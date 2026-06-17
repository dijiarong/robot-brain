"""In-process relay between Go2 aiortc connection and browser aiortc peer (no ffmpeg/UDP).

Optimizations:
- Cached transceiver lookup: audio senders/receivers are resolved once per
  Go2 connection and reused across browser peers — avoids O(N) iteration per attach.
- QueuedAudioTrack: bounded queue with oldest-first eviction to prevent memory
  growth when a browser peer is slow to consume.
"""
from __future__ import annotations

import asyncio
import fractions
import logging
from typing import Any

import av
from aiortc import MediaStreamTrack, RTCPeerConnection

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Track relay helpers
# ---------------------------------------------------------------------------


class RelayStreamTrack(MediaStreamTrack):
    """Forward frames from a source MediaStreamTrack (same event loop)."""

    def __init__(self, source: MediaStreamTrack) -> None:
        super().__init__()
        self._source = source
        self.kind = source.kind

    async def recv(self) -> Any:
        return await self._source.recv()


class QueuedAudioTrack(MediaStreamTrack):
    """Bounded async queue for decoded audio frames.

    When the queue is full, the oldest frame is dropped (ring-buffer behaviour)
    to prevent unbounded memory growth if a browser peer is slow to consume.
    """

    kind = "audio"

    def __init__(self, maxsize: int = 50) -> None:
        super().__init__()
        self._queue: asyncio.Queue[Any | None] = asyncio.Queue(maxsize=maxsize)
        self._timestamp = 0

    async def push_frame(self, frame: Any) -> None:
        try:
            self._queue.put_nowait(frame)
        except asyncio.QueueFull:
            # Drop oldest frame to make room (bounded memory).
            try:
                _ = self._queue.get_nowait()
            except asyncio.QueueEmpty:
                pass
            self._queue.put_nowait(frame)

    async def close(self) -> None:
        await self._queue.put(None)

    async def recv(self) -> Any:
        frame = await self._queue.get()
        if frame is None:
            raise Exception("audio relay closed")
        frame.pts = self._timestamp
        frame.time_base = fractions.Fraction(1, frame.sample_rate)
        self._timestamp += frame.samples
        return frame


# ---------------------------------------------------------------------------
# Cached transceiver handles
# ---------------------------------------------------------------------------


class _Go2MediaCache:
    """Resolve Go2 media transceivers once and reuse across browser attaches."""

    def __init__(self, go2_conn: Any) -> None:
        self.go2_conn = go2_conn
        self._go2_pc: Any = None
        self._video_receivers: list[Any] = []
        self._audio_sender: Any = None  # single audio transceiver sender for mic relay
        self._resolved = False

    @property
    def go2_pc(self) -> Any:
        if self._go2_pc is None:
            self._go2_pc = getattr(self.go2_conn, "pc", None)
        return self._go2_pc

    def resolve(self) -> None:
        """Scan transceivers once. Safe to call multiple times (idempotent)."""
        if self._resolved:
            return
        pc = self.go2_pc
        if pc is None:
            return

        # Cache video receivers
        self._video_receivers = [
            r for r in pc.getReceivers()
            if getattr(getattr(r, "track", None), "kind", None) == "video"
        ]

        # Cache audio sender
        for t in pc.getTransceivers():
            if t.kind == "audio":
                self._audio_sender = t.sender
                break

        self._resolved = True
        logger.info(
            "[Gateway] Go2 media cache resolved: %d video, audio=%s",
            len(self._video_receivers),
            "found" if self._audio_sender else "none",
        )

    def invalidate(self) -> None:
        """Clear cached handles (call after Go2 reconnect)."""
        self._go2_pc = None
        self._video_receivers.clear()
        self._audio_sender = None
        self._resolved = False


async def attach_go2_to_browser(
    go2_conn: Any,
    browser_pc: RTCPeerConnection,
    *,
    media_cache: _Go2MediaCache | None = None,
) -> None:
    """Enable Go2 A/V channels and attach relay tracks to the browser peer connection.

    Uses a shared ``_Go2MediaCache`` to avoid re-scanning transceivers for each
    browser peer attach. If not provided, a one-shot cache is created.
    """
    if media_cache is None:
        media_cache = _Go2MediaCache(go2_conn)

    # Enable Go2 media channels (idempotent).
    video = getattr(go2_conn, "video", None)
    audio = getattr(go2_conn, "audio", None)
    if video is not None and hasattr(video, "switchVideoChannel"):
        video.switchVideoChannel(True)
    if audio is not None and hasattr(audio, "switchAudioChannel"):
        audio.switchAudioChannel(True)

    media_cache.resolve()

    go2_pc = media_cache.go2_pc
    if go2_pc is None:
        raise RuntimeError("Go2 connection has no peer connection")

    # Video: relay Go2 camera → browser.
    for receiver in media_cache._video_receivers:
        track = getattr(receiver, "track", None)
        if track is None:
            continue
        browser_pc.addTrack(RelayStreamTrack(track))
        logger.info("[Gateway] relay Go2 video → browser")

    # Audio outbound: Go2 mic frames → browser.
    go2_out = QueuedAudioTrack()
    if audio is not None:
        async def _on_go2_audio(frame: Any) -> None:
            await go2_out.push_frame(frame)

        audio.add_track_callback(_on_go2_audio)
        browser_pc.addTrack(go2_out)
        logger.info("[Gateway] relay Go2 audio frames → browser")

    # Audio inbound: browser mic → Go2 speaker.
    audio_sender = media_cache._audio_sender

    @browser_pc.on("track")
    async def on_browser_track(track: MediaStreamTrack) -> None:
        if track.kind != "audio":
            return
        # Re-resolve the sender in case Go2 reconnected.
        sender = audio_sender
        if sender is None:
            media_cache.resolve()
            sender = media_cache._audio_sender
        if sender is None:
            logger.warning("[Gateway] no audio sender available for browser mic")
            return
        mic_relay = RelayStreamTrack(track)
        sender.replaceTrack(mic_relay)
        logger.info("[Gateway] browser mic → Go2 speaker")
