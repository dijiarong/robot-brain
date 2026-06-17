"""Bidirectional Go2 audio bridge for topsun_robot_service.

Outbound (Go2 mic → browser):
  aiortc audio frames → ffmpeg → Opus RTP on UDP :5005 (bin/robot audioTrack input)

Inbound (browser mic → Go2 speaker):
  Opus RTP on UDP :5010 ← bin/robot (browser mic) → ffmpeg decode → aiortc outbound track
"""
from __future__ import annotations

import asyncio
import fractions
import logging
import os
import shutil
import subprocess
import tempfile
from typing import Any

import av
import numpy as np
from aiortc import MediaStreamTrack

logger = logging.getLogger(__name__)

_PCM_CHUNK_BYTES = 3840  # 20 ms @ 48 kHz stereo s16le


def _ffmpeg_available() -> bool:
    return shutil.which("ffmpeg") is not None


class _PcmQueueAudioTrack(MediaStreamTrack):
    kind = "audio"

    def __init__(self) -> None:
        super().__init__()
        self._queue: asyncio.Queue[bytes | None] = asyncio.Queue(maxsize=100)
        self._timestamp = 0

    async def push_pcm(self, data: bytes) -> None:
        if not data:
            return
        try:
            self._queue.put_nowait(data)
        except asyncio.QueueFull:
            try:
                _ = self._queue.get_nowait()
            except asyncio.QueueEmpty:
                pass
            self._queue.put_nowait(data)

    async def end(self) -> None:
        await self._queue.put(None)

    async def recv(self) -> av.AudioFrame:
        chunk = await self._queue.get()
        if chunk is None:
            raise Exception("audio ingress ended")

        frame = av.AudioFrame(format="s16", layout="stereo", samples=960)
        frame.sample_rate = 48000
        frame.planes[0].update(chunk[: _PCM_CHUNK_BYTES].ljust(_PCM_CHUNK_BYTES, b"\x00"))
        frame.pts = self._timestamp
        frame.time_base = fractions.Fraction(1, 48000)
        self._timestamp += 960
        return frame


def _schedule(coro: Any) -> None:
    try:
        loop = asyncio.get_running_loop()
    except RuntimeError:
        loop = asyncio.get_event_loop()
    loop.create_task(coro)


async def _relay_audio_frame_to_rtp(
    frame: Any,
    *,
    host: str,
    port: int,
    proc_holder: dict[str, Any],
) -> None:
    """Pipe one aiortc AudioFrame into a long-lived ffmpeg Opus/RTP encoder."""
    proc = proc_holder.get("proc")
    if proc is None or proc.returncode is not None:
        cmd = [
            "ffmpeg",
            "-loglevel",
            "error",
            "-y",
            "-f",
            "s16le",
            "-ar",
            "48000",
            "-ac",
            "2",
            "-i",
            "pipe:0",
            "-c:a",
            "libopus",
            "-b:a",
            "64k",
            "-application",
            "voip",
            "-payload_type",
            "96",
            "-f",
            "rtp",
            f"rtp://{host}:{port}?pkt_size=1200",
        ]
        proc = await asyncio.create_subprocess_exec(
            *cmd,
            stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
        )
        proc_holder["proc"] = proc
        logger.info("[AudioRelay] Go2 mic → rtp://%s:%d", host, port)

    assert proc.stdin is not None
    pcm = _audio_frame_to_s16le_stereo(frame)
    proc.stdin.write(pcm)
    await proc.stdin.drain()


def _audio_frame_to_s16le_stereo(frame: Any) -> bytes:
    """Normalize aiortc/av audio to 48 kHz stereo s16le (960 samples = 20 ms)."""
    if hasattr(frame, "reformat"):
        frame = frame.reformat(format="s16", layout="stereo", rate=48000)
    arr = frame.to_ndarray()
    if arr.ndim == 1:
        stereo = arr
    elif arr.shape[0] <= 8:
        stereo = arr.reshape(-1)
    else:
        stereo = arr.T.reshape(-1)
    target_samples = 960 * 2
    if len(stereo) < target_samples:
        stereo = np.pad(stereo, (0, target_samples - len(stereo)))
    elif len(stereo) > target_samples:
        stereo = stereo[:target_samples]
    return np.asarray(stereo, dtype=np.int16).tobytes()


def register_go2_audio_relay(conn: Any, *, host: str = "127.0.0.1", port: int = 5005) -> None:
    """Register callback on conn.audio for Go2 → browser relay."""
    audio = getattr(conn, "audio", None)
    if audio is None:
        logger.warning("[AudioRelay] conn.audio not ready — skip outbound relay")
        return

    proc_holder: dict[str, Any] = {}

    async def _on_frame(frame: Any) -> None:
        try:
            await _relay_audio_frame_to_rtp(frame, host=host, port=port, proc_holder=proc_holder)
        except Exception as exc:
            logger.exception("[AudioRelay] outbound frame failed: %s", exc)

    audio.add_track_callback(_on_frame)
    logger.info("[AudioRelay] outbound callback registered → %s:%d", host, port)


async def _attach_outbound_audio_track(conn: Any, track: MediaStreamTrack) -> None:
    pc = getattr(conn, "pc", None)
    if pc is None:
        logger.warning("[AudioRelay] no peer connection — skip outbound track")
        return
    for transceiver in pc.getTransceivers():
        if transceiver.kind != "audio":
            continue
        # aiortc replaceTrack is synchronous (not a coroutine).
        transceiver.sender.replaceTrack(track)
        logger.info("[AudioRelay] outbound audio track attached to Go2 WebRTC")
        return
    logger.warning("[AudioRelay] no audio transceiver found on Go2 connection")


async def _start_audio_ingress(
    conn: Any,
    track: _PcmQueueAudioTrack,
    *,
    host: str,
    port: int,
) -> None:
    """Decode Opus RTP from bin/robot and feed Go2 outbound audio track."""
    sdp = (
        "v=0\r\n"
        f"o=- 0 0 IN IP4 {host}\r\n"
        "s=browser-mic\r\n"
        f"c=IN IP4 {host}\r\n"
        "t=0 0\r\n"
        f"m=audio {port} RTP/AVP 96\r\n"
        "a=rtpmap:96 opus/48000/2\r\n"
    )
    with tempfile.NamedTemporaryFile("w", suffix=".sdp", delete=False) as f:
        f.write(sdp)
        sdp_path = f.name

    cmd = [
        "ffmpeg",
        "-loglevel",
        "error",
        "-protocol_whitelist",
        "file,udp,rtp",
        "-i",
        sdp_path,
        "-f",
        "s16le",
        "-ar",
        "48000",
        "-ac",
        "2",
        "pipe:1",
    ]
    proc = await asyncio.create_subprocess_exec(
        *cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    logger.info("[AudioRelay] browser mic ingress listening on %s:%d → Go2 speaker", host, port)

    try:
        assert proc.stdout is not None
        while True:
            chunk = await proc.stdout.read(_PCM_CHUNK_BYTES)
            if not chunk:
                break
            await track.push_pcm(chunk)
    finally:
        await track.end()
        await proc.wait()
        try:
            os.unlink(sdp_path)
        except OSError:
            pass
        if proc.returncode not in (0, None):
            err = (await proc.stderr.read()).decode(errors="replace") if proc.stderr else ""
            logger.warning("[AudioRelay] ingress ffmpeg exit %s: %s", proc.returncode, err.strip())


class _SilentOutboundAudioTrack(MediaStreamTrack):
    """Placeholder outbound track until browser mic is attached via media_bridge."""

    kind = "audio"

    def __init__(self) -> None:
        super().__init__()
        self._timestamp = 0

    async def recv(self) -> av.AudioFrame:
        await asyncio.sleep(0.02)
        frame = av.AudioFrame(format="s16", layout="stereo", samples=960)
        frame.sample_rate = 48000
        frame.planes[0].update(b"\x00" * _PCM_CHUNK_BYTES)
        frame.pts = self._timestamp
        frame.time_base = fractions.Fraction(1, 48000)
        self._timestamp += 960
        return frame


def prime_go2_audio_for_connect(conn: Any) -> None:
    """Enable Go2 audio + attach silent outbound track (gateway, no ffmpeg/UDP)."""
    audio = getattr(conn, "audio", None)
    if audio is not None and hasattr(audio, "switchAudioChannel"):
        audio.switchAudioChannel(True)
        logger.info("[AudioRelay] Go2 audio channel enabled (in-process)")

    outbound = _SilentOutboundAudioTrack()

    async def _run() -> None:
        await _attach_outbound_audio_track(conn, outbound)

    _schedule(_run())


def start_go2_audio_relay(
    conn: Any,
    *,
    relay_host: str = "127.0.0.1",
    relay_port: int = 5005,
    ingress_host: str = "127.0.0.1",
    ingress_port: int = 5010,
) -> None:
    """Enable Go2 audio channel + bidirectional relay (call after await conn.connect())."""
    if not _ffmpeg_available():
        logger.warning(
            "[AudioRelay] ffmpeg not found — install ffmpeg for bidirectional voice "
            "(brew install ffmpeg)."
        )
        return

    audio = getattr(conn, "audio", None)
    if audio is not None and hasattr(audio, "switchAudioChannel"):
        audio.switchAudioChannel(True)
        logger.info("[AudioRelay] Go2 audio channel enabled")

    register_go2_audio_relay(conn, host=relay_host, port=relay_port)

    outbound = _PcmQueueAudioTrack()

    async def _run() -> None:
        await _attach_outbound_audio_track(conn, outbound)
        await _start_audio_ingress(conn, outbound, host=ingress_host, port=ingress_port)

    _schedule(_run())
