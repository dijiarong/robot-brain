"""Frame sources for the passability VLM.

A :class:`FrameSource` produces a single JPEG frame on demand. Phase A ships
file-based sources (CI / manual smoke); Phase B adds :class:`Go2VideoFrameSource`
which taps the Unitree WebRTC video track.

All sources expose :attr:`kind` (for status/diagnostics) and a synchronous
:meth:`stop` (idempotent; cancels background work). :attr:`last_frame_monotonic`
records when the most recent frame was captured so callers can report frame age.
"""
from __future__ import annotations

import asyncio
import logging
import time
from abc import ABC, abstractmethod
from io import BytesIO
from pathlib import Path
from typing import Any

logger = logging.getLogger(__name__)


class FrameSource(ABC):
    """Produces a JPEG frame for VLM analysis, or ``None`` if unavailable."""

    #: Short identifier for status/diagnostics (e.g. "file", "go2_tap", "null").
    kind: str = "unknown"
    #: ``time.monotonic()`` of the most recent captured frame, or ``None``.
    last_frame_monotonic: float | None = None

    @abstractmethod
    async def get_frame(self) -> bytes | None: ...

    def stop(self) -> None:
        """Synchronously release background resources. Idempotent, no-op by default."""

    @property
    def frame_age_ms(self) -> float | None:
        """Milliseconds since the last captured frame, or ``None`` if never."""
        if self.last_frame_monotonic is None:
            return None
        return (time.monotonic() - self.last_frame_monotonic) * 1000.0


class NullFrameSource(FrameSource):
    """Always returns ``None`` - graceful fallback when no camera is wired."""

    kind = "null"

    async def get_frame(self) -> bytes | None:
        return None


class FileFrameSource(FrameSource):
    """Reads a JPEG/PNG from disk on each call.

    Useful for CI fixtures and manual VLM smoke tests against a still image.
    """

    kind = "file"

    def __init__(self, path: str | Path) -> None:
        self._path = Path(path)

    async def get_frame(self) -> bytes | None:
        try:
            data = self._path.read_bytes()
        except OSError:
            return None
        self.last_frame_monotonic = time.monotonic()
        return data


#: Alias for clarity in test/CI wiring.
MockFrameSource = FileFrameSource


class Go2VideoFrameSource(FrameSource):
    """Taps the Unitree WebRTC video track and keeps the latest JPEG frame.

    A background drain task consumes ``track.recv()`` (av.VideoFrame -> PIL ->
    JPEG) and caches only the newest frame; :meth:`get_frame` returns a copy
    without blocking the drain. Explore reads at most ~1 Hz, so this is cheap.

    The av/PIL conversion is CPU-bound, so it runs in a worker thread instead
    of the asyncio event loop; otherwise encoding a frame would block teleop
    and HTTP handlers for tens of milliseconds per frame.

    Note: an aiortc video track delivers each frame to a single ``recv()``
    caller, so this source should be the track's consumer when VLM is active.
    Coexisting with the RTP relay on the same track requires a tee (future
    work); for now register this tap instead of, or alongside, the relay and
    accept that frames are split between consumers.
    """

    kind = "go2_tap"

    def __init__(self, *, minimum_frame_interval_s: float = 0.35) -> None:
        if minimum_frame_interval_s < 0:
            raise ValueError("minimum frame interval cannot be negative")
        self._lock = asyncio.Lock()
        self._latest_jpeg: bytes | None = None
        self._task: asyncio.Task[None] | None = None
        self._minimum_frame_interval_s = minimum_frame_interval_s
        self._requested_until = time.monotonic() + 0.35

    def attach_track(self, track: Any) -> None:
        """Start draining *track* if not already consuming one."""
        if self._task is not None and not self._task.done():
            return
        self._task = asyncio.create_task(self._consume(track))

    async def _consume(self, track: Any) -> None:
        try:
            while True:
                frame = await track.recv()
                now = time.monotonic()
                if now > self._requested_until:
                    continue
                if (
                    self.last_frame_monotonic is not None
                    and now-self.last_frame_monotonic < self._minimum_frame_interval_s
                ):
                    continue
                jpeg = await asyncio.to_thread(self._frame_to_jpeg, frame)
                if jpeg is not None:
                    async with self._lock:
                        self._latest_jpeg = jpeg
                        self.last_frame_monotonic = time.monotonic()
        except asyncio.CancelledError:
            raise
        except Exception as exc:  # noqa: BLE001 - track ended / decode error
            logger.warning("Go2VideoFrameSource drain stopped: %s", exc)

    @staticmethod
    def _frame_to_jpeg(frame: Any) -> bytes | None:
        try:
            img = frame.to_image()  # av.VideoFrame -> PIL.Image
            img.thumbnail((640, 480))
            buf = BytesIO()
            img.save(buf, format="JPEG", quality=72)
            return buf.getvalue()
        except Exception as exc:  # noqa: BLE001
            logger.debug("frame->jpeg failed: %s", exc)
            return None

    async def get_frame(self) -> bytes | None:
        self._requested_until = time.monotonic() + 0.35
        async with self._lock:
            return self._latest_jpeg

    def stop(self) -> None:
        if self._task is not None and not self._task.done():
            self._task.cancel()

    async def aclose(self) -> None:
        """Cancel and join the decoder task so no media work survives shutdown."""
        task = self._task
        self._task = None
        if task is None:
            return
        if not task.done():
            task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            pass
