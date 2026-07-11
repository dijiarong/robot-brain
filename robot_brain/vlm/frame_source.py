"""Frame sources for the passability VLM.

A :class:`FrameSource` produces a single JPEG frame on demand. Phase A ships
file-based sources (CI / manual smoke); Phase B adds :class:`Go2VideoFrameSource`
which taps the Unitree WebRTC video track.
"""
from __future__ import annotations

import asyncio
import logging
from abc import ABC, abstractmethod
from io import BytesIO
from pathlib import Path
from typing import Any

logger = logging.getLogger(__name__)


class FrameSource(ABC):
    """Produces a JPEG frame for VLM analysis, or ``None`` if unavailable."""

    @abstractmethod
    async def get_frame(self) -> bytes | None: ...


class NullFrameSource(FrameSource):
    """Always returns ``None`` - graceful fallback when no camera is wired."""

    async def get_frame(self) -> bytes | None:
        return None


class FileFrameSource(FrameSource):
    """Reads a JPEG/PNG from disk on each call.

    Useful for CI fixtures and manual VLM smoke tests against a still image.
    """

    def __init__(self, path: str | Path) -> None:
        self._path = Path(path)

    async def get_frame(self) -> bytes | None:
        try:
            return self._path.read_bytes()
        except OSError:
            return None


#: Alias for clarity in test/CI wiring.
MockFrameSource = FileFrameSource


class Go2VideoFrameSource(FrameSource):
    """Taps the Unitree WebRTC video track and keeps the latest JPEG frame.

    A background drain task consumes ``track.recv()`` (av.VideoFrame -> PIL ->
    JPEG) and caches only the newest frame; :meth:`get_frame` returns a copy
    without blocking the drain. Explore reads at most ~1 Hz, so this is cheap.

    Note: an aiortc video track delivers each frame to a single ``recv()``
    caller, so this source should be the track's consumer when VLM is active.
    Coexisting with the RTP relay on the same track requires a tee (future
    work); for now register this tap instead of, or alongside, the relay and
    accept that frames are split between consumers.
    """

    def __init__(self) -> None:
        self._lock = asyncio.Lock()
        self._latest_jpeg: bytes | None = None
        self._task: asyncio.Task[None] | None = None

    def attach_track(self, track: Any) -> None:
        """Start draining *track* if not already consuming one."""
        if self._task is not None and not self._task.done():
            return
        self._task = asyncio.create_task(self._consume(track))

    async def _consume(self, track: Any) -> None:
        try:
            while True:
                frame = await track.recv()
                jpeg = self._frame_to_jpeg(frame)
                if jpeg is not None:
                    async with self._lock:
                        self._latest_jpeg = jpeg
        except asyncio.CancelledError:
            raise
        except Exception as exc:  # noqa: BLE001 - track ended / decode error
            logger.warning("Go2VideoFrameSource drain stopped: %s", exc)

    @staticmethod
    def _frame_to_jpeg(frame: Any) -> bytes | None:
        try:
            img = frame.to_image()  # av.VideoFrame -> PIL.Image
            buf = BytesIO()
            img.save(buf, format="JPEG", quality=85)
            return buf.getvalue()
        except Exception as exc:  # noqa: BLE001
            logger.debug("frame->jpeg failed: %s", exc)
            return None

    async def get_frame(self) -> bytes | None:
        async with self._lock:
            return self._latest_jpeg

    def stop(self) -> None:
        if self._task is not None and not self._task.done():
            self._task.cancel()

