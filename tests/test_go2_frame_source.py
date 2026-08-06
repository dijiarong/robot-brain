"""Tests for Go2VideoFrameSource (WebRTC track -> latest JPEG)."""
from __future__ import annotations

import asyncio
import unittest

import pytest

# av / numpy / PIL are optional deps (only for the Go2 frame source). Skip the
# whole module gracefully on a minimal install instead of erroring at import.
pytest.importorskip("numpy")
pytest.importorskip("av")
pytest.importorskip("PIL")

import av  # noqa: E402
import numpy as np  # noqa: E402

from robot_brain.vlm.frame_source import Go2VideoFrameSource


def _make_frame(rgb: tuple[int, int, int]) -> av.VideoFrame:
    arr = np.zeros((8, 8, 3), dtype=np.uint8)
    arr[:, :] = rgb
    return av.VideoFrame.from_ndarray(arr, format="rgb24")


class _FakeTrack:
    """Async video track that yields a fixed list of frames, then ends."""

    kind = "video"

    def __init__(self, frames: list[av.VideoFrame]) -> None:
        self._frames = list(frames)

    async def recv(self) -> av.VideoFrame:
        if not self._frames:
            raise RuntimeError("track ended")
        return self._frames.pop(0)


class Go2VideoFrameSourceTests(unittest.IsolatedAsyncioTestCase):
    async def test_get_frame_none_before_attach(self):
        fs = Go2VideoFrameSource()
        self.assertIsNone(await fs.get_frame())

    async def test_drain_captures_latest_jpeg(self):
        fs = Go2VideoFrameSource(minimum_frame_interval_s=0.0)
        track = _FakeTrack([_make_frame((255, 0, 0)), _make_frame((0, 255, 0))])
        fs.attach_track(track)
        # Let the drain task process both frames.
        await asyncio.sleep(0.05)
        jpeg = await fs.get_frame()
        self.assertIsNotNone(jpeg)
        self.assertTrue(jpeg.startswith(b"\xff\xd8"))  # JPEG SOI marker

        # Verify it decoded to the last frame (green).
        from io import BytesIO

        from PIL import Image

        img = Image.open(BytesIO(jpeg)).convert("RGB")
        px = img.getpixel((0, 0))
        self.assertGreater(px[1], px[0])  # green > red

    async def test_drain_stops_on_track_end(self):
        fs = Go2VideoFrameSource()
        fs.attach_track(_FakeTrack([_make_frame((0, 0, 255))]))
        await asyncio.sleep(0.05)
        # Drain task ended cleanly; last frame remains available.
        self.assertIsNotNone(await fs.get_frame())
        self.assertIsNotNone(fs._task)
        self.assertTrue(fs._task.done())

    async def test_stop_cancels_drain(self):
        fs = Go2VideoFrameSource()
        # A track that never ends (blocks on recv).
        class _Blocking:
            kind = "video"

            async def recv(self):
                await asyncio.sleep(3600)
                raise RuntimeError("unreachable")

        fs.attach_track(_Blocking())
        await asyncio.sleep(0.01)
        fs.stop()
        await asyncio.sleep(0.01)
        self.assertTrue(fs._task.done() or fs._task.cancelled())


if __name__ == "__main__":
    unittest.main()
