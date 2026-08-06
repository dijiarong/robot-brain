"""Tests for the dashboard WebRTC camera bridge (Go2 track -> browser peer)."""
from __future__ import annotations

import unittest
import unittest.mock

import pytest

# aiortc is only installed with the unitree-webrtc extra.
pytest.importorskip("aiortc")

from aiortc import RTCPeerConnection, VideoStreamTrack  # noqa: E402

from robot_brain.media import dashboard_video  # noqa: E402
from robot_brain.media.dashboard_video import (  # noqa: E402
    DashboardVideoBridge,
    DashboardVideoUnavailable,
)


class _FakeReceiver:
    def __init__(self, track) -> None:
        self.track = track


class _FakePeerConnection:
    def __init__(self, track) -> None:
        self._receivers = [_FakeReceiver(track)] if track is not None else []

    def getReceivers(self):
        return self._receivers


class _FakeVideoChannel:
    def __init__(self) -> None:
        self.enabled = False

    def switchVideoChannel(self, on: bool) -> None:
        self.enabled = bool(on)


class _FakeGo2Connection:
    def __init__(self, track) -> None:
        self.pc = _FakePeerConnection(track)
        self.video = _FakeVideoChannel()


class _FakeTransport:
    """Stands in for the WebRTC transport; runs coroutines on the test loop."""

    def __init__(self, conn) -> None:
        self.webrtc_conn = conn

    async def run_on_conn_loop(self, coro, *, timeout: float = 30.0):
        return await coro


async def _browser_offer() -> tuple[RTCPeerConnection, str]:
    pc = RTCPeerConnection()
    pc.addTransceiver("video", direction="recvonly")
    await pc.setLocalDescription(await pc.createOffer())
    return pc, pc.localDescription.sdp


class DashboardVideoBridgeTests(unittest.IsolatedAsyncioTestCase):
    async def test_offer_answers_with_the_go2_video_track(self) -> None:
        conn = _FakeGo2Connection(VideoStreamTrack())
        bridge = DashboardVideoBridge(_FakeTransport(conn))
        browser, sdp = await _browser_offer()
        try:
            answer = await bridge.offer(sdp, "offer")
            self.assertEqual("answer", answer["type"])
            self.assertIn("m=video", answer["sdp"])
            self.assertTrue(conn.video.enabled)
            self.assertEqual(1, bridge.peer_count)
        finally:
            await bridge.close_all()
            await browser.close()
        self.assertEqual(0, bridge.peer_count)

    async def test_offer_without_connection_is_unavailable(self) -> None:
        bridge = DashboardVideoBridge(_FakeTransport(None))
        with self.assertRaises(DashboardVideoUnavailable):
            await bridge.offer("v=0", "offer")

    async def test_offer_rejects_non_offer_sdp(self) -> None:
        bridge = DashboardVideoBridge(_FakeTransport(_FakeGo2Connection(None)))
        with self.assertRaises(DashboardVideoUnavailable):
            await bridge.offer("v=0", "answer")

    async def test_missing_video_track_reports_reason(self) -> None:
        conn = _FakeGo2Connection(None)
        bridge = DashboardVideoBridge(_FakeTransport(conn))
        with unittest.mock.patch.object(
            dashboard_video, "_TRACK_WAIT_TIMEOUT_S", 0.05
        ):
            with self.assertRaises(DashboardVideoUnavailable) as ctx:
                await bridge.offer("v=0", "offer")
        self.assertIn("no video track", str(ctx.exception))
        self.assertTrue(conn.video.enabled)


if __name__ == "__main__":
    unittest.main()
