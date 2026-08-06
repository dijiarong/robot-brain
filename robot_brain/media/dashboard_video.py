"""Forward the Go2 WebRTC video track straight to dashboard browsers.

The dashboard originally polled ``/api/camera/frame`` for JPEG snapshots. Every
frame went through decode → PIL resize → JPEG encode → one HTTP round trip, and
:class:`~robot_brain.vlm.frame_source.Go2VideoFrameSource` additionally throttles
itself for the ~1 Hz VLM caller, so the panel could never show more than a few
frames per second. This bridge answers an SDP offer from the browser and relays
the robot's H.264 track untouched instead, so the panel plays the camera at the
robot's native frame rate with no transcoding.

Two constraints shape the implementation:

* The Go2 aiortc objects live on the transport's background event loop, and
  aiortc is not thread safe, so every negotiation step is scheduled there via
  ``transport.run_on_conn_loop``.
* An aiortc track delivers each frame to a single ``recv()`` caller, so the Go2
  track is fanned out through one shared :class:`~aiortc.contrib.media.MediaRelay`
  and browsers subscribe unbuffered (drop stale frames rather than grow latency).
"""
from __future__ import annotations

import asyncio
import logging
from typing import Any
from uuid import uuid4

logger = logging.getLogger(__name__)

# How long to wait for the Go2 to publish a video track after the camera
# channel is switched on.
_TRACK_WAIT_TIMEOUT_S = 6.0
_TRACK_POLL_INTERVAL_S = 0.2
# ICE gathering is host-candidate only on a LAN, so this only guards the
# pathological case where gathering never completes.
_ICE_GATHER_TIMEOUT_S = 5.0
_NEGOTIATE_TIMEOUT_S = 20.0


class DashboardVideoUnavailable(RuntimeError):
    """Raised when the Go2 camera cannot be negotiated for the dashboard."""


def _find_video_track(conn: Any) -> Any | None:
    """Return the first inbound video track on *conn*, or ``None``."""
    pc = getattr(conn, "pc", None)
    if pc is None:
        return None
    for receiver in pc.getReceivers():
        track = getattr(receiver, "track", None)
        if track is not None and getattr(track, "kind", None) == "video":
            return track
    return None


async def _wait_ice_gathering(pc: Any) -> None:
    if pc.iceGatheringState == "complete":
        return
    done: asyncio.Future[None] = asyncio.get_running_loop().create_future()

    @pc.on("icegatheringstatechange")
    async def _on_gathering() -> None:
        if pc.iceGatheringState == "complete" and not done.done():
            done.set_result(None)

    try:
        await asyncio.wait_for(done, timeout=_ICE_GATHER_TIMEOUT_S)
    except asyncio.TimeoutError:
        logger.warning(
            "[DashboardVideo] ICE gathering timed out (state=%s)", pc.iceGatheringState
        )


class DashboardVideoBridge:
    """Answers dashboard SDP offers with the live Go2 camera track.

    Peers negotiate on host candidates only: the browser already has a route to
    this process (it fetched the dashboard over HTTP), so no STUN/TURN detour is
    needed and the answer is not delayed by gathering reflexive candidates.
    """

    def __init__(self, transport: Any) -> None:
        self._transport = transport
        self._relay: Any = None
        self._peers: dict[str, Any] = {}

    @property
    def peer_count(self) -> int:
        return len(self._peers)

    async def offer(self, sdp: str, sdp_type: str = "offer") -> dict[str, str]:
        """Negotiate one browser peer and return the local answer."""
        if sdp_type != "offer":
            raise DashboardVideoUnavailable(f"unsupported sdp type '{sdp_type}'")
        return await self._transport.run_on_conn_loop(
            self._negotiate(sdp, sdp_type), timeout=_NEGOTIATE_TIMEOUT_S
        )

    async def close_all(self) -> None:
        """Close every browser peer; safe to call repeatedly."""
        if not self._peers:
            return
        try:
            await self._transport.run_on_conn_loop(self._close_peers(), timeout=10.0)
        except Exception as exc:  # noqa: BLE001 - shutdown is best effort
            logger.warning("[DashboardVideo] peer shutdown failed: %s", exc)
            self._peers.clear()

    # ------------------------------------------------------- Go2 connection loop
    async def _negotiate(self, sdp: str, sdp_type: str) -> dict[str, str]:
        from aiortc import RTCPeerConnection, RTCSessionDescription

        track = await self._await_video_track()
        pc = RTCPeerConnection()
        peer_id = uuid4().hex
        self._peers[peer_id] = pc

        @pc.on("connectionstatechange")
        async def _on_state() -> None:
            state = pc.connectionState
            logger.info("[DashboardVideo] peer %s state=%s", peer_id[:8], state)
            if state in ("failed", "closed", "disconnected"):
                self._peers.pop(peer_id, None)
                await _close_quietly(pc)

        try:
            pc.addTrack(self._subscribe(track))
            await pc.setRemoteDescription(RTCSessionDescription(sdp=sdp, type=sdp_type))
            await pc.setLocalDescription(await pc.createAnswer())
            await _wait_ice_gathering(pc)
        except Exception as exc:
            self._peers.pop(peer_id, None)
            await _close_quietly(pc)
            raise DashboardVideoUnavailable(f"WebRTC negotiation failed: {exc}") from exc

        logger.info("[DashboardVideo] answered dashboard peer %s", peer_id[:8])
        return {
            "sdp": pc.localDescription.sdp,
            "type": pc.localDescription.type,
            "peer_id": peer_id,
        }

    async def _await_video_track(self) -> Any:
        """Enable the camera channel and wait for the inbound video track."""
        conn = getattr(self._transport, "webrtc_conn", None)
        if conn is None:
            raise DashboardVideoUnavailable("Go2 WebRTC connection is not ready")

        video = getattr(conn, "video", None)
        if video is not None and hasattr(video, "switchVideoChannel"):
            video.switchVideoChannel(True)

        deadline = asyncio.get_running_loop().time() + _TRACK_WAIT_TIMEOUT_S
        while True:
            track = _find_video_track(conn)
            if track is not None:
                return track
            if asyncio.get_running_loop().time() >= deadline:
                raise DashboardVideoUnavailable(
                    "Go2 published no video track; check the camera channel"
                )
            await asyncio.sleep(_TRACK_POLL_INTERVAL_S)

    def _subscribe(self, track: Any) -> Any:
        """Fan the Go2 track out through the shared relay, newest frame wins."""
        from aiortc.contrib.media import MediaRelay

        if self._relay is None:
            self._relay = MediaRelay()
        return self._relay.subscribe(track, buffered=False)

    async def _close_peers(self) -> None:
        peers = list(self._peers.values())
        self._peers.clear()
        for pc in peers:
            await _close_quietly(pc)


async def _close_quietly(pc: Any) -> None:
    try:
        await pc.close()
    except Exception as exc:  # noqa: BLE001 - teardown is best effort
        logger.debug("[DashboardVideo] pc.close() error: %s", exc)
