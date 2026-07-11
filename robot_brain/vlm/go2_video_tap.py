"""Register the VLM frame tap on a Unitree WebRTC connection.

Mirrors ``robot_brain.media.go2_video_relay``: call after ``await conn.connect()``.
The tap attaches the Go2 video track to a :class:`Go2VideoFrameSource` so the
explore loop can grab a single JPEG for passability analysis.

Single-consumer note: an aiortc video track feeds one ``recv()`` caller. If the
RTP relay is also consuming the same track, frames are split between them. For
smooth browser video *and* VLM, a tee is needed (future work); until then,
prefer this tap when VLM is the priority, or accept degraded relay frames.
"""
from __future__ import annotations

import logging
from typing import TYPE_CHECKING, Any

logger = logging.getLogger(__name__)

if TYPE_CHECKING:
    from robot_brain.vlm.frame_source import Go2VideoFrameSource


def register_go2_frame_tap(conn: Any, frame_source: "Go2VideoFrameSource") -> None:
    """Register a callback so video tracks arriving after connect feed the source."""
    video = getattr(conn, "video", None)
    if video is None:
        logger.warning("[VLM] conn.video not ready - frame tap skipped")
        return

    async def _on_track(track: Any) -> None:
        if getattr(track, "kind", None) != "video":
            return
        frame_source.attach_track(track)
        logger.info("[VLM] Go2 video track attached to passability frame source")

    video.add_track_callback(_on_track)
    _attach_existing(conn, frame_source)


def _attach_existing(conn: Any, frame_source: "Go2VideoFrameSource") -> None:
    """Attach video tracks already on the peer connection (post-connect)."""
    pc = getattr(conn, "pc", None)
    if pc is None:
        return
    started = 0
    for receiver in pc.getReceivers():
        track = getattr(receiver, "track", None)
        if track is None or getattr(track, "kind", None) != "video":
            continue
        frame_source.attach_track(track)
        started += 1
    if started:
        logger.info("[VLM] attached %d existing Go2 video track(s) to frame source", started)


def prime_go2_video_for_passability(conn: Any, frame_source: "Go2VideoFrameSource") -> None:
    """Enable the Go2 front camera and register the VLM frame tap.

    Call after ``await conn.connect()``. Equivalent to
    ``prime_go2_video_for_connect`` but feeds the VLM frame source instead of a
    drain/relay.
    """
    video = getattr(conn, "video", None)
    if video is not None and hasattr(video, "switchVideoChannel"):
        video.switchVideoChannel(True)
        logger.info("[VLM] Go2 front camera channel enabled")
    register_go2_frame_tap(conn, frame_source)
