"""Relay Go2 front camera (Unitree WebRTC video track) to local H.264 RTP.

topsun_robot_service listens on UDP :5000 for H.264 RTP and forwards it to
browser clients over its own WebRTC peer connection.
"""
from __future__ import annotations

import asyncio
import logging
import shutil
import subprocess
from typing import Any

logger = logging.getLogger(__name__)


def _ffmpeg_available() -> bool:
    return shutil.which("ffmpeg") is not None


def _schedule_relay(track: Any, *, host: str, port: int) -> None:
    async def _run() -> None:
        logger.info("[VideoRelay] Go2 video track ready → rtp://%s:%d", host, port)
        try:
            await consume_go2_video_track(track, host=host, port=port)
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            logger.exception("[VideoRelay] relay stopped: %s", exc)

    try:
        loop = asyncio.get_running_loop()
    except RuntimeError:
        loop = asyncio.get_event_loop()
    loop.create_task(_run())


async def _drain_go2_video_track(track: Any) -> None:
    """Consume Go2 video frames in-process (no ffmpeg) so aiortc keeps flowing."""
    try:
        while True:
            await track.recv()
    except asyncio.CancelledError:
        raise
    except Exception:
        pass


def _schedule_video_drain(track: Any) -> None:
    async def _run() -> None:
        await _drain_go2_video_track(track)

    try:
        loop = asyncio.get_running_loop()
    except RuntimeError:
        loop = asyncio.get_event_loop()
    loop.create_task(_run())


def register_go2_video_relay(conn: Any, *, host: str = "127.0.0.1", port: int = 5000) -> None:
    """Register callback for video tracks that arrive after connect()."""
    video = getattr(conn, "video", None)
    if video is None:
        logger.warning("[VideoRelay] conn.video not ready — skip callback registration")
        return

    async def _on_track(track: Any) -> None:
        _schedule_relay(track, host=host, port=port)

    video.add_track_callback(_on_track)
    logger.info("[VideoRelay] callback registered")


def attach_existing_go2_video_tracks(
    conn: Any, *, host: str = "127.0.0.1", port: int = 5000
) -> int:
    """Start relay/drain for video tracks already on the peer connection (post-connect)."""
    pc = getattr(conn, "pc", None)
    if pc is None:
        return 0
    started = 0
    for receiver in pc.getReceivers():
        track = getattr(receiver, "track", None)
        if track is None or getattr(track, "kind", None) != "video":
            continue
        if port == 0:
            _schedule_video_drain(track)
        else:
            _schedule_relay(track, host=host, port=port)
        started += 1
    if started:
        label = "draining" if port == 0 else "attached"
        logger.info("[VideoRelay] %s %d existing Go2 video track(s)", label, started)
    return started


def prime_go2_video_for_connect(conn: Any) -> None:
    """Enable Go2 camera + drain tracks in-process (gateway mode, no ffmpeg/UDP)."""
    video = getattr(conn, "video", None)
    if video is not None and hasattr(video, "switchVideoChannel"):
        video.switchVideoChannel(True)
        logger.info("[VideoRelay] Go2 front camera channel enabled (in-process)")

    if video is None:
        return

    async def _on_track(track: Any) -> None:
        _schedule_video_drain(track)

    video.add_track_callback(_on_track)
    attach_existing_go2_video_tracks(conn, host="127.0.0.1", port=0)


def start_go2_video_relay(conn: Any, *, host: str = "127.0.0.1", port: int = 5000) -> None:
    """Enable Go2 front camera and relay to local RTP (call after await conn.connect())."""
    if not _ffmpeg_available():
        logger.warning(
            "[VideoRelay] ffmpeg not found — install ffmpeg to enable Go2 camera relay "
            "(brew install ffmpeg). Browser video will stay black."
        )
        return

    video = getattr(conn, "video", None)
    if video is not None and hasattr(video, "switchVideoChannel"):
        video.switchVideoChannel(True)
        logger.info("[VideoRelay] Go2 front camera channel enabled")

    register_go2_video_relay(conn, host=host, port=port)
    attach_existing_go2_video_tracks(conn, host=host, port=port)


async def consume_go2_video_track(track: Any, *, host: str, port: int) -> None:
    """Read aiortc video frames and pipe raw YUV into ffmpeg → RTP/UDP."""
    first = await track.recv()
    width = int(first.width)
    height = int(first.height)
    if width <= 0 or height <= 0:
        raise RuntimeError(f"invalid video frame size {width}x{height}")

    cmd = [
        "ffmpeg",
        "-loglevel",
        "error",
        "-y",
        "-f",
        "rawvideo",
        "-pix_fmt",
        "yuv420p",
        "-s",
        f"{width}x{height}",
        "-r",
        "15",
        "-i",
        "pipe:0",
        "-c:v",
        "libx264",
        "-preset",
        "ultrafast",
        "-tune",
        "zerolatency",
        "-profile:v",
        "baseline",
        "-g",
        "15",
        "-keyint_min",
        "15",
        "-sc_threshold",
        "0",
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
    assert proc.stdin is not None

    frames = 0
    try:
        while True:
            frame = first if frames == 0 else await track.recv()
            yuv = _frame_to_yuv420p(frame)
            proc.stdin.write(yuv)
            await proc.stdin.drain()
            frames += 1
            if frames == 1:
                logger.info("[VideoRelay] streaming %dx%d → %s:%d", width, height, host, port)
    finally:
        if proc.stdin:
            proc.stdin.close()
        await proc.wait()
        if proc.returncode not in (0, None):
            err = (await proc.stderr.read()).decode(errors="replace") if proc.stderr else ""
            logger.warning("[VideoRelay] ffmpeg exit %s: %s", proc.returncode, err.strip())


def _frame_to_yuv420p(frame: Any) -> bytes:
    """Convert aiortc/av VideoFrame to contiguous yuv420p bytes."""
    if hasattr(frame, "reformat"):
        frame = frame.reformat(format="yuv420p")
    if hasattr(frame, "planes"):
        return b"".join(bytes(plane) for plane in frame.planes)
    arr = frame.to_ndarray(format="yuv420p")
    return arr.tobytes()
