"""Shared helpers for Go2 skills: precondition checks, segment planning, drive execution."""
from __future__ import annotations

import logging
from typing import Any

from config.settings import Settings
from robot_brain.actuation.unitree import UnitreeRobot
from robot_brain.core.world_state import WorldState

logger = logging.getLogger(__name__)

# Fixed motion parameters — below the safety max to leave headroom.
LINEAR_SPEED = 0.15   # m/s   (< RDB_UNITREE_MAX_SPEED 0.2)
YAW_SPEED = 0.3       # rad/s (= RDB_UNITREE_MAX_YAW_SPEED 0.3)

# Tiny float threshold for segment remainder noise.
_EPS = 0.001


def check_robot_self_state(world: WorldState, settings: Settings) -> str | None:
    """Return ``None`` if the robot is ready for motion, or a rejection reason."""
    ss = world.robot_self_state
    if ss is None:
        return "robot self-state not available (set RDB_PERCEPTION=unitree)"

    if ss.is_standing is not None and not ss.is_standing:
        return "robot not standing"

    if ss.error_code is not None and ss.error_code != 0:
        return f"robot error_code={ss.error_code}"

    if (
        ss.state_age_seconds is not None
        and ss.state_age_seconds > settings.unitree_state_max_age_seconds
    ):
        return (
            f"robot state stale ({ss.state_age_seconds:.1f}s"
            f" > {settings.unitree_state_max_age_seconds}s)"
        )

    return None


def plan_linear_segments(distance_m: float, segment_duration: float) -> list[float]:
    """Chop a linear move into segments each ≤ *segment_duration* seconds.

    Returns one or more durations (seconds).  The total of all segments equals
    the time needed to cover *distance_m* at ``LINEAR_SPEED``.
    """
    total_time = distance_m / LINEAR_SPEED
    return _chop(total_time, segment_duration)


def plan_yaw_segments(yaw_rad: float, segment_duration: float) -> list[float]:
    """Chop a rotation into segments each ≤ *segment_duration* seconds."""
    total_time = abs(yaw_rad) / YAW_SPEED
    return _chop(total_time, segment_duration)


def _chop(total_time: float, segment_duration: float) -> list[float]:
    if total_time <= _EPS:
        return []
    full = int(total_time // segment_duration)
    remainder = total_time - full * segment_duration
    durations: list[float] = [segment_duration] * full
    if remainder > _EPS:
        durations.append(round(remainder, 3))
    if not durations:
        # Very short total_time — one segment of the requested duration.
        durations.append(min(total_time, segment_duration))
    return durations


async def run_go2_drive_segments(
    robot: UnitreeRobot,
    *,
    vx: float = 0.0,
    vy: float = 0.0,
    vyaw: float = 0.0,
    durations: list[float],
) -> dict[str, Any]:
    """Execute a sequence of ``drive()`` calls and return a segment audit dict."""
    segments: list[dict[str, Any]] = []
    for i, d in enumerate(durations):
        try:
            await robot.drive(vx=vx, vy=vy, vyaw=vyaw, duration=d)
            raw_reason = getattr(
                robot.transport, "last_drive_end_reason", None
            )
            end_reason = str(raw_reason) if raw_reason is not None else "completed"
            segments.append(
                {
                    "index": i,
                    "vx": vx,
                    "vy": vy,
                    "vyaw": vyaw,
                    "duration": round(d, 3),
                    "end_reason": end_reason,
                }
            )
        except Exception as exc:
            segments.append(
                {
                    "index": i,
                    "vx": vx,
                    "vy": vy,
                    "vyaw": vyaw,
                    "duration": round(d, 3),
                    "end_reason": "error",
                    "error": str(exc),
                }
            )
            return {
                "segments": segments,
                "segment_count": len(segments),
                "success": False,
            }
    return {
        "segments": segments,
        "segment_count": len(segments),
        "success": True,
    }
