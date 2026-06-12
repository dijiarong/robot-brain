"""Motion control lifecycle helpers for Unitree WebRTC velocity teleop."""
from __future__ import annotations

from enum import StrEnum


class MotionEndReason(StrEnum):
    """Structured end reason for a drive/stop lifecycle."""

    COMPLETED = "completed"
    PREEMPTED = "preempted"
    OPERATOR_STOP = "operator_stop"
    WATCHDOG = "watchdog"
    CANCELLED = "cancelled"
    TRANSPORT_ERROR = "transport_error"
    DISCONNECT = "disconnect"
    # Joystick zeroed without StopMove sport API (DimOS release semantics).
    RELEASE = "release"
