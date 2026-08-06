"""Process-wide motion authority: one shared TeleopSession for all ingresses.

Dashboard HTTP, gRPC control, and the WebRTC gateway must inject the same
session so leases are mutually visible. Navigation preemption and emergency
stop then apply once for the whole process.
"""
from __future__ import annotations

import threading
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from config.settings import Settings
    from robot_brain.actuation.unitree import UnitreeRobot
    from robot_brain.navigation.base import NavigationClient
    from robot_brain.teleop.session import TeleopSession

_lock = threading.RLock()
_session: TeleopSession | None = None


def get_motion_session() -> TeleopSession | None:
    """Return the installed session, or ``None`` if none yet."""
    with _lock:
        return _session


def install_motion_session(session: TeleopSession) -> TeleopSession:
    """Install *session* as the process authority (idempotent if same object)."""
    global _session
    with _lock:
        if _session is not None and _session is not session:
            raise RuntimeError(
                "motion authority already installed with a different TeleopSession"
            )
        _session = session
        return _session


def session_or_create(
    robot: UnitreeRobot,
    settings: Settings,
    navigation: NavigationClient | None = None,
) -> TeleopSession:
    """Return the shared session, creating and installing one if needed."""
    from robot_brain.teleop.session import TeleopSession

    global _session
    with _lock:
        if _session is not None:
            return _session
        _session = TeleopSession(robot, settings, navigation)
        return _session


def reset_motion_authority_for_tests() -> None:
    """Clear the process singleton (unit tests only)."""
    global _session
    with _lock:
        _session = None
