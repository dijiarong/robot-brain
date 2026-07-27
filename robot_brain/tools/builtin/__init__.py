"""Built-in atomic tools."""
from __future__ import annotations

from robot_brain.tools.base import Tool
from robot_brain.tools.builtin.control import Go2DriveSegmentTool, StopMotionTool
from robot_brain.tools.builtin.navigation import NavigationGetStateTool

__all__ = [
    "Go2DriveSegmentTool",
    "NavigationGetStateTool",
    "StopMotionTool",
    "default_tools",
    "go2_tools",
]


def default_tools() -> list[Tool]:
    """Built-in tools available on every backend."""
    return [StopMotionTool()]


def go2_tools() -> list[Tool]:
    """Low-level Go2 motion tools (unitree backend only)."""
    return [Go2DriveSegmentTool()]
