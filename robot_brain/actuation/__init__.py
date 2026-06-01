"""Robot actuation adapter interfaces and built-in implementations."""

from .base import RobotInterface, RobotState
from .mock import MockRobot

__all__ = ["MockRobot", "RobotInterface", "RobotState"]
