"""Independent emergency-stop hook outside the planner."""
from __future__ import annotations

from robot_brain.actuation.base import RobotInterface
from robot_brain.core.world_state import WorldState


class EmergencyStop:
    def __init__(self) -> None:
        self.active = False
        self.reason = ""

    async def activate(self, reason: str, robot: RobotInterface, world: WorldState) -> None:
        self.active = True
        self.reason = reason
        world.estop_active = True
        await robot.stop(reason)

    def reset(self, world: WorldState) -> None:
        self.active = False
        self.reason = ""
        world.estop_active = False
