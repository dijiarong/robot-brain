"""In-memory robot implementation for tests and local demos."""
from __future__ import annotations

from typing import Any

from robodog_brain.actuation.base import RobotInterface, RobotState
from robodog_brain.core.world_state import Position


class MockRobot(RobotInterface):
    def __init__(self, state: RobotState | None = None) -> None:
        self.state = state or RobotState()
        self._action_history: list[dict[str, Any]] = []

    @property
    def action_history(self) -> list[dict[str, Any]]:
        return self._action_history

    async def move_to(self, target: Position, speed: float) -> None:
        self._record("move_to", target=target.model_dump(), speed=speed)
        self.state.position = target.model_copy(deep=True)
        self.state.stopped = False
        self.state.docked = False

    async def turn(self, heading_degrees: float) -> None:
        self._record("turn", heading_degrees=heading_degrees)
        self.state.heading_degrees = heading_degrees

    async def stop(self, reason: str = "") -> None:
        self._record("stop", reason=reason)
        self.state.stopped = True

    async def dock(self, station: str) -> None:
        self._record("dock", station=station)
        self.state.position = Position()
        self.state.battery_level = 100.0
        self.state.stopped = True
        self.state.docked = True

    async def follow(self, target_id: str, distance: float) -> None:
        self._record("follow", target_id=target_id, distance=distance)
        self.state.stopped = False

    async def report(self, message: str, severity: str) -> None:
        self._record("report", message=message, severity=severity)

    async def get_state(self) -> RobotState:
        return self.state.model_copy(deep=True)

    def _record(self, action: str, **parameters: Any) -> None:
        self._action_history.append({"action": action, **parameters})
