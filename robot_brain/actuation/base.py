"""Robot SDK boundary used by cognition skills."""
from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any

from pydantic import BaseModel, Field

from robot_brain.core.world_state import Position


class RobotState(BaseModel):
    position: Position = Field(default_factory=Position)
    heading_degrees: float = 0.0
    battery_level: float = Field(default=100.0, ge=0.0, le=100.0)
    payload: str | None = None
    stopped: bool = False
    docked: bool = False


class RobotInterface(ABC):
    @abstractmethod
    async def move_to(self, target: Position, speed: float) -> None: ...

    @abstractmethod
    async def turn(self, heading_degrees: float) -> None: ...

    @abstractmethod
    async def stop(self, reason: str = "") -> None: ...

    @abstractmethod
    async def dock(self, station: str) -> None: ...

    @abstractmethod
    async def follow(self, target_id: str, distance: float) -> None: ...

    @abstractmethod
    async def report(self, message: str, severity: str) -> None: ...

    @abstractmethod
    async def get_state(self) -> RobotState: ...

    @property
    @abstractmethod
    def action_history(self) -> list[dict[str, Any]]: ...
