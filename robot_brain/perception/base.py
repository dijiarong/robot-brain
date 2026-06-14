"""Perception adapter contract."""
from __future__ import annotations

from abc import ABC, abstractmethod
from datetime import datetime, timezone

from pydantic import BaseModel, Field

from robot_brain.core.robot_self_state import ImuRPY, RobotSelfState, Velocity
from robot_brain.core.world_state import DetectedObject, Position


class Observation(BaseModel):
    observed_at: datetime = Field(default_factory=lambda: datetime.now(timezone.utc))
    position: Position | None = None
    heading_degrees: float | None = None
    battery_level: float | None = Field(default=None, ge=0.0, le=100.0)
    payload: str | None = None
    detected_objects: list[DetectedObject] = Field(default_factory=list)
    alerts: list[str] = Field(default_factory=list)
    self_state: RobotSelfState | None = None


class PerceptionAdapter(ABC):
    @abstractmethod
    async def observe(self) -> Observation:
        """Return a normalized sensor observation."""
