"""Perception adapter contract."""
from __future__ import annotations

from abc import ABC, abstractmethod

from pydantic import BaseModel, Field

from robot_brain.core.world_state import DetectedObject, Position


class Observation(BaseModel):
    position: Position | None = None
    heading_degrees: float | None = None
    battery_level: float | None = Field(default=None, ge=0.0, le=100.0)
    payload: str | None = None
    detected_objects: list[DetectedObject] = Field(default_factory=list)
    alerts: list[str] = Field(default_factory=list)


class PerceptionAdapter(ABC):
    @abstractmethod
    async def observe(self) -> Observation:
        """Return a normalized sensor observation."""
