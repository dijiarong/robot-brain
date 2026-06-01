"""Structured world model: the cognition layer's single source of truth."""
from __future__ import annotations

from typing import Any, Literal, TYPE_CHECKING

from pydantic import BaseModel, Field

if TYPE_CHECKING:
    from robot_brain.perception.base import Observation


class Position(BaseModel):
    x: float = 0.0
    y: float = 0.0

    def distance_to(self, other: "Position") -> float:
        return ((self.x - other.x) ** 2 + (self.y - other.y) ** 2) ** 0.5


class DetectedObject(BaseModel):
    object_id: str
    kind: str
    position: Position = Field(default_factory=Position)
    confidence: float = Field(default=1.0, ge=0.0, le=1.0)
    attributes: dict[str, Any] = Field(default_factory=dict)


class TaskProgress(BaseModel):
    objective: str
    status: Literal["idle", "running", "paused", "completed", "failed"] = "idle"
    completed_skills: list[str] = Field(default_factory=list)
    last_message: str = ""


class WorldState(BaseModel):
    position: Position = Field(default_factory=Position)
    heading_degrees: float = 0.0
    battery_level: float = Field(default=100.0, ge=0.0, le=100.0)
    payload: str | None = None
    known_objects: dict[str, DetectedObject] = Field(default_factory=dict)
    alerts: list[str] = Field(default_factory=list)
    current_task: TaskProgress | None = None
    estop_active: bool = False

    def apply_observation(self, observation: "Observation") -> None:
        if observation.position is not None:
            self.position = observation.position.model_copy(deep=True)
        if observation.heading_degrees is not None:
            self.heading_degrees = observation.heading_degrees
        if observation.battery_level is not None:
            self.battery_level = observation.battery_level
        if observation.payload is not None:
            self.payload = observation.payload
        for item in observation.detected_objects:
            self.known_objects[item.object_id] = item.model_copy(deep=True)
        self.alerts = list(dict.fromkeys(observation.alerts))

    def snapshot(self) -> dict[str, Any]:
        return self.model_dump(mode="json")
