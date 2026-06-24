"""Structured world model: the cognition layer's single source of truth."""
from __future__ import annotations

from datetime import datetime, timezone
from typing import Any, Literal, TYPE_CHECKING

from pydantic import BaseModel, Field

from robot_brain.core.robot_self_state import RobotSelfState

if TYPE_CHECKING:
    from config.settings import Settings
    from robot_brain.core.state_interpreter import StateInterpretation
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
    last_seen_at: datetime | None = None


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
    robot_self_state: RobotSelfState | None = None

    def apply_observation(self, observation: "Observation", *, object_ttl_seconds: float | None = None) -> None:
        if observation.position is not None:
            self.position = observation.position.model_copy(deep=True)
        if observation.heading_degrees is not None:
            self.heading_degrees = observation.heading_degrees
        if observation.battery_level is not None:
            self.battery_level = observation.battery_level
        if observation.payload is not None:
            self.payload = observation.payload
        if observation.self_state is not None:
            self.robot_self_state = observation.self_state.model_copy(deep=True)
        for item in observation.detected_objects:
            perceived = item.model_copy(deep=True)
            perceived.last_seen_at = observation.observed_at
            self.known_objects[item.object_id] = perceived
        if object_ttl_seconds is not None:
            self.expire_stale_objects(object_ttl_seconds, now=observation.observed_at)
        self.alerts = list(dict.fromkeys(observation.alerts))

    def expire_stale_objects(self, ttl_seconds: float, *, now: datetime | None = None) -> list[str]:
        now = now or datetime.now(timezone.utc)
        expired = [
            object_id
            for object_id, item in self.known_objects.items()
            if item.last_seen_at is None or (now - item.last_seen_at).total_seconds() >= ttl_seconds
        ]
        for object_id in expired:
            del self.known_objects[object_id]
        return expired

    def is_object_fresh(self, object_id: str, ttl_seconds: float, *, now: datetime | None = None) -> bool:
        item = self.known_objects.get(object_id)
        if item is None or item.last_seen_at is None:
            return False
        now = now or datetime.now(timezone.utc)
        return (now - item.last_seen_at).total_seconds() < ttl_seconds

    def snapshot(self) -> dict[str, Any]:
        return self.model_dump(mode="json")

    def cognitive_snapshot(self, settings: "Settings | None" = None) -> dict[str, Any]:
        """Return a snapshot optimized for LLM consumption with state interpretation hints."""
        data = self.snapshot()
        data["_state_summary"] = self.state_summary(settings)
        return data

    def state_summary(self, settings: "Settings | None" = None) -> dict[str, str]:
        """Public API: generate human-readable interpretation using configured thresholds.

        When *settings* is provided the interpretation uses thresholds from the
        central config; otherwise falls back to hardcoded defaults for backward
        compatibility (e.g. in tests that don't have a Settings instance handy).
        """
        from robot_brain.core.state_interpreter import StateInterpreter

        if settings is not None:
            interpreter = StateInterpreter(settings)
            return interpreter.interpret(self).summary

        # Legacy fallback — construct a minimal Settings with defaults.
        from config.settings import Settings
        interpreter = StateInterpreter(Settings())
        return interpreter.interpret(self).summary

    # Backward-compat alias (deprecated — use state_summary(settings) instead).
    def _build_state_summary(self) -> dict[str, str]:
        return self.state_summary()

    @property
    def state_age_seconds(self) -> float | None:
        """Convenience accessor for the robot-reported state age."""
        if self.robot_self_state is None:
            return None
        return self.robot_self_state.state_age_seconds

    @property
    def robot_error_code(self) -> int | None:
        """Convenience accessor for the robot error code."""
        if self.robot_self_state is None:
            return None
        return self.robot_self_state.error_code
