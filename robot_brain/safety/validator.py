"""Deterministic veto layer between planning and actuation."""
from __future__ import annotations

from typing import Any

from pydantic import BaseModel, Field, ValidationError

from config.settings import Settings
from robot_brain.core.world_state import Position, WorldState
from robot_brain.llm.base import ToolCall
from robot_brain.skills.registry import SkillRegistry


class ValidationResult(BaseModel):
    allowed: bool
    reason: str = ""
    requires_confirmation: bool = False
    normalized_parameters: dict[str, Any] = Field(default_factory=dict)


class SafetyValidator:
    ALWAYS_ALLOWED = {"stop", "dock", "report"}

    def __init__(self, settings: Settings, skills: SkillRegistry) -> None:
        self.settings = settings
        self.skills = skills

    def validate(
        self,
        call: ToolCall,
        world: WorldState,
        *,
        confirmation_granted: bool = False,
    ) -> ValidationResult:
        skill = self.skills.get(call.skill_name)
        if skill is None:
            return ValidationResult(allowed=False, reason=f"skill is not whitelisted: {call.skill_name}")

        try:
            params = skill.parse_params(call.parameters)
        except ValidationError as exc:
            return ValidationResult(allowed=False, reason=f"invalid parameters: {exc.errors()}")
        normalized = params.model_dump(mode="json")

        if world.estop_active and call.skill_name not in self.ALWAYS_ALLOWED:
            return ValidationResult(allowed=False, reason="emergency stop is active")
        if (
            world.battery_level <= self.settings.critical_battery_threshold
            and call.skill_name not in self.ALWAYS_ALLOWED
        ):
            return ValidationResult(allowed=False, reason="critical battery only permits stop, dock, or report")
        if not skill.preconditions(world):
            return ValidationResult(allowed=False, reason=f"preconditions failed for {call.skill_name}")

        error = self._validate_motion(call.skill_name, normalized, world)
        if error:
            return ValidationResult(allowed=False, reason=error)
        if call.skill_name in self.settings.require_confirmation_for and not confirmation_granted:
            return ValidationResult(
                allowed=False,
                reason=f"{call.skill_name} requires operator confirmation",
                requires_confirmation=True,
                normalized_parameters=normalized,
            )
        return ValidationResult(allowed=True, normalized_parameters=normalized)

    def _validate_motion(self, skill_name: str, params: dict[str, Any], world: WorldState) -> str:
        if skill_name == "navigate":
            if params["speed"] > self.settings.max_linear_speed:
                return "requested speed exceeds max_linear_speed"
            target = Position.model_validate(params["target"])
            if world.position.distance_to(target) > self.settings.max_step_distance:
                return "navigate target exceeds max_step_distance"
        if skill_name == "patrol":
            if params["speed"] > self.settings.max_linear_speed:
                return "requested speed exceeds max_linear_speed"
            previous = world.position
            for raw_waypoint in params["waypoints"]:
                waypoint = Position.model_validate(raw_waypoint)
                if previous.distance_to(waypoint) > self.settings.max_step_distance:
                    return "patrol waypoint exceeds max_step_distance"
                previous = waypoint
        if skill_name == "follow" and not world.is_object_fresh(
            params["target_id"],
            self.settings.object_ttl_seconds,
        ):
            return "follow target has not been perceived recently"
        return ""
