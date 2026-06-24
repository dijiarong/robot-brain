"""Deterministic veto layer between planning and actuation."""
from __future__ import annotations

from typing import Any

from pydantic import BaseModel, Field, ValidationError

from config.settings import Settings
from robot_brain.core.errors import ErrorCode
from robot_brain.core.world_state import Position, WorldState
from robot_brain.llm.base import ToolCall
from robot_brain.skills.registry import SkillRegistry, UNITREE_HIDDEN_SKILLS


class ValidationResult(BaseModel):
    allowed: bool
    reason: str = ""
    error_code: ErrorCode | None = None
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
            return ValidationResult(
                allowed=False,
                reason=f"skill is not whitelisted: {call.skill_name}",
                error_code=ErrorCode.SAFETY_NOT_WHITELISTED,
            )

        backend_err = self._validate_backend(call.skill_name)
        if backend_err:
            return ValidationResult(
                allowed=False,
                reason=backend_err,
                error_code=ErrorCode.SAFETY_NOT_WHITELISTED,
            )

        try:
            params = skill.parse_params(call.parameters)
        except ValidationError as exc:
            return ValidationResult(
                allowed=False,
                reason=f"invalid parameters: {exc.errors()}",
                error_code=ErrorCode.SAFETY_INVALID_PARAMS,
            )
        normalized = params.model_dump(mode="json")

        if world.estop_active and call.skill_name not in self.ALWAYS_ALLOWED:
            return ValidationResult(
                allowed=False,
                reason="emergency stop is active",
                error_code=ErrorCode.SAFETY_ESTOP_ACTIVE,
            )
        if (
            world.battery_level <= self.settings.critical_battery_threshold
            and call.skill_name not in self.ALWAYS_ALLOWED
        ):
            return ValidationResult(
                allowed=False,
                reason="critical battery only permits stop, dock, or report",
                error_code=ErrorCode.SAFETY_BATTERY_CRITICAL,
            )
        if not skill.preconditions(world):
            return ValidationResult(
                allowed=False,
                reason=f"preconditions failed for {call.skill_name}",
                error_code=ErrorCode.SAFETY_PRECONDITION_FAILED,
            )

        error = self._validate_motion(call.skill_name, normalized, world)
        if error:
            return ValidationResult(
                allowed=False,
                reason=error,
                error_code=ErrorCode.SAFETY_MOTION_VIOLATION,
            )
        if call.skill_name in self.settings.require_confirmation_for and not confirmation_granted:
            return ValidationResult(
                allowed=False,
                reason=f"{call.skill_name} requires operator confirmation",
                error_code=ErrorCode.SAFETY_CONFIRMATION_REQUIRED,
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
        # Go2 motion skills
        if skill_name == "nudge":
            d = params.get("distance_cm", 20.0)
            if not (10.0 <= d <= 50.0):
                return "nudge distance must be 10–50 cm"
        if skill_name == "scan":
            y = params.get("yaw_degrees", 45.0)
            if not (-90.0 <= y <= 90.0):
                return "scan angle must be ±90°"
        if skill_name == "retreat":
            d = params.get("distance_cm", 30.0)
            if not (10.0 <= d <= 100.0):
                return "retreat distance must be 10–100 cm"
        if skill_name == "explore":
            ms = params.get("max_steps", 5)
            if ms > self.settings.explore_max_steps:
                return (
                    f"explore max_steps ({ms}) exceeds settings limit "
                    f"({self.settings.explore_max_steps})"
                )
            sd = params.get("step_distance_cm", 20.0)
            if not (10.0 <= sd <= 50.0):
                return "explore step_distance_cm must be 10–50 cm"
            deg = params.get("scan_degrees", 45.0)
            if not (10.0 <= deg <= 90.0):
                return "explore scan_degrees must be 10–90°"
        return ""

    def _validate_backend(self, skill_name: str) -> str:
        """Reject skills that are hidden on the current backend.

        On unitree, generic motion skills (navigate/patrol/follow/dock)
        are hidden from the LLM tool list *and* hard-rejected here so a
        direct API call cannot bypass the filter.
        """
        if self.settings.robot_backend == "unitree" and skill_name in UNITREE_HIDDEN_SKILLS:
            return f"skill '{skill_name}' is unsupported on unitree backend"
        return ""
