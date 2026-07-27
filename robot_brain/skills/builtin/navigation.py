"""Planner-facing skills backed by a replaceable NavigationClient."""
from __future__ import annotations

import asyncio
import time

from pydantic import BaseModel

from robot_brain.actuation.base import RobotInterface
from robot_brain.core.world_state import WorldState
from robot_brain.navigation import (
    AbsoluteNavigationGoal,
    NavigationClient,
    NavigationStatus,
    RelativeNavigationGoal,
)
from robot_brain.skills.base import Skill, SkillResult
from robot_brain.tools.base import CapabilityMetadata, MotionKind, RiskLevel


class NavigateRelativeParams(RelativeNavigationGoal):
    pass


class CancelNavigationParams(BaseModel):
    goal_id: str | None = None


class NavigateAbsoluteParams(AbsoluteNavigationGoal):
    pass


class NavigateRelativeSkill(Skill):
    name = "nav_go_relative"
    description = (
        "Navigate to a bounded relative goal in the robot frame. "
        "Use for deliberate local motion, not small manual adjustments."
    )
    params_model = NavigateRelativeParams
    capability_metadata = CapabilityMetadata(
        risk_level=RiskLevel.MEDIUM,
        motion_kind=MotionKind.LINEAR,
        requires_confirmation=True,
        planner_visible=True,
        tags=frozenset({"navigation", "relative_goal"}),
    )

    def __init__(
        self,
        client: NavigationClient,
        *,
        poll_interval_s: float = 0.1,
    ) -> None:
        self._client = client
        self._poll_interval_s = poll_interval_s

    async def execute(
        self,
        params: NavigateRelativeParams,
        robot: RobotInterface,
        world: WorldState,
    ) -> SkillResult:
        try:
            handle = await self._client.set_relative_goal(
                RelativeNavigationGoal.model_validate(params.model_dump())
            )
            if not handle.accepted:
                return SkillResult(
                    success=False,
                    message=handle.message or "navigation goal rejected",
                    data={"goal_id": handle.goal_id, "stop_reason": "rejected"},
                )
            deadline = time.monotonic() + params.max_duration_s
            while True:
                state = await self._client.get_state()
                if state.status.terminal:
                    break
                if time.monotonic() >= deadline:
                    await self._client.cancel(handle.goal_id)
                    state = state.model_copy(
                        update={
                            "status": NavigationStatus.TIMED_OUT,
                            "message": "navigation timed out",
                            "error_code": "timed_out",
                        }
                    )
                    break
                await asyncio.sleep(self._poll_interval_s)
        except Exception as exc:
            return SkillResult(
                success=False,
                message=f"navigation provider error: {exc}",
                data={"stop_reason": "provider_error", "error": str(exc)},
            )

        success = state.status == NavigationStatus.SUCCEEDED
        return SkillResult(
            success=success,
            message=state.message or f"navigation {state.status.value}",
            data={
                "goal_id": handle.goal_id,
                "stop_reason": state.status.value,
                "navigation_state": state.model_dump(mode="json"),
            },
        )


class CancelNavigationSkill(Skill):
    name = "nav_cancel"
    description = "Cancel the active navigation goal. Safe to call when no goal is active."
    params_model = CancelNavigationParams
    capability_metadata = CapabilityMetadata(
        risk_level=RiskLevel.LOW,
        motion_kind=MotionKind.STOP,
        planner_visible=True,
        tags=frozenset({"navigation", "cancel", "safety"}),
    )

    def __init__(self, client: NavigationClient) -> None:
        self._client = client

    def preconditions(self, world: WorldState) -> bool:
        return True

    async def execute(
        self,
        params: CancelNavigationParams,
        robot: RobotInterface,
        world: WorldState,
    ) -> SkillResult:
        try:
            before = await self._client.get_state()
            state = await self._client.cancel(params.goal_id)
        except Exception as exc:
            return SkillResult(
                success=False,
                message=f"navigation cancel failed: {exc}",
                data={"stop_reason": "provider_error", "error": str(exc)},
            )

        requested_matches = params.goal_id is None or params.goal_id == before.goal_id
        success = before.status != NavigationStatus.ACTIVE or (
            requested_matches and state.status == NavigationStatus.CANCELED
        )
        return SkillResult(
            success=success,
            message=state.message or "no active navigation goal",
            data={
                "goal_id": state.goal_id,
                "stop_reason": state.status.value,
                "navigation_state": state.model_dump(mode="json"),
            },
        )


class NavigateAbsoluteSkill(Skill):
    name = "nav_go_to_pose"
    description = "Navigate to an absolute pose on the currently localized persistent map."
    params_model = NavigateAbsoluteParams
    capability_metadata = CapabilityMetadata(
        risk_level=RiskLevel.MEDIUM,
        motion_kind=MotionKind.LINEAR,
        requires_confirmation=True,
        planner_visible=True,
        tags=frozenset({"navigation", "absolute_goal", "map"}),
    )

    def __init__(self, client: NavigationClient, *, poll_interval_s: float = 0.1) -> None:
        self._client = client
        self._poll_interval_s = poll_interval_s

    async def execute(
        self,
        params: NavigateAbsoluteParams,
        robot: RobotInterface,
        world: WorldState,
    ) -> SkillResult:
        try:
            handle = await self._client.set_absolute_goal(
                AbsoluteNavigationGoal.model_validate(params.model_dump())
            )
            if not handle.accepted:
                return SkillResult(
                    success=False, message=handle.message or "absolute goal rejected",
                    data={"goal_id": handle.goal_id, "stop_reason": "rejected"},
                )
            deadline = time.monotonic() + params.max_duration_s
            while True:
                state = await self._client.get_state()
                if state.status.terminal:
                    break
                if time.monotonic() >= deadline:
                    await self._client.cancel(handle.goal_id)
                    state = state.model_copy(update={
                        "status": NavigationStatus.TIMED_OUT,
                        "message": "absolute navigation timed out",
                        "error_code": "timed_out",
                    })
                    break
                await asyncio.sleep(self._poll_interval_s)
        except Exception as exc:
            return SkillResult(
                success=False, message=f"absolute navigation provider error: {exc}",
                data={"stop_reason": "provider_error", "error": str(exc)},
            )
        return SkillResult(
            success=state.status == NavigationStatus.SUCCEEDED,
            message=state.message or f"navigation {state.status.value}",
            data={
                "goal_id": handle.goal_id,
                "stop_reason": state.status.value,
                "navigation_state": state.model_dump(mode="json"),
            },
        )


def navigation_skills(client: NavigationClient) -> list[Skill]:
    result: list[Skill] = [NavigateRelativeSkill(client), CancelNavigationSkill(client)]
    if client.supports_absolute_goals:
        result.append(NavigateAbsoluteSkill(client))
    return result
