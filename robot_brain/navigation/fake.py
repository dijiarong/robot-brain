"""Deterministic navigation backend for Agent tests and evaluation scenarios."""
from __future__ import annotations

from collections import deque
from collections.abc import Iterable
from datetime import datetime, timezone
import math
from uuid import uuid4

from robot_brain.navigation.base import (
    NavigationClient,
    AbsoluteNavigationGoal,
    LocalizationState,
    LocalizationStatus,
    MapIdentity,
    NavigationGoalHandle,
    NavigationPose,
    NavigationState,
    NavigationStatus,
    NavigationUnavailableError,
    RelativeNavigationGoal,
)


class FakeNavigationClient(NavigationClient):
    """Scriptable provider whose next state is selected per submitted goal."""

    def __init__(
        self,
        outcomes: Iterable[NavigationStatus] | None = None,
        *,
        ready: bool = True,
        pose: NavigationPose | None = None,
        map_identity: MapIdentity | None = None,
    ) -> None:
        self._outcomes = deque(outcomes or [NavigationStatus.SUCCEEDED])
        self._state = NavigationState(
            provider="fake",
            ready=ready,
            status=NavigationStatus.IDLE if ready else NavigationStatus.UNAVAILABLE,
            pose=pose or NavigationPose(),
        )
        self._next_goal_number = 1
        self._map_identity = map_identity or MapIdentity(
            map_id=f"fake-session-{uuid4().hex}",
            frame_id="odom",
            persistent=False,
        )
        self._active_goal: RelativeNavigationGoal | None = None
        self._active_absolute_goal: AbsoluteNavigationGoal | None = None
        self._pending_outcome: NavigationStatus | None = None
        self.command_history: list[dict[str, object]] = []

    def queue_outcome(self, status: NavigationStatus) -> None:
        self._outcomes.append(status)

    @property
    def supports_absolute_goals(self) -> bool:
        return self._map_identity.persistent

    async def get_state(self) -> NavigationState:
        if self._state.status == NavigationStatus.ACTIVE and self._pending_outcome is not None:
            outcome = self._pending_outcome
            self._pending_outcome = None
            if outcome != NavigationStatus.ACTIVE:
                self._complete(outcome)
        return self._state.model_copy(deep=True)

    async def set_relative_goal(
        self, goal: RelativeNavigationGoal
    ) -> NavigationGoalHandle:
        if not self._state.ready:
            raise NavigationUnavailableError("fake navigation provider is unavailable")
        if self._state.status == NavigationStatus.ACTIVE:
            return NavigationGoalHandle(
                goal_id=self._state.goal_id or "",
                accepted=False,
                message="another navigation goal is active",
            )

        goal_id = f"fake-nav-{self._next_goal_number}"
        self._next_goal_number += 1
        self._active_goal = goal.model_copy(deep=True)
        self._active_absolute_goal = None
        self._pending_outcome = (
            self._outcomes.popleft() if self._outcomes else NavigationStatus.SUCCEEDED
        )
        self._state = NavigationState(
            provider="fake",
            ready=True,
            status=NavigationStatus.ACTIVE,
            goal_id=goal_id,
            pose=self._state.pose,
            progress=0.0,
            message="relative goal accepted",
        )
        self.command_history.append(
            {"action": "set_relative_goal", "goal_id": goal_id, "goal": goal.model_dump()}
        )
        return NavigationGoalHandle(goal_id=goal_id, message="relative goal accepted")

    async def get_localization_state(self) -> LocalizationState:
        return LocalizationState(
            status=(
                LocalizationStatus.LOCALIZED
                if self._map_identity.persistent
                else LocalizationStatus.LOCAL
            ),
            map_identity=self._map_identity,
            pose=self._state.pose,
            confidence=1.0,
            message="fake localization ready",
        )

    async def set_absolute_goal(
        self, goal: AbsoluteNavigationGoal
    ) -> NavigationGoalHandle:
        if not self._state.ready:
            raise NavigationUnavailableError("fake navigation provider is unavailable")
        if goal.map_id != self._map_identity.map_id or (
            goal.map_version is not None
            and goal.map_version != self._map_identity.version
        ):
            raise NavigationUnavailableError("absolute goal belongs to a different map")
        if goal.pose.frame_id != self._map_identity.frame_id:
            raise NavigationUnavailableError("absolute goal frame does not match map frame")
        if self._state.status == NavigationStatus.ACTIVE:
            return NavigationGoalHandle(
                goal_id=self._state.goal_id or "", accepted=False,
                message="another navigation goal is active",
            )
        goal_id = f"fake-nav-{self._next_goal_number}"
        self._next_goal_number += 1
        self._active_goal = None
        self._active_absolute_goal = goal.model_copy(deep=True)
        self._pending_outcome = (
            self._outcomes.popleft() if self._outcomes else NavigationStatus.SUCCEEDED
        )
        self._state = NavigationState(
            provider="fake", ready=True, status=NavigationStatus.ACTIVE,
            goal_id=goal_id, pose=self._state.pose, progress=0.0,
            message="absolute goal accepted",
        )
        self.command_history.append(
            {"action": "set_absolute_goal", "goal_id": goal_id, "goal": goal.model_dump()}
        )
        return NavigationGoalHandle(goal_id=goal_id, message="absolute goal accepted")

    async def cancel(self, goal_id: str | None = None) -> NavigationState:
        active_id = self._state.goal_id
        if self._state.status != NavigationStatus.ACTIVE:
            self.command_history.append({"action": "cancel", "goal_id": goal_id, "noop": True})
            return self._state.model_copy(deep=True)
        if goal_id is not None and goal_id != active_id:
            return self._state.model_copy(deep=True)

        self._pending_outcome = None
        self._complete(NavigationStatus.CANCELED)
        self.command_history.append({"action": "cancel", "goal_id": active_id, "noop": False})
        return self._state.model_copy(deep=True)

    def _complete(self, status: NavigationStatus) -> None:
        pose = self._state.pose
        if status == NavigationStatus.SUCCEEDED and pose is not None and self._active_goal is not None:
            yaw_rad = math.radians(pose.yaw_degrees)
            dx = (
                self._active_goal.forward_m * math.cos(yaw_rad)
                - self._active_goal.left_m * math.sin(yaw_rad)
            )
            dy = (
                self._active_goal.forward_m * math.sin(yaw_rad)
                + self._active_goal.left_m * math.cos(yaw_rad)
            )
            pose = pose.model_copy(
                update={
                    "x_m": pose.x_m + dx,
                    "y_m": pose.y_m + dy,
                    "yaw_degrees": (
                        pose.yaw_degrees + self._active_goal.yaw_degrees + 180.0
                    ) % 360.0 - 180.0,
                }
            )
        elif status == NavigationStatus.SUCCEEDED and self._active_absolute_goal is not None:
            pose = self._active_absolute_goal.pose.model_copy(deep=True)
        messages = {
            NavigationStatus.SUCCEEDED: "relative goal reached",
            NavigationStatus.FAILED: "navigation failed",
            NavigationStatus.CANCELED: "navigation canceled",
            NavigationStatus.TIMED_OUT: "navigation timed out",
            NavigationStatus.NO_PROGRESS: "navigation made no progress",
            NavigationStatus.UNAVAILABLE: "navigation provider unavailable",
        }
        self._state = self._state.model_copy(
            update={
                "status": status,
                "ready": status != NavigationStatus.UNAVAILABLE,
                "pose": pose,
                "progress": 1.0 if status == NavigationStatus.SUCCEEDED else self._state.progress,
                "message": messages.get(status, status.value),
                "error_code": None if status == NavigationStatus.SUCCEEDED else status.value,
                "updated_at": datetime.now(timezone.utc),
            }
        )
        self._active_goal = None
        self._active_absolute_goal = None
