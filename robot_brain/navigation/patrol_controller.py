"""Cancelable patrol execution over native or replaceable navigation providers."""
from __future__ import annotations

import asyncio
from dataclasses import dataclass
import math
from typing import Callable

from robot_brain.navigation.base import (
    AbsoluteNavigationGoal,
    NavigationClient,
    NavigationPose,
    NavigationStatus,
    RelativeNavigationGoal,
)


@dataclass(frozen=True)
class PatrolExecutionResult:
    completed: bool
    canceled: bool
    attempted: int
    reached: int
    failed: int
    cycles_completed: int
    last_status: NavigationStatus | None


class PatrolController:
    def __init__(
        self,
        navigation: NavigationClient,
        *,
        poll_interval_s: float = 0.10,
        goal_timeout_s: float = 60.0,
        event_sink: Callable[[str, dict[str, object]], None] | None = None,
    ) -> None:
        self._navigation = navigation
        self._poll_interval = poll_interval_s
        self._goal_timeout = goal_timeout_s
        self._event_sink = event_sink
        self._cancel = asyncio.Event()
        self._active_goal_id: str | None = None

    async def cancel(self) -> None:
        self._cancel.set()
        if self._active_goal_id is not None:
            await self._navigation.cancel(self._active_goal_id)

    async def run(
        self,
        route: list[NavigationPose],
        *,
        cycles: int = 1,
        continue_on_failure: bool = False,
    ) -> PatrolExecutionResult:
        self._cancel = asyncio.Event()
        attempted = reached = failed = cycles_completed = 0
        last_status: NavigationStatus | None = None
        if not route or cycles <= 0:
            return self._finish(True, False, 0, 0, 0, 0, None)
        for _ in range(cycles):
            for waypoint in route:
                if self._cancel.is_set():
                    return self._finish(False, True, attempted, reached, failed,
                                        cycles_completed, last_status)
                handle = await self._submit(waypoint)
                attempted += 1
                self._event("waypoint_command", attempt=attempted, cycle=cycles_completed+1,
                            x_m=waypoint.x_m, y_m=waypoint.y_m,
                            frame_id=waypoint.frame_id)
                if not handle.accepted:
                    failed += 1
                    last_status = NavigationStatus.FAILED
                    if not continue_on_failure:
                        return self._finish(False, False, attempted, reached, failed,
                                            cycles_completed, last_status)
                    continue
                self._active_goal_id = handle.goal_id
                last_status = await self._wait_terminal()
                self._event("waypoint_terminal", attempt=attempted, status=last_status.value)
                self._active_goal_id = None
                if self._cancel.is_set() or last_status == NavigationStatus.CANCELED:
                    return self._finish(False, True, attempted, reached, failed,
                                        cycles_completed, last_status)
                if last_status == NavigationStatus.SUCCEEDED:
                    reached += 1
                else:
                    failed += 1
                    if not continue_on_failure:
                        return self._finish(False, False, attempted, reached, failed,
                                            cycles_completed, last_status)
            cycles_completed += 1
        return self._finish(True, False, attempted, reached, failed,
                            cycles_completed, last_status)

    async def _submit(self, waypoint: NavigationPose):
        localization = await self._navigation.get_localization_state()
        identity = localization.map_identity
        if (
            self._navigation.supports_absolute_goals
            and localization.usable_for_persistent_memory
            and identity is not None
        ):
            return await self._navigation.set_absolute_goal(AbsoluteNavigationGoal(
                pose=waypoint.model_copy(update={"frame_id": identity.frame_id}),
                map_id=identity.map_id, map_version=identity.version,
                max_duration_s=self._goal_timeout,
            ))
        if localization.pose is None:
            raise RuntimeError("patrol localization unavailable")
        pose = localization.pose
        dx, dy = waypoint.x_m - pose.x_m, waypoint.y_m - pose.y_m
        yaw = math.radians(pose.yaw_degrees)
        return await self._navigation.set_relative_goal(RelativeNavigationGoal(
            forward_m=dx * math.cos(yaw) + dy * math.sin(yaw),
            left_m=-dx * math.sin(yaw) + dy * math.cos(yaw),
            yaw_degrees=(waypoint.yaw_degrees - pose.yaw_degrees + 180.0) % 360.0 - 180.0,
            max_duration_s=self._goal_timeout,
        ))

    async def _wait_terminal(self) -> NavigationStatus:
        while True:
            if self._cancel.is_set():
                await self._navigation.cancel(self._active_goal_id)
            state = await self._navigation.get_state()
            if state.status.terminal:
                return state.status
            await asyncio.sleep(self._poll_interval)

    def _event(self, kind: str, **fields: object) -> None:
        if self._event_sink is None:
            return
        try:
            self._event_sink(f"patrol_{kind}", fields)
        except Exception:
            pass

    def _finish(self, completed, canceled, attempted, reached, failed,
                cycles_completed, last_status):
        result = PatrolExecutionResult(
            completed, canceled, attempted, reached, failed,
            cycles_completed, last_status,
        )
        self._event("finished", **_patrol_fields(result))
        return result


def _patrol_fields(result: PatrolExecutionResult) -> dict[str, object]:
    return {"completed": result.completed, "canceled": result.canceled,
            "attempted": result.attempted, "reached": result.reached,
            "failed": result.failed, "cycles_completed": result.cycles_completed,
            "last_status": result.last_status.value if result.last_status else None}


def evaluate_patrol_trace(events) -> dict[str, object]:
    routes = [row for row in events if row.get("event") == "patrol_route"]
    commands = [row for row in events if row.get("event") == "patrol_waypoint_command"]
    terminals = [row for row in events if row.get("event") == "patrol_waypoint_terminal"]
    finished = [row for row in events if row.get("event") == "patrol_finished"]
    final = finished[-1] if finished else None
    failures = []
    if not routes:
        failures.append("patrol_route_evidence_missing")
    elif not isinstance(routes[-1].get("route_evaluation"), dict) or not routes[-1][
        "route_evaluation"
    ].get("ok"):
        failures.append("patrol_route_semantics_not_verified")
    if len(commands) != len(terminals):
        failures.append("incomplete_patrol_waypoint_chain")
    if any(row.get("status") != NavigationStatus.SUCCEEDED.value for row in terminals):
        failures.append("patrol_waypoint_failed")
    if final is None or not final.get("completed") or final.get("failed") != 0:
        failures.append("patrol_not_completed")
    elif final.get("attempted") != final.get("reached"):
        failures.append("patrol_waypoint_count_mismatch")
    return {"ok": not failures, "failures": failures,
            "strategy": routes[-1].get("strategy") if routes else None,
            "waypoints": len(commands),
            "cycles_completed": final.get("cycles_completed") if final else None}
