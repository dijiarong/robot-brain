"""Cancelable execution adapter for validated 3D surface paths."""
from __future__ import annotations

import asyncio
from dataclasses import dataclass
import math
from typing import Callable

from robot_brain.navigation.base import NavigationClient, NavigationStatus, RelativeNavigationGoal
from robot_brain.navigation.terrain3d import SurfacePath


@dataclass(frozen=True)
class TerrainExecutionResult:
    completed: bool
    canceled: bool
    attempted: int
    reached: int
    last_status: NavigationStatus | None
    stop_reason: str


class TerrainPathController:
    """Follow the XY projection of an already safety-validated MLS path.

    Go2 controls horizontal velocity; the surface planner is responsible for
    ensuring that the corresponding Z transition is physically traversable.
    Localization is checked before every segment so commands remain relative to
    the current body heading rather than an assumed open-loop pose.
    """

    def __init__(self, navigation: NavigationClient, *, poll_interval_s: float = 0.1,
                 segment_timeout_s: float = 30.0, max_segment_m: float = 3.0,
                 event_sink: Callable[[str, dict[str, object]], None] | None = None) -> None:
        if poll_interval_s <= 0 or segment_timeout_s <= 0 or not 0 < max_segment_m <= 3.0:
            raise ValueError("invalid terrain controller timing or segment limit")
        self._navigation = navigation
        self._poll_interval = poll_interval_s
        self._timeout = segment_timeout_s
        self._max_segment = max_segment_m
        self._event_sink = event_sink
        self._cancel = asyncio.Event()
        self._goal_id: str | None = None

    async def cancel(self) -> None:
        self._cancel.set()
        if self._goal_id is not None:
            await self._navigation.cancel(self._goal_id)

    async def run(self, path: SurfacePath) -> TerrainExecutionResult:
        self._cancel = asyncio.Event()
        attempted = reached = 0
        last: NavigationStatus | None = None
        if len(path.nodes) < 2:
            return self._finish(True, False, 0, 0, None, "already_at_goal")
        for target in path.nodes[1:]:
            if self._cancel.is_set():
                return self._finish(False, True, attempted, reached, last, "canceled")
            localization = await self._navigation.get_localization_state()
            pose = localization.pose
            if pose is None:
                return self._finish(False, False, attempted, reached, last,
                                    "localization_unavailable")
            dx, dy = target.x_m - pose.x_m, target.y_m - pose.y_m
            distance = math.hypot(dx, dy)
            if distance > self._max_segment + 1e-9:
                return self._finish(False, False, attempted, reached, last,
                                    "segment_exceeds_safety_limit")
            yaw = math.radians(pose.yaw_degrees)
            handle = await self._navigation.set_relative_goal(RelativeNavigationGoal(
                forward_m=dx * math.cos(yaw) + dy * math.sin(yaw),
                left_m=-dx * math.sin(yaw) + dy * math.cos(yaw),
                max_duration_s=self._timeout,
            ))
            attempted += 1
            self._event("segment_command", segment_index=attempted, distance_m=distance,
                        target_xyz=(target.x_m, target.y_m, target.z_m))
            if not handle.accepted:
                return self._finish(False, False, attempted, reached,
                                    NavigationStatus.FAILED, "goal_rejected")
            self._goal_id = handle.goal_id
            last = await self._wait()
            self._goal_id = None
            self._event("segment_terminal", segment_index=attempted, status=last.value)
            if self._cancel.is_set() or last == NavigationStatus.CANCELED:
                return self._finish(False, True, attempted, reached, last, "canceled")
            if last != NavigationStatus.SUCCEEDED:
                return self._finish(False, False, attempted, reached, last,
                                    "navigation_failed")
            reached += 1
        return self._finish(True, False, attempted, reached, last, "goal_reached")

    async def _wait(self) -> NavigationStatus:
        while True:
            if self._cancel.is_set():
                await self._navigation.cancel(self._goal_id)
            state = await self._navigation.get_state()
            if state.status.terminal:
                return state.status
            await asyncio.sleep(self._poll_interval)

    def _finish(self, completed, canceled, attempted, reached, last, reason):
        result = TerrainExecutionResult(completed, canceled, attempted, reached, last, reason)
        self._event("finished", completed=completed, canceled=canceled,
                    attempted=attempted, reached=reached,
                    last_status=last.value if last else None, stop_reason=reason)
        return result

    def _event(self, kind: str, **fields: object) -> None:
        if self._event_sink is None:
            return
        try:
            self._event_sink(f"terrain_execution_{kind}", fields)
        except Exception:
            pass


def evaluate_terrain_execution_trace(events) -> dict[str, object]:
    commands = [row for row in events if row.get("event") == "terrain_execution_segment_command"]
    terminals = [row for row in events if row.get("event") == "terrain_execution_segment_terminal"]
    finished = [row for row in events if row.get("event") == "terrain_execution_finished"]
    final = finished[-1] if finished else None
    failures = []
    if len(commands) != len(terminals):
        failures.append("incomplete_terrain_segment_chain")
    if any(row.get("status") != NavigationStatus.SUCCEEDED.value for row in terminals):
        failures.append("terrain_segment_failed")
    if final is None or not final.get("completed") or final.get("stop_reason") not in {
        "goal_reached", "already_at_goal",
    }:
        failures.append("terrain_execution_not_completed")
    elif final.get("attempted") != final.get("reached"):
        failures.append("terrain_waypoint_count_mismatch")
    return {"ok": not failures, "failures": failures,
            "segments_attempted": len(commands), "segments_terminal": len(terminals),
            "stop_reason": final.get("stop_reason") if final else None}
