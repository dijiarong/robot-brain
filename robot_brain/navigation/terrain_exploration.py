"""Bounded TARE-style multi-level frontier exploration orchestration."""
from __future__ import annotations

import asyncio
from dataclasses import dataclass
from typing import Callable

from robot_brain.navigation.base import NavigationUnavailableError
from robot_brain.navigation.terrain_controller import TerrainPathController


@dataclass(frozen=True)
class TerrainExplorationResult:
    completed: bool
    canceled: bool
    goals_attempted: int
    goals_reached: int
    stop_reason: str


class TerrainFrontierExplorationController:
    def __init__(self, navigation, *, max_goals: int = 5,
                 exploration_range_m: float = 3.0,
                 event_sink: Callable[[str, dict[str, object]], None] | None = None,
                 **planner_overlays) -> None:
        if max_goals <= 0:
            raise ValueError("terrain exploration goal budget must be positive")
        self._navigation = navigation
        self._max_goals = max_goals
        self._range = exploration_range_m
        self._overlays = planner_overlays
        self._event_sink = event_sink or getattr(navigation, "record_diagnostic_event", None)
        self._cancel = asyncio.Event()
        self._active: TerrainPathController | None = None

    async def cancel(self) -> None:
        self._cancel.set()
        if self._active is not None:
            await self._active.cancel()

    async def run(self) -> TerrainExplorationResult:
        self._cancel = asyncio.Event()
        visited: list[tuple[float, float]] = []
        attempted = reached = 0
        for _ in range(self._max_goals):
            if self._cancel.is_set():
                return self._finish(False, True, attempted, reached, "canceled")
            try:
                path = await self._navigation.plan_terrain_frontier(
                    exploration_range_m=self._range,
                    visited_xy=tuple(visited), **self._overlays,
                )
            except NavigationUnavailableError:
                return self._finish(reached > 0, False, attempted, reached,
                                    "no_terrain_frontier")
            attempted += 1
            target = path.nodes[-1]
            self._event("goal_planned", attempt=attempted, path_nodes=len(path.nodes),
                        target_xyz=(target.x_m, target.y_m, target.z_m),
                        visited_count=len(visited))
            self._active = TerrainPathController(
                self._navigation,
                event_sink=getattr(self._navigation, "record_diagnostic_event", None),
            )
            execution = await self._active.run(path)
            self._active = None
            if execution.canceled or self._cancel.is_set():
                return self._finish(False, True, attempted, reached, "canceled")
            if not execution.completed:
                return self._finish(False, False, attempted, reached,
                                    f"terrain_{execution.stop_reason}")
            reached += 1
            self._event("goal_reached", attempt=attempted, reached=reached)
            visited.append((target.x_m, target.y_m))
        return self._finish(True, False, attempted, reached, "max_goals")

    def _finish(self, completed, canceled, attempted, reached, reason):
        result = TerrainExplorationResult(completed, canceled, attempted, reached, reason)
        self._event("finished", completed=completed, canceled=canceled,
                    goals_attempted=attempted, goals_reached=reached,
                    stop_reason=reason)
        return result

    def _event(self, kind: str, **fields: object) -> None:
        if self._event_sink is None:
            return
        try:
            self._event_sink(f"terrain_exploration_{kind}", fields)
        except Exception:
            pass


def evaluate_terrain_exploration_trace(events) -> dict[str, object]:
    frontier = [row for row in events if row.get("event") == "terrain_frontier_plan"]
    planned = [row for row in events if row.get("event") == "terrain_exploration_goal_planned"]
    reached = [row for row in events if row.get("event") == "terrain_exploration_goal_reached"]
    finished = [row for row in events if row.get("event") == "terrain_exploration_finished"]
    final = finished[-1] if finished else None
    failures = []
    if len(frontier) < len(planned):
        failures.append("terrain_frontier_score_evidence_missing")
    if len(planned) != len(reached):
        failures.append("incomplete_terrain_exploration_goal_chain")
    if final is None or not final.get("completed") or final.get("goals_reached", 0) <= 0:
        failures.append("terrain_exploration_not_completed")
    elif final.get("goals_attempted") != final.get("goals_reached"):
        failures.append("terrain_exploration_goal_count_mismatch")
    return {"ok": not failures, "failures": failures,
            "frontier_plans": len(frontier), "goals_planned": len(planned),
            "goals_reached": len(reached),
            "stop_reason": final.get("stop_reason") if final else None}
