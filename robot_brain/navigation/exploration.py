"""Bounded frontier exploration orchestrated through NavigationClient."""
from __future__ import annotations

import asyncio
from dataclasses import dataclass
from enum import StrEnum
from typing import Awaitable, Callable
import math

from robot_brain.navigation.base import NavigationClient, NavigationStatus, RelativeNavigationGoal
from robot_brain.navigation.frontier import find_frontier_goals
from robot_brain.navigation.grid import OccupancyGrid2D

GridProvider = Callable[[], OccupancyGrid2D | Awaitable[OccupancyGrid2D]]


class ExplorationStopReason(StrEnum):
    COMPLETE = "complete"
    CANCELED = "canceled"
    MAX_GOALS = "max_goals"
    NO_INFORMATION_GAIN = "no_information_gain"
    NAVIGATION_FAILED = "navigation_failed"
    NO_FRONTIER = "no_frontier"


@dataclass(frozen=True)
class ExplorationResult:
    stop_reason: ExplorationStopReason
    goals_attempted: int
    goals_reached: int
    initial_known_cells: int
    final_known_cells: int
    visited_goals: tuple[tuple[float, float], ...]
    last_navigation_status: NavigationStatus | None = None


class FrontierExplorationController:
    def __init__(
        self,
        navigation: NavigationClient,
        grid_provider: GridProvider,
        *,
        max_goals: int = 20,
        max_no_gain_attempts: int = 2,
        min_information_gain_cells: int = 5,
        goal_timeout_s: float = 30.0,
        poll_interval_s: float = 0.10,
        event_sink: Callable[[str, dict[str, object]], None] | None = None,
    ) -> None:
        self._navigation = navigation
        self._grid_provider = grid_provider
        self._max_goals = max_goals
        self._max_no_gain = max_no_gain_attempts
        self._min_gain = min_information_gain_cells
        self._goal_timeout = goal_timeout_s
        self._poll_interval = poll_interval_s
        self._event_sink = event_sink
        self._cancel = asyncio.Event()
        self._active_goal_id: str | None = None

    async def cancel(self) -> None:
        self._cancel.set()
        if self._active_goal_id is not None:
            await self._navigation.cancel(self._active_goal_id)

    async def run(self) -> ExplorationResult:
        self._cancel = asyncio.Event()
        visited: list[tuple[float, float]] = []
        first_grid = await _resolve_grid(self._grid_provider)
        initial_known = len(first_grid.known_free) + len(first_grid.occupied)
        previous_known = initial_known
        no_gain = attempted = reached = 0
        last_status: NavigationStatus | None = None
        reason = ExplorationStopReason.MAX_GOALS
        for _ in range(self._max_goals):
            if self._cancel.is_set():
                reason = ExplorationStopReason.CANCELED
                break
            grid = await _resolve_grid(self._grid_provider)
            localization = await self._navigation.get_localization_state()
            if localization.pose is None:
                reason = ExplorationStopReason.NAVIGATION_FAILED
                break
            if grid.frame_id != localization.pose.frame_id:
                reason = ExplorationStopReason.NAVIGATION_FAILED
                break
            robot_xy = (localization.pose.x_m, localization.pose.y_m)
            goals = find_frontier_goals(
                grid, robot_xy, visited=tuple(visited),
                min_frontier_length_m=max(0.3, grid.resolution_m),
                obstacle_clearance_m=0.0,
            )
            if not goals:
                reason = (
                    ExplorationStopReason.COMPLETE if visited
                    else ExplorationStopReason.NO_FRONTIER
                )
                break
            goal = goals[0]
            self._event("frontier_selected", x_m=goal.x_m, y_m=goal.y_m,
                        frontier_cells=goal.cell_count,
                        frontier_length_m=goal.cell_count*grid.resolution_m,
                        score=goal.score,
                        known_cells=previous_known, visited_count=len(visited))
            dx, dy = goal.x_m - robot_xy[0], goal.y_m - robot_xy[1]
            yaw = math.radians(localization.pose.yaw_degrees)
            handle = await self._navigation.set_relative_goal(RelativeNavigationGoal(
                forward_m=dx * math.cos(yaw) + dy * math.sin(yaw),
                left_m=-dx * math.sin(yaw) + dy * math.cos(yaw),
                max_duration_s=self._goal_timeout,
            ))
            attempted += 1
            if not handle.accepted:
                reason = ExplorationStopReason.NAVIGATION_FAILED
                break
            self._active_goal_id = handle.goal_id
            last_status = await self._wait_terminal()
            self._event("goal_terminal", attempt=attempted, status=last_status.value)
            self._active_goal_id = None
            if self._cancel.is_set() or last_status == NavigationStatus.CANCELED:
                reason = ExplorationStopReason.CANCELED
                break
            if last_status != NavigationStatus.SUCCEEDED:
                reason = ExplorationStopReason.NAVIGATION_FAILED
                break
            reached += 1
            visited.append((goal.x_m, goal.y_m))
            updated = await _resolve_grid(self._grid_provider)
            known = len(updated.known_free) + len(updated.occupied)
            self._event("information_gain", attempt=attempted,
                        previous_known_cells=previous_known,
                        known_cells=known, gained_cells=known-previous_known)
            if known - previous_known < self._min_gain:
                no_gain += 1
            else:
                no_gain = 0
            previous_known = known
            if no_gain >= self._max_no_gain:
                reason = ExplorationStopReason.NO_INFORMATION_GAIN
                break
        final_grid = await _resolve_grid(self._grid_provider)
        result = ExplorationResult(
            stop_reason=reason, goals_attempted=attempted, goals_reached=reached,
            initial_known_cells=initial_known,
            final_known_cells=len(final_grid.known_free) + len(final_grid.occupied),
            visited_goals=tuple(visited), last_navigation_status=last_status,
        )
        self._event("finished", stop_reason=reason.value, goals_attempted=attempted,
                    goals_reached=reached, initial_known_cells=initial_known,
                    final_known_cells=result.final_known_cells)
        return result

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
            self._event_sink(f"exploration_{kind}", fields)
        except Exception:
            pass


def evaluate_exploration_trace(events) -> dict[str, object]:
    selected = [row for row in events if row.get("event") == "exploration_frontier_selected"]
    terminals = [row for row in events if row.get("event") == "exploration_goal_terminal"]
    gains = [row for row in events if row.get("event") == "exploration_information_gain"]
    finished = [row for row in events if row.get("event") == "exploration_finished"]
    final = finished[-1] if finished else None
    failures = []
    if len(terminals) != len(selected):
        failures.append("incomplete_frontier_goal_chain")
    if any(row.get("status") != NavigationStatus.SUCCEEDED.value for row in terminals):
        failures.append("frontier_navigation_failed")
    if len(gains) != len(terminals):
        failures.append("information_gain_evidence_missing")
    allowed = {reason.value for reason in (
        ExplorationStopReason.COMPLETE, ExplorationStopReason.MAX_GOALS,
        ExplorationStopReason.NO_INFORMATION_GAIN,
    )}
    if final is None or final.get("stop_reason") not in allowed:
        failures.append("exploration_terminal_not_accepted")
    elif final.get("goals_reached", 0) <= 0:
        failures.append("no_frontier_goal_reached")
    return {"ok": not failures, "failures": failures,
            "frontiers_selected": len(selected), "goals_terminal": len(terminals),
            "gain_samples": len(gains),
            "known_cell_gain": ((final.get("final_known_cells", 0)-final.get("initial_known_cells", 0))
                                if final else None),
            "stop_reason": final.get("stop_reason") if final else None}


async def _resolve_grid(provider: GridProvider) -> OccupancyGrid2D:
    value = provider()
    if isinstance(value, Awaitable):
        return await value
    return value
