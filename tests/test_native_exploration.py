from __future__ import annotations

import asyncio
import unittest

from robot_brain.navigation import FakeNavigationClient, NavigationStatus
from robot_brain.navigation.exploration import (
    ExplorationStopReason,
    FrontierExplorationController,
    evaluate_exploration_trace,
)
from robot_brain.navigation.grid import OccupancyGrid2D


def _grid(size: int) -> OccupancyGrid2D:
    start = 15 - size // 2
    known = {(x, y) for x in range(start, start + size) for y in range(start, start + size)}
    return OccupancyGrid2D(
        resolution_m=0.1, width=30, height=30,
        origin_x_m=-1.5, origin_y_m=-1.5,
        occupied=frozenset(), known_free=frozenset(known), frame_id="odom",
    )


class _GrowingGrid:
    def __init__(self, sizes):
        self.sizes = list(sizes)
        self.index = 0

    def __call__(self):
        value = _grid(self.sizes[min(self.index, len(self.sizes) - 1)])
        self.index += 1
        return value


class NativeExplorationTests(unittest.IsolatedAsyncioTestCase):
    async def test_exploration_reaches_bounded_number_of_frontiers(self) -> None:
        events = []
        controller = FrontierExplorationController(
            FakeNavigationClient(outcomes=[NavigationStatus.SUCCEEDED] * 4),
            _GrowingGrid([9, 11, 13, 15, 17, 19]),
            max_goals=2, min_information_gain_cells=1,
            event_sink=lambda kind, fields: events.append({"event": kind, **fields}),
        )
        result = await controller.run()
        self.assertEqual(ExplorationStopReason.MAX_GOALS, result.stop_reason)
        self.assertEqual(2, result.goals_reached)
        self.assertGreater(result.final_known_cells, result.initial_known_cells)
        report = evaluate_exploration_trace(events)
        self.assertTrue(report["ok"], report)
        self.assertGreater(report["known_cell_gain"], 0)

    async def test_no_information_gain_stops_loop(self) -> None:
        controller = FrontierExplorationController(
            FakeNavigationClient(outcomes=[NavigationStatus.SUCCEEDED] * 4),
            lambda: _grid(11), max_goals=4, max_no_gain_attempts=2,
            min_information_gain_cells=1,
        )
        result = await controller.run()
        self.assertEqual(ExplorationStopReason.NO_INFORMATION_GAIN, result.stop_reason)
        self.assertEqual(2, result.goals_reached)

    async def test_navigation_failure_stops_exploration(self) -> None:
        controller = FrontierExplorationController(
            FakeNavigationClient(outcomes=[NavigationStatus.FAILED]),
            lambda: _grid(11), max_goals=4,
        )
        result = await controller.run()
        self.assertEqual(ExplorationStopReason.NAVIGATION_FAILED, result.stop_reason)
        self.assertEqual(NavigationStatus.FAILED, result.last_navigation_status)

    def test_exploration_trace_rejects_missing_gain_evidence(self) -> None:
        report = evaluate_exploration_trace([
            {"event": "exploration_frontier_selected"},
            {"event": "exploration_goal_terminal", "status": "succeeded"},
            {"event": "exploration_finished", "stop_reason": "max_goals",
             "goals_reached": 1, "initial_known_cells": 10, "final_known_cells": 10},
        ])
        self.assertFalse(report["ok"])
        self.assertIn("information_gain_evidence_missing", report["failures"])

    async def test_cancel_propagates_to_active_navigation(self) -> None:
        navigation = FakeNavigationClient(outcomes=[NavigationStatus.ACTIVE])
        controller = FrontierExplorationController(
            navigation, lambda: _grid(11), max_goals=4, poll_interval_s=0.001,
        )
        task = asyncio.create_task(controller.run())
        await asyncio.sleep(0.01)
        await controller.cancel()
        result = await asyncio.wait_for(task, timeout=1.0)
        self.assertEqual(ExplorationStopReason.CANCELED, result.stop_reason)
        self.assertEqual(NavigationStatus.CANCELED, result.last_navigation_status)
