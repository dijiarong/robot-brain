from __future__ import annotations

import asyncio
import unittest

from robot_brain.navigation import (
    FakeNavigationClient, NavigationStatus, NavigationUnavailableError,
    SurfaceNode, SurfacePath, TerrainFrontierExplorationController,
    evaluate_terrain_exploration_trace,
)


def _path(offset: float) -> SurfacePath:
    nodes = (
        SurfaceNode(offset, 0, 0, (int(offset*10), 0, 0)),
        SurfaceNode(offset+.2, 0, 0, (int(offset*10)+2, 0, 0)),
    )
    return SurfacePath(nodes, .2, 0)


class _TerrainExplorationNavigation(FakeNavigationClient):
    def __init__(self, paths, **kwargs):
        super().__init__(**kwargs)
        self.paths = list(paths)
        self.plan_calls = []

    async def plan_terrain_frontier(self, **kwargs):
        self.plan_calls.append(kwargs)
        if not self.paths:
            raise NavigationUnavailableError("no safe reachable terrain frontier")
        return self.paths.pop(0)


class NativeTerrainExplorationTests(unittest.IsolatedAsyncioTestCase):
    async def test_runs_bounded_frontier_goals_and_passes_visited_blacklist(self) -> None:
        events = [{"event": "terrain_frontier_plan"},
                  {"event": "terrain_frontier_plan"}]
        navigation = _TerrainExplorationNavigation(
            [_path(0), _path(.2)],
            outcomes=[NavigationStatus.SUCCEEDED, NavigationStatus.SUCCEEDED],
        )
        result = await TerrainFrontierExplorationController(
            navigation, max_goals=2, exploration_range_m=2,
            event_sink=lambda kind, fields: events.append({"event": kind, **fields}),
        ).run()
        self.assertTrue(result.completed)
        self.assertEqual(2, result.goals_reached)
        self.assertEqual((), navigation.plan_calls[0]["visited_xy"])
        self.assertEqual(((.2, 0),), navigation.plan_calls[1]["visited_xy"])
        report = evaluate_terrain_exploration_trace(events)
        self.assertTrue(report["ok"], report)
        self.assertEqual(2, report["goals_reached"])

    def test_trace_rejects_goal_without_frontier_score_or_reach(self) -> None:
        report = evaluate_terrain_exploration_trace([
            {"event": "terrain_exploration_goal_planned"},
            {"event": "terrain_exploration_finished", "completed": True,
             "goals_attempted": 1, "goals_reached": 1, "stop_reason": "max_goals"},
        ])
        self.assertFalse(report["ok"])
        self.assertIn("terrain_frontier_score_evidence_missing", report["failures"])
        self.assertIn("incomplete_terrain_exploration_goal_chain", report["failures"])

    async def test_no_frontier_and_navigation_failure_are_structured(self) -> None:
        empty = _TerrainExplorationNavigation([])
        result = await TerrainFrontierExplorationController(empty).run()
        self.assertFalse(result.completed)
        self.assertEqual("no_terrain_frontier", result.stop_reason)

        failed = _TerrainExplorationNavigation(
            [_path(0)], outcomes=[NavigationStatus.FAILED],
        )
        result = await TerrainFrontierExplorationController(failed).run()
        self.assertEqual("terrain_navigation_failed", result.stop_reason)

    async def test_cancel_propagates_to_active_terrain_segment(self) -> None:
        navigation = _TerrainExplorationNavigation(
            [_path(0)], outcomes=[NavigationStatus.ACTIVE],
        )
        controller = TerrainFrontierExplorationController(navigation)
        task = asyncio.create_task(controller.run())
        await asyncio.sleep(.02)
        await controller.cancel()
        result = await asyncio.wait_for(task, 1)
        self.assertTrue(result.canceled)
        self.assertEqual("canceled", result.stop_reason)


if __name__ == "__main__":
    unittest.main()
