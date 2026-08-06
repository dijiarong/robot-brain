from __future__ import annotations

import asyncio
import unittest

from robot_brain.navigation import (
    FakeNavigationClient,
    NavigationPose,
    NavigationStatus,
    SurfaceNode,
    SurfacePath,
    TerrainPathController,
    evaluate_terrain_execution_trace,
)


def _path(*points: tuple[float, float, float]) -> SurfacePath:
    nodes = tuple(SurfaceNode(x, y, z, (i, 0, i)) for i, (x, y, z) in enumerate(points))
    return SurfacePath(nodes, 1.0, 0.1)


class TerrainPathControllerTests(unittest.IsolatedAsyncioTestCase):
    async def test_executes_surface_waypoints_through_navigation_contract(self) -> None:
        navigation = FakeNavigationClient(
            outcomes=[NavigationStatus.SUCCEEDED, NavigationStatus.SUCCEEDED],
            pose=NavigationPose(frame_id="odom"),
        )
        result = await TerrainPathController(navigation, poll_interval_s=0.001).run(
            _path((0, 0, 0), (0.4, 0, 0.05), (0.8, 0, 0.10))
        )
        self.assertTrue(result.completed)
        self.assertEqual(2, result.reached)
        self.assertEqual("goal_reached", result.stop_reason)

    async def test_execution_trace_requires_every_segment_to_succeed(self) -> None:
        events = []
        navigation = FakeNavigationClient(
            outcomes=[NavigationStatus.SUCCEEDED, NavigationStatus.SUCCEEDED],
            pose=NavigationPose(frame_id="odom"),
        )
        result = await TerrainPathController(
            navigation, poll_interval_s=.001,
            event_sink=lambda kind, fields: events.append({"event": kind, **fields}),
        ).run(_path((0, 0, 0), (.4, 0, .05), (.8, 0, .1)))
        self.assertTrue(result.completed)
        report = evaluate_terrain_execution_trace(events)
        self.assertTrue(report["ok"], report)
        self.assertEqual(2, report["segments_attempted"])

    def test_execution_trace_rejects_missing_terminal(self) -> None:
        report = evaluate_terrain_execution_trace([
            {"event": "terrain_execution_segment_command"},
            {"event": "terrain_execution_finished", "completed": True,
             "attempted": 1, "reached": 1, "stop_reason": "goal_reached"},
        ])
        self.assertFalse(report["ok"])
        self.assertIn("incomplete_terrain_segment_chain", report["failures"])

    async def test_cancel_propagates_to_active_segment(self) -> None:
        navigation = FakeNavigationClient(outcomes=[NavigationStatus.ACTIVE])
        controller = TerrainPathController(navigation, poll_interval_s=0.001)
        task = asyncio.create_task(controller.run(_path((0, 0, 0), (0.5, 0, 0))))
        await asyncio.sleep(0.01)
        await controller.cancel()
        result = await asyncio.wait_for(task, 1.0)
        self.assertTrue(result.canceled)
        self.assertEqual("canceled", result.stop_reason)

    async def test_rejects_oversized_segment_before_motion(self) -> None:
        navigation = FakeNavigationClient()
        result = await TerrainPathController(navigation).run(
            _path((0, 0, 0), (3.1, 0, 0)))
        self.assertFalse(result.completed)
        self.assertEqual("segment_exceeds_safety_limit", result.stop_reason)
        self.assertEqual([], navigation.command_history)
