from __future__ import annotations

import asyncio
import unittest

from robot_brain.navigation import (
    FakeNavigationClient,
    MapIdentity,
    NavigationPose,
    NavigationStatus,
)
from robot_brain.navigation.patrol_controller import PatrolController, evaluate_patrol_trace


class NativePatrolControllerTests(unittest.IsolatedAsyncioTestCase):
    async def test_runs_route_for_requested_cycles(self) -> None:
        events = [{"event": "patrol_route", "strategy": "coverage",
                   "route_evaluation": {"ok": True}}]
        navigation = FakeNavigationClient(
            outcomes=[NavigationStatus.SUCCEEDED] * 4,
            pose=NavigationPose(frame_id="map"),
            map_identity=MapIdentity(map_id="office", version="v1", frame_id="map"),
        )
        controller = PatrolController(
            navigation, poll_interval_s=0.001,
            event_sink=lambda kind, fields: events.append({"event": kind, **fields}),
        )
        result = await controller.run([
            NavigationPose(x_m=0.5, y_m=0.0, frame_id="map"),
            NavigationPose(x_m=0.5, y_m=0.5, frame_id="map"),
        ], cycles=2)
        self.assertTrue(result.completed)
        self.assertEqual(4, result.reached)
        self.assertEqual(2, result.cycles_completed)
        self.assertTrue(all(
            row["action"] == "set_absolute_goal"
            for row in navigation.command_history
        ))
        report = evaluate_patrol_trace(events)
        self.assertTrue(report["ok"], report)
        self.assertEqual("coverage", report["strategy"])
        self.assertEqual(2, report["cycles_completed"])

    def test_patrol_trace_requires_route_and_complete_waypoint_chain(self) -> None:
        report = evaluate_patrol_trace([
            {"event": "patrol_waypoint_command"},
            {"event": "patrol_finished", "completed": True, "attempted": 1,
             "reached": 1, "failed": 0, "cycles_completed": 1},
        ])
        self.assertFalse(report["ok"])
        self.assertIn("patrol_route_evidence_missing", report["failures"])
        self.assertIn("incomplete_patrol_waypoint_chain", report["failures"])

    async def test_failure_stops_route_by_default(self) -> None:
        navigation = FakeNavigationClient(outcomes=[NavigationStatus.FAILED])
        controller = PatrolController(navigation, poll_interval_s=0.001)
        result = await controller.run([
            NavigationPose(x_m=0.5), NavigationPose(x_m=1.0),
        ])
        self.assertFalse(result.completed)
        self.assertEqual(1, result.attempted)
        self.assertEqual(1, result.failed)

    async def test_cancel_propagates_to_active_goal(self) -> None:
        navigation = FakeNavigationClient(outcomes=[NavigationStatus.ACTIVE])
        controller = PatrolController(navigation, poll_interval_s=0.001)
        task = asyncio.create_task(controller.run([NavigationPose(x_m=0.5)]))
        await asyncio.sleep(0.01)
        await controller.cancel()
        result = await asyncio.wait_for(task, timeout=1.0)
        self.assertTrue(result.canceled)
        self.assertEqual(NavigationStatus.CANCELED, result.last_status)
