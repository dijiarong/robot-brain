from __future__ import annotations

import asyncio
from collections import deque
import time
import unittest

from robot_brain.navigation import (
    CameraIntrinsics,
    ContinuousVisualServoController,
    FakeNavigationClient,
    NavigationStatus,
    VisualTargetObservation,
    compute_visual_servo_3d,
    evaluate_visual_servo_trace,
)


CAMERA = CameraIntrinsics(fx=500, fy=500, cx=320, cy=240, width=640, height=480)


def _bbox(distance: float, center_x: float = 320.0):
    width = 0.45 * CAMERA.fx / distance
    return (center_x-width/2, 100.0, center_x+width/2, 300.0)


def _observation(distance: float = 1.5, center_x: float = 320.0):
    return VisualTargetObservation(time.monotonic(), bbox=_bbox(distance, center_x))


class NativeVisualControllerTests(unittest.IsolatedAsyncioTestCase):
    async def test_requires_stable_reached_frames_without_motion(self) -> None:
        navigation = FakeNavigationClient()
        controller = ContinuousVisualServoController(
            navigation, lambda: _observation(), CAMERA,
            stable_frames=3, poll_interval_s=0.001,
        )
        result = await controller.run()
        self.assertTrue(result.completed)
        self.assertEqual("target_reached", result.stop_reason)
        self.assertEqual(0, result.commands)

    async def test_approach_segment_then_reacquires_and_finishes(self) -> None:
        observations = deque([_observation(3.0), _observation(), _observation()])
        navigation = FakeNavigationClient(outcomes=[NavigationStatus.SUCCEEDED])
        controller = ContinuousVisualServoController(
            navigation, lambda: observations.popleft(), CAMERA,
            stable_frames=2, poll_interval_s=0.001,
        )
        result = await controller.run()
        self.assertTrue(result.completed)
        self.assertEqual(1, result.commands)
        self.assertTrue(any(row["action"] == "set_relative_goal"
                            for row in navigation.command_history))

    async def test_trace_proves_reacquisition_and_stable_convergence(self) -> None:
        events = []
        observations = deque([_observation(3.0, 400), _observation(), _observation()])
        controller = ContinuousVisualServoController(
            FakeNavigationClient(outcomes=[NavigationStatus.SUCCEEDED]),
            lambda: observations.popleft(), CAMERA, stable_frames=2,
            poll_interval_s=.001,
            event_sink=lambda kind, fields: events.append({"event": kind, **fields}),
        )
        self.assertTrue((await controller.run()).completed)
        report = evaluate_visual_servo_trace(events)
        self.assertTrue(report["ok"], report)
        self.assertEqual(1, report["commands"])
        self.assertEqual(3, report["observations"])
        self.assertLess(report["final_center_error"], report["initial_center_error"])

    def test_trace_rejects_planned_command_without_terminal_or_reacquisition(self) -> None:
        report = evaluate_visual_servo_trace([
            {"event": "visual_servo_observation", "estimated_distance_m": 3.0,
             "normalized_center_error": .2},
            {"event": "visual_servo_command"},
            {"event": "visual_servo_finished", "completed": True,
             "stop_reason": "target_reached"},
        ])
        self.assertFalse(report["ok"])
        self.assertIn("incomplete_navigation_command_chain", report["failures"])
        self.assertIn("target_not_reacquired_after_motion", report["failures"])

    async def test_lost_or_stale_detection_fails_closed(self) -> None:
        navigation = FakeNavigationClient()
        lost = ContinuousVisualServoController(
            navigation, lambda: None, CAMERA, max_lost_frames=1,
            poll_interval_s=0.001,
        )
        self.assertEqual("target_lost", (await lost.run()).stop_reason)
        stale_value = VisualTargetObservation(
            time.monotonic()-10.0, bbox=_bbox(2.0),
        )
        stale = ContinuousVisualServoController(
            navigation, lambda: stale_value, CAMERA, max_lost_frames=0,
            poll_interval_s=0.001,
        )
        self.assertEqual("stale_detection", (await stale.run()).stop_reason)

    async def test_detection_timeout_and_error_are_structured(self) -> None:
        async def slow():
            await asyncio.sleep(1.0)

        timeout = ContinuousVisualServoController(
            FakeNavigationClient(), slow, CAMERA,
            observation_timeout_s=0.001, max_lost_frames=0,
            poll_interval_s=0.001,
        )
        self.assertEqual("detection_timeout", (await timeout.run()).stop_reason)

        def broken():
            raise RuntimeError("detector unavailable")

        error = ContinuousVisualServoController(
            FakeNavigationClient(), broken, CAMERA,
            max_lost_frames=0, poll_interval_s=0.001,
        )
        self.assertEqual("detection_provider_error", (await error.run()).stop_reason)

    async def test_cancel_propagates_to_active_short_navigation_segment(self) -> None:
        navigation = FakeNavigationClient(outcomes=[NavigationStatus.ACTIVE])
        controller = ContinuousVisualServoController(
            navigation, lambda: _observation(3.0), CAMERA, poll_interval_s=0.001,
        )
        task = asyncio.create_task(controller.run())
        await asyncio.sleep(0.01)
        await controller.cancel()
        result = await asyncio.wait_for(task, 1.0)
        self.assertTrue(result.canceled)
        self.assertEqual("canceled", result.stop_reason)

    def test_3d_servo_turns_toward_body_frame_target(self) -> None:
        command = compute_visual_servo_3d((2.0, 1.0, 0.8))
        self.assertTrue(command.valid)
        self.assertGreater(command.forward_mps, 0.0)
        self.assertGreater(command.yaw_rps, 0.0)


if __name__ == "__main__":
    unittest.main()
