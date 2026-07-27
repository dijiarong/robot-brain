"""Offline contract tests for the Navigation repository Nav2 adapter."""
from __future__ import annotations

import unittest

from config.settings import Settings
from robot_brain.navigation import (
    AbsoluteNavigationGoal,
    LocalizationStatus,
    NavigationPose,
    NavigationStatus,
    Nav2GoalSnapshot,
    Nav2GoalSubmission,
    Nav2NavigationClient,
    RelativeNavigationGoal,
)
from robot_brain.runtime.loop import AgentRuntime


class FakeNav2Bridge:
    def __init__(self) -> None:
        self.ready = True
        self.pose = NavigationPose(x_m=2.0, y_m=3.0, yaw_degrees=90.0, frame_id="odom")
        self.submission = Nav2GoalSubmission("nav2-1", True, "accepted")
        self.snapshot = Nav2GoalSnapshot(2, distance_remaining_m=0.5, message="active")
        self.sent_pose: NavigationPose | None = None
        self.canceled: list[str] = []
        self.closed = False

    def is_ready(self, timeout_s: float) -> bool:
        return self.ready

    def get_pose(self, timeout_s: float) -> NavigationPose | None:
        return self.pose

    def get_pose_in_frame(self, frame_id: str, timeout_s: float) -> NavigationPose | None:
        if self.pose is None:
            return None
        return self.pose.model_copy(update={"frame_id": frame_id})

    def send_goal(self, pose: NavigationPose, *, timeout_s: float) -> Nav2GoalSubmission:
        self.sent_pose = pose
        return self.submission

    def get_goal(self, goal_id: str) -> Nav2GoalSnapshot:
        return self.snapshot

    def cancel_goal(self, goal_id: str, *, timeout_s: float) -> bool:
        self.canceled.append(goal_id)
        return True

    def close(self) -> None:
        self.closed = True


class Nav2NavigationClientTests(unittest.IsolatedAsyncioTestCase):
    async def test_relative_goal_becomes_odom_goal(self):
        bridge = FakeNav2Bridge()
        client = Nav2NavigationClient(bridge)
        handle = await client.set_relative_goal(
            RelativeNavigationGoal(forward_m=1.0, left_m=0.2, yaw_degrees=30.0)
        )

        self.assertTrue(handle.accepted)
        self.assertAlmostEqual(1.8, bridge.sent_pose.x_m, places=6)  # type: ignore[union-attr]
        self.assertAlmostEqual(4.0, bridge.sent_pose.y_m, places=6)  # type: ignore[union-attr]
        self.assertAlmostEqual(120.0, bridge.sent_pose.yaw_degrees)  # type: ignore[union-attr]
        self.assertEqual("odom", bridge.sent_pose.frame_id)  # type: ignore[union-attr]

    async def test_active_feedback_maps_progress(self):
        bridge = FakeNav2Bridge()
        client = Nav2NavigationClient(bridge)
        await client.set_relative_goal(RelativeNavigationGoal(forward_m=1.0))
        state = await client.get_state()
        self.assertEqual(NavigationStatus.ACTIVE, state.status)
        self.assertAlmostEqual(0.5, state.progress)

    async def test_success_and_cancel_status_mapping(self):
        bridge = FakeNav2Bridge()
        client = Nav2NavigationClient(bridge)
        handle = await client.set_relative_goal(RelativeNavigationGoal(forward_m=0.4))
        bridge.snapshot = Nav2GoalSnapshot(4, distance_remaining_m=0.0, message="done")
        state = await client.get_state()
        self.assertEqual(NavigationStatus.SUCCEEDED, state.status)

        bridge.submission = Nav2GoalSubmission("nav2-2", True)
        await client.set_relative_goal(RelativeNavigationGoal(forward_m=0.2))
        state = await client.cancel("nav2-2")
        self.assertEqual(NavigationStatus.CANCELED, state.status)
        self.assertEqual(["nav2-2"], bridge.canceled)
        self.assertEqual("nav2-1", handle.goal_id)

    async def test_aborted_progress_failure_maps_no_progress(self):
        bridge = FakeNav2Bridge()
        client = Nav2NavigationClient(bridge)
        await client.set_relative_goal(RelativeNavigationGoal(forward_m=0.4))
        bridge.snapshot = Nav2GoalSnapshot(6, message="Failed to make progress")
        state = await client.get_state()
        self.assertEqual(NavigationStatus.NO_PROGRESS, state.status)

    async def test_unavailable_action_and_missing_odom_are_explicit(self):
        bridge = FakeNav2Bridge()
        bridge.ready = False
        client = Nav2NavigationClient(bridge)
        state = await client.get_state()
        self.assertFalse(state.ready)
        self.assertEqual(NavigationStatus.UNAVAILABLE, state.status)

        bridge.ready = True
        bridge.pose = None
        with self.assertRaisesRegex(Exception, "odometry"):
            await client.set_relative_goal(RelativeNavigationGoal(forward_m=0.2))

    async def test_close_delegates_to_bridge(self):
        bridge = FakeNav2Bridge()
        await Nav2NavigationClient(bridge).aclose()
        self.assertTrue(bridge.closed)

    async def test_map_localization_and_absolute_goal_require_matching_identity(self):
        bridge = FakeNav2Bridge()
        client = Nav2NavigationClient(
            bridge, map_id="office", map_version="v1", map_frame="map"
        )
        localization = await client.get_localization_state()
        self.assertEqual(LocalizationStatus.LOCALIZED, localization.status)
        self.assertTrue(localization.usable_for_persistent_memory)

        handle = await client.set_absolute_goal(AbsoluteNavigationGoal(
            pose=NavigationPose(x_m=4.0, y_m=5.0, frame_id="map"),
            map_id="office",
            map_version="v1",
        ))
        self.assertTrue(handle.accepted)
        self.assertEqual("map", bridge.sent_pose.frame_id)  # type: ignore[union-attr]

    async def test_absolute_goal_rejects_wrong_map(self):
        bridge = FakeNav2Bridge()
        client = Nav2NavigationClient(bridge, map_id="office", map_frame="map")
        with self.assertRaisesRegex(Exception, "different map"):
            await client.set_absolute_goal(AbsoluteNavigationGoal(
                pose=NavigationPose(frame_id="map"), map_id="warehouse"
            ))


class Nav2RuntimeWiringTests(unittest.TestCase):
    def test_nav2_setting_registers_provider_without_importing_rclpy(self):
        runtime = AgentRuntime.create(
            settings=Settings(
                robot_backend="mock",
                navigation_backend="nav2",
                memory_db_path=":memory:",
            )
        )
        self.assertIsInstance(runtime.context.navigation, Nav2NavigationClient)
        self.assertIsNotNone(runtime.context.skills.get("nav_go_relative"))
        self.assertEqual(
            "Nav2NavigationClient", runtime.diagnostics()["navigation"]["provider"]
        )
        runtime.close()


if __name__ == "__main__":
    unittest.main()
