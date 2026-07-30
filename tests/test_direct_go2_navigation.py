from __future__ import annotations

import asyncio
import time
import unittest

from config.settings import Settings
from robot_brain.actuation.unitree import FakeUnitreeTransport, UnitreeRobot, UnitreeState
from robot_brain.navigation import NavigationStatus, RelativeNavigationGoal
from robot_brain.navigation.direct_go2 import DirectGo2NavigationClient
from robot_brain.navigation.sensors import UnitreeNavigationSensorProvider
from robot_brain.perception.pointcloud import PointCloudSnapshot
from robot_brain.runtime.loop import AgentRuntime


class _LidarFakeTransport(FakeUnitreeTransport):
    def __init__(self, points=((2.0, 2.0, 0.2),)) -> None:
        super().__init__(UnitreeState(
            connected=True,
            is_standing=True,
            pose_frame_id="odom",
            pose_source="unitree_robotodom",
        ))
        self.points = points

    def read_lidar_snapshot(self) -> PointCloudSnapshot:
        return PointCloudSnapshot(
            points_xyz=tuple(self.points),
            frame_id="base_link",
            sensor_timestamp=time.time(),
            received_monotonic=time.monotonic(),
            source="fake",
            timestamp_valid=True,
        )

    def lidar_age_seconds(self) -> float:
        return 0.0


class _StuckTransport(_LidarFakeTransport):
    def _apply_command(self, command) -> None:
        if command.action == "stop":
            super()._apply_command(command)


class _LaggedOdomTransport(_LidarFakeTransport):
    """Apply drive immediately, but expose odometry after a short settle."""

    def __init__(self) -> None:
        super().__init__()
        self._pending: UnitreeState | None = None

    def _apply_command(self, command) -> None:
        if command.action != "drive":
            super()._apply_command(command)
            self._pending = None
            return
        before = self._state.model_copy(deep=True)
        super()._apply_command(command)
        after = self._state.model_copy(deep=True)
        self._state = before
        self._pending = after

    def publish_pending(self) -> None:
        if self._pending is not None:
            self._state = self._pending
            self._pending = None


class _AppearingObstacleTransport(_LidarFakeTransport):
    def __init__(self) -> None:
        super().__init__()
        self.drive_count = 0

    def _apply_command(self, command) -> None:
        super()._apply_command(command)
        if command.action == "drive":
            self.drive_count += 1
            if self.drive_count == 1:
                self.points = ((0.18, 0.0, 0.2),)


async def _client(transport=None, **kwargs):
    transport = transport or _LidarFakeTransport()
    await transport.connect()
    settings = Settings(
        robot_backend="unitree",
        unitree_enable_motion=True,
        unitree_dry_run=False,
        memory_db_path=":memory:",
    )
    robot = UnitreeRobot(transport, settings)
    sensors = UnitreeNavigationSensorProvider(transport)
    return DirectGo2NavigationClient(robot, sensors, **kwargs), robot


async def _terminal(client, timeout=2.0):
    deadline = time.monotonic() + timeout
    state = await client.get_state()
    while not state.status.terminal and time.monotonic() < deadline:
        await asyncio.sleep(0.01)
        state = await client.get_state()
    return state


class DirectGo2NavigationTests(unittest.IsolatedAsyncioTestCase):
    async def test_relative_translation_and_yaw_close_on_real_odom_delta(self) -> None:
        client, _ = await _client(segment_duration_s=0.25, odom_settle_s=0.0)
        handle = await client.set_relative_goal(
            RelativeNavigationGoal(forward_m=0.12, yaw_degrees=10.0, max_duration_s=5.0)
        )
        state = await _terminal(client, timeout=6.0)

        self.assertTrue(handle.accepted)
        self.assertEqual(NavigationStatus.SUCCEEDED, state.status)
        self.assertEqual(1.0, state.progress)
        self.assertGreater(state.pose.x_m, 0.10)  # type: ignore[union-attr]
        self.assertGreater(state.pose.yaw_degrees, 8.0)  # type: ignore[union-attr]

    async def test_closed_loop_keeps_driving_until_odom_reaches_goal(self) -> None:
        # Each drive only advances half the commanded distance; open-loop would
        # stop early, closed-loop must keep going until odom reaches the goal.
        class _HalfSpeedTransport(_LidarFakeTransport):
            def _apply_command(self, command) -> None:
                if command.action == "drive":
                    params = dict(command.parameters)
                    params["duration"] = float(params.get("duration", 0.0)) * 0.5
                    command = command.model_copy(update={"parameters": params})
                super()._apply_command(command)

        client, robot = await _client(
            _HalfSpeedTransport(),
            segment_duration_s=0.2,
            linear_speed_mps=0.2,
            odom_settle_s=0.0,
            reach_tolerance_m=0.015,
        )
        await client.set_relative_goal(
            RelativeNavigationGoal(forward_m=0.10, max_duration_s=5.0)
        )
        state = await _terminal(client, timeout=6.0)
        self.assertEqual(NavigationStatus.SUCCEEDED, state.status)
        self.assertGreaterEqual(state.pose.x_m, 0.085)  # type: ignore[union-attr]
        drives = [a for a in robot.action_history if a["action"] == "drive"]
        self.assertGreaterEqual(len(drives), 2)

    async def test_goal_times_out_if_odom_never_reaches_tolerance(self) -> None:
        class _CappedTransport(_LidarFakeTransport):
            def _apply_command(self, command) -> None:
                if command.action == "drive":
                    super()._apply_command(command)
                    if self._state.position.x > 0.03:
                        self._state.position = type(self._state.position)(
                            x=0.03, y=self._state.position.y
                        )
                    return
                super()._apply_command(command)

        client, _ = await _client(
            _CappedTransport(),
            segment_duration_s=0.2,
            linear_speed_mps=0.2,
            odom_settle_s=0.0,
            min_progress_m=0.001,
            max_no_progress_segments=20,
            reach_tolerance_m=0.015,
        )
        await client.set_relative_goal(
            RelativeNavigationGoal(forward_m=0.10, max_duration_s=1.0)
        )
        state = await _terminal(client, timeout=3.0)
        self.assertIn(
            state.status,
            {NavigationStatus.TIMED_OUT, NavigationStatus.NO_PROGRESS},
        )
        self.assertNotEqual(NavigationStatus.SUCCEEDED, state.status)
        self.assertLess(state.pose.x_m, 0.05)  # type: ignore[union-attr]

    async def test_obstacle_in_requested_corridor_rejects_goal(self) -> None:
        transport = _LidarFakeTransport(points=((0.2, 0.0, 0.2),))
        client, _ = await _client(transport)

        with self.assertRaisesRegex(Exception, "obstacle"):
            await client.set_relative_goal(RelativeNavigationGoal(forward_m=0.3))

    async def test_side_obstacle_does_not_block_forward_corridor(self) -> None:
        transport = _LidarFakeTransport(points=((0.2, 0.8, 0.2),))
        client, _ = await _client(transport, odom_settle_s=0.0)
        await client.set_relative_goal(RelativeNavigationGoal(forward_m=0.08))
        state = await _terminal(client)
        self.assertEqual(NavigationStatus.SUCCEEDED, state.status)

    async def test_stuck_robot_stops_after_bounded_no_progress(self) -> None:
        client, robot = await _client(
            _StuckTransport(),
            segment_duration_s=0.25,
            max_no_progress_segments=2,
            odom_settle_s=0.0,
        )
        await client.set_relative_goal(RelativeNavigationGoal(forward_m=0.3))
        state = await _terminal(client, timeout=5.0)

        self.assertEqual(NavigationStatus.NO_PROGRESS, state.status)
        self.assertTrue(any(a["action"] == "stop" for a in robot.action_history))

    async def test_lagged_odom_publishes_during_settle_window(self) -> None:
        transport = _LaggedOdomTransport()
        client, _ = await _client(
            transport,
            segment_duration_s=0.2,
            linear_speed_mps=0.2,
            min_progress_m=0.02,
            max_no_progress_segments=2,
            odom_settle_s=0.3,
        )

        async def release_lag() -> None:
            await asyncio.sleep(0.05)
            transport.publish_pending()

        original_drive = client._robot.drive

        async def drive_and_release(*args, **kwargs):
            await original_drive(*args, **kwargs)
            asyncio.create_task(release_lag())

        client._robot.drive = drive_and_release  # type: ignore[method-assign]
        await client.set_relative_goal(RelativeNavigationGoal(forward_m=0.08))
        state = await _terminal(client, timeout=5.0)
        self.assertEqual(NavigationStatus.SUCCEEDED, state.status)

    async def test_dynamic_obstacle_between_segments_stops_goal(self) -> None:
        transport = _AppearingObstacleTransport()
        client, robot = await _client(
            transport, segment_duration_s=0.25, odom_settle_s=0.0
        )
        await client.set_relative_goal(RelativeNavigationGoal(forward_m=0.3))
        state = await _terminal(client)

        self.assertEqual(NavigationStatus.FAILED, state.status)
        self.assertEqual("obstacle", state.error_code)
        drives = [a for a in robot.action_history if a["action"] == "drive"]
        self.assertEqual(1, len(drives))
        self.assertTrue(any(a["action"] == "stop" for a in robot.action_history))

    async def test_cancel_is_idempotent_and_stops_motion(self) -> None:
        client, robot = await _client(segment_duration_s=0.05, odom_settle_s=0.0)
        handle = await client.set_relative_goal(RelativeNavigationGoal(forward_m=1.0))
        state = await client.cancel(handle.goal_id)

        self.assertEqual(NavigationStatus.CANCELED, state.status)
        again = await client.cancel(handle.goal_id)
        self.assertEqual(NavigationStatus.CANCELED, again.status)
        self.assertTrue(any(a["action"] == "stop" for a in robot.action_history))


class DirectGo2RuntimeWiringTests(unittest.TestCase):
    def test_explicit_backend_registers_replaceable_navigation_skills(self) -> None:
        transport = _LidarFakeTransport()
        settings = Settings(
            robot_backend="unitree",
            perception_backend="mock",
            navigation_backend="direct_go2",
            unitree_enable_motion=True,
            unitree_dry_run=False,
            memory_db_path=":memory:",
        )
        robot = UnitreeRobot(transport, settings)
        runtime = AgentRuntime.create(settings=settings, robot=robot)

        self.assertIsInstance(runtime.context.navigation, DirectGo2NavigationClient)
        self.assertIsNotNone(runtime.context.skills.get("nav_go_relative"))
        self.assertIsNotNone(runtime.context.skills.get("nav_cancel"))
        runtime.close()

    def test_backend_rejects_transport_without_lidar(self) -> None:
        settings = Settings(
            robot_backend="unitree",
            perception_backend="mock",
            navigation_backend="direct_go2",
            memory_db_path=":memory:",
        )
        robot = UnitreeRobot(FakeUnitreeTransport(), settings)
        with self.assertRaisesRegex(ValueError, "built-in LiDAR"):
            AgentRuntime.create(settings=settings, robot=robot)
