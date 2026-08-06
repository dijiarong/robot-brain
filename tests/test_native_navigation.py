from __future__ import annotations

import asyncio
import math
import time
import unittest

from config.settings import Settings
from robot_brain.actuation.unitree import FakeUnitreeTransport, UnitreeRobot, UnitreeState
from robot_brain.core.world_state import Position
from robot_brain.core.robot_self_state import RobotPose
from robot_brain.navigation import (
    AbsoluteNavigationGoal,
    NavigationPose,
    NavigationStatus,
    RelativeNavigationGoal,
    SparseVoxelMap,
    OnlinePoseGraphTracker,
    PoseGraphTrackerConfig,
    NavigationMotionSafetySignal,
    OccupancyGrid2D,
    path_collision_fraction,
    path_corridor_cells,
    path_minimum_clearance_m,
)
from robot_brain.navigation.grid import costmap_from_pointcloud
from robot_brain.navigation.native_go2 import NativeGo2NavigationClient
from robot_brain.navigation.planner import astar_path
from robot_brain.navigation.sensors import UnitreeNavigationSensorProvider
from robot_brain.perception.pointcloud import PointCloudSnapshot
from robot_brain.runtime.loop import AgentRuntime


def _cloud(points):
    return PointCloudSnapshot(
        points_xyz=tuple(points), frame_id="base_link", sensor_timestamp=time.time(),
        received_monotonic=time.monotonic(), source="test", timestamp_valid=True,
    )


class NativeGridPlannerTests(unittest.TestCase):
    def test_soft_obstacle_cost_makes_astar_choose_clearer_route(self) -> None:
        costs = [0]*(7*5)
        for column in range(1, 6):
            costs[2*7+column] = 99
        grid = OccupancyGrid2D(
            resolution_m=1, width=7, height=5,
            origin_x_m=0, origin_y_m=0, occupied=frozenset(),
            traversal_cost_values=tuple(costs),
        )
        path = astar_path(grid, (.5, 2.5), (6.5, 2.5))
        self.assertIsNotNone(path)
        self.assertTrue(any(abs(y-2.5) > .5 for _, y in path))

    def test_pointcloud_cost_gradient_decreases_away_from_inflated_obstacle(self) -> None:
        grid = costmap_from_pointcloud(
            _cloud(((1, 0, .2),)), size_m=4, resolution_m=.1,
            robot_radius_m=.1, obstacle_cost_radius_m=1,
        )
        near = grid.world_to_cell(.7, 0)
        far = grid.world_to_cell(-1, 0)
        self.assertGreater(grid.traversal_cost(near), grid.traversal_cost(far))

    def test_path_corridor_checks_segments_and_full_robot_width(self) -> None:
        grid = OccupancyGrid2D(
            resolution_m=1, width=7, height=6,
            origin_x_m=0, origin_y_m=0,
            occupied=frozenset({(3, 3)}),
        )
        path = [(.5, 2.5), (6.5, 2.5)]
        centerline = path_corridor_cells(grid, path, robot_width_m=0)
        corridor = path_corridor_cells(grid, path, robot_width_m=2)
        self.assertNotIn((3, 3), centerline)
        self.assertIn((3, 3), corridor)
        self.assertGreater(path_collision_fraction(grid, path, robot_width_m=2), 0)

        blocked_center = grid.__class__(**{
            **grid.__dict__, "occupied": frozenset({(3, 2)}),
        })
        self.assertEqual(0, path_minimum_clearance_m(blocked_center, path))

    def test_path_clearance_only_exempts_occupied_start_cell(self) -> None:
        grid = OccupancyGrid2D(
            resolution_m=1, width=5, height=3,
            origin_x_m=0, origin_y_m=0,
            occupied=frozenset({(0, 1), (3, 1)}),
        )
        path = [(.5, 1.5), (4.5, 1.5)]
        self.assertEqual(0, path_minimum_clearance_m(grid, path))
        start_only = grid.__class__(**{
            **grid.__dict__, "occupied": frozenset({(0, 1)}),
        })
        self.assertGreater(path_minimum_clearance_m(start_only, path), 0)

    def test_straight_path(self) -> None:
        grid = costmap_from_pointcloud(_cloud(((2.0, 2.0, 0.2),)), robot_radius_m=0.2)
        path = astar_path(grid, (0.0, 0.0), (1.0, 0.0))
        self.assertIsNotNone(path)
        self.assertLessEqual(len(path or []), 3)

    def test_path_detours_around_inflated_obstacle(self) -> None:
        grid = costmap_from_pointcloud(
            _cloud(((0.5, 0.0, 0.2),)), resolution_m=0.1, robot_radius_m=0.25
        )
        path = astar_path(grid, (0.0, 0.0), (1.0, 0.0))
        self.assertIsNotNone(path)
        self.assertTrue(any(abs(y) > 0.2 for _, y in path or []))

    def test_full_wall_has_no_path(self) -> None:
        wall = tuple((0.5, -3.0 + index * 0.1, 0.2) for index in range(61))
        grid = costmap_from_pointcloud(_cloud(wall), size_m=6.0, robot_radius_m=0.2)
        self.assertIsNone(astar_path(grid, (0.0, 0.0), (1.0, 0.0)))


class _WorldCloudTransport(FakeUnitreeTransport):
    def __init__(self, obstacles=(), *, stuck=False, dynamic=False) -> None:
        super().__init__(UnitreeState(
            connected=True, is_standing=True, pose_frame_id="odom",
            pose_source="unitree_robotodom", position=Position(),
        ))
        self.obstacles = list(obstacles)
        self.stuck = stuck
        self.dynamic = dynamic
        self.drive_count = 0
        self.stale = False

    def read_lidar_snapshot(self):
        yaw = math.radians(self._state.heading_degrees)
        points = []
        for wx, wy, wz in self.obstacles:
            dx, dy = wx - self._state.position.x, wy - self._state.position.y
            points.append((dx * math.cos(yaw) + dy * math.sin(yaw),
                           -dx * math.sin(yaw) + dy * math.cos(yaw), wz))
        if not points:
            points = [(3.0, 3.0, 0.2)]
        return _cloud(points)

    def lidar_age_seconds(self) -> float:
        return 5.0 if self.stale else 0.0

    def _apply_command(self, command) -> None:
        if command.action == "drive":
            self.drive_count += 1
            if self.dynamic and self.drive_count == 1:
                self.obstacles.append((0.35, 0.0, 0.2))
            if self.stuck:
                return
        super()._apply_command(command)


async def _client(transport: _WorldCloudTransport, **kwargs):
    await transport.connect()
    robot = UnitreeRobot(transport, Settings(
        robot_backend="unitree", unitree_enable_motion=True, unitree_dry_run=False,
        unitree_max_speed=0.5, unitree_max_drive_duration=0.5, memory_db_path=":memory:",
    ))
    sensors = UnitreeNavigationSensorProvider(transport)
    return NativeGo2NavigationClient(
        robot, sensors, linear_speed_mps=0.5, segment_duration_s=0.2,
        robot_radius_m=0.15, reach_tolerance_m=0.08, settle_s=0.0, **kwargs,
    )


async def _terminal(client, timeout=3.0):
    deadline = time.monotonic() + timeout
    state = await client.get_state()
    while not state.status.terminal and time.monotonic() < deadline:
        await asyncio.sleep(0.01)
        state = await client.get_state()
    return state


class NativeGo2ClientTests(unittest.IsolatedAsyncioTestCase):
    async def test_live_local_planner_uses_safe_goal_for_occupied_requested_cell(self) -> None:
        transport = _WorldCloudTransport(obstacles=((0.5, 0.0, 0.2),))
        client = await _client(transport, max_no_path_replans=1)
        await client.set_relative_goal(
            RelativeNavigationGoal(forward_m=0.5, max_duration_s=0.5)
        )
        await _terminal(client, timeout=1.0)
        adjustments = [
            row for row in client.trace if row["event"] == "safe_goal_adjusted"
        ]
        self.assertTrue(adjustments)
        self.assertLessEqual(adjustments[0]["adjustment_m"], 0.5)

    async def test_external_safety_stop_prevents_motion(self) -> None:
        transport = _WorldCloudTransport()
        client = await _client(transport)
        client.set_motion_safety_signal(NavigationMotionSafetySignal.now(
            stop_requested=True, reason="bumper_stop",
        ))
        await client.set_relative_goal(RelativeNavigationGoal(forward_m=.3, max_duration_s=2))
        state = await _terminal(client)
        self.assertEqual(NavigationStatus.FAILED, state.status)
        self.assertEqual("external_safety_stop", state.stop_reason)
        self.assertEqual(0, transport.drive_count)

    async def test_stale_external_safety_signal_fails_closed(self) -> None:
        transport = _WorldCloudTransport()
        client = await _client(transport, safety_signal_max_age_s=.01)
        client.set_motion_safety_signal(NavigationMotionSafetySignal(
            speed_scale=.5, observed_monotonic=time.monotonic()-1,
        ))
        await client.set_relative_goal(RelativeNavigationGoal(forward_m=.3, max_duration_s=2))
        state = await _terminal(client)
        self.assertEqual(NavigationStatus.UNAVAILABLE, state.status)
        self.assertEqual("stale_safety_signal", state.stop_reason)
        self.assertEqual(0, transport.drive_count)

    async def test_slowdown_and_acceleration_are_visible_in_command_trace(self) -> None:
        transport = _WorldCloudTransport()
        client = await _client(transport, max_acceleration_mps2=.5,
                               safety_signal_max_age_s=10)
        client.set_motion_safety_signal(NavigationMotionSafetySignal.now(speed_scale=.5))
        await client.set_relative_goal(RelativeNavigationGoal(forward_m=.3, max_duration_s=4))
        state = await _terminal(client, timeout=5)
        self.assertEqual(NavigationStatus.SUCCEEDED, state.status)
        commands = [row for row in client.trace if row["event"] == "command"
                    and (row.get("vx_mps") or row.get("vy_mps"))]
        speeds = [math.hypot(row["vx_mps"], row["vy_mps"]) for row in commands]
        self.assertTrue(speeds)
        self.assertLessEqual(max(speeds), .25+1e-9)
        self.assertTrue(all(row["speed_scale"] == .5 for row in commands))

    async def test_optional_pose_graph_only_changes_global_mapping_frame(self) -> None:
        tracker = OnlinePoseGraphTracker(PoseGraphTrackerConfig(
            keyframe_translation_m=0.1, keyframe_yaw_degrees=5,
            minimum_loop_age_s=1,
        ))
        transport = _WorldCloudTransport()
        client = await _client(transport, pose_graph_tracker=tracker)
        grid = await client.get_costmap()
        localization = await client.get_localization_state()
        self.assertEqual("map", grid.frame_id)
        self.assertEqual("map", localization.pose.frame_id)
        self.assertEqual("map", localization.map_identity.frame_id)
        self.assertTrue(any(row["event"] == "pose_graph_update" for row in client.trace))

        await client.set_relative_goal(RelativeNavigationGoal(forward_m=0.3, max_duration_s=3))
        state = await _terminal(client)
        self.assertEqual(NavigationStatus.SUCCEEDED, state.status)
        self.assertEqual("odom", state.pose.frame_id)

    async def test_invalid_safety_configuration_fails_closed_at_construction(self) -> None:
        transport = _WorldCloudTransport()
        await transport.connect()
        settings = Settings(robot_backend="unitree", memory_db_path=":memory:")
        robot = UnitreeRobot(transport, settings)
        sensors = UnitreeNavigationSensorProvider(transport)
        with self.assertRaisesRegex(ValueError, "safety parameter"):
            NativeGo2NavigationClient(robot, sensors, linear_speed_mps=-0.1)
        with self.assertRaisesRegex(ValueError, "too small"):
            NativeGo2NavigationClient(
                robot, sensors, map_size_m=0.5, robot_radius_m=0.3,
                resolution_m=0.1,
            )

    async def test_success_records_path_replans_and_stop_reason(self) -> None:
        client = await _client(_WorldCloudTransport())
        await client.set_relative_goal(RelativeNavigationGoal(forward_m=0.5, max_duration_s=3))
        state = await _terminal(client)
        self.assertEqual(NavigationStatus.SUCCEEDED, state.status)
        self.assertGreater(state.replan_count, 0)
        self.assertTrue(state.path)
        self.assertEqual("goal_reached", state.stop_reason)
        clearances = [row for row in client.trace if row["event"] == "path_clearance"]
        self.assertTrue(clearances)
        self.assertTrue(all(row["minimum_clearance_m"] > 0 for row in clearances))

    async def test_relative_goal_finishes_requested_yaw(self) -> None:
        transport = _WorldCloudTransport()
        client = await _client(transport)
        await client.set_relative_goal(RelativeNavigationGoal(
            forward_m=0.2, yaw_degrees=20.0, max_duration_s=4,
        ))
        state = await _terminal(client, timeout=5)
        self.assertEqual(NavigationStatus.SUCCEEDED, state.status)
        self.assertAlmostEqual(20.0, transport._state.heading_degrees, delta=3.0)
        rotation_commands = [index for index, row in enumerate(client.trace)
                             if row["event"] == "command" and row.get("yaw_rps")]
        rotation_samples = [index for index, row in enumerate(client.trace)
                            if row["event"] == "motion_sample"
                            and row.get("motion_kind") == "rotation"]
        self.assertTrue(rotation_commands)
        self.assertEqual(len(rotation_commands), len(rotation_samples))
        self.assertTrue(all(command < sample for command, sample
                            in zip(rotation_commands, rotation_samples)))

    async def test_relative_goal_can_leave_terminal_yaw_unspecified(self) -> None:
        transport = _WorldCloudTransport()
        client = await _client(transport)
        await client.set_relative_goal(RelativeNavigationGoal(
            forward_m=0.2, yaw_degrees=20.0, require_final_yaw=False,
            max_duration_s=2,
        ))
        state = await _terminal(client)
        self.assertEqual(NavigationStatus.SUCCEEDED, state.status)
        self.assertFalse(any(
            row["event"] == "command" and row.get("yaw_rps")
            for row in client.trace
        ))

    async def test_configured_final_yaw_tolerance_avoids_sub_gait_pulses(self) -> None:
        transport = _WorldCloudTransport()
        transport._state.heading_degrees = 4.0
        client = await _client(transport, reach_tolerance_yaw_deg=5.0)
        await client.set_relative_goal(
            RelativeNavigationGoal(forward_m=0.2, yaw_degrees=-4.0, max_duration_s=2)
        )
        state = await _terminal(client)
        self.assertEqual(NavigationStatus.SUCCEEDED, state.status)
        self.assertFalse(any(
            row["event"] == "command" and row.get("yaw_rps")
            for row in client.trace
        ))

    async def test_static_obstacle_is_avoided(self) -> None:
        transport = _WorldCloudTransport(obstacles=((0.3, 0.0, 0.2),))
        client = await _client(transport, emergency_stop_m=0.12)
        await client.set_relative_goal(RelativeNavigationGoal(forward_m=0.7, max_duration_s=5))
        state = await _terminal(client, timeout=6)
        self.assertEqual(NavigationStatus.SUCCEEDED, state.status)
        self.assertGreater(abs(transport._state.position.y), 0.05)

    async def test_dynamic_obstacle_causes_replan_and_detour(self) -> None:
        transport = _WorldCloudTransport(dynamic=True)
        client = await _client(transport, emergency_stop_m=0.12)
        await client.set_relative_goal(RelativeNavigationGoal(forward_m=0.7, max_duration_s=5))
        state = await _terminal(client, timeout=6)
        self.assertEqual(NavigationStatus.SUCCEEDED, state.status)
        self.assertGreater(state.replan_count, 2)

    async def test_no_path_fails_closed(self) -> None:
        ring = tuple(
            (0.3 * math.cos(index * math.pi / 8),
             0.3 * math.sin(index * math.pi / 8), 0.2)
            for index in range(16)
        )
        client = await _client(_WorldCloudTransport(ring), max_no_path_replans=2)
        await client.set_relative_goal(RelativeNavigationGoal(forward_m=0.7, max_duration_s=3))
        state = await _terminal(client)
        self.assertEqual(NavigationStatus.FAILED, state.status)
        self.assertEqual("no_path", state.stop_reason)

    async def test_stuck_robot_reports_no_progress(self) -> None:
        client = await _client(_WorldCloudTransport(stuck=True), max_no_progress_segments=2)
        await client.set_relative_goal(RelativeNavigationGoal(forward_m=0.5, max_duration_s=3))
        state = await _terminal(client)
        self.assertEqual(NavigationStatus.NO_PROGRESS, state.status)

    async def test_stale_sensor_fails_closed(self) -> None:
        transport = _WorldCloudTransport()
        client = await _client(transport)
        handle = await client.set_relative_goal(RelativeNavigationGoal(forward_m=0.5))
        self.assertTrue(handle.accepted)
        transport.stale = True
        state = await _terminal(client)
        self.assertEqual(NavigationStatus.UNAVAILABLE, state.status)
        self.assertEqual("stale_pointcloud", state.stop_reason)

    async def test_cancel_active_goal(self) -> None:
        client = await _client(_WorldCloudTransport(stuck=True), max_no_progress_segments=100)
        handle = await client.set_relative_goal(RelativeNavigationGoal(forward_m=1.0))
        started = time.monotonic()
        state = await client.cancel(handle.goal_id)
        self.assertEqual(NavigationStatus.CANCELED, state.status)
        self.assertLess(time.monotonic() - started, 0.5)

    async def test_persistent_map_exposes_and_executes_absolute_goal(self) -> None:
        transport = _WorldCloudTransport()
        voxel_map = SparseVoxelMap(resolution_m=0.1, map_id="office")
        for corridor_x in (9.45 + index * 0.1 for index in range(12)):
            voxel_map.integrate(
                _cloud(((1.0, 0.0, 0.2),)),
                RobotPose(x_m=corridor_x, y_m=5.0, yaw_deg=90.0, frame_id="map"),
                carve_free_space=True,
            )
        client = await _client(
            transport, voxel_map=voxel_map, persistent_map=True,
            map_from_odom=NavigationPose(
                x_m=10.0, y_m=5.0, yaw_degrees=90.0, frame_id="map",
            ),
        )
        localization = await client.get_localization_state()
        self.assertTrue(localization.usable_for_persistent_memory)
        identity = voxel_map.identity()
        handle = await client.set_absolute_goal(AbsoluteNavigationGoal(
            pose=NavigationPose(
                x_m=10.0, y_m=5.4, yaw_degrees=90.0, frame_id="map",
            ),
            map_id=identity.map_id, map_version=identity.version,
            max_duration_s=4,
        ))
        state = await _terminal(client, timeout=5)
        self.assertTrue(handle.accepted)
        self.assertEqual(NavigationStatus.SUCCEEDED, state.status)
        self.assertAlmostEqual(0.4, transport._state.position.x, delta=0.15)

    async def test_persistent_map_version_survives_live_observation_merge(self) -> None:
        transport = _WorldCloudTransport()
        voxel_map = SparseVoxelMap(resolution_m=0.1, map_id="office")
        client = await _client(
            transport, voxel_map=voxel_map, persistent_map=True,
            map_from_odom=NavigationPose(frame_id="map"),
        )
        before = await client.get_localization_state()
        await client.get_costmap()
        after = await client.get_localization_state()
        self.assertEqual(before.map_identity.version, after.map_identity.version)  # type: ignore[union-attr]
        self.assertEqual(before.map_identity.map_id, after.map_identity.map_id)  # type: ignore[union-attr]

    async def test_absolute_goal_beyond_local_window_uses_known_free_global_route(self) -> None:
        transport = _WorldCloudTransport()
        voxel_map = SparseVoxelMap(resolution_m=0.1, map_id="long-hall")
        for offset in (-0.4, -0.3, -0.2, -0.1, 0.0, 0.1, 0.2, 0.3, 0.4):
            voxel_map.integrate(
                _cloud(((5.0, 0.0, 0.2),)),
                RobotPose(y_m=offset, frame_id="map"), carve_free_space=True,
            )
        client = await _client(
            transport, voxel_map=voxel_map, persistent_map=True,
            map_from_odom=NavigationPose(frame_id="map"),
        )
        identity = voxel_map.identity()
        handle = await client.set_absolute_goal(AbsoluteNavigationGoal(
            pose=NavigationPose(x_m=4.0, y_m=0.0, frame_id="map"),
            map_id=identity.map_id, map_version=identity.version,
            max_duration_s=20.0,
        ))
        state = await _terminal(client, timeout=5)
        self.assertTrue(handle.accepted)
        self.assertEqual(NavigationStatus.SUCCEEDED, state.status)
        self.assertAlmostEqual(4.0, transport._state.position.x, delta=0.2)
        self.assertTrue(any(row["event"] == "global_waypoint_reached" for row in client.trace))

    async def test_absolute_goal_in_unknown_space_fails_closed(self) -> None:
        transport = _WorldCloudTransport()
        voxel_map = SparseVoxelMap(resolution_m=0.1, map_id="partial")
        client = await _client(
            transport, voxel_map=voxel_map, persistent_map=True,
            map_from_odom=NavigationPose(frame_id="map"),
        )
        identity = voxel_map.identity()
        with self.assertRaisesRegex(Exception, "known-free"):
            await client.set_absolute_goal(AbsoluteNavigationGoal(
                pose=NavigationPose(x_m=4.0, y_m=0.0, frame_id="map"),
                map_id=identity.map_id, map_version=identity.version,
            ))

    async def test_local_no_path_triggers_bounded_global_replan(self) -> None:
        wall = tuple(
            (0.3 * math.cos(index * math.pi / 8),
             0.3 * math.sin(index * math.pi / 8), 0.2)
            for index in range(16)
        )
        transport = _WorldCloudTransport(wall)
        client = await _client(
            transport, persistent_map=True,
            map_from_odom=NavigationPose(frame_id="map"),
            max_no_path_replans=2,
        )
        replans = 0

        def replacement_route(_pose, _goal):
            nonlocal replans
            replans += 1
            return [(0.0, 0.5), (1.0, 0.5)]

        client._plan_global_route = replacement_route  # type: ignore[method-assign]
        initial = await client._sensors.get_snapshot()  # type: ignore[attr-defined]
        await client._start_goal(  # type: ignore[attr-defined]
            RelativeNavigationGoal(max_duration_s=2.0), initial,
            1.0, 0.0, 0.0, route_targets=[(1.0, 0.0)],
            absolute_goal_map=NavigationPose(x_m=1.0, frame_id="map"),
        )
        state = await _terminal(client)
        self.assertTrue(state.status.terminal)
        self.assertGreaterEqual(replans, 1)
        self.assertTrue(any(row["event"] == "global_replan" for row in client.trace))


class NativeGo2RuntimeWiringTests(unittest.TestCase):
    def test_native_backend_registers_navigation_skills(self) -> None:
        transport = _WorldCloudTransport()
        settings = Settings(
            robot_backend="unitree", perception_backend="mock",
            navigation_backend="native_go2", unitree_enable_motion=True,
            unitree_dry_run=False, memory_db_path=":memory:",
        )
        runtime = AgentRuntime.create(
            settings=settings, robot=UnitreeRobot(transport, settings)
        )
        self.assertIsInstance(runtime.context.navigation, NativeGo2NavigationClient)
        self.assertIsNotNone(runtime.context.skills.get("nav_go_relative"))
        self.assertIsNotNone(runtime.context.skills.get("nav_cancel"))
        self.assertIsNotNone(runtime.context.skills.get("nav_explore"))
        self.assertIsNotNone(runtime.context.skills.get("nav_patrol"))
        self.assertIsNotNone(runtime.context.skills.get("nav_go_to_bbox"))
        self.assertIsNotNone(runtime.context.skills.get("nav_relocalize"))
        self.assertIsNotNone(runtime.context.skills.get("nav_go_terrain_relative"))
        self.assertIsNotNone(runtime.context.tools.get("native_map_get_state"))
        self.assertIsNotNone(runtime.context.tools.get("native_terrain_plan"))
        runtime.close()

    def test_non_mcf_mode_hides_terrain_motion_but_keeps_read_only_planner(self) -> None:
        transport = _WorldCloudTransport()
        settings = Settings(
            robot_backend="unitree", perception_backend="mock",
            navigation_backend="native_go2", unitree_motion_mode="ai-w",
            memory_db_path=":memory:",
        )
        runtime = AgentRuntime.create(
            settings=settings, robot=UnitreeRobot(transport, settings)
        )
        self.assertIsNone(runtime.context.skills.get("nav_go_terrain_relative"))
        self.assertIsNotNone(runtime.context.tools.get("native_terrain_plan"))
        runtime.close()
