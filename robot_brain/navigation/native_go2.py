"""Robot-brain owned 2-D Go2 navigation; no DIMOS, ROS, or Nav2 runtime."""
from __future__ import annotations

import asyncio
from datetime import datetime, timezone
import math
import time
from uuid import uuid4

from robot_brain.actuation.unitree import UnitreeRobot
from robot_brain.core.robot_self_state import RobotPose
from robot_brain.navigation.base import (
    AbsoluteNavigationGoal,
    LocalizationState,
    LocalizationStatus,
    MapIdentity,
    NavigationClient,
    NavigationGoalHandle,
    NavigationPose,
    NavigationState,
    NavigationStatus,
    NavigationUnavailableError,
    RelativeNavigationGoal,
)
from robot_brain.navigation.grid import (
    OccupancyGrid2D, costmap_from_pointcloud, with_obstacle_distance_costs,
)
from robot_brain.navigation.diagnostics import NavigationTraceWriter
from robot_brain.navigation.map_store import SparseVoxelMap
from robot_brain.navigation.relocalization import (
    RelocalizationResult,
    relocalize_global,
    relocalize_with_initial,
)
from robot_brain.navigation.replay import NavigationReplayWriter
from robot_brain.navigation.planner import (
    astar_path,
    find_nearest_safe_goal,
    path_minimum_clearance_m,
)
from robot_brain.navigation.pose_graph import OnlinePoseGraphTracker, PlanarPose
from robot_brain.navigation.motion_safety import (
    LinearVelocityRampLimiter,
    NavigationMotionSafetySignal,
)
from robot_brain.navigation.sensors import NavigationSensorProvider, NavigationSensorSnapshot
from robot_brain.navigation.terrain3d import (
    MultiLevelTerrainPlanner,
    SurfacePath,
    TerrainPlannerConfig,
)


class NativeGo2NavigationClient(NavigationClient):
    """Plan and follow short local paths from fresh Go2 odometry and LiDAR."""

    def __init__(
        self,
        robot: UnitreeRobot,
        sensors: NavigationSensorProvider,
        *,
        linear_speed_mps: float = 0.15,
        yaw_speed_rps: float = 0.30,
        segment_duration_s: float = 0.25,
        map_size_m: float = 7.0,
        resolution_m: float = 0.10,
        robot_radius_m: float = 0.30,
        emergency_stop_m: float = 0.25,
        reach_tolerance_m: float = 0.10,
        reach_tolerance_yaw_deg: float = 5.0,
        min_progress_m: float = 0.02,
        max_no_progress_segments: int = 4,
        max_no_path_replans: int = 3,
        min_replan_interval_s: float = 0.0,
        settle_s: float = 0.20,
        trace_writer: NavigationTraceWriter | None = None,
        voxel_map: SparseVoxelMap | None = None,
        persistent_map: bool = False,
        map_from_odom: NavigationPose | None = None,
        replay_writer: NavigationReplayWriter | None = None,
        pose_graph_tracker: OnlinePoseGraphTracker | None = None,
        max_acceleration_mps2: float = 1.0,
        safety_signal_max_age_s: float = 0.5,
    ) -> None:
        positive = {
            "linear_speed_mps": linear_speed_mps,
            "yaw_speed_rps": yaw_speed_rps,
            "segment_duration_s": segment_duration_s,
            "map_size_m": map_size_m,
            "resolution_m": resolution_m,
            "reach_tolerance_m": reach_tolerance_m,
            "reach_tolerance_yaw_deg": reach_tolerance_yaw_deg,
            "max_acceleration_mps2": max_acceleration_mps2,
            "safety_signal_max_age_s": safety_signal_max_age_s,
        }
        invalid = [name for name, value in positive.items()
                   if not math.isfinite(value) or value <= 0]
        nonnegative = {
            "robot_radius_m": robot_radius_m,
            "emergency_stop_m": emergency_stop_m,
            "min_progress_m": min_progress_m,
            "min_replan_interval_s": min_replan_interval_s,
            "settle_s": settle_s,
        }
        invalid += [name for name, value in nonnegative.items()
                    if not math.isfinite(value) or value < 0]
        if invalid:
            raise ValueError(f"invalid native navigation safety parameter(s): {', '.join(invalid)}")
        if map_size_m <= 2 * robot_radius_m + 2 * resolution_m:
            raise ValueError("native navigation map is too small for the robot safety envelope")
        if max_no_progress_segments <= 0 or max_no_path_replans <= 0:
            raise ValueError("native navigation retry limits must be positive")
        self._robot = robot
        self._sensors = sensors
        self._linear_speed = linear_speed_mps
        self._yaw_speed = yaw_speed_rps
        self._segment_duration = segment_duration_s
        self._map_size = map_size_m
        self._resolution = resolution_m
        self._robot_radius = robot_radius_m
        self._emergency_stop = emergency_stop_m
        self._reach_tolerance = reach_tolerance_m
        self._reach_tolerance_yaw = reach_tolerance_yaw_deg
        self._min_progress = min_progress_m
        self._max_no_progress = max_no_progress_segments
        self._max_no_path = max_no_path_replans
        self._min_replan_interval = max(0.0, min_replan_interval_s)
        self._settle_s = settle_s
        self._trace_writer = trace_writer
        self._voxel_map = voxel_map or SparseVoxelMap(resolution_m=resolution_m)
        self._persistent_map = persistent_map
        self._map_from_odom = map_from_odom
        self._replay_writer = replay_writer
        self._pose_graph_tracker = pose_graph_tracker
        self._pgo_last_cloud_monotonic: float | None = None
        self._velocity_limiter = LinearVelocityRampLimiter(max_acceleration_mps2)
        self._safety_signal_max_age = safety_signal_max_age_s
        self._motion_safety_signal: NavigationMotionSafetySignal | None = None
        self._task: asyncio.Task[None] | None = None
        self._cancel_event = asyncio.Event()
        self._state = NavigationState(
            provider="native_go2", ready=False, status=NavigationStatus.UNAVAILABLE,
            message="native Go2 navigation has not been queried",
        )
        identity = self._voxel_map.identity()
        self._map_identity = MapIdentity(
            map_id=identity.map_id if persistent_map else f"native-go2-session-{uuid4().hex}",
            version=identity.version if persistent_map else None,
            frame_id="map" if persistent_map or pose_graph_tracker is not None else "odom",
            persistent=persistent_map,
        )
        self._trace: list[dict[str, object]] = []
        self._last_plan_monotonic = 0.0
        self._last_viewer_cloud_monotonic = 0.0

    @property
    def trace(self) -> tuple[dict[str, object], ...]:
        return tuple(dict(event) for event in self._trace)

    @property
    def supports_absolute_goals(self) -> bool:
        return self._persistent_map

    @property
    def voxel_map(self) -> SparseVoxelMap:
        return self._voxel_map

    async def get_sensor_snapshot(self) -> NavigationSensorSnapshot:
        """Expose the shared read-only sensor snapshot for local observability."""
        return await self._sensors.get_snapshot()

    def integrate_viewer_snapshot(self, snapshot: NavigationSensorSnapshot) -> bool:
        """Fuse each fresh LiDAR frame at most once for live map visualization."""
        cloud = snapshot.pointcloud
        if not snapshot.ready or snapshot.pose is None or cloud is None:
            return False
        if cloud.received_monotonic <= self._last_viewer_cloud_monotonic:
            return False
        self._last_viewer_cloud_monotonic = cloud.received_monotonic
        self._voxel_map.integrate(
            cloud,
            self._mapping_pose(snapshot),
            carve_free_space=True,
            carve_misses=3,
        )
        return True

    def set_motion_safety_signal(self, signal: NavigationMotionSafetySignal | None) -> None:
        """Update or detach an external local-planner safety source."""
        self._motion_safety_signal = signal

    def close(self) -> None:
        if self._trace_writer is not None:
            self._trace_writer.close()

    async def aclose(self) -> None:
        if self._state.status == NavigationStatus.ACTIVE:
            await self.cancel(self._state.goal_id)
        self.close()

    async def get_costmap(self):
        """Return a fresh map/odom-aligned grid for exploration and patrol."""
        snapshot = await self._sensors.get_snapshot()
        if not snapshot.ready or snapshot.pose is None or snapshot.pointcloud is None:
            raise NavigationUnavailableError(snapshot.reason or "navigation sensors unavailable")
        pose = self._mapping_pose(snapshot)
        self._voxel_map.integrate(
            snapshot.pointcloud, pose, carve_free_space=True, carve_misses=3
        )
        return self._voxel_map.occupancy_grid(
            center_x_m=pose.x_m, center_y_m=pose.y_m, size_m=self._map_size,
            robot_radius_m=self._robot_radius,
            frame_id=pose.frame_id,
        )

    async def get_viewer_costmap(self):
        """Build a read-only grid without reintegrating the latest point cloud."""
        state = await self.get_state()
        if state.pose is None:
            raise NavigationUnavailableError("navigation pose is unavailable")
        return self._voxel_map.occupancy_grid(
            center_x_m=state.pose.x_m,
            center_y_m=state.pose.y_m,
            size_m=self._map_size,
            robot_radius_m=self._robot_radius,
            frame_id=state.pose.frame_id,
        )

    def save_map(self, path: str):
        identity = self._voxel_map.save(path)
        self._event(
            "map_saved", map_id=identity.map_id, map_version=identity.version,
            map_revision=identity.revision, voxel_count=identity.voxel_count, path=path,
        )
        return identity

    async def plan_terrain_relative(
        self, *, forward_m: float, left_m: float, up_m: float = 0.0,
        navigation_boundary_xy: tuple[tuple[float, float], ...] = (),
        added_obstacles_xyz: tuple[tuple[float, float, float], ...] = (),
        added_obstacle_radius_m: float = 0.30,
    ) -> SurfacePath:
        """Plan a multi-level surface path without issuing motion commands."""
        if not all(math.isfinite(value) for value in (forward_m, left_m, up_m)):
            raise ValueError("terrain goal must be finite")
        if math.hypot(forward_m, left_m) > 3.0 or abs(up_m) > 1.5:
            raise ValueError("terrain goal exceeds bounded planning envelope")
        snapshot = await self._sensors.get_snapshot()
        if not snapshot.ready or snapshot.pose is None or snapshot.pointcloud is None:
            raise NavigationUnavailableError(snapshot.reason or "navigation sensors unavailable")
        pose = self._mapping_pose(snapshot)
        self._voxel_map.integrate(snapshot.pointcloud, pose, carve_free_space=True)
        planner = MultiLevelTerrainPlanner(TerrainPlannerConfig(
            voxel_size_m=self._voxel_map.resolution_m,
            robot_height_m=0.30,
            surface_closing_radius_m=0.30,
            wall_clearance_m=0.10,
            wall_buffer_m=0.75,
            wall_buffer_weight=100.0,
            step_threshold_m=0.16,
            step_penalty_weight=4.0,
            goal_tolerance_m=0.35,
        ))
        goal_x, goal_y = _body_to_world(
            pose.x_m, pose.y_m, pose.yaw_deg, forward_m, left_m
        )
        start = (pose.x_m, pose.y_m, pose.z_m - 0.30)
        goal = (goal_x, goal_y, start[2] + up_m)
        local_radius = max(1.0, math.hypot(forward_m, left_m) + 1.0)
        local_points = self._voxel_map.points_in_cylinder(
            center_x_m=(start[0]+goal[0])/2.0,
            center_y_m=(start[1]+goal[1])/2.0,
            radius_m=local_radius,
            z_min_m=min(start[2], goal[2])-1.0,
            z_max_m=max(start[2], goal[2])+2.0,
            max_points=120_000,
        )
        planner.update_global_map(local_points)
        if navigation_boundary_xy:
            planner.set_navigation_boundary(navigation_boundary_xy)
        if added_obstacles_xyz:
            planner.set_added_obstacles(
                added_obstacles_xyz, radius_m=added_obstacle_radius_m,
            )
        path = planner.plan(start, goal)
        if path is None:
            self._event("terrain_plan_failed", stop_reason=planner.state.stop_reason)
            raise NavigationUnavailableError(
                planner.state.stop_reason or "no traversable 3D surface path"
            )
        self._event(
            "terrain_plan", path_points=len(path.nodes), length_m=path.length_m,
            elevation_gain_m=path.elevation_gain_m,
            minimum_clearance_m=path.minimum_clearance_m,
        )
        return path

    async def plan_terrain_frontier(
        self, *, exploration_range_m: float = 3.0,
        visited_xy: tuple[tuple[float, float], ...] = (),
        navigation_boundary_xy: tuple[tuple[float, float], ...] = (),
        added_obstacles_xyz: tuple[tuple[float, float, float], ...] = (),
        added_obstacle_radius_m: float = 0.30,
    ) -> SurfacePath:
        """Plan one TARE-style MLS frontier step without issuing motion."""
        if not math.isfinite(exploration_range_m) or not 0 < exploration_range_m <= 10:
            raise ValueError("terrain exploration range must be in (0, 10]")
        snapshot = await self._sensors.get_snapshot()
        if not snapshot.ready or snapshot.pose is None or snapshot.pointcloud is None:
            raise NavigationUnavailableError(snapshot.reason or "navigation sensors unavailable")
        pose = self._mapping_pose(snapshot)
        self._voxel_map.integrate(snapshot.pointcloud, pose, carve_free_space=True)
        planner = MultiLevelTerrainPlanner(TerrainPlannerConfig(
            voxel_size_m=self._voxel_map.resolution_m, robot_height_m=0.30,
            surface_closing_radius_m=0.30, wall_clearance_m=0.10,
            wall_buffer_m=0.75, wall_buffer_weight=100.0,
            step_threshold_m=0.16, step_penalty_weight=4.0,
            goal_tolerance_m=0.35,
        ))
        points = self._voxel_map.points_in_cylinder(
            center_x_m=pose.x_m, center_y_m=pose.y_m,
            radius_m=exploration_range_m+1.0,
            z_min_m=pose.z_m-1.3, z_max_m=pose.z_m+2.0,
            max_points=120_000,
        )
        planner.update_global_map(points)
        if navigation_boundary_xy:
            planner.set_navigation_boundary(navigation_boundary_xy)
        if added_obstacles_xyz:
            planner.set_added_obstacles(added_obstacles_xyz,
                                        radius_m=added_obstacle_radius_m)
        start = (pose.x_m, pose.y_m, pose.z_m-0.30)
        goals = planner.frontier_goals(
            start, exploration_range_m=exploration_range_m,
            visited_xy=visited_xy, max_goals=20,
        )
        for goal in goals:
            path = planner.plan(start, (goal.node.x_m, goal.node.y_m, goal.node.z_m))
            if path is not None and len(path.nodes) >= 2:
                self._event(
                    "terrain_frontier_plan", information_gain=goal.information_gain,
                    score=goal.score, distance_m=goal.distance_m,
                    path_points=len(path.nodes),
                )
                return path
        self._event("terrain_frontier_unavailable", stop_reason="no_terrain_frontier")
        raise NavigationUnavailableError("no safe reachable terrain frontier")

    async def get_state(self) -> NavigationState:
        if self._task is None or self._task.done():
            snapshot = await self._sensors.get_snapshot()
            if self._state.goal_id is None:
                self._state = self._state.model_copy(update={
                    "ready": snapshot.ready,
                    "status": NavigationStatus.IDLE if snapshot.ready else NavigationStatus.UNAVAILABLE,
                    "pose": _pose(snapshot),
                    "message": "native Go2 navigation ready" if snapshot.ready else snapshot.reason or "sensors unavailable",
                    "error_code": None if snapshot.ready else snapshot.reason,
                    "stop_reason": None if snapshot.ready else snapshot.reason,
                })
        return self._state.model_copy(deep=True)

    async def set_relative_goal(self, goal: RelativeNavigationGoal) -> NavigationGoalHandle:
        if self._task is not None and not self._task.done():
            return NavigationGoalHandle(
                goal_id=self._state.goal_id or "", accepted=False,
                message="another navigation goal is active",
            )
        initial = await self._sensors.get_snapshot()
        if not initial.ready or initial.pose is None:
            raise NavigationUnavailableError(initial.reason or "navigation sensors unavailable")
        distance = math.hypot(goal.forward_m, goal.left_m)
        if distance > self._map_size / 2.0 - self._robot_radius:
            raise NavigationUnavailableError("relative goal exceeds local costmap")
        target_x, target_y = _body_to_world(
            initial.pose.x_m, initial.pose.y_m, initial.pose.yaw_deg,
            goal.forward_m, goal.left_m,
        )
        target_yaw = _normalize(initial.pose.yaw_deg + goal.yaw_degrees)
        return await self._start_goal(goal, initial, target_x, target_y, target_yaw)

    async def set_absolute_goal(self, goal: AbsoluteNavigationGoal) -> NavigationGoalHandle:
        if not self._persistent_map or self._map_from_odom is None:
            raise NavigationUnavailableError("native persistent map localization is not ready")
        current_identity = self._voxel_map.identity()
        if goal.map_id != current_identity.map_id or (
            goal.map_version is not None and goal.map_version != current_identity.version
        ):
            raise NavigationUnavailableError("absolute goal belongs to a different map version")
        if goal.pose.frame_id != "map":
            raise NavigationUnavailableError("absolute goal must use map frame")
        initial = await self._sensors.get_snapshot()
        if not initial.ready or initial.pose is None:
            raise NavigationUnavailableError(initial.reason or "navigation sensors unavailable")
        target_odom = _map_to_odom(goal.pose, self._map_from_odom)  # type: ignore[arg-type]
        route_odom = self._plan_global_route(initial.pose, goal.pose)
        relative = RelativeNavigationGoal(
            forward_m=0.0, left_m=0.0,
            yaw_degrees=0.0, max_duration_s=goal.max_duration_s,
        )
        return await self._start_goal(
            relative, initial, target_odom.x_m, target_odom.y_m, target_odom.yaw_degrees,
            route_targets=route_odom, absolute_goal_map=goal.pose,
        )

    def _plan_global_route(
        self, current_odom: RobotPose, goal_map: NavigationPose,
    ) -> list[tuple[float, float]]:
        if self._map_from_odom is None:
            raise NavigationUnavailableError("persistent map localization is unavailable")
        current_map = _robot_pose_in_map(current_odom, self._map_from_odom)
        distance = math.hypot(goal_map.x_m-current_map.x_m, goal_map.y_m-current_map.y_m)
        if distance > 30.0:
            raise NavigationUnavailableError("absolute goal exceeds bounded global range")
        grid = self._voxel_map.occupancy_grid(
            center_x_m=(current_map.x_m+goal_map.x_m)/2.0,
            center_y_m=(current_map.y_m+goal_map.y_m)/2.0,
            size_m=max(self._map_size, distance + 4.0),
            robot_radius_m=self._robot_radius, frame_id="map",
        )
        grid = with_obstacle_distance_costs(grid, maximum_distance_m=1.0)
        safe_grid = _known_free_only_grid(
            grid, (current_map.x_m, current_map.y_m),
            clearance_m=self._robot_radius,
        )
        global_path = astar_path(
            safe_grid, (current_map.x_m, current_map.y_m),
            (goal_map.x_m, goal_map.y_m),
        )
        if global_path is None:
            raise NavigationUnavailableError("no known-free global path to absolute goal")
        global_path = _resample_polyline(global_path, maximum_segment_m=1.5)
        route = [
            (mapped.x_m, mapped.y_m)
            for x, y in global_path[1:]
            for mapped in [_map_to_odom(
                NavigationPose(x_m=x, y_m=y, frame_id="map"), self._map_from_odom
            )]
        ]
        target = _map_to_odom(goal_map, self._map_from_odom)
        if not route or math.hypot(route[-1][0]-target.x_m, route[-1][1]-target.y_m) > 0.05:
            route.append((target.x_m, target.y_m))
        return route

    async def _start_goal(
        self, goal: RelativeNavigationGoal, initial: NavigationSensorSnapshot,
        target_x: float, target_y: float, target_yaw: float,
        route_targets: list[tuple[float, float]] | None = None,
        absolute_goal_map: NavigationPose | None = None,
    ) -> NavigationGoalHandle:
        goal_id = f"native-go2-{uuid4().hex}"
        self._cancel_event = asyncio.Event()
        self._trace = []
        self._state = NavigationState(
            provider="native_go2", ready=True, status=NavigationStatus.ACTIVE,
            goal_id=goal_id, pose=_pose(initial), progress=0.0,
            message="native Go2 goal accepted",
        )
        self._event(
            "goal_accepted", target_x_m=target_x, target_y_m=target_y,
            target_yaw_degrees=target_yaw,
        )
        self._task = asyncio.create_task(
            self._execute(
                goal_id, goal, initial, target_x, target_y, target_yaw,
                route_targets, absolute_goal_map,
            )
        )
        return NavigationGoalHandle(goal_id=goal_id, message=self._state.message)

    async def cancel(self, goal_id: str | None = None) -> NavigationState:
        if self._state.status != NavigationStatus.ACTIVE:
            return self._state.model_copy(deep=True)
        if goal_id is not None and goal_id != self._state.goal_id:
            return self._state.model_copy(deep=True)
        self._cancel_event.set()
        await self._robot.stop("native navigation canceled")
        task = self._task
        if task is not None and not task.done():
            # Physical stop has completed.  Do not wait for the execution loop
            # to finish an odometry-settle cycle and issue a duplicate stop;
            # terminate it now so cancellation latency reflects the safety
            # action rather than background bookkeeping.
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass
        if self._state.status == NavigationStatus.ACTIVE:
            self._finish(NavigationStatus.CANCELED, "navigation canceled", "canceled")
        return self._state.model_copy(deep=True)

    async def get_localization_state(self) -> LocalizationState:
        snapshot = await self._sensors.get_snapshot()
        localized = (
            snapshot.pose_ready and self._persistent_map
            and self._map_from_odom is not None
        )
        mapped = self._mapping_pose(snapshot) if snapshot.ready else None
        localized_pose = (
            NavigationPose(x_m=mapped.x_m, y_m=mapped.y_m,
                           yaw_degrees=mapped.yaw_deg, frame_id=mapped.frame_id)
            if mapped is not None else _pose(snapshot)
        )
        if self._persistent_map:
            identity = self._voxel_map.identity()
            self._map_identity = MapIdentity(
                map_id=identity.map_id, version=identity.version,
                frame_id="map", persistent=True,
            )
        return LocalizationState(
            status=(
                LocalizationStatus.LOCALIZED if localized
                else LocalizationStatus.LOST if self._persistent_map
                else LocalizationStatus.LOCAL if snapshot.pose_ready
                else LocalizationStatus.STALE
            ),
            map_identity=self._map_identity,
            pose=localized_pose,
            confidence=1.0 if snapshot.pose_ready else 0.0,
            message=(
                "native persistent map localization"
                if localized
                else "session-local Go2 odometry"
                if snapshot.pose_ready else snapshot.reason or "odometry unavailable"
            ),
        )

    async def relocalize(
        self,
        initial_map_pose: NavigationPose | None = None,
        *,
        allow_global_fallback: bool = False,
    ) -> RelocalizationResult:
        """Establish map->odom from the current body cloud and saved map."""
        if not self._persistent_map:
            raise NavigationUnavailableError("no persistent native map loaded")
        snapshot = await self._sensors.get_snapshot()
        if not snapshot.ready or snapshot.pose is None or snapshot.pointcloud is None:
            raise NavigationUnavailableError(snapshot.reason or "navigation sensors unavailable")
        if initial_map_pose is not None:
            result = relocalize_with_initial(
                self._voxel_map,
                snapshot.pointcloud,
                RobotPose(
                    x_m=initial_map_pose.x_m, y_m=initial_map_pose.y_m,
                    yaw_deg=initial_map_pose.yaw_degrees, frame_id="map",
                ),
            )
            if not result.accepted and allow_global_fallback:
                result = relocalize_global(self._voxel_map, snapshot.pointcloud)
        elif allow_global_fallback:
            result = relocalize_global(self._voxel_map, snapshot.pointcloud)
        else:
            raise NavigationUnavailableError(
                "initial map pose is required unless global fallback is enabled"
            )
        if result.accepted and result.pose is not None:
            self._map_from_odom = _map_from_odom_transform(result.pose, snapshot.pose)
            self._event(
                "relocalized", mode=result.mode, fitness=result.fitness,
                rmse_m=result.rmse_m,
            )
        else:
            self._event("relocalization_failed", mode=result.mode, reason=result.reason)
        return result

    async def _execute(
        self, goal_id: str, goal: RelativeNavigationGoal,
        initial: NavigationSensorSnapshot, target_x: float, target_y: float,
        target_yaw: float, route_targets: list[tuple[float, float]] | None = None,
        absolute_goal_map: NavigationPose | None = None,
    ) -> None:
        started = time.monotonic()
        initial_distance = math.hypot(target_x - initial.pose.x_m, target_y - initial.pose.y_m)  # type: ignore[union-attr]
        previous = initial
        no_progress = 0
        no_path = 0
        replans = 0
        route = list(route_targets or [(target_x, target_y)])
        route_index = 0
        try:
            while True:
                snapshot = await self._sensors.get_snapshot()
                failure = self._guard(snapshot, goal, started)
                if failure is not None:
                    await self._robot.stop(failure[1])
                    self._finish(*failure, snapshot=snapshot)
                    return
                assert snapshot.pose is not None and snapshot.pointcloud is not None
                if self._replay_writer is not None:
                    self._replay_writer.record(snapshot)
                map_pose = self._mapping_pose(snapshot)
                self._voxel_map.integrate(snapshot.pointcloud, map_pose)
                active_x, active_y = route[route_index]
                dx_world = active_x - snapshot.pose.x_m
                dy_world = active_y - snapshot.pose.y_m
                remaining = math.hypot(dx_world, dy_world)
                # A grid planner cannot reliably distinguish sub-cell residuals;
                # accept the configured tolerance or 1.5 cells, whichever is safer.
                if remaining <= max(self._reach_tolerance, self._resolution * 1.5):
                    if route_index < len(route) - 1:
                        route_index += 1
                        self._event("global_waypoint_reached", route_index=route_index,
                                    route_points=len(route))
                        continue
                    if goal.require_final_yaw:
                        rotated = await self._rotate_to(target_yaw, goal, started, snapshot)
                        if rotated is None:
                            return
                    await self._robot.stop("native navigation goal reached")
                    self._finish(
                        NavigationStatus.SUCCEEDED, "relative goal reached", None,
                        snapshot=snapshot, progress=1.0,
                    )
                    return
                goal_body = _world_delta_to_body(dx_world, dy_world, snapshot.pose.yaw_deg)
                until_plan = (
                    self._last_plan_monotonic + self._min_replan_interval
                    - time.monotonic()
                )
                if until_plan > 0:
                    await self._robot.stop("waiting for bounded replan interval")
                    await asyncio.sleep(until_plan)
                    continue
                grid = costmap_from_pointcloud(
                    snapshot.pointcloud, size_m=self._map_size,
                    resolution_m=self._resolution, robot_radius_m=self._robot_radius,
                )
                # Match DIMOS' ReplanningAStar pipeline: do not fail the whole
                # plan merely because the exact requested cell is occupied by
                # a noisy return or inflated obstacle edge.  Select the nearest
                # safe local endpoint, then continuously replan toward the
                # original world-frame target on the next sensor frame.
                safe_goal_body = find_nearest_safe_goal(
                    grid, goal_body, search_radius_m=0.50,
                    clearance_m=self._resolution,
                )
                path = (
                    astar_path(grid, (0.0, 0.0), safe_goal_body)
                    if safe_goal_body is not None else None
                )
                self._last_plan_monotonic = time.monotonic()
                replans += 1
                if safe_goal_body is not None and math.dist(safe_goal_body, goal_body) > 1e-6:
                    self._event(
                        "safe_goal_adjusted", replan_count=replans,
                        requested_xy=list(goal_body), selected_xy=list(safe_goal_body),
                        adjustment_m=math.dist(safe_goal_body, goal_body),
                    )
                if path is None or len(path) < 2:
                    no_path += 1
                    self._event("plan_failed", replan_count=replans, occupied_cells=len(grid.occupied))
                    await self._robot.stop("no safe local path")
                    if absolute_goal_map is not None:
                        try:
                            route = self._plan_global_route(snapshot.pose, absolute_goal_map)
                            route_index = 0
                            self._event(
                                "global_replan", replan_count=replans,
                                route_points=len(route),
                            )
                        except NavigationUnavailableError as exc:
                            self._event(
                                "global_replan_failed", replan_count=replans,
                                reason=str(exc),
                            )
                    if no_path >= self._max_no_path:
                        self._finish(
                            NavigationStatus.FAILED, "no safe local path", "no_path",
                            snapshot=snapshot, replan_count=replans,
                        )
                        return
                    await asyncio.sleep(min(0.25, self._segment_duration))
                    continue
                clearance = path_minimum_clearance_m(grid, path)
                self._event(
                    "path_clearance", replan_count=replans,
                    minimum_clearance_m=clearance,
                    clearance_reference="inflated_costmap",
                )
                if clearance is None or clearance <= 0:
                    no_path += 1
                    self._event(
                        "path_clearance_failed", replan_count=replans,
                        stop_reason="path_clearance_failed",
                    )
                    await self._robot.stop("planned path failed corridor clearance")
                    if no_path >= self._max_no_path:
                        self._finish(
                            NavigationStatus.FAILED,
                            "planned path failed corridor clearance",
                            "path_clearance_failed", snapshot=snapshot,
                            replan_count=replans,
                        )
                        return
                    continue
                no_path = 0
                world_path = [
                    _path_pose(snapshot, x, y) for x, y in path
                ]
                waypoint = path[1]
                waypoint_distance = math.hypot(*waypoint)
                if waypoint_distance <= 1e-6:
                    waypoint = path[-1]
                    waypoint_distance = math.hypot(*waypoint)
                ux, uy = waypoint[0] / waypoint_distance, waypoint[1] / waypoint_distance
                if _emergency_corridor_blocked(
                    snapshot, ux, uy, self._emergency_stop, self._robot_radius * 0.7
                ):
                    self._event("emergency_stop", replan_count=replans)
                    self._velocity_limiter.reset()
                    await self._robot.stop("obstacle entered emergency corridor")
                    await asyncio.sleep(min(0.20, self._segment_duration))
                    continue
                duration = min(
                    self._segment_duration,
                    max(0.05, waypoint_distance / max(self._linear_speed, 1e-3)),
                )
                safety_failure, speed_scale = self._motion_safety()
                if safety_failure is not None:
                    self._velocity_limiter.reset()
                    await self._robot.stop(safety_failure[1])
                    self._finish(*safety_failure, snapshot=snapshot,
                                 replan_count=replans)
                    return
                command_vx, command_vy = self._velocity_limiter.step(
                    self._linear_speed*ux, self._linear_speed*uy, duration,
                    speed_scale=speed_scale,
                )
                progress = 1.0 if initial_distance <= 1e-6 else min(
                    0.99, max(0.0, (route_index + 1.0 - min(1.0, remaining / 1.5)) / len(route))
                )
                self._state = self._state.model_copy(update={
                    "pose": _pose(snapshot), "path": world_path,
                    "replan_count": replans, "progress": progress,
                    "message": "following replanned local path",
                    "updated_at": datetime.now(timezone.utc),
                })
                self._event("plan", replan_count=replans, path_points=len(path), remaining_m=remaining)
                self._event(
                    "plan_geometry", replan_count=replans,
                    path_xy=[[item.x_m, item.y_m] for item in world_path],
                )
                self._event(
                    "command", vx_mps=command_vx, vy_mps=command_vy,
                    yaw_rps=0.0, duration_s=duration, speed_scale=speed_scale,
                )
                await self._robot.drive(
                    vx=command_vx, vy=command_vy,
                    duration=duration,
                )
                after = await self._settled_snapshot(duration)
                if after.pose is not None:
                    self._event("motion_sample", x_m=after.pose.x_m, y_m=after.pose.y_m,
                                yaw_degrees=after.pose.yaw_deg)
                moved = _distance(previous, after)
                if moved < min(self._min_progress, self._linear_speed * duration * 0.35):
                    no_progress += 1
                else:
                    no_progress = 0
                if no_progress >= self._max_no_progress:
                    await self._robot.stop("native navigation made no progress")
                    self._finish(
                        NavigationStatus.NO_PROGRESS, "native navigation made no progress",
                        "no_progress", snapshot=after, replan_count=replans,
                    )
                    return
                previous = after
        except asyncio.CancelledError:
            self._finish(NavigationStatus.CANCELED, "navigation canceled", "canceled")
            raise
        except Exception as exc:
            await self._robot.stop("native navigation provider error")
            self._finish(NavigationStatus.FAILED, f"native navigation failed: {exc}", "provider_error")

    def _mapping_pose(self, snapshot: NavigationSensorSnapshot) -> RobotPose:
        """Return global mapping pose; local motion continues to use raw odom."""
        assert snapshot.pose is not None
        pose = snapshot.pose
        cloud = snapshot.pointcloud
        if self._pose_graph_tracker is not None and cloud is not None:
            timestamp = cloud.received_monotonic
            if self._pgo_last_cloud_monotonic is None or timestamp > self._pgo_last_cloud_monotonic:
                update = self._pose_graph_tracker.process(
                    timestamp, PlanarPose(pose.x_m, pose.y_m, pose.yaw_deg),
                    cloud.points_xyz,
                )
                self._pgo_last_cloud_monotonic = timestamp
                self._event(
                    "pose_graph_update", reason=update.reason,
                    keyframe_added=update.keyframe_added,
                    keyframe_index=update.keyframe_index,
                    loop_added=update.loop_added,
                    loop_source_index=update.loop_source_index,
                    corrected_x_m=update.corrected_pose.x_m,
                    corrected_y_m=update.corrected_pose.y_m,
                    corrected_yaw_degrees=update.corrected_pose.yaw_degrees,
                    graph_accepted=(update.graph_result.accepted
                                    if update.graph_result is not None else None),
                )
            corrected = self._pose_graph_tracker.corrected_pose(
                PlanarPose(pose.x_m, pose.y_m, pose.yaw_deg)
            )
            pose = RobotPose(
                x_m=corrected.x_m, y_m=corrected.y_m, z_m=pose.z_m,
                yaw_deg=corrected.yaw_degrees, frame_id="map",
                timestamp=pose.timestamp,
            )
        if self._persistent_map and self._map_from_odom is not None:
            pose = _robot_pose_in_map(pose, self._map_from_odom)
        return pose

    def _motion_safety(self):
        signal = self._motion_safety_signal
        if signal is None:
            return None, 1.0
        age = time.monotonic()-signal.observed_monotonic
        if age < 0 or age > self._safety_signal_max_age:
            return (NavigationStatus.UNAVAILABLE, "external safety signal is stale",
                    "stale_safety_signal"), 0.0
        if signal.stop_requested or signal.speed_scale <= 0:
            return (NavigationStatus.FAILED, signal.reason,
                    "external_safety_stop"), 0.0
        return None, signal.speed_scale

    async def _rotate_to(
        self, target_yaw: float, goal: RelativeNavigationGoal, started: float,
        snapshot: NavigationSensorSnapshot,
    ) -> NavigationSensorSnapshot | None:
        no_progress = 0
        previous_error = abs(_angle_error(target_yaw, snapshot.pose.yaw_deg))  # type: ignore[union-attr]
        while previous_error > self._reach_tolerance_yaw:
            failure = self._guard(snapshot, goal, started)
            if failure is not None:
                await self._robot.stop(failure[1])
                self._finish(*failure, snapshot=snapshot)
                return None
            assert snapshot.pose is not None
            error = _angle_error(target_yaw, snapshot.pose.yaw_deg)
            duration = min(
                self._segment_duration,
                max(0.05, math.radians(abs(error)) / max(self._yaw_speed, 1e-3)),
            )
            self._event("command", vx_mps=0.0, vy_mps=0.0,
                        yaw_rps=self._yaw_speed if error > 0 else -self._yaw_speed,
                        duration_s=duration)
            await self._robot.drive(
                vyaw=self._yaw_speed if error > 0 else -self._yaw_speed,
                duration=duration,
            )
            after = await self._settled_snapshot(duration)
            if after.pose is None:
                self._finish(
                    NavigationStatus.UNAVAILABLE, "odometry unavailable during rotation",
                    "stale_odometry", snapshot=after,
                )
                return None
            self._event("motion_sample", x_m=after.pose.x_m, y_m=after.pose.y_m,
                        yaw_degrees=after.pose.yaw_deg, motion_kind="rotation")
            new_error = abs(_angle_error(target_yaw, after.pose.yaw_deg))
            if previous_error - new_error < 1.0:
                no_progress += 1
            else:
                no_progress = 0
            if no_progress >= self._max_no_progress:
                await self._robot.stop("native rotation made no progress")
                self._finish(
                    NavigationStatus.NO_PROGRESS, "native rotation made no progress",
                    "no_progress", snapshot=after,
                )
                return None
            snapshot, previous_error = after, new_error
        return snapshot

    async def _settled_snapshot(self, duration: float) -> NavigationSensorSnapshot:
        await asyncio.sleep(min(self._settle_s, max(0.0, duration)))
        return await self._sensors.get_snapshot()

    def _guard(
        self, snapshot: NavigationSensorSnapshot, goal: RelativeNavigationGoal, started: float,
    ) -> tuple[NavigationStatus, str, str] | None:
        if self._cancel_event.is_set():
            return NavigationStatus.CANCELED, "navigation canceled", "canceled"
        if time.monotonic() - started >= goal.max_duration_s:
            return NavigationStatus.TIMED_OUT, "navigation timed out", "timed_out"
        if not snapshot.ready:
            reason = snapshot.reason or "sensors_unavailable"
            return NavigationStatus.UNAVAILABLE, reason, reason
        return None

    def _finish(
        self, status: NavigationStatus, message: str, error_code: str | None,
        snapshot: NavigationSensorSnapshot | None = None, *, progress: float | None = None,
        replan_count: int | None = None,
    ) -> None:
        self._velocity_limiter.reset()
        self._event("finished", status=status.value, stop_reason=error_code)
        self._state = self._state.model_copy(update={
            "ready": status != NavigationStatus.UNAVAILABLE,
            "status": status, "message": message, "error_code": error_code,
            "stop_reason": error_code or ("goal_reached" if status == NavigationStatus.SUCCEEDED else None),
            "pose": _pose(snapshot) if snapshot is not None else self._state.pose,
            "progress": self._state.progress if progress is None else progress,
            "replan_count": self._state.replan_count if replan_count is None else replan_count,
            "updated_at": datetime.now(timezone.utc),
        })

    def _event(self, kind: str, **fields: object) -> None:
        correlation = {"goal_id": self._state.goal_id} if self._state.goal_id else {}
        self._trace.append({"event": kind, "monotonic": time.monotonic(),
                            **correlation, **fields})
        if self._trace_writer is not None:
            self._trace_writer.record(kind, **correlation, **fields)

    def record_diagnostic_event(self, kind: str, fields: dict[str, object]) -> None:
        """Attach bounded higher-level navigation evidence to the same trace."""
        if not kind.startswith(("visual_servo_", "terrain_execution_",
                                "terrain_exploration_", "exploration_", "patrol_")):
            raise ValueError("unsupported diagnostic event namespace")
        self._event(kind, **fields)


def _pose(snapshot: NavigationSensorSnapshot | None) -> NavigationPose | None:
    if snapshot is None or snapshot.pose is None:
        return None
    value = snapshot.pose
    return NavigationPose(x_m=value.x_m, y_m=value.y_m, yaw_degrees=value.yaw_deg, frame_id=value.frame_id)


def _body_to_world(x: float, y: float, yaw_deg: float, forward: float, left: float) -> tuple[float, float]:
    yaw = math.radians(yaw_deg)
    return x + forward * math.cos(yaw) - left * math.sin(yaw), y + forward * math.sin(yaw) + left * math.cos(yaw)


def _world_delta_to_body(dx: float, dy: float, yaw_deg: float) -> tuple[float, float]:
    yaw = math.radians(yaw_deg)
    return dx * math.cos(yaw) + dy * math.sin(yaw), -dx * math.sin(yaw) + dy * math.cos(yaw)


def _path_pose(snapshot: NavigationSensorSnapshot, x: float, y: float) -> NavigationPose:
    assert snapshot.pose is not None
    wx, wy = _body_to_world(snapshot.pose.x_m, snapshot.pose.y_m, snapshot.pose.yaw_deg, x, y)
    return NavigationPose(x_m=wx, y_m=wy, yaw_degrees=snapshot.pose.yaw_deg, frame_id=snapshot.pose.frame_id)


def _distance(before: NavigationSensorSnapshot, after: NavigationSensorSnapshot) -> float:
    if before.pose is None or after.pose is None:
        return 0.0
    return math.hypot(after.pose.x_m - before.pose.x_m, after.pose.y_m - before.pose.y_m)


def _emergency_corridor_blocked(
    snapshot: NavigationSensorSnapshot, ux: float, uy: float,
    distance_m: float, half_width_m: float,
) -> bool:
    if snapshot.pointcloud is None:
        return True
    for x, y, z in snapshot.pointcloud.points_xyz:
        if not 0.05 <= z <= 1.20:
            continue
        along = x * ux + y * uy
        lateral = abs(-x * uy + y * ux)
        if 0.0 < along <= distance_m and lateral <= half_width_m:
            return True
    return False


def _angle_error(target: float, current: float) -> float:
    return (target - current + 180.0) % 360.0 - 180.0


def _normalize(value: float) -> float:
    return (value + 180.0) % 360.0 - 180.0


def _known_free_only_grid(
    grid: OccupancyGrid2D, start_xy: tuple[float, float], *, clearance_m: float,
) -> OccupancyGrid2D:
    start = grid.world_to_cell(*start_xy)
    radius = math.ceil(max(0.0, clearance_m) / grid.resolution_m)
    traversable = {
        cell for cell in grid.known_free
        if cell not in grid.occupied and all(
            (cell[0]+dx, cell[1]+dy) in grid.known_free
            and (cell[0]+dx, cell[1]+dy) not in grid.occupied
            for dx in range(-radius, radius + 1)
            for dy in range(-radius, radius + 1)
            if dx*dx + dy*dy <= radius*radius
        )
    }
    if start is not None:
        traversable.update(
            cell for cell in grid.known_free
            if cell not in grid.occupied
            and (cell[0]-start[0]) ** 2 + (cell[1]-start[1]) ** 2 <= radius*radius
        )
    blocked = {
        (column, row)
        for column in range(grid.width)
        for row in range(grid.height)
        if (column, row) not in traversable
    }
    blocked.update(grid.occupied)
    if start is not None:
        blocked.discard(start)
    return OccupancyGrid2D(
        resolution_m=grid.resolution_m, width=grid.width, height=grid.height,
        origin_x_m=grid.origin_x_m, origin_y_m=grid.origin_y_m,
        occupied=frozenset(blocked), raw_occupied=grid.raw_occupied,
        known_free=frozenset(traversable),
        traversal_cost_values=grid.traversal_cost_values, frame_id=grid.frame_id,
    )


def _resample_polyline(
    points: list[tuple[float, float]], *, maximum_segment_m: float,
) -> list[tuple[float, float]]:
    if maximum_segment_m <= 0:
        raise ValueError("maximum route segment must be positive")
    if not points:
        return []
    result = [points[0]]
    for start, end in zip(points, points[1:]):
        distance = math.hypot(end[0]-start[0], end[1]-start[1])
        steps = max(1, math.ceil(distance / maximum_segment_m))
        result.extend((
            start[0] + (end[0]-start[0]) * index / steps,
            start[1] + (end[1]-start[1]) * index / steps,
        ) for index in range(1, steps + 1))
    return result


def _odom_to_map(pose: NavigationPose | None, transform: NavigationPose) -> NavigationPose | None:
    if pose is None:
        return None
    x, y = _body_to_world(
        transform.x_m, transform.y_m, transform.yaw_degrees, pose.x_m, pose.y_m
    )
    return NavigationPose(
        x_m=x, y_m=y,
        yaw_degrees=_normalize(transform.yaw_degrees + pose.yaw_degrees),
        frame_id="map",
    )


def _map_to_odom(pose: NavigationPose, transform: NavigationPose) -> NavigationPose:
    dx, dy = pose.x_m - transform.x_m, pose.y_m - transform.y_m
    x, y = _world_delta_to_body(dx, dy, transform.yaw_degrees)
    return NavigationPose(
        x_m=x, y_m=y,
        yaw_degrees=_normalize(pose.yaw_degrees - transform.yaw_degrees),
        frame_id="odom",
    )


def _robot_pose_in_map(pose, transform: NavigationPose):
    mapped = _odom_to_map(
        NavigationPose(
            x_m=pose.x_m, y_m=pose.y_m,
            yaw_degrees=pose.yaw_deg, frame_id=pose.frame_id,
        ),
        transform,
    )
    assert mapped is not None
    return RobotPose(
        x_m=mapped.x_m, y_m=mapped.y_m, z_m=pose.z_m,
        yaw_deg=mapped.yaw_degrees, frame_id="map", timestamp=pose.timestamp,
    )


def _map_from_odom_transform(map_pose: RobotPose, odom_pose: RobotPose) -> NavigationPose:
    yaw = _normalize(map_pose.yaw_deg - odom_pose.yaw_deg)
    rotated_x, rotated_y = _body_to_world(0.0, 0.0, yaw, odom_pose.x_m, odom_pose.y_m)
    return NavigationPose(
        x_m=map_pose.x_m - rotated_x,
        y_m=map_pose.y_m - rotated_y,
        yaw_degrees=yaw,
        frame_id="map",
    )
