"""Sensor boundary used by local navigation providers."""
from __future__ import annotations

from dataclasses import dataclass
import math
import time
from typing import Protocol

from robot_brain.core.robot_self_state import RobotPose
from robot_brain.perception.pointcloud import PointCloudSnapshot


@dataclass(frozen=True)
class NavigationSensorSnapshot:
    pose: RobotPose | None
    pointcloud: PointCloudSnapshot | None
    pose_age_seconds: float
    pointcloud_age_seconds: float
    pose_ready: bool
    obstacle_data_ready: bool
    obstacle_frame: str | None
    pose_source: str | None = None
    reason: str | None = None

    @property
    def ready(self) -> bool:
        return self.pose_ready and self.obstacle_data_ready


class NavigationSensorProvider(Protocol):
    async def get_snapshot(self) -> NavigationSensorSnapshot: ...


class UnitreeNavigationSensorProvider:
    """Expose Go2 transport odometry and built-in LiDAR with strict freshness gates."""

    def __init__(
        self,
        transport,
        *,
        max_pose_age_s: float = 1.0,
        max_pointcloud_age_s: float = 0.5,
        obstacle_frames: frozenset[str] = frozenset(
            {"base", "base_link", "unitree_lidar", "utlidar"}
        ),
        max_world_origin_error_m: float = 0.75,
        require_authoritative_odom: bool = True,
    ) -> None:
        if any(not math.isfinite(value) or value <= 0 for value in (
            max_pose_age_s, max_pointcloud_age_s,
        )) or not math.isfinite(max_world_origin_error_m) or max_world_origin_error_m < 0:
            raise ValueError("invalid navigation sensor freshness/frame limits")
        if not obstacle_frames:
            raise ValueError("at least one trusted obstacle frame is required")
        self._transport = transport
        self._max_pose_age_s = max_pose_age_s
        self._max_pointcloud_age_s = max_pointcloud_age_s
        self._obstacle_frames = obstacle_frames
        self._max_world_origin_error_m = max_world_origin_error_m
        self._require_authoritative_odom = require_authoritative_odom

    async def get_snapshot(self) -> NavigationSensorSnapshot:
        now_wall = time.time()
        state = await self._transport.read_state()
        odom_age = getattr(self._transport, "odometry_age_seconds", None)
        pose_age = float(
            odom_age() if callable(odom_age) else self._transport.state_age_seconds()
        )
        pose = RobotPose(
            x_m=state.position.x,
            y_m=state.position.y,
            yaw_deg=state.heading_degrees,
            frame_id=state.pose_frame_id,
            timestamp=now_wall - pose_age if math.isfinite(pose_age) else None,
        )
        cloud = self._transport.read_lidar_snapshot()
        cloud_age = (
            float(self._transport.lidar_age_seconds())
            if cloud is not None
            else float("inf")
        )
        source_ready = (
            not self._require_authoritative_odom
            or state.pose_source == "unitree_robotodom"
        )
        pose_values_ready = all(math.isfinite(value) for value in (
            pose.x_m, pose.y_m, pose.z_m, pose.yaw_deg,
        ))
        pose_ready = (
            math.isfinite(pose_age)
            and pose_age >= 0.0
            and pose_age <= self._max_pose_age_s
            and source_ready
            and pose_values_ready
        )
        cloud = self._normalize_obstacle_cloud(cloud, pose)
        cloud = self._finite_cloud(cloud)
        frame_ready = cloud is not None and cloud.frame_id in self._obstacle_frames
        cloud_ready = (
            cloud is not None
            and cloud.point_count > 0
            and math.isfinite(cloud_age)
            and cloud_age >= 0.0
            and cloud_age <= self._max_pointcloud_age_s
            and frame_ready
        )

        reason = None
        if not source_ready:
            reason = "authoritative_robotodom_unavailable"
        elif not pose_values_ready:
            reason = "invalid_odometry"
        elif not math.isfinite(pose_age) or pose_age < 0.0:
            reason = "invalid_odometry_timestamp"
        elif not pose_ready:
            reason = "stale_odometry"
        elif cloud is None:
            reason = "missing_pointcloud"
        elif not math.isfinite(cloud_age) or cloud_age < 0.0:
            reason = "invalid_pointcloud_timestamp"
        elif cloud_age > self._max_pointcloud_age_s:
            reason = "stale_pointcloud"
        elif not frame_ready:
            reason = "untrusted_obstacle_frame"
        elif cloud.point_count == 0:
            reason = "invalid_pointcloud"
        return NavigationSensorSnapshot(
            pose=pose,
            pointcloud=cloud,
            pose_age_seconds=pose_age,
            pointcloud_age_seconds=cloud_age,
            pose_ready=pose_ready,
            obstacle_data_ready=cloud_ready,
            obstacle_frame=cloud.frame_id if cloud is not None else None,
            pose_source=state.pose_source,
            reason=reason,
        )

    @staticmethod
    def _finite_cloud(cloud: PointCloudSnapshot | None) -> PointCloudSnapshot | None:
        if cloud is None:
            return None
        filtered: list[tuple[float, float, float]] = []
        for raw in cloud.points_xyz:
            try:
                if len(raw) != 3:
                    continue
                point = tuple(float(value) for value in raw)
            except (TypeError, ValueError, OverflowError):
                continue
            if all(math.isfinite(value) for value in point):
                filtered.append(point)  # type: ignore[arg-type]
        points = tuple(filtered)
        if points == cloud.points_xyz:
            return cloud
        return PointCloudSnapshot(
            points_xyz=points, frame_id=cloud.frame_id,
            sensor_timestamp=cloud.sensor_timestamp,
            received_monotonic=cloud.received_monotonic,
            source=f"{cloud.source}:finite_filter",
            timestamp_valid=cloud.timestamp_valid,
            origin_xyz=cloud.origin_xyz,
        )

    _MAP_FRAMES = frozenset({"world", "odom"})

    def _normalize_obstacle_cloud(
        self,
        cloud: PointCloudSnapshot | None,
        pose: RobotPose,
    ) -> PointCloudSnapshot | None:
        """Convert Unitree's odom/world voxel map into the robot body frame.

        Go2 built-in voxel maps commonly arrive as ``world`` or ``odom``.

        For ``odom`` clouds that share the session-local ``robot_pose`` frame,
        always transform with the live robot pose.  The payload ``origin`` field
        on Unitree voxel maps is typically the grid AABB corner (e.g. -3.225 m),
        not a TF sensor origin, so it must not gate or drive the transform.

        For ``world`` clouds, keep the stricter gate: only convert when a
        payload origin is present and close to the simultaneous Go2 odometry.
        """
        if cloud is None or cloud.frame_id in self._obstacle_frames:
            return cloud
        if cloud.frame_id not in self._MAP_FRAMES:
            return cloud

        if cloud.frame_id == "odom" and pose.frame_id == "odom":
            ox, oy, oz = pose.x_m, pose.y_m, pose.z_m
        elif cloud.frame_id == "world" and cloud.origin_xyz is not None:
            ox, oy, oz = cloud.origin_xyz
            if math.hypot(ox - pose.x_m, oy - pose.y_m) > self._max_world_origin_error_m:
                return cloud
        else:
            return cloud

        yaw = math.radians(pose.yaw_deg)
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
        body_points: list[tuple[float, float, float]] = []
        for x, y, z in cloud.points_xyz:
            dx, dy = x - ox, y - oy
            body_points.append(
                (
                    dx * cos_yaw + dy * sin_yaw,
                    -dx * sin_yaw + dy * cos_yaw,
                    z - oz,
                )
            )
        return PointCloudSnapshot(
            points_xyz=tuple(body_points),
            frame_id="base_link",
            sensor_timestamp=cloud.sensor_timestamp,
            received_monotonic=cloud.received_monotonic,
            source=f"{cloud.source}:{cloud.frame_id}_to_base",
            timestamp_valid=cloud.timestamp_valid,
            origin_xyz=(0.0, 0.0, 0.0),
        )
