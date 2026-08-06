"""Bounded sensor/pose replay artifacts for native navigation verification."""
from __future__ import annotations

from dataclasses import dataclass
import gzip
import json
import math
from pathlib import Path
from typing import Iterable

from robot_brain.core.robot_self_state import RobotPose
from robot_brain.navigation.grid import costmap_from_pointcloud
from robot_brain.navigation.planner import astar_path
from robot_brain.navigation.map_store import SparseVoxelMap
from robot_brain.navigation.pose_graph import (
    OnlinePoseGraphTracker, PlanarPose, PoseGraphTrackerConfig,
)
from robot_brain.navigation.relocalization import relocalize_global, relocalize_with_initial
from robot_brain.navigation.sensors import NavigationSensorSnapshot
from robot_brain.perception.pointcloud import PointCloudSnapshot


@dataclass(frozen=True)
class NavigationReplayFrame:
    sequence: int
    pose: RobotPose
    points_xyz: tuple[tuple[float, float, float], ...]
    pose_age_seconds: float
    pointcloud_age_seconds: float
    obstacle_frame: str


class NavigationReplayWriter:
    def __init__(self, path: str | Path, *, max_points_per_frame: int = 20_000) -> None:
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._max_points = max_points_per_frame
        self._sequence = 0
        self._started = False

    def record(self, snapshot: NavigationSensorSnapshot) -> bool:
        if not snapshot.ready or snapshot.pose is None or snapshot.pointcloud is None:
            return False
        points = tuple(
            point for point in snapshot.pointcloud.points_xyz
            if len(point) == 3 and all(math.isfinite(float(value)) for value in point)
        )
        stride = max(1, math.ceil(len(points) / self._max_points))
        row = {
            "schema_version": 1,
            "sequence": self._sequence,
            "pose": snapshot.pose.model_dump(mode="json"),
            "points_xyz": points[::stride],
            "pose_age_seconds": snapshot.pose_age_seconds,
            "pointcloud_age_seconds": snapshot.pointcloud_age_seconds,
            "obstacle_frame": snapshot.obstacle_frame,
        }
        self._sequence += 1
        # The first frame reserves a new artifact atomically. Reusing a path
        # must fail instead of silently mixing independent hardware sessions.
        with gzip.open(self.path, "at" if self._started else "xt", encoding="utf-8") as stream:
            stream.write(json.dumps(row, separators=(",", ":"), allow_nan=False) + "\n")
        self._started = True
        return True


def load_navigation_replay(path: str | Path) -> list[NavigationReplayFrame]:
    result: list[NavigationReplayFrame] = []
    with gzip.open(path, "rt", encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, 1):
            try:
                row = json.loads(line)
                if row.get("schema_version") != 1:
                    raise ValueError("unsupported replay schema")
                result.append(NavigationReplayFrame(
                    sequence=int(row["sequence"]),
                    pose=RobotPose.model_validate(row["pose"]),
                    points_xyz=tuple(tuple(float(value) for value in point) for point in row["points_xyz"]),
                    pose_age_seconds=float(row["pose_age_seconds"]),
                    pointcloud_age_seconds=float(row["pointcloud_age_seconds"]),
                    obstacle_frame=str(row["obstacle_frame"]),
                ))
            except Exception as exc:
                raise ValueError(f"invalid replay frame at line {line_number}: {exc}") from exc
    return result


def evaluate_replay_planning(
    frames: Iterable[NavigationReplayFrame],
    *,
    goal_body_xy: tuple[float, float],
    map_size_m: float = 7.0,
    resolution_m: float = 0.10,
    robot_radius_m: float = 0.30,
) -> dict[str, object]:
    total = paths = blocked = 0
    minimum_path_points: int | None = None
    for frame in frames:
        total += 1
        cloud = PointCloudSnapshot(
            points_xyz=frame.points_xyz, frame_id=frame.obstacle_frame,
            sensor_timestamp=frame.pose.timestamp, received_monotonic=0.0,
            source="native_replay", timestamp_valid=frame.pose.timestamp is not None,
        )
        grid = costmap_from_pointcloud(
            cloud, size_m=map_size_m, resolution_m=resolution_m,
            robot_radius_m=robot_radius_m,
        )
        path = astar_path(grid, (0.0, 0.0), goal_body_xy)
        if path is None:
            blocked += 1
        else:
            paths += 1
            minimum_path_points = len(path) if minimum_path_points is None else min(
                minimum_path_points, len(path)
            )
    return {
        "frames": total, "paths_found": paths, "blocked_frames": blocked,
        "minimum_path_points": minimum_path_points,
    }


def evaluate_replay_relocalization(
    frames: Iterable[NavigationReplayFrame], reference: SparseVoxelMap, *,
    initial_map_pose: RobotPose | None = None, allow_global_fallback: bool = False,
) -> dict[str, object]:
    """Run the production relocalizer against the first recorded fresh scan."""
    frame = next(iter(frames), None)
    if frame is None:
        return {"ok": False, "reason": "empty_replay"}
    cloud = _frame_cloud(frame)
    if initial_map_pose is not None:
        result = relocalize_with_initial(reference, cloud, initial_map_pose)
        if not result.accepted and allow_global_fallback:
            result = relocalize_global(reference, cloud)
    elif allow_global_fallback:
        result = relocalize_global(reference, cloud)
    else:
        return {"ok": False, "reason": "initial_pose_or_global_fallback_required"}
    identity = reference.identity()
    return {
        "ok": result.accepted, "reason": result.reason, "mode": result.mode,
        "fitness": result.fitness, "rmse_m": result.rmse_m,
        "inlier_count": result.inlier_count, "source_count": result.source_count,
        "candidates_evaluated": result.candidates_evaluated,
        "pose": result.pose.model_dump(mode="json") if result.pose else None,
        "map_identity": {
            "map_id": identity.map_id, "version": identity.version,
            "content_revision": identity.revision,
        },
    }


def evaluate_replay_mapping(
    frames: Iterable[NavigationReplayFrame], *, resolution_m: float = .10,
    max_voxels: int = 500_000,
) -> dict[str, object]:
    """Rebuild a bounded native voxel map from recorded body scans and odom."""
    target = SparseVoxelMap(resolution_m=resolution_m, map_id="native-replay-audit",
                            max_voxels=max_voxels)
    integrated = input_points = 0
    for frame in frames:
        if frame.obstacle_frame not in {"base", "base_link", "unitree_lidar", "utlidar"}:
            return {"ok": False, "reason": "untrusted_replay_obstacle_frame",
                    "frames_integrated": integrated}
        input_points += len(frame.points_xyz)
        target.integrate(_frame_cloud(frame), frame.pose, carve_free_space=True)
        integrated += 1
    identity = target.identity()
    ok = integrated > 0 and input_points > 0 and 0 < identity.voxel_count <= max_voxels
    return {
        "ok": ok, "reason": "mapped" if ok else "empty_replay_mapping",
        "frames_integrated": integrated, "input_points": input_points,
        "voxel_count": identity.voxel_count, "max_voxels": max_voxels,
        "map_identity": {"map_id": identity.map_id, "version": identity.version,
                         "content_revision": identity.revision},
    }


def evaluate_replay_pose_graph(
    frames: Iterable[NavigationReplayFrame], *,
    config: PoseGraphTrackerConfig | None = None,
    verification_overrides: dict[str, object] | None = None,
) -> dict[str, object]:
    """Replay recorded odom/scans through the production online loop tracker."""
    tracker = OnlinePoseGraphTracker(config)
    events: list[dict[str, object]] = []
    accepted_loops = 0
    previous_timestamp = -math.inf
    for frame in frames:
        timestamp = float(frame.pose.timestamp if frame.pose.timestamp is not None else frame.sequence)
        if not math.isfinite(timestamp) or timestamp <= previous_timestamp:
            return {"ok": False, "reason": "non_monotonic_replay_timestamps",
                    "frames_processed": len(events), "events": events}
        previous_timestamp = timestamp
        update = tracker.process(
            timestamp, PlanarPose(frame.pose.x_m, frame.pose.y_m, frame.pose.yaw_deg),
            frame.points_xyz, **(verification_overrides or {}),
        )
        result = update.graph_result
        verification = update.loop_verification
        accepted = bool(update.loop_added and result is not None and result.accepted)
        accepted_loops += int(accepted)
        events.append({
            "sequence": frame.sequence, "reason": update.reason,
            "keyframe_added": update.keyframe_added, "loop_added": update.loop_added,
            "loop_source_index": update.loop_source_index,
            "loop_accepted": accepted,
            "loop_fitness": verification.fitness if verification else None,
            "loop_rmse_m": verification.rmse_m if verification else None,
            "initial_graph_rmse": result.initial_rmse if result else None,
            "optimized_graph_rmse": result.optimized_rmse if result else None,
            "translation_correction_m": result.max_translation_correction_m if result else None,
            "yaw_correction_degrees": result.max_yaw_correction_degrees if result else None,
        })
    return {
        "ok": accepted_loops > 0, "reason": "accepted_loop_observed" if accepted_loops else "no_accepted_loop",
        "frames_processed": len(events), "keyframes": len(tracker.graph.keyframes),
        "accepted_loops": accepted_loops, "events": events,
    }


def _frame_cloud(frame: NavigationReplayFrame) -> PointCloudSnapshot:
    return PointCloudSnapshot(
        points_xyz=frame.points_xyz, frame_id=frame.obstacle_frame,
        sensor_timestamp=frame.pose.timestamp, received_monotonic=0.0,
        source="native_replay", timestamp_valid=frame.pose.timestamp is not None,
    )
