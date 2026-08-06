"""Dependency-free planar scan matching for native map relocalization."""
from __future__ import annotations

from dataclasses import dataclass
import math

from robot_brain.core.robot_self_state import RobotPose
from robot_brain.navigation.map_store import SparseVoxelMap
from robot_brain.perception.pointcloud import PointCloudSnapshot


@dataclass(frozen=True)
class RelocalizationResult:
    accepted: bool
    pose: RobotPose | None
    fitness: float
    rmse_m: float | None
    inlier_count: int
    source_count: int
    candidates_evaluated: int
    mode: str
    reason: str


def relocalize_with_initial(
    reference: SparseVoxelMap,
    local_cloud: PointCloudSnapshot,
    initial_pose: RobotPose,
    *,
    search_radius_m: float = 1.0,
    yaw_search_deg: float = 30.0,
    translation_step_m: float = 0.20,
    yaw_step_deg: float = 10.0,
    max_correspondence_m: float = 0.20,
    min_fitness: float = 0.45,
    max_rmse_m: float = 0.16,
    min_points: int = 8,
) -> RelocalizationResult:
    """Refine a cached/fixed start pose with coarse-to-fine planar matching."""
    return _search(
        reference, local_cloud,
        x_values=_axis(initial_pose.x_m, search_radius_m, translation_step_m),
        y_values=_axis(initial_pose.y_m, search_radius_m, translation_step_m),
        yaw_values=_axis(initial_pose.yaw_deg, yaw_search_deg, yaw_step_deg),
        initial_pose=initial_pose,
        max_correspondence_m=max_correspondence_m,
        min_fitness=min_fitness,
        max_rmse_m=max_rmse_m,
        min_points=min_points,
        mode="initial",
        refine=True,
    )


def relocalize_global(
    reference: SparseVoxelMap,
    local_cloud: PointCloudSnapshot,
    *,
    xy_step_m: float = 0.50,
    yaw_step_deg: float = 30.0,
    margin_m: float = 0.50,
    max_correspondence_m: float = 0.25,
    min_fitness: float = 0.50,
    max_rmse_m: float = 0.18,
    min_points: int = 8,
    max_candidates: int = 50_000,
) -> RelocalizationResult:
    """Bounded global fallback over the saved map extent.

    This deliberately favors deterministic bounded work over DIMOS/Open3D's
    very large stochastic RANSAC budget. Ambiguous or oversized searches fail
    closed and are visible in the returned reason.
    """
    target = reference.points(min_hits=1)
    if not target:
        return _rejected("empty_reference_map", "global")
    min_x = min(point[0] for point in target) - margin_m
    max_x = max(point[0] for point in target) + margin_m
    min_y = min(point[1] for point in target) - margin_m
    max_y = max(point[1] for point in target) + margin_m
    xs = _range(min_x, max_x, xy_step_m)
    ys = _range(min_y, max_y, xy_step_m)
    yaws = _range(-180.0, 180.0 - yaw_step_deg, yaw_step_deg)
    if len(xs) * len(ys) * len(yaws) > max_candidates:
        return _rejected("global_search_budget_exceeded", "global")
    return _search(
        reference, local_cloud, x_values=xs, y_values=ys, yaw_values=yaws,
        initial_pose=RobotPose(frame_id="map"),
        max_correspondence_m=max_correspondence_m,
        min_fitness=min_fitness, max_rmse_m=max_rmse_m,
        min_points=min_points, mode="global", refine=True,
    )


def merge_local_observation(
    target: SparseVoxelMap,
    local_cloud: PointCloudSnapshot,
    localized_pose: RobotPose,
) -> int:
    if localized_pose.frame_id != "map":
        raise ValueError("localized pose must be in map frame")
    return target.integrate(local_cloud, localized_pose)


def _search(
    reference: SparseVoxelMap,
    local_cloud: PointCloudSnapshot,
    *,
    x_values: list[float], y_values: list[float], yaw_values: list[float],
    initial_pose: RobotPose, max_correspondence_m: float,
    min_fitness: float, max_rmse_m: float, min_points: int,
    mode: str, refine: bool,
) -> RelocalizationResult:
    if local_cloud.frame_id not in {"base", "base_link", "unitree_lidar", "utlidar"}:
        return _rejected("untrusted_local_cloud_frame", mode)
    source = _planar_points(local_cloud.points_xyz)
    target = _planar_points(reference.points(min_hits=1))
    if len(source) < min_points:
        return _rejected("insufficient_local_points", mode, len(source))
    if len(target) < min_points:
        return _rejected("insufficient_reference_points", mode, len(source))
    # Bound runtime and reduce repeated points from dense walls.
    source = _downsample(source, reference.resolution_m, limit=500)
    target = _downsample(target, reference.resolution_m, limit=10_000)
    target_index = _spatial_index(target, max_correspondence_m)
    best: tuple[float, float, int, float, float, float] | None = None
    evaluated = 0
    for x in x_values:
        for y in y_values:
            for yaw in yaw_values:
                score = _score(source, target_index, x, y, yaw, max_correspondence_m)
                evaluated += 1
                candidate = (score[0], -score[1], score[2], x, y, _normalize(yaw))
                if best is None or candidate[:3] > best[:3]:
                    best = candidate
    if best is None:
        return _rejected("no_candidates", mode, len(source), evaluated)
    if refine:
        _, _, _, bx, by, byaw = best
        for trans_step, yaw_step in (
            (max(reference.resolution_m, 0.10), 5.0),
            (max(reference.resolution_m / 2.0, 0.05), 2.0),
        ):
            for dx in (-trans_step, 0.0, trans_step):
                for dy in (-trans_step, 0.0, trans_step):
                    for dyaw in (-yaw_step, 0.0, yaw_step):
                        score = _score(
                            source, target_index, bx + dx, by + dy, byaw + dyaw,
                            max_correspondence_m,
                        )
                        evaluated += 1
                        candidate = (
                            score[0], -score[1], score[2], bx + dx, by + dy,
                            _normalize(byaw + dyaw),
                        )
                        if candidate[:3] > best[:3]:
                            best = candidate
            _, _, _, bx, by, byaw = best
    fitness, negative_rmse, inliers, x, y, yaw = best
    rmse = -negative_rmse if inliers else None
    accepted = fitness >= min_fitness and rmse is not None and rmse <= max_rmse_m
    reason = "accepted" if accepted else (
        "fitness_below_threshold" if fitness < min_fitness else "rmse_above_threshold"
    )
    return RelocalizationResult(
        accepted=accepted,
        pose=RobotPose(x_m=x, y_m=y, yaw_deg=yaw, frame_id="map") if accepted else None,
        fitness=fitness, rmse_m=rmse, inlier_count=inliers,
        source_count=len(source), candidates_evaluated=evaluated,
        mode=mode, reason=reason,
    )


def _score(source, target_index, x, y, yaw_deg, max_distance):
    yaw = math.radians(yaw_deg)
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    squared: list[float] = []
    cell_size = max_distance
    limit_squared = max_distance * max_distance
    for sx, sy in source:
        wx = x + sx * cos_yaw - sy * sin_yaw
        wy = y + sx * sin_yaw + sy * cos_yaw
        cell = (math.floor(wx / cell_size), math.floor(wy / cell_size))
        best = float("inf")
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for tx, ty in target_index.get((cell[0] + dx, cell[1] + dy), ()):
                    distance = (wx - tx) ** 2 + (wy - ty) ** 2
                    best = min(best, distance)
        if best <= limit_squared:
            squared.append(best)
    fitness = len(squared) / len(source) if source else 0.0
    rmse = math.sqrt(sum(squared) / len(squared)) if squared else float("inf")
    return fitness, rmse, len(squared)


def _spatial_index(points, cell_size):
    result = {}
    for x, y in points:
        key = (math.floor(x / cell_size), math.floor(y / cell_size))
        result.setdefault(key, []).append((x, y))
    return result


def _planar_points(points):
    return [(float(x), float(y)) for x, y, z in points if -0.2 <= z <= 1.8 and math.isfinite(x) and math.isfinite(y)]


def _downsample(points, resolution, *, limit):
    result = {}
    for x, y in points:
        result.setdefault((math.floor(x / resolution), math.floor(y / resolution)), (x, y))
        if len(result) >= limit:
            break
    return list(result.values())


def _axis(center, radius, step):
    if radius < 0 or step <= 0:
        raise ValueError("invalid search axis")
    return _range(center - radius, center + radius, step)


def _range(start, stop, step):
    if step <= 0:
        raise ValueError("step must be positive")
    count = max(0, math.floor((stop - start) / step + 1e-9))
    return [start + index * step for index in range(count + 1)]


def _normalize(value):
    return (value + 180.0) % 360.0 - 180.0


def _rejected(reason, mode, source_count=0, evaluated=0):
    return RelocalizationResult(
        accepted=False, pose=None, fitness=0.0, rmse_m=None,
        inlier_count=0, source_count=source_count,
        candidates_evaluated=evaluated, mode=mode, reason=reason,
    )
