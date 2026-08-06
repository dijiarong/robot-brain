"""Dependency-free, bounded planar pose-graph correction."""
from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class PlanarPose:
    x_m: float
    y_m: float
    yaw_degrees: float

    def __post_init__(self) -> None:
        if not all(map(math.isfinite, (self.x_m, self.y_m, self.yaw_degrees))):
            raise ValueError("pose values must be finite")


@dataclass(frozen=True)
class PoseGraphKeyframe:
    timestamp_s: float
    raw: PlanarPose

    def __post_init__(self) -> None:
        if not math.isfinite(self.timestamp_s):
            raise ValueError("keyframe timestamp must be finite")


@dataclass(frozen=True)
class PoseGraphConstraint:
    source_index: int
    target_index: int
    relative: PlanarPose
    weight: float = 1.0
    kind: str = "loop"

    def __post_init__(self) -> None:
        if min(self.source_index, self.target_index) < 0 or self.source_index == self.target_index:
            raise ValueError("invalid constraint indices")
        if not math.isfinite(self.weight) or self.weight <= 0:
            raise ValueError("constraint weight must be positive and finite")


@dataclass(frozen=True)
class PoseGraphResult:
    accepted: bool
    reason: str
    optimized: tuple[PlanarPose, ...]
    initial_rmse: float | None
    optimized_rmse: float | None
    iterations: int
    loop_constraints: int
    max_translation_correction_m: float
    max_yaw_correction_degrees: float


@dataclass(frozen=True)
class LoopVerificationResult:
    accepted: bool
    reason: str
    relative: PlanarPose | None
    fitness: float
    rmse_m: float | None
    inlier_count: int
    candidates_evaluated: int


@dataclass(frozen=True)
class PoseGraphTrackerConfig:
    keyframe_translation_m: float = 0.5
    keyframe_yaw_degrees: float = 10.0
    loop_search_radius_m: float = 1.5
    minimum_loop_age_s: float = 20.0
    minimum_keyframes_for_loop: int = 8
    max_loop_candidates_per_keyframe: int = 4
    max_keyframes: int = 2000

    def __post_init__(self) -> None:
        positive = (self.keyframe_translation_m, self.keyframe_yaw_degrees,
                    self.loop_search_radius_m, self.minimum_loop_age_s)
        if not all(math.isfinite(value) and value > 0 for value in positive):
            raise ValueError("invalid pose graph tracker geometry limits")
        if min(self.minimum_keyframes_for_loop, self.max_loop_candidates_per_keyframe,
               self.max_keyframes) <= 0:
            raise ValueError("invalid pose graph tracker capacity")


@dataclass(frozen=True)
class PoseGraphTrackerUpdate:
    keyframe_added: bool
    keyframe_index: int | None
    loop_added: bool
    loop_source_index: int | None
    corrected_pose: PlanarPose
    graph_result: PoseGraphResult | None
    loop_verification: LoopVerificationResult | None
    reason: str


class PlanarPoseGraph:
    """Optimizes odometry plus independently scan-verified loop constraints."""

    def __init__(self) -> None:
        self._keyframes: list[PoseGraphKeyframe] = []
        self._constraints: list[PoseGraphConstraint] = []

    @property
    def keyframes(self) -> tuple[PoseGraphKeyframe, ...]:
        return tuple(self._keyframes)

    @property
    def constraints(self) -> tuple[PoseGraphConstraint, ...]:
        return tuple(self._constraints)

    def add_keyframe(self, timestamp_s: float, pose: PlanarPose) -> int:
        if self._keyframes and timestamp_s <= self._keyframes[-1].timestamp_s:
            raise ValueError("keyframe timestamps must be strictly increasing")
        index = len(self._keyframes)
        self._keyframes.append(PoseGraphKeyframe(timestamp_s, pose))
        if index:
            self._constraints.append(PoseGraphConstraint(
                index-1, index, relative_pose(self._keyframes[index-1].raw, pose),
                weight=4.0, kind="odometry",
            ))
        return index

    def add_loop_constraint(self, source_index: int, target_index: int,
                            relative: PlanarPose, *, confidence: float) -> None:
        if max(source_index, target_index) >= len(self._keyframes):
            raise ValueError("constraint references missing keyframe")
        if not math.isfinite(confidence) or not 0 < confidence <= 1:
            raise ValueError("loop confidence must be in (0, 1]")
        self._constraints.append(PoseGraphConstraint(
            source_index, target_index, relative,
            weight=max(1.0, confidence*20.0), kind="loop",
        ))

    def optimize(self, *, max_iterations: int = 80, relaxation: float = 0.35,
                 convergence_tolerance: float = 1e-4,
                 minimum_improvement: float = 0.05,
                 max_translation_correction_m: float = 5.0,
                 max_yaw_correction_degrees: float = 90.0) -> PoseGraphResult:
        if max_iterations <= 0 or not 0 < relaxation <= 1:
            raise ValueError("invalid optimization budget")
        raw = tuple(frame.raw for frame in self._keyframes)
        loops = sum(edge.kind == "loop" for edge in self._constraints)
        if len(raw) < 2:
            return _rejected("insufficient_keyframes", raw, loops)
        if not loops:
            return _rejected("no_verified_loop_constraints", raw, loops)
        initial_rmse = self._rmse(raw)
        poses = list(raw)
        iterations = 0
        for iterations in range(1, max_iterations+1):
            proposals: list[list[tuple[PlanarPose, float]]] = [[] for _ in poses]
            for edge in self._constraints:
                source, target = poses[edge.source_index], poses[edge.target_index]
                proposals[edge.target_index].append((compose_pose(source, edge.relative), edge.weight))
                proposals[edge.source_index].append((compose_pose(target, inverse_pose(edge.relative)), edge.weight))
            next_poses = [poses[0]]  # hard map anchor
            max_delta = 0.0
            for index in range(1, len(poses)):
                updated = _interpolate_pose(poses[index], _weighted_pose(proposals[index]), relaxation)
                max_delta = max(max_delta, _pose_delta(poses[index], updated))
                next_poses.append(updated)
            poses = next_poses
            if max_delta <= convergence_tolerance:
                break
        optimized = tuple(poses)
        optimized_rmse = self._rmse(optimized)
        max_trans = max(math.hypot(a.x_m-b.x_m, a.y_m-b.y_m) for a, b in zip(raw, optimized))
        max_yaw = max(abs(_angle_delta(a.yaw_degrees, b.yaw_degrees)) for a, b in zip(raw, optimized))
        improvement = (initial_rmse-optimized_rmse)/max(initial_rmse, 1e-9)
        accepted, reason = True, "accepted"
        if not all(map(math.isfinite, (initial_rmse, optimized_rmse, max_trans, max_yaw))):
            accepted, reason = False, "non_finite_solution"
        elif optimized_rmse >= initial_rmse or improvement < minimum_improvement:
            accepted, reason = False, "insufficient_residual_improvement"
        elif max_trans > max_translation_correction_m:
            accepted, reason = False, "translation_correction_exceeds_limit"
        elif max_yaw > max_yaw_correction_degrees:
            accepted, reason = False, "yaw_correction_exceeds_limit"
        return PoseGraphResult(accepted, reason, optimized if accepted else raw,
                               initial_rmse, optimized_rmse, iterations, loops,
                               max_trans, max_yaw)

    def correction(self, result: PoseGraphResult, timestamp_s: float) -> PlanarPose:
        if not result.accepted or len(result.optimized) != len(self._keyframes):
            raise ValueError("only an accepted graph result can publish correction")
        if not math.isfinite(timestamp_s):
            raise ValueError("correction timestamp must be finite")
        if timestamp_s <= self._keyframes[0].timestamp_s:
            index, alpha = 0, 0.0
        elif timestamp_s >= self._keyframes[-1].timestamp_s:
            index, alpha = len(self._keyframes)-2, 1.0
        else:
            index = next(i for i in range(len(self._keyframes)-1)
                         if self._keyframes[i+1].timestamp_s >= timestamp_s)
            lo, hi = self._keyframes[index:index+2]
            alpha = (timestamp_s-lo.timestamp_s)/(hi.timestamp_s-lo.timestamp_s)
        raw = _interpolate_pose(self._keyframes[index].raw, self._keyframes[index+1].raw, alpha)
        corrected = _interpolate_pose(result.optimized[index], result.optimized[index+1], alpha)
        return compose_pose(corrected, inverse_pose(raw))

    def _rmse(self, poses) -> float:
        squared, total_weight = 0.0, 0.0
        for edge in self._constraints:
            actual = relative_pose(poses[edge.source_index], poses[edge.target_index])
            translation = math.hypot(actual.x_m-edge.relative.x_m, actual.y_m-edge.relative.y_m)
            yaw = math.radians(_angle_delta(actual.yaw_degrees, edge.relative.yaw_degrees))
            squared += edge.weight*(translation*translation+yaw*yaw)
            total_weight += edge.weight
        return math.sqrt(squared/max(total_weight, 1.0))


class OnlinePoseGraphTracker:
    """Bounded keyframe/loop tracker; proximity alone never creates an edge."""

    def __init__(self, config: PoseGraphTrackerConfig | None = None) -> None:
        self.config = config or PoseGraphTrackerConfig()
        self.graph = PlanarPoseGraph()
        self._scans: list[tuple[tuple[float, float, float], ...]] = []
        self._last_result: PoseGraphResult | None = None
        self._latest_correction = PlanarPose(0.0, 0.0, 0.0)

    def process(self, timestamp_s: float, raw_pose: PlanarPose, points_xyz,
                **verification_overrides) -> PoseGraphTrackerUpdate:
        if not math.isfinite(timestamp_s):
            raise ValueError("tracker timestamp must be finite")
        if self.graph.keyframes:
            previous = self.graph.keyframes[-1].raw
            moved = math.hypot(raw_pose.x_m-previous.x_m, raw_pose.y_m-previous.y_m)
            turned = abs(_angle_delta(raw_pose.yaw_degrees, previous.yaw_degrees))
            if moved < self.config.keyframe_translation_m and turned < self.config.keyframe_yaw_degrees:
                return self._update(False, None, False, None, raw_pose, None,
                                    "below_keyframe_threshold")
        if len(self.graph.keyframes) >= self.config.max_keyframes:
            return self._update(False, None, False, None, raw_pose, None,
                                "keyframe_capacity_reached")
        scan = tuple(_finite_xyz(points_xyz))
        index = self.graph.add_keyframe(timestamp_s, raw_pose)
        self._scans.append(scan)
        candidates = self._loop_candidates(index)
        verification = None
        loop_source = None
        for candidate in candidates:
            initial = relative_pose(self.graph.keyframes[candidate].raw, raw_pose)
            verification = verify_loop_constraint(
                self._scans[candidate], scan, initial, **verification_overrides,
            )
            if verification.accepted and verification.relative is not None:
                loop_source = candidate
                self.graph.add_loop_constraint(
                    candidate, index, verification.relative,
                    confidence=max(0.01, min(1.0, verification.fitness)),
                )
                break
        loop_added = loop_source is not None
        result = None
        if loop_added or any(edge.kind == "loop" for edge in self.graph.constraints):
            result = self.graph.optimize()
            if result.accepted:
                self._last_result = result
                self._latest_correction = self.graph.correction(result, timestamp_s)
        reason = ("loop_accepted" if loop_added and result is not None and result.accepted
                  else "loop_optimization_rejected" if loop_added
                  else verification.reason if verification is not None
                  else "keyframe_added")
        return self._update(True, index, loop_added, loop_source, raw_pose,
                            verification, reason, result)

    def corrected_pose(self, raw_pose: PlanarPose) -> PlanarPose:
        return compose_pose(self._latest_correction, raw_pose)

    def _loop_candidates(self, current: int) -> list[int]:
        if len(self.graph.keyframes) < self.config.minimum_keyframes_for_loop:
            return []
        frame = self.graph.keyframes[current]
        ranked = []
        for index, candidate in enumerate(self.graph.keyframes[:current]):
            age = frame.timestamp_s-candidate.timestamp_s
            distance = math.hypot(frame.raw.x_m-candidate.raw.x_m,
                                  frame.raw.y_m-candidate.raw.y_m)
            if age >= self.config.minimum_loop_age_s and distance <= self.config.loop_search_radius_m:
                ranked.append((distance, index))
        return [index for _, index in sorted(ranked)[:self.config.max_loop_candidates_per_keyframe]]

    def _update(self, added, index, loop_added, source, raw, verification,
                reason, result=None):
        return PoseGraphTrackerUpdate(added, index, loop_added, source,
                                      self.corrected_pose(raw), result,
                                      verification, reason)


def compose_pose(parent: PlanarPose, child: PlanarPose) -> PlanarPose:
    yaw = math.radians(parent.yaw_degrees)
    return PlanarPose(parent.x_m+math.cos(yaw)*child.x_m-math.sin(yaw)*child.y_m,
                      parent.y_m+math.sin(yaw)*child.x_m+math.cos(yaw)*child.y_m,
                      _normalize(parent.yaw_degrees+child.yaw_degrees))


def inverse_pose(pose: PlanarPose) -> PlanarPose:
    yaw = math.radians(pose.yaw_degrees)
    return PlanarPose(-math.cos(yaw)*pose.x_m-math.sin(yaw)*pose.y_m,
                      math.sin(yaw)*pose.x_m-math.cos(yaw)*pose.y_m,
                      _normalize(-pose.yaw_degrees))


def relative_pose(source: PlanarPose, target: PlanarPose) -> PlanarPose:
    return compose_pose(inverse_pose(source), target)


def verify_loop_constraint(
    source_points_xyz, target_points_xyz, initial_relative: PlanarPose, *,
    translation_radius_m: float = 0.6, yaw_radius_degrees: float = 20.0,
    translation_step_m: float = 0.1, yaw_step_degrees: float = 5.0,
    max_correspondence_m: float = 0.20, minimum_points: int = 12,
    minimum_fitness: float = 0.55, maximum_rmse_m: float = 0.14,
    minimum_score_margin: float = 0.02, max_candidates: int = 5000,
) -> LoopVerificationResult:
    """Verify a loop edge by bounded planar scan matching near odometry."""
    values = (translation_radius_m, yaw_radius_degrees, translation_step_m,
              yaw_step_degrees, max_correspondence_m)
    if not all(math.isfinite(value) and value > 0 for value in values):
        raise ValueError("invalid loop verification limits")
    source = _downsample_points(source_points_xyz, max_correspondence_m/2, 4000)
    target = _downsample_points(target_points_xyz, max_correspondence_m/2, 4000)
    if len(source) < minimum_points or len(target) < minimum_points:
        return LoopVerificationResult(False, "insufficient_scan_points", None,
                                      0.0, None, 0, 0)
    xs = _axis(initial_relative.x_m, translation_radius_m, translation_step_m)
    ys = _axis(initial_relative.y_m, translation_radius_m, translation_step_m)
    yaws = _axis(initial_relative.yaw_degrees, yaw_radius_degrees, yaw_step_degrees)
    if len(xs)*len(ys)*len(yaws) > max_candidates:
        return LoopVerificationResult(False, "loop_search_budget_exceeded", None,
                                      0.0, None, 0, 0)
    index = _point_index(source, max_correspondence_m)
    ranked = []
    for x in xs:
        for y in ys:
            for yaw in yaws:
                fitness, rmse, inliers = _scan_score(
                    target, index, PlanarPose(x, y, yaw), max_correspondence_m,
                )
                ranked.append((fitness, -rmse, inliers, x, y, yaw))
    ranked.sort(reverse=True)
    best = ranked[0]
    fitness, negative_rmse, inliers, x, y, yaw = best
    rmse = -negative_rmse if inliers else None
    # Compare against a spatially distinct runner-up, not an adjacent grid cell.
    runner = next((item for item in ranked[1:]
                   if math.hypot(item[3]-x, item[4]-y) >= translation_step_m*1.5
                   or abs(_angle_delta(item[5], yaw)) >= yaw_step_degrees*1.5), None)
    margin = fitness-(runner[0] if runner is not None else 0.0)
    accepted = (fitness >= minimum_fitness and rmse is not None
                and rmse <= maximum_rmse_m and inliers >= minimum_points
                and margin >= minimum_score_margin)
    reason = "accepted"
    if fitness < minimum_fitness or inliers < minimum_points:
        reason = "loop_fitness_below_threshold"
    elif rmse is None or rmse > maximum_rmse_m:
        reason = "loop_rmse_above_threshold"
    elif margin < minimum_score_margin:
        reason = "ambiguous_loop_match"
    return LoopVerificationResult(accepted, reason,
                                  PlanarPose(x, y, yaw) if accepted else None,
                                  fitness, rmse, inliers, len(ranked))


def _weighted_pose(values: list[tuple[PlanarPose, float]]) -> PlanarPose:
    if not values:
        raise ValueError("pose graph node has no constraints")
    total = sum(weight for _, weight in values)
    return PlanarPose(sum(p.x_m*w for p, w in values)/total,
                      sum(p.y_m*w for p, w in values)/total,
                      math.degrees(math.atan2(
                          sum(math.sin(math.radians(p.yaw_degrees))*w for p, w in values),
                          sum(math.cos(math.radians(p.yaw_degrees))*w for p, w in values))))


def _interpolate_pose(start: PlanarPose, end: PlanarPose, alpha: float) -> PlanarPose:
    return PlanarPose(start.x_m+(end.x_m-start.x_m)*alpha,
                      start.y_m+(end.y_m-start.y_m)*alpha,
                      _normalize(start.yaw_degrees+_angle_delta(end.yaw_degrees, start.yaw_degrees)*alpha))


def _pose_delta(a: PlanarPose, b: PlanarPose) -> float:
    return max(math.hypot(a.x_m-b.x_m, a.y_m-b.y_m),
               abs(math.radians(_angle_delta(a.yaw_degrees, b.yaw_degrees))))


def _angle_delta(target: float, source: float) -> float:
    return _normalize(target-source)


def _normalize(value: float) -> float:
    return (value+180.0) % 360.0-180.0


def _rejected(reason: str, poses: tuple[PlanarPose, ...], loops: int) -> PoseGraphResult:
    return PoseGraphResult(False, reason, poses, None, None, 0, loops, 0.0, 0.0)


def _downsample_points(points, resolution: float, limit: int):
    cells = {}
    for point in points:
        try:
            x, y, z = map(float, point)
        except (TypeError, ValueError):
            continue
        if all(map(math.isfinite, (x, y, z))) and -0.3 <= z <= 1.8:
            cells.setdefault((math.floor(x/resolution), math.floor(y/resolution)), (x, y))
        if len(cells) >= limit:
            break
    return list(cells.values())


def _finite_xyz(points):
    for point in points:
        try:
            x, y, z = map(float, point)
        except (TypeError, ValueError):
            continue
        if all(map(math.isfinite, (x, y, z))):
            yield x, y, z


def _axis(center: float, radius: float, step: float) -> list[float]:
    count = math.floor(2*radius/step+1e-9)
    return [center-radius+i*step for i in range(count+1)]


def _point_index(points, cell_size: float):
    result = {}
    for x, y in points:
        result.setdefault((math.floor(x/cell_size), math.floor(y/cell_size)), []).append((x, y))
    return result


def _scan_score(points, index, transform: PlanarPose, max_distance: float):
    yaw = math.radians(transform.yaw_degrees)
    cosine, sine = math.cos(yaw), math.sin(yaw)
    squared = []
    limit = max_distance*max_distance
    for px, py in points:
        x = transform.x_m+cosine*px-sine*py
        y = transform.y_m+sine*px+cosine*py
        cell = math.floor(x/max_distance), math.floor(y/max_distance)
        best = float("inf")
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for sx, sy in index.get((cell[0]+dx, cell[1]+dy), ()):
                    best = min(best, (x-sx)**2+(y-sy)**2)
        if best <= limit:
            squared.append(best)
    return (len(squared)/len(points),
            math.sqrt(sum(squared)/len(squared)) if squared else float("inf"),
            len(squared))
