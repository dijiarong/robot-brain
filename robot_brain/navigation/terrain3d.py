"""Dependency-free multi-level surface (MLS) terrain planning.

The implementation deliberately owns its data model.  It extracts standable
surfaces from an occupied voxel cloud, applies headroom/step/slope/edge safety
gates, and supports bounded incremental replacement of a local map region.
"""
from __future__ import annotations

from dataclasses import dataclass, replace
import heapq
import math
import time
from typing import Iterable

Cell3D = tuple[int, int, int]
Point3D = tuple[float, float, float]


class TerrainPlanningBudgetExceeded(RuntimeError):
    pass


@dataclass(frozen=True)
class TerrainPlannerConfig:
    voxel_size_m: float = 0.08
    robot_height_m: float = 0.30
    max_overhead_m: float = 2.0
    surface_closing_radius_m: float = 0.30
    wall_clearance_m: float = 0.10
    wall_buffer_m: float = 0.75
    wall_buffer_weight: float = 100.0
    step_threshold_m: float = 0.16
    step_penalty_weight: float = 4.0
    max_slope_degrees: float = 25.0
    goal_tolerance_m: float = 0.30
    max_search_expansions: int = 100_000

    def __post_init__(self) -> None:
        positive = (self.voxel_size_m, self.robot_height_m, self.goal_tolerance_m)
        if any(value <= 0 for value in positive):
            raise ValueError("voxel size, robot height, and goal tolerance must be positive")
        nonnegative = (
            self.max_overhead_m, self.surface_closing_radius_m,
            self.wall_clearance_m, self.wall_buffer_m, self.wall_buffer_weight,
            self.step_threshold_m, self.step_penalty_weight,
        )
        if any(value < 0 for value in nonnegative):
            raise ValueError("terrain safety parameters cannot be negative")
        if self.wall_buffer_weight and not self.wall_buffer_m:
            raise ValueError("wall_buffer_weight requires wall_buffer_m")
        if not 0 < self.max_slope_degrees < 90:
            raise ValueError("max_slope_degrees must be between 0 and 90")
        if self.max_search_expansions <= 0:
            raise ValueError("terrain search expansion budget must be positive")


@dataclass(frozen=True)
class TerrainRegion:
    origin_x_m: float
    origin_y_m: float
    radius_m: float
    z_min_m: float
    z_max_m: float

    def __post_init__(self) -> None:
        if self.radius_m <= 0 or self.z_max_m < self.z_min_m:
            raise ValueError("invalid terrain update region")

    def contains(self, point: Point3D) -> bool:
        x, y, z = point
        return (
            self.z_min_m <= z <= self.z_max_m
            and math.hypot(x - self.origin_x_m, y - self.origin_y_m) <= self.radius_m
        )


@dataclass(frozen=True)
class SurfaceNode:
    x_m: float
    y_m: float
    z_m: float
    cell: Cell3D
    wall_clearance_m: float = math.inf
    traversal_cost: float = 1.0


@dataclass(frozen=True)
class SurfacePath:
    nodes: tuple[SurfaceNode, ...]
    length_m: float
    elevation_gain_m: float
    cost: float = 0.0
    minimum_clearance_m: float = math.inf


@dataclass(frozen=True)
class TerrainFrontierGoal:
    node: SurfaceNode
    information_gain: int
    distance_m: float
    score: float


@dataclass(frozen=True)
class TerrainPlanState:
    voxel_count: int
    surface_count: int
    generation: int
    update_ms: float
    plan_ms: float
    stop_reason: str | None


@dataclass(frozen=True)
class TerrainMapConfig:
    voxel_size_m: float = 0.10
    sensor_range_m: float = 20.0
    decay_time_s: float = 4.0
    no_decay_distance_m: float = 0.0
    lower_relative_z_m: float = -1.5
    upper_relative_z_m: float = 1.0
    distance_z_ratio: float = 0.10
    local_merge_radius_m: float = 0.5
    max_voxels: int = 250_000

    def __post_init__(self) -> None:
        if self.voxel_size_m <= 0 or self.sensor_range_m <= 0 or self.decay_time_s < 0:
            raise ValueError("invalid terrain map dimensions or decay")
        if self.max_voxels <= 0 or self.upper_relative_z_m <= self.lower_relative_z_m:
            raise ValueError("invalid terrain map bounds")


class RollingTerrainMap:
    """Bounded, decaying world-frame terrain accumulator for CMU-style input."""

    def __init__(self, config: TerrainMapConfig | None = None) -> None:
        self.config = config or TerrainMapConfig()
        self._voxels: dict[Cell3D, tuple[Point3D, float]] = {}

    def update(self, points_xyz: Iterable[Point3D], pose_xyz: Point3D, timestamp_s: float,
               *, local_terrain: Iterable[Point3D] = ()) -> tuple[Point3D, ...]:
        if not math.isfinite(timestamp_s):
            raise ValueError("terrain timestamp must be finite")
        px, py, pz = pose_xyz
        self._evict(timestamp_s, pose_xyz)
        for point in points_xyz:
            x, y, z = (float(value) for value in point)
            if not all(math.isfinite(value) for value in (x, y, z)):
                continue
            distance = math.hypot(x - px, y - py)
            relative_z = z - pz
            if distance > self.config.sensor_range_m:
                continue
            lower = self.config.lower_relative_z_m - self.config.distance_z_ratio * distance
            upper = self.config.upper_relative_z_m + self.config.distance_z_ratio * distance
            if not lower < relative_z < upper:
                continue
            self._put((x, y, z), timestamp_s)
        # The analyzed near-field terrain replaces raw returns inside its merge
        # radius, matching the extended-map responsibility split.
        local = tuple(tuple(float(v) for v in point) for point in local_terrain)
        if local:
            radius = self.config.local_merge_radius_m
            for cell, (point, _) in tuple(self._voxels.items()):
                if math.hypot(point[0] - px, point[1] - py) <= radius:
                    self._voxels.pop(cell, None)
            for point in local:
                if len(point) == 3 and all(math.isfinite(v) for v in point):
                    self._put(point, timestamp_s)  # type: ignore[arg-type]
        self._limit_capacity(pose_xyz)
        return self.points

    def clear_within(self, pose_xyz: Point3D, radius_m: float) -> int:
        if radius_m < 0:
            raise ValueError("clear radius cannot be negative")
        px, py, _ = pose_xyz
        keys = [cell for cell, (point, _) in self._voxels.items()
                if math.hypot(point[0] - px, point[1] - py) <= radius_m]
        for cell in keys:
            self._voxels.pop(cell, None)
        return len(keys)

    @property
    def points(self) -> tuple[Point3D, ...]:
        return tuple(point for point, _ in self._voxels.values())

    def _put(self, point: Point3D, timestamp_s: float) -> None:
        size = self.config.voxel_size_m
        cell = tuple(math.floor(value / size) for value in point)
        previous = self._voxels.get(cell)  # type: ignore[arg-type]
        if previous is None:
            self._voxels[cell] = (point, timestamp_s)  # type: ignore[index]
        else:
            old, _ = previous
            self._voxels[cell] = (tuple((a + b) / 2 for a, b in zip(old, point)), timestamp_s)  # type: ignore[index]

    def _evict(self, timestamp_s: float, pose_xyz: Point3D) -> None:
        px, py, _ = pose_xyz
        keep_near = self.config.no_decay_distance_m
        stale = [cell for cell, (point, seen) in self._voxels.items()
                 if timestamp_s - seen > self.config.decay_time_s
                 and math.hypot(point[0] - px, point[1] - py) > keep_near]
        for cell in stale:
            self._voxels.pop(cell, None)

    def _limit_capacity(self, pose_xyz: Point3D) -> None:
        excess = len(self._voxels) - self.config.max_voxels
        if excess <= 0:
            return
        px, py, _ = pose_xyz
        victims = sorted(self._voxels, key=lambda cell: (
            self._voxels[cell][1],
            -math.hypot(self._voxels[cell][0][0] - px, self._voxels[cell][0][1] - py),
        ))[:excess]
        for cell in victims:
            self._voxels.pop(cell, None)


def build_surface_graph(
    points_xyz: tuple[Point3D, ...],
    *,
    resolution_m: float = 0.20,
    layer_height_m: float = 0.10,
    robot_height_m: float = 0.0,
    wall_clearance_m: float = 0.0,
    wall_buffer_m: float = 0.0,
    wall_buffer_weight: float = 0.0,
    surface_closing_radius_m: float = 0.0,
) -> dict[Cell3D, SurfaceNode]:
    """Build a multi-layer graph while retaining the historical public API.

    With ``robot_height_m`` set, only the top occupied voxel before sufficient
    vertical free space is a surface. Occupied columns still retain multiple
    surfaces (floor, landing, bridge) when headroom exists between them.
    """
    if resolution_m <= 0 or layer_height_m <= 0 or robot_height_m < 0:
        raise ValueError("surface graph dimensions must be positive")
    buckets: dict[Cell3D, list[Point3D]] = {}
    for point in points_xyz:
        if not all(math.isfinite(value) for value in point):
            continue
        x, y, z = point
        key = (math.floor(x / resolution_m), math.floor(y / resolution_m), math.floor(z / layer_height_m))
        buckets.setdefault(key, []).append(point)

    headroom_cells = math.ceil(robot_height_m / layer_height_m)
    by_column: dict[tuple[int, int], list[int]] = {}
    for ix, iy, iz in buckets:
        by_column.setdefault((ix, iy), []).append(iz)
    surface_keys: set[Cell3D] = set()
    for (ix, iy), levels in by_column.items():
        levels.sort()
        for index, iz in enumerate(levels):
            if index == len(levels) - 1 or levels[index + 1] - iz > headroom_cells:
                surface_keys.add((ix, iy, iz))

    if surface_closing_radius_m > 0:
        surface_keys = _close_surface_holes(
            surface_keys, by_column,
            passes=math.ceil(surface_closing_radius_m / resolution_m),
            headroom_cells=headroom_cells,
        )

    # A column is a wall/overhang hazard when occupied material spans enough
    # vertical cells to intrude into the robot envelope. A lone return is floor
    # support and must not be mistaken for a wall.
    obstacle_xy = {
        column for column, levels in by_column.items()
        if len(levels) > 1 and max(levels) - min(levels) <= headroom_cells
    }
    surface_levels_by_xy: dict[tuple[int, int], set[int]] = {}
    for sx, sy, sz in surface_keys:
        surface_levels_by_xy.setdefault((sx, sy), set()).add(sz)
    safety_radius = wall_clearance_m + wall_buffer_m
    graph: dict[Cell3D, SurfaceNode] = {}
    for key in surface_keys:
        values = buckets.get(key)
        if values:
            x = sum(p[0] for p in values) / len(values)
            y = sum(p[1] for p in values) / len(values)
            z = sum(p[2] for p in values) / len(values)
        else:
            x = (key[0] + 0.5) * resolution_m
            y = (key[1] + 0.5) * resolution_m
            z = (key[2] + 0.5) * layer_height_m
        clearance = _surface_clearance(
            key, surface_levels_by_xy, obstacle_xy, resolution_m, safety_radius,
        )
        # The source column is support, not a wall; clearance looks outward.
        if not math.isfinite(clearance):
            penalty = 1.0
        elif clearance <= wall_clearance_m:
            penalty = math.inf
        elif safety_radius > wall_clearance_m and clearance < safety_radius:
            ratio = (safety_radius - clearance) / wall_buffer_m
            penalty = 1.0 + wall_buffer_weight * ratio
        else:
            penalty = 1.0
        graph[key] = SurfaceNode(x, y, z, key, clearance, penalty)
    return graph


class MultiLevelTerrainPlanner:
    """Stateful MLS planner with global rebuild and bounded regional updates."""

    def __init__(self, config: TerrainPlannerConfig | None = None) -> None:
        self.config = config or TerrainPlannerConfig()
        self._voxels: set[Cell3D] = set()
        self._points: dict[Cell3D, Point3D] = {}
        self._graph: dict[Cell3D, SurfaceNode] = {}
        self._generation = 0
        self._last_path: SurfacePath | None = None
        self._last_goal: Point3D | None = None
        self._update_ms = 0.0
        self._plan_ms = 0.0
        self._stop_reason: str | None = "map_empty"
        self._navigation_boundary: tuple[tuple[float, float], ...] | None = None
        self._added_obstacles: tuple[Point3D, ...] = ()
        self._added_obstacle_radius_m = 0.30

    @property
    def graph(self) -> dict[Cell3D, SurfaceNode]:
        return dict(self._graph)

    @property
    def state(self) -> TerrainPlanState:
        return TerrainPlanState(len(self._voxels), len(self._graph), self._generation,
                                self._update_ms, self._plan_ms, self._stop_reason)

    def update_global_map(self, points_xyz: Iterable[Point3D]) -> None:
        started = time.perf_counter()
        self._replace_all(points_xyz)
        self._rebuild()
        self._update_ms = (time.perf_counter() - started) * 1000.0

    def set_navigation_boundary(self, polygon_xy: Iterable[tuple[float, float]] | None) -> None:
        """Restrict traversable surfaces to a finite simple polygon."""
        if polygon_xy is None:
            self._navigation_boundary = None
        else:
            polygon = tuple((float(x), float(y)) for x, y in polygon_xy)
            if len(polygon) < 3 or not all(math.isfinite(v) for point in polygon for v in point):
                raise ValueError("navigation boundary requires at least three finite vertices")
            if abs(_polygon_area(polygon)) <= 1e-9 or _polygon_self_intersects(polygon):
                raise ValueError("navigation boundary must be a non-self-intersecting polygon")
            self._navigation_boundary = polygon
        self._rebuild()

    def set_added_obstacles(self, points_xyz: Iterable[Point3D], *, radius_m: float = 0.30) -> None:
        """Atomically replace operator/perception supplied obstacle overlays."""
        if not math.isfinite(radius_m) or radius_m < 0:
            raise ValueError("added obstacle radius must be finite and nonnegative")
        obstacles = []
        for point in points_xyz:
            try:
                value = tuple(float(v) for v in point)
            except (TypeError, ValueError):
                continue
            if len(value) == 3 and all(math.isfinite(v) for v in value):
                obstacles.append(value)
        self._added_obstacles = tuple(obstacles)  # type: ignore[assignment]
        self._added_obstacle_radius_m = radius_m
        self._rebuild()

    def update_region(self, points_xyz: Iterable[Point3D], region: TerrainRegion,
                      *, sensor_z_m: float | None = None) -> None:
        """Atomically replace voxels in a cylindrical local-map region."""
        started = time.perf_counter()
        ceiling = region.z_max_m
        if sensor_z_m is not None:
            ceiling = min(ceiling, sensor_z_m + self.config.max_overhead_m)
        bounded = replace(region, z_max_m=ceiling)
        remove = [cell for cell, point in self._points.items() if bounded.contains(point)]
        for cell in remove:
            self._voxels.discard(cell)
            self._points.pop(cell, None)
        self._insert(points_xyz, bounded)
        self._rebuild()
        self._update_ms = (time.perf_counter() - started) * 1000.0

    def plan(self, start_xyz: Point3D, goal_xyz: Point3D) -> SurfacePath | None:
        started = time.perf_counter()
        try:
            path = plan_surface_path(
                self._graph, start_xyz, goal_xyz,
                max_step_height_m=self.config.step_threshold_m,
                max_slope_degrees=self.config.max_slope_degrees,
                max_endpoint_distance_m=self.config.goal_tolerance_m,
                step_penalty_weight=self.config.step_penalty_weight,
                max_expansions=self.config.max_search_expansions,
            )
        except TerrainPlanningBudgetExceeded:
            self._plan_ms = (time.perf_counter() - started) * 1000.0
            self._stop_reason = "terrain_search_budget_exceeded"
            return None
        self._plan_ms = (time.perf_counter() - started) * 1000.0
        if path is not None:
            self._last_path, self._last_goal = path, goal_xyz
            self._stop_reason = None
            return path
        self._stop_reason = "no_traversable_surface_path"
        return self._safe_prefix(start_xyz, goal_xyz)

    def frontier_goals(self, robot_xyz: Point3D, *, exploration_range_m: float = 20.0,
                       minimum_information_gain: int = 2,
                       max_goals: int = 20,
                       visited_xy: Iterable[tuple[float, float]] = (),
                       visited_radius_m: float = 0.75) -> tuple[TerrainFrontierGoal, ...]:
        """Rank safe MLS boundary nodes by unknown surface coverage and travel cost."""
        if not math.isfinite(exploration_range_m) or exploration_range_m <= 0:
            raise ValueError("exploration range must be positive and finite")
        if minimum_information_gain <= 0 or max_goals <= 0 or visited_radius_m < 0:
            raise ValueError("invalid terrain frontier limits")
        visited = tuple((float(x), float(y)) for x, y in visited_xy)
        by_xy = {(node.cell[0], node.cell[1]) for node in self._graph.values()
                 if math.isfinite(node.traversal_cost)}
        candidates = []
        for node in self._graph.values():
            if not math.isfinite(node.traversal_cost):
                continue
            distance = math.dist((node.x_m, node.y_m, node.z_m), robot_xyz)
            if distance > exploration_range_m or any(
                math.hypot(node.x_m-x, node.y_m-y) <= visited_radius_m for x, y in visited
            ):
                continue
            unknown = sum((node.cell[0]+dx, node.cell[1]+dy) not in by_xy
                          for dx in range(-2, 3) for dy in range(-2, 3)
                          if dx or dy)
            if unknown < minimum_information_gain:
                continue
            # Prefer information-rich, close, high-clearance surfaces.
            clearance = (2.0 if not math.isfinite(node.wall_clearance_m)
                         else min(2.0, node.wall_clearance_m))
            score = unknown*2.0+clearance-distance-node.traversal_cost*.05
            candidates.append(TerrainFrontierGoal(node, unknown, distance, score))
        candidates.sort(key=lambda item: (-item.score, item.distance_m, item.node.cell))
        return tuple(candidates[:max_goals])

    def _safe_prefix(self, start: Point3D, goal: Point3D) -> SurfacePath | None:
        """Return only the still-valid prefix of the last path for the same goal."""
        if self._last_path is None or self._last_goal is None:
            return None
        if math.dist(self._last_goal, goal) > self.config.goal_tolerance_m:
            return None
        old = self._last_path.nodes
        resume = min(range(len(old)), key=lambda index: math.dist(
            (old[index].x_m, old[index].y_m, old[index].z_m), start))
        valid: list[SurfaceNode] = []
        blocked = False
        for node in old[resume:]:
            current = self._graph.get(node.cell)
            if current is None or not math.isfinite(current.traversal_cost):
                blocked = True
                break
            if valid and not _edge_is_safe(
                valid[-1], current, self.config.step_threshold_m,
                self.config.max_slope_degrees,
            ):
                blocked = True
                break
            valid.append(current)
        if len(valid) < 2:
            return None
        if blocked:
            valid = _back_off_nodes(valid, 1.0)
            if len(valid) < 2:
                return None
        nodes = tuple(valid)
        return _path_result(nodes, 0.0)

    def _replace_all(self, points: Iterable[Point3D]) -> None:
        self._voxels.clear()
        self._points.clear()
        self._insert(points, None)

    def _insert(self, points: Iterable[Point3D], region: TerrainRegion | None) -> None:
        size = self.config.voxel_size_m
        for raw in points:
            point = tuple(float(v) for v in raw)
            if len(point) != 3 or not all(math.isfinite(v) for v in point):
                continue
            if region is not None and not region.contains(point):
                continue
            cell = tuple(math.floor(v / size) for v in point)
            self._voxels.add(cell)  # type: ignore[arg-type]
            self._points[cell] = point  # type: ignore[index]

    def _rebuild(self) -> None:
        cfg = self.config
        self._graph = build_surface_graph(
            tuple(self._points.values()), resolution_m=cfg.voxel_size_m,
            layer_height_m=cfg.voxel_size_m, robot_height_m=cfg.robot_height_m,
            wall_clearance_m=cfg.wall_clearance_m,
            wall_buffer_m=cfg.wall_buffer_m, wall_buffer_weight=cfg.wall_buffer_weight,
            surface_closing_radius_m=cfg.surface_closing_radius_m,
        )
        if self._navigation_boundary is not None:
            polygon = self._navigation_boundary
            self._graph = {cell: node for cell, node in self._graph.items()
                           if _point_in_polygon_or_edge((node.x_m, node.y_m), polygon)}
        if self._added_obstacles:
            radius = self._added_obstacle_radius_m
            self._graph = {
                cell: node for cell, node in self._graph.items()
                if not any(math.hypot(node.x_m-x, node.y_m-y) <= radius
                           and abs(node.z_m-z) <= self.config.robot_height_m
                           for x, y, z in self._added_obstacles)
            }
        self._generation += 1
        self._stop_reason = None if self._graph else "map_empty"


def plan_surface_path(
    graph: dict[Cell3D, SurfaceNode],
    start_xyz: Point3D,
    goal_xyz: Point3D,
    *,
    max_step_height_m: float = 0.20,
    max_slope_degrees: float = 25.0,
    max_endpoint_distance_m: float = 0.50,
    step_penalty_weight: float = 0.0,
    max_expansions: int = 100_000,
) -> SurfacePath | None:
    if max_expansions <= 0:
        raise ValueError("terrain max_expansions must be positive")
    if not graph or any(value < 0 for value in (max_step_height_m, step_penalty_weight)):
        return None
    start = _nearest(graph, start_xyz, max_endpoint_distance_m)
    goal = _nearest(graph, goal_xyz, max_endpoint_distance_m)
    if start is None or goal is None or not math.isfinite(start.traversal_cost) or not math.isfinite(goal.traversal_cost):
        return None
    frontier: list[tuple[float, int, Cell3D]] = [(0.0, 0, start.cell)]
    by_xy: dict[tuple[int, int], list[SurfaceNode]] = {}
    for node in graph.values():
        by_xy.setdefault((node.cell[0], node.cell[1]), []).append(node)
    costs = {start.cell: 0.0}
    came_from: dict[Cell3D, Cell3D] = {}
    serial = 0
    expansions = 0
    while frontier:
        expansions += 1
        if expansions > max_expansions:
            raise TerrainPlanningBudgetExceeded("terrain A* expansion budget exceeded")
        _, _, current_key = heapq.heappop(frontier)
        if current_key == goal.cell:
            keys = [current_key]
            while keys[-1] != start.cell:
                keys.append(came_from[keys[-1]])
            keys.reverse()
            return _path_result(tuple(graph[key] for key in keys), costs[current_key])
        current = graph[current_key]
        for neighbor in _neighbors(by_xy, current, max_step_height_m, max_slope_degrees):
            vertical = max(0.0, neighbor.z_m - current.z_m)
            step = _distance(current, neighbor) * neighbor.traversal_cost + vertical * step_penalty_weight
            candidate = costs[current_key] + step
            if candidate >= costs.get(neighbor.cell, math.inf):
                continue
            costs[neighbor.cell] = candidate
            came_from[neighbor.cell] = current_key
            serial += 1
            heapq.heappush(frontier, (candidate + _distance(neighbor, goal), serial, neighbor.cell))
    return None


def _path_result(nodes: tuple[SurfaceNode, ...], cost: float) -> SurfacePath:
    return SurfacePath(
        nodes, sum(_distance(a, b) for a, b in zip(nodes, nodes[1:])),
        sum(max(0.0, b.z_m - a.z_m) for a, b in zip(nodes, nodes[1:])), cost,
        min((node.wall_clearance_m for node in nodes), default=math.inf),
    )


def _neighbors(by_xy: dict[tuple[int, int], list[SurfaceNode]], node: SurfaceNode,
               max_step: float, max_slope: float) -> list[SurfaceNode]:
    result: list[SurfaceNode] = []
    ix, iy, _ = node.cell
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            if dx == 0 and dy == 0:
                continue
            for candidate in by_xy.get((ix + dx, iy + dy), ()):
                if not math.isfinite(candidate.traversal_cost):
                    continue
                horizontal = math.hypot(candidate.x_m - node.x_m, candidate.y_m - node.y_m)
                vertical = abs(candidate.z_m - node.z_m)
                if horizontal <= 1e-6 or vertical > max_step:
                    continue
                if math.degrees(math.atan2(vertical, horizontal)) <= max_slope:
                    result.append(candidate)
    return result


def _surface_clearance(cell: Cell3D, levels_by_xy: dict[tuple[int, int], set[int]],
                       occupied_xy: set[tuple[int, int]], resolution: float,
                       search_radius_m: float) -> float:
    ix, iy, _ = cell
    if search_radius_m <= 0:
        return math.inf
    radius = math.ceil(search_radius_m / resolution)
    # Missing surface beyond a short sensor hole is a cliff/map edge. Tall
    # occupied columns are walls. Both seed the same clearance distance field.
    wall = _is_real_edge(cell, levels_by_xy) or (ix, iy) in occupied_xy
    if wall:
        return 0.0
    distances = []
    for dx in range(-radius, radius + 1):
        for dy in range(-radius, radius + 1):
            if not (dx or dy):
                continue
            levels = levels_by_xy.get((ix + dx, iy + dy), ())
            if (ix + dx, iy + dy) in occupied_xy or any(
                _is_real_edge((ix + dx, iy + dy, level), levels_by_xy) for level in levels
            ):
                distances.append(math.hypot(dx, dy) * resolution)
    return min(distances, default=math.inf)


def _is_real_edge(cell: Cell3D, levels_by_xy: dict[tuple[int, int], set[int]],
                  hole_span: int = 4) -> bool:
    ix, iy, iz = cell
    for dx, dy in ((-1, 0), (1, 0), (0, -1), (0, 1)):
        if any(abs(level - iz) <= 1 for level in levels_by_xy.get((ix + dx, iy + dy), ())):
            continue
        found = False
        for distance in range(2, hole_span + 1):
            if any(abs(level - iz) <= 1 for level in
                   levels_by_xy.get((ix + dx * distance, iy + dy * distance), ())):
                found = True
                break
        if not found:
            return True
    return False


def _close_surface_holes(surfaces: set[Cell3D], by_column: dict[tuple[int, int], list[int]],
                         *, passes: int, headroom_cells: int) -> set[Cell3D]:
    """Conservative closing: fill enclosed one-cell holes with nearby support."""
    result = set(surfaces)
    for _ in range(max(0, passes)):
        candidates: set[Cell3D] = set()
        for ix, iy, iz in result:
            for dx, dy in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                candidate = (ix + dx, iy + dy, iz)
                if candidate not in result:
                    candidates.add(candidate)
        additions = set()
        for ix, iy, iz in candidates:
            neighbors = sum(
                any(c[0] == ix + dx and c[1] == iy + dy and abs(c[2] - iz) <= 1
                    for c in result)
                for dx, dy in ((-1, 0), (1, 0), (0, -1), (0, 1))
            )
            support = any(
                abs(level - iz) <= 3
                for cx in range(ix - 3, ix + 4)
                for cy in range(iy - 3, iy + 4)
                if (cx - ix) ** 2 + (cy - iy) ** 2 <= 9
                for level in by_column.get((cx, cy), ())
            )
            occupied_above = any(iz < level <= iz + headroom_cells
                                 for level in by_column.get((ix, iy), ()))
            if neighbors == 4 and support and not occupied_above:
                additions.add((ix, iy, iz))
        if not additions:
            break
        result.update(additions)
    return result


def _edge_is_safe(left: SurfaceNode, right: SurfaceNode,
                  max_step: float, max_slope: float) -> bool:
    horizontal = math.hypot(right.x_m-left.x_m, right.y_m-left.y_m)
    vertical = abs(right.z_m-left.z_m)
    return (horizontal > 1e-6 and vertical <= max_step
            and math.degrees(math.atan2(vertical, horizontal)) <= max_slope
            and math.isfinite(right.traversal_cost))


def _back_off_nodes(nodes: list[SurfaceNode], distance_m: float) -> list[SurfaceNode]:
    remaining = distance_m
    end = len(nodes) - 1
    while end > 0:
        segment = _distance(nodes[end - 1], nodes[end])
        if remaining < segment:
            break
        remaining -= segment
        end -= 1
    return nodes[:end + 1]


def _nearest(graph: dict[Cell3D, SurfaceNode], point: Point3D, maximum: float) -> SurfaceNode | None:
    candidates = [node for node in graph.values() if math.isfinite(node.traversal_cost)]
    if not candidates:
        return None
    node = min(candidates, key=lambda value: math.dist((value.x_m, value.y_m, value.z_m), point))
    return node if math.dist((node.x_m, node.y_m, node.z_m), point) <= maximum else None


def _polygon_area(polygon) -> float:
    return 0.5*sum(x1*y2-x2*y1 for (x1, y1), (x2, y2) in
                   zip(polygon, polygon[1:]+polygon[:1]))


def _point_in_polygon_or_edge(point, polygon) -> bool:
    x, y = point
    inside = False
    for (x1, y1), (x2, y2) in zip(polygon, polygon[1:]+polygon[:1]):
        cross = (x-x1)*(y2-y1)-(y-y1)*(x2-x1)
        if (abs(cross) <= 1e-9 and min(x1, x2)-1e-9 <= x <= max(x1, x2)+1e-9
                and min(y1, y2)-1e-9 <= y <= max(y1, y2)+1e-9):
            return True
        if (y1 > y) != (y2 > y):
            if x < x1+(y-y1)*(x2-x1)/(y2-y1):
                inside = not inside
    return inside


def _polygon_self_intersects(polygon) -> bool:
    edges = list(zip(polygon, polygon[1:]+polygon[:1]))
    for i, first in enumerate(edges):
        for j, second in enumerate(edges):
            if j <= i+1 or (i == 0 and j == len(edges)-1):
                continue
            if _segments_intersect(first[0], first[1], second[0], second[1]):
                return True
    return False


def _segments_intersect(a, b, c, d) -> bool:
    def orientation(p, q, r):
        return (q[0]-p[0])*(r[1]-p[1])-(q[1]-p[1])*(r[0]-p[0])
    values = (orientation(a, b, c), orientation(a, b, d),
              orientation(c, d, a), orientation(c, d, b))
    return values[0]*values[1] < 0 and values[2]*values[3] < 0


def _distance(left: SurfaceNode, right: SurfaceNode) -> float:
    return math.dist((left.x_m, left.y_m, left.z_m), (right.x_m, right.y_m, right.z_m))
