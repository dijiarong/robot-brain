"""Small dependency-free 2-D costmap used by native Go2 navigation."""
from __future__ import annotations

from dataclasses import dataclass
import heapq
import math

from robot_brain.perception.pointcloud import PointCloudSnapshot

GridCell = tuple[int, int]


@dataclass(frozen=True)
class OccupancyGrid2D:
    resolution_m: float
    width: int
    height: int
    origin_x_m: float
    origin_y_m: float
    occupied: frozenset[GridCell]
    raw_occupied: frozenset[GridCell] = frozenset()
    known_free: frozenset[GridCell] = frozenset()
    traversal_cost_values: tuple[int, ...] = ()
    frame_id: str = "base_link"

    def world_to_cell(self, x_m: float, y_m: float) -> GridCell | None:
        col = math.floor((x_m - self.origin_x_m) / self.resolution_m)
        row = math.floor((y_m - self.origin_y_m) / self.resolution_m)
        if 0 <= col < self.width and 0 <= row < self.height:
            return (col, row)
        return None

    def cell_to_world(self, cell: GridCell) -> tuple[float, float]:
        col, row = cell
        return (
            self.origin_x_m + (col + 0.5) * self.resolution_m,
            self.origin_y_m + (row + 0.5) * self.resolution_m,
        )

    def blocked(self, cell: GridCell) -> bool:
        col, row = cell
        return not (0 <= col < self.width and 0 <= row < self.height) or cell in self.occupied

    def traversal_cost(self, cell: GridCell) -> int:
        col, row = cell
        index = row*self.width+col
        return self.traversal_cost_values[index] if 0 <= index < len(self.traversal_cost_values) else 0


def costmap_from_pointcloud(
    cloud: PointCloudSnapshot,
    *,
    size_m: float = 6.0,
    resolution_m: float = 0.10,
    robot_radius_m: float = 0.30,
    obstacle_min_z_m: float = 0.05,
    obstacle_max_z_m: float = 1.20,
    obstacle_cost_radius_m: float = 1.0,
) -> OccupancyGrid2D:
    """Project a trusted body-frame cloud and inflate obstacles for Go2."""
    if cloud.frame_id not in {"base", "base_link", "unitree_lidar", "utlidar"}:
        raise ValueError(f"point cloud is not robot-relative: {cloud.frame_id}")
    if size_m <= 0 or resolution_m <= 0 or robot_radius_m < 0 or obstacle_cost_radius_m < 0:
        raise ValueError("invalid costmap geometry")
    cells = max(3, math.ceil(size_m / resolution_m))
    origin = -cells * resolution_m / 2.0
    occupied_raw: set[GridCell] = set()
    free_raw: set[GridCell] = set()
    sensor_cell = (cells // 2, cells // 2)
    for x, y, z in cloud.points_xyz:
        if not all(math.isfinite(v) for v in (x, y, z)):
            continue
        if not obstacle_min_z_m <= z <= obstacle_max_z_m:
            continue
        col = math.floor((x - origin) / resolution_m)
        row = math.floor((y - origin) / resolution_m)
        if 0 <= col < cells and 0 <= row < cells:
            occupied_raw.add((col, row))
            free_raw.update(_bresenham(sensor_cell, (col, row))[:-1])

    inflation = math.ceil(robot_radius_m / resolution_m)
    occupied: set[GridCell] = set()
    for col, row in occupied_raw:
        for dc in range(-inflation, inflation + 1):
            for dr in range(-inflation, inflation + 1):
                if dc * dc + dr * dr > inflation * inflation:
                    continue
                candidate = (col + dc, row + dr)
                if 0 <= candidate[0] < cells and 0 <= candidate[1] < cells:
                    occupied.add(candidate)
    costs = _distance_cost_values(cells, cells, occupied,
                                  math.ceil(obstacle_cost_radius_m/resolution_m))
    return OccupancyGrid2D(
        resolution_m=resolution_m,
        width=cells,
        height=cells,
        origin_x_m=origin,
        origin_y_m=origin,
        occupied=frozenset(occupied),
        raw_occupied=frozenset(occupied_raw),
        known_free=frozenset(free_raw - occupied),
        traversal_cost_values=tuple(costs),
    )


def with_obstacle_distance_costs(
    grid: OccupancyGrid2D, *, maximum_distance_m: float = 1.0,
) -> OccupancyGrid2D:
    if not math.isfinite(maximum_distance_m) or maximum_distance_m < 0:
        raise ValueError("maximum obstacle cost distance must be finite and nonnegative")
    values = _distance_cost_values(
        grid.width, grid.height, set(grid.occupied),
        math.ceil(maximum_distance_m/grid.resolution_m),
    )
    return OccupancyGrid2D(
        resolution_m=grid.resolution_m, width=grid.width, height=grid.height,
        origin_x_m=grid.origin_x_m, origin_y_m=grid.origin_y_m,
        occupied=grid.occupied, raw_occupied=grid.raw_occupied,
        known_free=grid.known_free,
        traversal_cost_values=tuple(values), frame_id=grid.frame_id,
    )


def _bresenham(start: GridCell, end: GridCell) -> list[GridCell]:
    x0, y0 = start
    x1, y1 = end
    dx, dy = abs(x1 - x0), -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    error = dx + dy
    result: list[GridCell] = []
    while True:
        result.append((x0, y0))
        if x0 == x1 and y0 == y1:
            return result
        doubled = 2 * error
        if doubled >= dy:
            error += dy
            x0 += sx
        if doubled <= dx:
            error += dx
            y0 += sy


def _distance_cost_values(width: int, height: int, occupied: set[GridCell], radius: int):
    costs = [0]*(width*height)
    if not occupied or radius <= 0:
        return costs
    # Bounded multi-source Dijkstra keeps persistent-map cost construction
    # proportional to the affected cells instead of O(cells * obstacles).
    distances = [math.inf]*(width*height)
    frontier: list[tuple[float, int, int]] = []
    for col, row in occupied:
        if not (0 <= col < width and 0 <= row < height):
            continue
        distances[row*width+col] = 0.0
        heapq.heappush(frontier, (0.0, col, row))
    neighbors = (
        (dc, dr, math.sqrt(2.0) if dc and dr else 1.0)
        for dc in (-1, 0, 1) for dr in (-1, 0, 1) if dc or dr
    )
    neighbors = tuple(neighbors)
    while frontier:
        distance, col, row = heapq.heappop(frontier)
        if distance != distances[row*width+col] or distance > radius:
            continue
        if distance > 0:
            costs[row*width+col] = max(
                1, min(99, round(99*(1-distance/radius)))
            )
        for dc, dr, step in neighbors:
            next_col, next_row = col+dc, row+dr
            next_distance = distance+step
            if (next_distance > radius or not (0 <= next_col < width)
                    or not (0 <= next_row < height)):
                continue
            index = next_row*width+next_col
            if next_distance >= distances[index]:
                continue
            distances[index] = next_distance
            heapq.heappush(frontier, (next_distance, next_col, next_row))
    return costs
