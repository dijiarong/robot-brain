"""Deterministic A* planner for the native Go2 local costmap."""
from __future__ import annotations

import heapq
import math

from robot_brain.navigation.grid import GridCell, OccupancyGrid2D


def astar_path(
    grid: OccupancyGrid2D,
    start_xy: tuple[float, float],
    goal_xy: tuple[float, float],
    *, obstacle_cost_weight: float = 3.0,
) -> list[tuple[float, float]] | None:
    if not math.isfinite(obstacle_cost_weight) or obstacle_cost_weight < 0:
        raise ValueError("obstacle cost weight must be finite and nonnegative")
    start = grid.world_to_cell(*start_xy)
    goal = grid.world_to_cell(*goal_xy)
    if start is None or goal is None or grid.blocked(goal):
        return None
    # Sensor returns on the robot can mark its own footprint; the current cell
    # is always safe because freshness and collision guards are checked outside.
    blocked = set(grid.occupied)
    blocked.discard(start)
    frontier: list[tuple[float, int, GridCell]] = [(0.0, 0, start)]
    came_from: dict[GridCell, GridCell] = {}
    cost: dict[GridCell, float] = {start: 0.0}
    serial = 0
    while frontier:
        _, _, current = heapq.heappop(frontier)
        if current == goal:
            cells = [current]
            while current != start:
                current = came_from[current]
                cells.append(current)
            cells.reverse()
            return _simplify([grid.cell_to_world(cell) for cell in cells])
        for dc, dr, step in _NEIGHBORS:
            nxt = (current[0] + dc, current[1] + dr)
            if grid.blocked(nxt) or nxt in blocked:
                continue
            if dc and dr and (
                (current[0] + dc, current[1]) in blocked
                or (current[0], current[1] + dr) in blocked
            ):
                continue
            candidate = cost[current] + step*(1+obstacle_cost_weight*grid.traversal_cost(nxt)/100)
            if candidate >= cost.get(nxt, float("inf")):
                continue
            cost[nxt] = candidate
            came_from[nxt] = current
            serial += 1
            heuristic = math.hypot(goal[0] - nxt[0], goal[1] - nxt[1])
            heapq.heappush(frontier, (candidate + heuristic, serial, nxt))
    return None


def find_nearest_safe_goal(
    grid: OccupancyGrid2D,
    requested_xy: tuple[float, float],
    *,
    search_radius_m: float = 0.50,
    clearance_m: float = 0.10,
) -> tuple[float, float] | None:
    """Find the nearest free goal cell with required obstacle clearance."""
    requested = grid.world_to_cell(*requested_xy)
    if requested is None:
        return None
    radius = max(0, math.ceil(search_radius_m / grid.resolution_m))
    clearance = max(0, math.ceil(clearance_m / grid.resolution_m))
    candidates: list[tuple[float, GridCell]] = []
    for dx in range(-radius, radius + 1):
        for dy in range(-radius, radius + 1):
            if dx * dx + dy * dy > radius * radius:
                continue
            cell = (requested[0] + dx, requested[1] + dy)
            if grid.blocked(cell) or not _cell_has_clearance(grid, cell, clearance):
                continue
            candidates.append((math.hypot(dx, dy), cell))
    if not candidates:
        return None
    _, selected = min(candidates, key=lambda row: (row[0], row[1]))
    return grid.cell_to_world(selected)


def path_minimum_clearance_m(
    grid: OccupancyGrid2D,
    path: list[tuple[float, float]],
    *,
    maximum_search_m: float = 2.0,
) -> float | None:
    if not path:
        return None
    occupied = set(grid.occupied)
    # The local planner deliberately permits the robot to leave its current
    # cell when inflation touches the footprint. Apply the same narrow
    # exception here; every later path cell and obstacle remains checked.
    start = grid.world_to_cell(*path[0])
    if start is not None:
        occupied.discard(start)
    if not occupied:
        return maximum_search_m
    minimum = maximum_search_m
    for cell in path_corridor_cells(grid, path, robot_width_m=0.0):
        distance_cells = min(
            math.hypot(cell[0] - obstacle[0], cell[1] - obstacle[1])
            for obstacle in occupied
        )
        minimum = min(minimum, distance_cells * grid.resolution_m)
    return minimum


def path_corridor_cells(
    grid: OccupancyGrid2D, path: list[tuple[float, float]], *, robot_width_m: float,
    maximum_length_m: float = math.inf,
) -> frozenset[GridCell]:
    """Rasterize every path segment and dilate by half robot width."""
    if robot_width_m < 0 or maximum_length_m < 0:
        raise ValueError("path corridor dimensions cannot be negative")
    if len(path) < 2:
        return frozenset()
    radius = math.ceil(robot_width_m/2/grid.resolution_m)
    result: set[GridCell] = set()
    traveled = 0.0
    for start_xy, end_xy in zip(path, path[1:]):
        segment = math.dist(start_xy, end_xy)
        if traveled+segment > maximum_length_m:
            break
        start, end = grid.world_to_cell(*start_xy), grid.world_to_cell(*end_xy)
        if start is None or end is None:
            continue
        for center in _bresenham_cells(start, end):
            for dx in range(-radius, radius+1):
                for dy in range(-radius, radius+1):
                    if dx*dx+dy*dy <= radius*radius:
                        cell = center[0]+dx, center[1]+dy
                        if 0 <= cell[0] < grid.width and 0 <= cell[1] < grid.height:
                            result.add(cell)
        traveled += segment
    return frozenset(result)


def path_collision_fraction(grid: OccupancyGrid2D, path: list[tuple[float, float]], *,
                            robot_width_m: float) -> float:
    corridor = path_corridor_cells(grid, path, robot_width_m=robot_width_m)
    return (sum(cell in grid.occupied for cell in corridor)/len(corridor)
            if corridor else 0.0)


def _cell_has_clearance(grid: OccupancyGrid2D, cell: GridCell, radius: int) -> bool:
    return all(
        not grid.blocked((cell[0] + dx, cell[1] + dy))
        for dx in range(-radius, radius + 1)
        for dy in range(-radius, radius + 1)
        if dx * dx + dy * dy <= radius * radius
    )


_NEIGHBORS = tuple(
    (dc, dr, math.sqrt(2.0) if dc and dr else 1.0)
    for dc in (-1, 0, 1)
    for dr in (-1, 0, 1)
    if dc or dr
)


def _simplify(points: list[tuple[float, float]]) -> list[tuple[float, float]]:
    if len(points) <= 2:
        return points
    result = [points[0]]
    previous_direction: tuple[int, int] | None = None
    for index in range(1, len(points)):
        dx = round(points[index][0] - points[index - 1][0], 6)
        dy = round(points[index][1] - points[index - 1][1], 6)
        direction = (_sign(dx), _sign(dy))
        if previous_direction is not None and direction != previous_direction:
            result.append(points[index - 1])
        previous_direction = direction
    result.append(points[-1])
    return result


def _sign(value: float) -> int:
    return 1 if value > 0 else -1 if value < 0 else 0


def _bresenham_cells(start: GridCell, end: GridCell):
    x0, y0 = start
    x1, y1 = end
    dx, dy = abs(x1-x0), -abs(y1-y0)
    sx, sy = (1 if x0 < x1 else -1), (1 if y0 < y1 else -1)
    error = dx+dy
    while True:
        yield x0, y0
        if (x0, y0) == (x1, y1):
            return
        doubled = 2*error
        if doubled >= dy:
            error += dy
            x0 += sx
        if doubled <= dx:
            error += dx
            y0 += sy
