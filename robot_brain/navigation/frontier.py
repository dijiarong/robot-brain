"""Wavefront-style frontier extraction without DIMOS runtime dependencies."""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math

from robot_brain.navigation.grid import GridCell, OccupancyGrid2D


@dataclass(frozen=True)
class FrontierGoal:
    x_m: float
    y_m: float
    cell_count: int
    distance_m: float
    score: float


def find_frontier_goals(
    grid: OccupancyGrid2D,
    robot_xy: tuple[float, float] = (0.0, 0.0),
    *,
    min_frontier_length_m: float = 0.5,
    obstacle_clearance_m: float = 0.30,
    visited: tuple[tuple[float, float], ...] = (),
    visited_radius_m: float = 0.50,
) -> list[FrontierGoal]:
    """Return reachable free-space boundaries adjacent to unknown cells."""
    start = grid.world_to_cell(*robot_xy)
    if start is None or start not in grid.known_free:
        return []
    reachable = _reachable_free(grid, start)
    candidates = {
        cell for cell in reachable
        if any(neighbor not in grid.known_free and neighbor not in grid.occupied
               and not grid.blocked(neighbor) for neighbor in _neighbors4(cell))
    }
    minimum_cells = max(1, math.ceil(min_frontier_length_m / grid.resolution_m))
    clearance_cells = math.ceil(obstacle_clearance_m / grid.resolution_m)
    groups = _components(candidates)
    goals: list[FrontierGoal] = []
    for group in groups:
        if len(group) < minimum_cells:
            continue
        safe = [cell for cell in group if _clear_of_obstacles(grid, cell, clearance_cells)]
        if not safe:
            continue
        safe = [
            cell for cell in safe
            if not any(
                math.hypot(
                    grid.cell_to_world(cell)[0] - vx,
                    grid.cell_to_world(cell)[1] - vy,
                ) <= visited_radius_m
                for vx, vy in visited
            )
        ]
        if not safe:
            continue
        center = min(safe, key=lambda cell: _centroid_distance(cell, safe))
        x, y = grid.cell_to_world(center)
        distance = math.hypot(x - robot_xy[0], y - robot_xy[1])
        information_gain = len(group) * grid.resolution_m
        score = information_gain / max(distance, grid.resolution_m)
        goals.append(FrontierGoal(x, y, len(group), distance, score))
    return sorted(goals, key=lambda goal: (-goal.score, goal.distance_m, goal.x_m, goal.y_m))


def _reachable_free(grid: OccupancyGrid2D, start: GridCell) -> set[GridCell]:
    queue = deque([start])
    reached = {start}
    while queue:
        cell = queue.popleft()
        for neighbor in _neighbors4(cell):
            if neighbor in reached or neighbor not in grid.known_free:
                continue
            reached.add(neighbor)
            queue.append(neighbor)
    return reached


def _components(cells: set[GridCell]) -> list[list[GridCell]]:
    remaining = set(cells)
    result: list[list[GridCell]] = []
    while remaining:
        seed = min(remaining)
        remaining.remove(seed)
        group = [seed]
        queue = deque([seed])
        while queue:
            for neighbor in _neighbors8(queue.popleft()):
                if neighbor in remaining:
                    remaining.remove(neighbor)
                    group.append(neighbor)
                    queue.append(neighbor)
        result.append(group)
    return result


def _clear_of_obstacles(grid: OccupancyGrid2D, cell: GridCell, radius: int) -> bool:
    return all(
        (cell[0] + dx, cell[1] + dy) not in grid.occupied
        for dx in range(-radius, radius + 1)
        for dy in range(-radius, radius + 1)
        if dx * dx + dy * dy <= radius * radius
    )


def _centroid_distance(cell: GridCell, group: list[GridCell]) -> float:
    cx = sum(value[0] for value in group) / len(group)
    cy = sum(value[1] for value in group) / len(group)
    return (cell[0] - cx) ** 2 + (cell[1] - cy) ** 2


def _neighbors4(cell: GridCell):
    x, y = cell
    return ((x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1))


def _neighbors8(cell: GridCell):
    x, y = cell
    return tuple(
        (x + dx, y + dy)
        for dx in (-1, 0, 1)
        for dy in (-1, 0, 1)
        if dx or dy
    )
