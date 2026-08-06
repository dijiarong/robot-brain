"""Native patrol route generation over known free-space grids."""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import StrEnum
import math
import random

from robot_brain.navigation.frontier import find_frontier_goals
from robot_brain.navigation.grid import GridCell, OccupancyGrid2D


class PatrolStrategy(StrEnum):
    COVERAGE = "coverage"
    FRONTIER = "frontier"
    RANDOM = "random"
    LEAST_VISITED = "least_visited"


@dataclass
class VisitationHistory:
    resolution_m: float = 0.5
    counts: dict[tuple[int, int], int] = field(default_factory=dict)

    def record(self, x_m: float, y_m: float) -> None:
        key = self._key(x_m, y_m)
        self.counts[key] = self.counts.get(key, 0) + 1

    def count(self, x_m: float, y_m: float) -> int:
        return self.counts.get(self._key(x_m, y_m), 0)

    def _key(self, x_m: float, y_m: float) -> tuple[int, int]:
        return (
            math.floor(x_m / self.resolution_m),
            math.floor(y_m / self.resolution_m),
        )


def create_patrol_route(
    grid: OccupancyGrid2D,
    *,
    strategy: PatrolStrategy,
    robot_xy: tuple[float, float] = (0.0, 0.0),
    spacing_m: float = 0.75,
    max_waypoints: int = 20,
    seed: int = 0,
    history: VisitationHistory | None = None,
) -> list[tuple[float, float]]:
    free = sorted(grid.known_free)
    if not free or max_waypoints <= 0:
        return []
    if strategy == PatrolStrategy.FRONTIER:
        return [
            (goal.x_m, goal.y_m)
            for goal in find_frontier_goals(
                grid, robot_xy, min_frontier_length_m=max(grid.resolution_m, 0.3),
                obstacle_clearance_m=0.0,
            )[:max_waypoints]
        ]
    stride = max(1, round(spacing_m / grid.resolution_m))
    candidates = _spaced_cells(free, stride)
    if strategy == PatrolStrategy.COVERAGE:
        route_cells = _coverage_order(candidates)
    elif strategy == PatrolStrategy.RANDOM:
        route_cells = list(candidates)
        random.Random(seed).shuffle(route_cells)
    elif strategy == PatrolStrategy.LEAST_VISITED:
        visits = history or VisitationHistory()
        route_cells = sorted(
            candidates,
            key=lambda cell: (
                visits.count(*grid.cell_to_world(cell)),
                _cell_distance(grid, cell, robot_xy),
                cell,
            ),
        )
    else:
        raise ValueError(f"unsupported patrol strategy: {strategy}")
    return [grid.cell_to_world(cell) for cell in route_cells[:max_waypoints]]


def evaluate_patrol_route(
    grid: OccupancyGrid2D, route: list[tuple[float, float]], *,
    strategy: PatrolStrategy, history: VisitationHistory | None = None,
) -> dict[str, object]:
    cells = [grid.world_to_cell(x, y) for x, y in route]
    failures = []
    if not cells:
        failures.append("empty_patrol_route")
    if any(cell is None or cell not in grid.known_free for cell in cells):
        failures.append("route_outside_known_free")
    concrete = [cell for cell in cells if cell is not None]
    if len(set(concrete)) != len(concrete):
        failures.append("duplicate_patrol_waypoint")
    if strategy == PatrolStrategy.COVERAGE:
        rows: dict[int, list[int]] = {}
        for col, row in concrete:
            rows.setdefault(row, []).append(col)
        directions = []
        for row in sorted(rows):
            values = rows[row]
            if len(values) >= 2:
                directions.append(1 if values[-1] > values[0] else -1)
        if any(left == right for left, right in zip(directions, directions[1:])):
            failures.append("coverage_route_not_boustrophedon")
    elif strategy == PatrolStrategy.FRONTIER:
        if any(not any(
            neighbor not in grid.known_free and neighbor not in grid.occupied
            and not grid.blocked(neighbor)
            for neighbor in ((cell[0]-1, cell[1]), (cell[0]+1, cell[1]),
                             (cell[0], cell[1]-1), (cell[0], cell[1]+1))
        ) for cell in concrete):
            failures.append("frontier_route_not_adjacent_to_unknown")
    elif strategy == PatrolStrategy.LEAST_VISITED:
        visits = history or VisitationHistory()
        counts = [visits.count(*grid.cell_to_world(cell)) for cell in concrete]
        if counts != sorted(counts):
            failures.append("least_visited_route_out_of_order")
    return {
        "ok": not failures, "failures": failures, "strategy": strategy.value,
        "waypoints": len(route), "unique_cells": len(set(concrete)),
        "known_free_cells": len(grid.known_free),
    }


def _spaced_cells(free: list[GridCell], stride: int) -> list[GridCell]:
    return [cell for cell in free if cell[0] % stride == 0 and cell[1] % stride == 0]


def _coverage_order(cells: list[GridCell]) -> list[GridCell]:
    rows: dict[int, list[GridCell]] = {}
    for cell in cells:
        rows.setdefault(cell[1], []).append(cell)
    result: list[GridCell] = []
    for index, row in enumerate(sorted(rows)):
        ordered = sorted(rows[row], reverse=bool(index % 2))
        result.extend(ordered)
    return result


def _cell_distance(grid: OccupancyGrid2D, cell: GridCell, point: tuple[float, float]) -> float:
    x, y = grid.cell_to_world(cell)
    return math.hypot(x - point[0], y - point[1])
