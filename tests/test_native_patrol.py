from __future__ import annotations

import unittest

from robot_brain.navigation.grid import OccupancyGrid2D
from robot_brain.navigation.patrol import (
    PatrolStrategy,
    VisitationHistory,
    create_patrol_route,
    evaluate_patrol_route,
)


def _grid():
    return OccupancyGrid2D(
        resolution_m=0.25, width=12, height=8,
        origin_x_m=0.0, origin_y_m=0.0, occupied=frozenset(),
        known_free=frozenset((x, y) for x in range(12) for y in range(8)),
    )


class NativePatrolTests(unittest.TestCase):
    def test_coverage_route_snakes_between_rows(self) -> None:
        route = create_patrol_route(
            _grid(), strategy=PatrolStrategy.COVERAGE,
            spacing_m=0.5, max_waypoints=20,
        )
        self.assertGreater(len(route), 4)
        first_row = [point for point in route if point[1] == route[0][1]]
        second_y = next(point[1] for point in route if point[1] != route[0][1])
        second_row = [point for point in route if point[1] == second_y]
        self.assertLess(first_row[0][0], first_row[-1][0])
        self.assertGreater(second_row[0][0], second_row[-1][0])
        self.assertTrue(evaluate_patrol_route(
            _grid(), route, strategy=PatrolStrategy.COVERAGE,
        )["ok"])

    def test_random_route_is_seeded(self) -> None:
        first = create_patrol_route(_grid(), strategy=PatrolStrategy.RANDOM, seed=7)
        second = create_patrol_route(_grid(), strategy=PatrolStrategy.RANDOM, seed=7)
        self.assertEqual(first, second)
        self.assertTrue(evaluate_patrol_route(
            _grid(), first, strategy=PatrolStrategy.RANDOM,
        )["ok"])

    def test_least_visited_prioritizes_unvisited_cells(self) -> None:
        grid = _grid()
        history = VisitationHistory(resolution_m=0.25)
        route = create_patrol_route(
            grid, strategy=PatrolStrategy.LEAST_VISITED,
            spacing_m=0.5, max_waypoints=10, history=history,
        )
        for x, y in route:
            history.record(x, y)
        next_route = create_patrol_route(
            grid, strategy=PatrolStrategy.LEAST_VISITED,
            spacing_m=0.5, max_waypoints=10, history=history,
        )
        self.assertTrue(next_route)
        self.assertTrue(all(point not in route for point in next_route))
        self.assertTrue(evaluate_patrol_route(
            grid, next_route, strategy=PatrolStrategy.LEAST_VISITED,
            history=history,
        )["ok"])

    def test_frontier_strategy_returns_boundary_goals(self) -> None:
        grid = OccupancyGrid2D(
            resolution_m=0.25, width=12, height=8,
            origin_x_m=0.0, origin_y_m=0.0, occupied=frozenset(),
            known_free=frozenset((x, y) for x in range(1, 8) for y in range(1, 7)),
        )
        route = create_patrol_route(
            grid, strategy=PatrolStrategy.FRONTIER, max_waypoints=4,
            robot_xy=(1.0, 1.0),
        )
        self.assertTrue(route)
        self.assertLessEqual(len(route), 4)
        self.assertTrue(evaluate_patrol_route(
            grid, route, strategy=PatrolStrategy.FRONTIER,
        )["ok"])
