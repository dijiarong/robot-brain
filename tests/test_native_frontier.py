from __future__ import annotations

import unittest

from robot_brain.navigation.frontier import find_frontier_goals
from robot_brain.navigation.grid import OccupancyGrid2D


def _grid(known, occupied=()):
    return OccupancyGrid2D(
        resolution_m=0.1, width=30, height=30,
        origin_x_m=-1.5, origin_y_m=-1.5,
        occupied=frozenset(occupied), known_free=frozenset(known),
    )


class NativeFrontierTests(unittest.TestCase):
    def test_extracts_boundary_of_reachable_known_free_space(self) -> None:
        known = {(x, y) for x in range(8, 23) for y in range(8, 23)}
        goals = find_frontier_goals(
            _grid(known), min_frontier_length_m=0.3, obstacle_clearance_m=0.0
        )
        self.assertTrue(goals)
        self.assertGreaterEqual(goals[0].cell_count, 10)

    def test_unreachable_free_is_not_selected(self) -> None:
        near = {(x, y) for x in range(12, 19) for y in range(12, 19)}
        remote = {(x, y) for x in range(2, 6) for y in range(2, 6)}
        goals = find_frontier_goals(
            _grid(near | remote), min_frontier_length_m=0.2,
            obstacle_clearance_m=0.0,
        )
        self.assertTrue(goals)
        self.assertTrue(all(goal.x_m > -0.5 and goal.y_m > -0.5 for goal in goals))

    def test_visited_frontier_is_blacklisted(self) -> None:
        known = {(x, y) for x in range(10, 21) for y in range(10, 21)}
        first = find_frontier_goals(
            _grid(known), min_frontier_length_m=0.2, obstacle_clearance_m=0.0
        )
        self.assertTrue(first)
        filtered = find_frontier_goals(
            _grid(known), min_frontier_length_m=0.2, obstacle_clearance_m=0.0,
            visited=((first[0].x_m, first[0].y_m),), visited_radius_m=10.0,
        )
        self.assertEqual([], filtered)

    def test_obstacle_clearance_can_reject_frontier(self) -> None:
        known = {(x, y) for x in range(10, 21) for y in range(10, 21)}
        boundary = {
            (x, y) for x in range(9, 22) for y in range(9, 22)
            if x in {9, 21} or y in {9, 21}
        }
        goals = find_frontier_goals(
            _grid(known, boundary), min_frontier_length_m=0.2,
            obstacle_clearance_m=0.3,
        )
        self.assertEqual([], goals)
