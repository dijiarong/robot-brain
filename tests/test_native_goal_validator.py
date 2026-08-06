from __future__ import annotations

import unittest

from robot_brain.navigation.grid import OccupancyGrid2D
from robot_brain.navigation.planner import (
    astar_path,
    find_nearest_safe_goal,
    path_minimum_clearance_m,
)


class NativeGoalValidatorTests(unittest.TestCase):
    def setUp(self) -> None:
        self.grid = OccupancyGrid2D(
            resolution_m=0.1, width=30, height=30,
            origin_x_m=-1.5, origin_y_m=-1.5,
            occupied=frozenset({(20, 15)}),
        )

    def test_moves_occupied_goal_to_nearest_clear_cell(self) -> None:
        requested = self.grid.cell_to_world((20, 15))
        safe = find_nearest_safe_goal(
            self.grid, requested, search_radius_m=0.5, clearance_m=0.1
        )
        self.assertIsNotNone(safe)
        assert safe is not None
        self.assertNotEqual((20, 15), self.grid.world_to_cell(*safe))

    def test_returns_none_when_search_region_is_fully_blocked(self) -> None:
        occupied = frozenset(
            (x, y) for x in range(10, 21) for y in range(10, 21)
        )
        grid = OccupancyGrid2D(
            resolution_m=0.1, width=30, height=30,
            origin_x_m=-1.5, origin_y_m=-1.5, occupied=occupied,
        )
        self.assertIsNone(find_nearest_safe_goal(
            grid, (0.0, 0.0), search_radius_m=0.4
        ))

    def test_path_clearance_reports_distance_to_inflated_obstacles(self) -> None:
        path = astar_path(self.grid, (0.0, 0.0), (1.0, 0.0))
        self.assertIsNotNone(path)
        clearance = path_minimum_clearance_m(self.grid, path or [])
        self.assertIsNotNone(clearance)
        self.assertGreater(clearance or 0.0, 0.0)
        self.assertLess(clearance or 99.0, 1.0)
