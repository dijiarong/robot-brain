from __future__ import annotations

import unittest

from robot_brain.navigation.terrain3d import (
    MultiLevelTerrainPlanner,
    RollingTerrainMap,
    TerrainMapConfig,
    TerrainPlannerConfig,
    TerrainRegion,
    build_surface_graph,
    plan_surface_path,
)


def _surface(columns, *, resolution=0.2):
    return tuple(
        (x * resolution, y * resolution, z)
        for x, y, z in columns
    )


class NativeTerrain3DTests(unittest.TestCase):
    def test_plans_across_gentle_multilevel_surface(self) -> None:
        points = _surface((
            (0, 0, 0.0), (1, 0, 0.05), (2, 0, 0.10),
            (3, 0, 0.15), (4, 0, 0.20),
        ))
        graph = build_surface_graph(points, resolution_m=0.2, layer_height_m=0.05)
        path = plan_surface_path(
            graph, (0.0, 0.0, 0.0), (0.8, 0.0, 0.2),
            max_step_height_m=0.08, max_slope_degrees=20.0,
        )
        self.assertIsNotNone(path)
        self.assertEqual(5, len(path.nodes))  # type: ignore[union-attr]
        self.assertAlmostEqual(0.2, path.elevation_gain_m, places=2)  # type: ignore[union-attr]

    def test_rejects_step_above_go2_limit(self) -> None:
        graph = build_surface_graph(
            _surface(((0, 0, 0.0), (1, 0, 0.35), (2, 0, 0.35))),
            resolution_m=0.2, layer_height_m=0.05,
        )
        self.assertIsNone(plan_surface_path(
            graph, (0.0, 0.0, 0.0), (0.4, 0.0, 0.35),
            max_step_height_m=0.2,
        ))

    def test_supports_multiple_layers_at_same_xy_cell(self) -> None:
        graph = build_surface_graph(
            ((0.05, 0.05, 0.0), (0.05, 0.05, 1.0)),
            resolution_m=0.2, layer_height_m=0.1,
        )
        self.assertEqual(2, len(graph))

    def test_far_endpoint_fails_closed(self) -> None:
        graph = build_surface_graph(((0.0, 0.0, 0.0),))
        self.assertIsNone(plan_surface_path(
            graph, (5.0, 5.0, 0.0), (0.0, 0.0, 0.0),
        ))

    def test_headroom_excludes_surface_below_low_ceiling(self) -> None:
        graph = build_surface_graph(
            ((0.0, 0.0, 0.0), (0.0, 0.0, 0.2)),
            resolution_m=0.1, layer_height_m=0.1, robot_height_m=0.3,
        )
        self.assertNotIn((0, 0, 0), graph)
        self.assertIn((0, 0, 2), graph)

    def test_incremental_region_replaces_stale_surface(self) -> None:
        config = TerrainPlannerConfig(
            voxel_size_m=0.2, robot_height_m=0.3,
            wall_clearance_m=0.0, wall_buffer_m=0.0,
            wall_buffer_weight=0.0, goal_tolerance_m=0.3,
        )
        planner = MultiLevelTerrainPlanner(config)
        planner.update_global_map(_surface(((0, 0, 0.0), (1, 0, 0.0), (2, 0, 0.0))))
        self.assertIsNotNone(planner.plan((0, 0, 0), (0.4, 0, 0)))

        planner.update_region(
            _surface(((0, 0, 0.0),)),
            TerrainRegion(0.3, 0.0, 0.25, -0.2, 0.5),
        )
        self.assertIsNone(planner.plan((0, 0, 0), (0.4, 0, 0)))
        self.assertGreaterEqual(planner.state.generation, 2)
        self.assertEqual("no_traversable_surface_path", planner.state.stop_reason)

    def test_navigation_boundary_excludes_outside_surface(self) -> None:
        planner = MultiLevelTerrainPlanner(TerrainPlannerConfig(
            voxel_size_m=0.2, robot_height_m=0.3, surface_closing_radius_m=0,
            wall_clearance_m=0, wall_buffer_m=0, wall_buffer_weight=0,
            goal_tolerance_m=0.25,
        ))
        points = tuple((x*.2, y*.2, 0.0) for x in range(6) for y in range(5))
        planner.update_global_map(points)
        self.assertIsNotNone(planner.plan((.2, .4, 0), (1.0, .4, 0)))
        planner.set_navigation_boundary(((0, 0), (.6, 0), (.6, .8), (0, .8)))
        self.assertTrue(all(node.x_m <= .6+1e-9 for node in planner.graph.values()))
        self.assertIsNone(planner.plan((.2, .4, 0), (1.0, .4, 0)))

    def test_added_obstacle_overlay_invalidates_path_and_can_be_cleared(self) -> None:
        planner = MultiLevelTerrainPlanner(TerrainPlannerConfig(
            voxel_size_m=0.2, robot_height_m=0.3, surface_closing_radius_m=0,
            wall_clearance_m=0, wall_buffer_m=0, wall_buffer_weight=0,
            goal_tolerance_m=0.25,
        ))
        planner.update_global_map(tuple((x*.2, y*.2, 0.0)
                                        for x in range(7) for y in range(3)))
        self.assertIsNotNone(planner.plan((0, .2, 0), (1.2, .2, 0)))
        planner.set_added_obstacles(((.6, .2, 0.0),), radius_m=.35)
        blocked = planner.plan((0, .2, 0), (1.2, .2, 0))
        self.assertTrue(blocked is None or blocked.nodes[-1].x_m < .6)
        planner.set_added_obstacles(())
        self.assertIsNotNone(planner.plan((0, .2, 0), (1.2, .2, 0)))

    def test_invalid_navigation_boundary_fails_closed(self) -> None:
        planner = MultiLevelTerrainPlanner()
        with self.assertRaises(ValueError):
            planner.set_navigation_boundary(((0, 0), (1, 1), (0, 1), (1, 0)))

    def test_multilevel_frontiers_rank_information_and_respect_visited_blacklist(self) -> None:
        planner = MultiLevelTerrainPlanner(TerrainPlannerConfig(
            voxel_size_m=.2, robot_height_m=.3, surface_closing_radius_m=0,
            wall_clearance_m=0, wall_buffer_m=0, wall_buffer_weight=0,
        ))
        floor = tuple((x*.2, y*.2, 0.0) for x in range(6) for y in range(6))
        landing = tuple((x*.2, y*.2, 1.0) for x in range(2) for y in range(2))
        planner.update_global_map(floor+landing)
        goals = planner.frontier_goals((.4, .4, 0), max_goals=10)
        self.assertTrue(goals)
        self.assertTrue(all(goal.information_gain >= 2 for goal in goals))
        best = goals[0]
        filtered = planner.frontier_goals(
            (.4, .4, 0), visited_xy=((best.node.x_m, best.node.y_m),),
            visited_radius_m=.3,
        )
        self.assertNotIn(best.node.cell, {goal.node.cell for goal in filtered})

    def test_terrain_frontier_range_and_limits_are_bounded(self) -> None:
        planner = MultiLevelTerrainPlanner(TerrainPlannerConfig(
            voxel_size_m=.2, robot_height_m=.3, surface_closing_radius_m=0,
            wall_clearance_m=0, wall_buffer_m=0, wall_buffer_weight=0,
        ))
        planner.update_global_map(tuple((x*.2, y*.2, 0) for x in range(10) for y in range(3)))
        goals = planner.frontier_goals((0, 0, 0), exploration_range_m=.7, max_goals=2)
        self.assertLessEqual(len(goals), 2)
        self.assertTrue(all(goal.distance_m <= .7 for goal in goals))
        with self.assertRaises(ValueError):
            planner.frontier_goals((0, 0, 0), exploration_range_m=0)

    def test_region_caps_points_above_sensor_overhead_budget(self) -> None:
        planner = MultiLevelTerrainPlanner(TerrainPlannerConfig(
            voxel_size_m=0.1, robot_height_m=0.3, max_overhead_m=0.5,
            wall_clearance_m=0.0, wall_buffer_m=0.0, wall_buffer_weight=0.0,
        ))
        planner.update_region(
            ((0.0, 0.0, 0.0), (0.0, 0.0, 2.0)),
            TerrainRegion(0.0, 0.0, 1.0, -1.0, 3.0), sensor_z_m=0.2,
        )
        self.assertEqual(1, planner.state.voxel_count)

    def test_rolling_map_filters_decays_and_merges_local_terrain(self) -> None:
        terrain = RollingTerrainMap(TerrainMapConfig(
            voxel_size_m=0.1, sensor_range_m=2.0, decay_time_s=1.0,
            local_merge_radius_m=0.5,
        ))
        points = terrain.update(
            ((0.2, 0.0, 0.0), (3.0, 0.0, 0.0), (0.2, 0.0, 3.0)),
            (0.0, 0.0, 0.0), 1.0,
        )
        self.assertEqual(1, len(points))
        points = terrain.update((), (0.0, 0.0, 0.0), 3.0,
                                local_terrain=((0.1, 0.0, 0.05),))
        self.assertEqual(((0.1, 0.0, 0.05),), points)

    def test_rolling_map_capacity_and_manual_clear_are_bounded(self) -> None:
        terrain = RollingTerrainMap(TerrainMapConfig(max_voxels=2))
        terrain.update(((0, 0, 0), (1, 0, 0), (2, 0, 0)), (0, 0, 0), 1.0)
        self.assertEqual(2, len(terrain.points))
        self.assertGreaterEqual(terrain.clear_within((0, 0, 0), 5.0), 1)
        self.assertEqual(0, len(terrain.points))

    def test_conservative_closing_fills_enclosed_supported_hole(self) -> None:
        points = tuple(
            (x * 0.1, y * 0.1, 0.0)
            for x in range(3) for y in range(3) if (x, y) != (1, 1)
        )
        open_graph = build_surface_graph(
            points, resolution_m=0.1, layer_height_m=0.1,
            surface_closing_radius_m=0.0,
        )
        closed_graph = build_surface_graph(
            points, resolution_m=0.1, layer_height_m=0.1,
            surface_closing_radius_m=0.1,
        )
        self.assertNotIn((1, 1, 0), open_graph)
        self.assertIn((1, 1, 0), closed_graph)

    def test_hard_edge_clearance_rejects_narrow_surface(self) -> None:
        points = tuple((x * 0.1, y * 0.1, 0.0) for x in range(9) for y in range(9))
        graph = build_surface_graph(
            points, resolution_m=0.1, layer_height_m=0.1,
            wall_clearance_m=0.2, wall_buffer_m=0.1, wall_buffer_weight=2.0,
        )
        self.assertFalse(graph[(0, 0, 0)].traversal_cost < float("inf"))
        self.assertTrue(graph[(4, 4, 0)].traversal_cost < float("inf"))

    def test_failed_replan_returns_current_safe_prefix_with_standoff(self) -> None:
        planner = MultiLevelTerrainPlanner(TerrainPlannerConfig(
            voxel_size_m=0.2, robot_height_m=0.3, surface_closing_radius_m=0.0,
            wall_clearance_m=0.0, wall_buffer_m=0.0, wall_buffer_weight=0.0,
            goal_tolerance_m=0.25,
        ))
        points = tuple((index * 0.2, 0.0, 0.0) for index in range(11))
        planner.update_global_map(points)
        self.assertIsNotNone(planner.plan((0, 0, 0), (2.0, 0, 0)))
        planner.update_region((), TerrainRegion(1.4, 0.0, 0.11, -0.1, 0.1))
        prefix = planner.plan((0.0, 0.0, 0.0), (2.0, 0.0, 0.0))
        self.assertIsNotNone(prefix)
        self.assertGreaterEqual(len(prefix.nodes), 2)  # type: ignore[union-attr]
        self.assertLessEqual(prefix.nodes[-1].x_m, 0.4 + 1e-9)  # type: ignore[union-attr]

    def test_search_expansion_budget_fails_closed_without_cached_fallback(self) -> None:
        planner = MultiLevelTerrainPlanner(TerrainPlannerConfig(
            voxel_size_m=0.2, robot_height_m=0.3,
            surface_closing_radius_m=0.0, wall_clearance_m=0.0,
            wall_buffer_m=0.0, wall_buffer_weight=0.0,
            goal_tolerance_m=0.3, max_search_expansions=2,
        ))
        planner.update_global_map(tuple((index * 0.2, 0.0, 0.0) for index in range(20)))
        self.assertIsNone(planner.plan((0, 0, 0), (3.8, 0, 0)))
        self.assertEqual("terrain_search_budget_exceeded", planner.state.stop_reason)
