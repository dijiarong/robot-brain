from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from robot_brain.core.world_state import WorldState
from robot_brain.tools.base import EmptyParams, ToolContext
from robot_brain.tools.builtin.native_navigation import (
    NativeMapGetStateTool,
    NativeMapSaveTool,
    NativeTerrainPlanParams,
    NativeTerrainPlanTool,
)
from tests.test_native_navigation import _WorldCloudTransport, _client


class NativeNavigationToolsTests(unittest.IsolatedAsyncioTestCase):
    async def test_map_state_reports_coverage(self) -> None:
        client = await _client(_WorldCloudTransport())
        context = ToolContext(settings=None, world=WorldState(), robot=None)  # type: ignore[arg-type]
        result = await NativeMapGetStateTool(client).execute(EmptyParams(), context)
        self.assertTrue(result.success)
        self.assertIn("voxel_count", result.data)
        self.assertGreater(result.data["costmap"]["known_free_cells"], 0)

    async def test_terrain_plan_is_read_only_and_structured(self) -> None:
        floor = tuple(
            (x * 0.1, y * 0.1, -0.1)
            for x in range(-5, 11) for y in range(-5, 6)
        )
        transport = _WorldCloudTransport(floor)
        client = await _client(transport)
        context = ToolContext(settings=None, world=WorldState(), robot=None)  # type: ignore[arg-type]
        result = await NativeTerrainPlanTool(client).execute(
            NativeTerrainPlanParams(forward_m=0.5, left_m=0.0), context,
        )
        self.assertTrue(result.success, result.message)
        self.assertGreaterEqual(len(result.data["path"]), 2)
        self.assertEqual(0, transport.drive_count)

    async def test_terrain_boundary_and_added_obstacles_fail_closed_without_motion(self) -> None:
        floor = tuple((x*.1, y*.1, -.1) for x in range(-5, 11) for y in range(-5, 6))
        transport = _WorldCloudTransport(floor)
        client = await _client(transport)
        context = ToolContext(settings=None, world=WorldState(), robot=None)  # type: ignore[arg-type]
        result = await NativeTerrainPlanTool(client).execute(
            NativeTerrainPlanParams(
                forward_m=.5, left_m=0,
                navigation_boundary_xy=((-0.2, -0.2), (0.2, -0.2),
                                        (0.2, 0.2), (-0.2, 0.2)),
                added_obstacles_xyz=((0.1, 0.0, -0.1),),
                added_obstacle_radius_m=.2,
            ), context,
        )
        self.assertFalse(result.success)
        self.assertEqual("terrain_plan_failed", result.data["stop_reason"])
        self.assertEqual(0, transport.drive_count)

    async def test_native_client_plans_reachable_terrain_frontier_read_only(self) -> None:
        floor = tuple(((x+.5)*.1, (y+.5)*.1, -.05)
                      for x in range(-10, 21) for y in range(-10, 11))
        transport = _WorldCloudTransport(floor)
        client = await _client(transport)
        path = await client.plan_terrain_frontier(exploration_range_m=2)
        self.assertGreaterEqual(len(path.nodes), 2)
        self.assertGreater(path.length_m, 0)
        self.assertEqual(0, transport.drive_count)
        self.assertTrue(any(row["event"] == "terrain_frontier_plan"
                            for row in client.trace))

    async def test_map_save_uses_fixed_configured_path(self) -> None:
        client = await _client(_WorldCloudTransport())
        await client.get_costmap()
        context = ToolContext(settings=None, world=WorldState(), robot=None)  # type: ignore[arg-type]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "office.native-map.json"
            result = await NativeMapSaveTool(client, str(path)).execute(EmptyParams(), context)
            self.assertTrue(path.exists())
        self.assertTrue(result.success)
