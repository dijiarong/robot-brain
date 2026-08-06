"""Operator/internal tools for native navigation maps."""
from __future__ import annotations

import math
from pydantic import BaseModel, Field

from robot_brain.navigation.native_go2 import NativeGo2NavigationClient
from robot_brain.tools.base import (
    CapabilityMetadata,
    EmptyParams,
    RiskLevel,
    Tool,
    ToolContext,
    ToolResult,
)


class NativeTerrainPlanParams(BaseModel):
    forward_m: float = Field(ge=-3.0, le=3.0)
    left_m: float = Field(ge=-3.0, le=3.0)
    up_m: float = Field(default=0.0, ge=-1.5, le=1.5)
    navigation_boundary_xy: tuple[tuple[float, float], ...] = Field(default=(), max_length=100)
    added_obstacles_xyz: tuple[tuple[float, float, float], ...] = Field(default=(), max_length=1000)
    added_obstacle_radius_m: float = Field(default=0.30, ge=0.0, le=2.0)


class NativeMapGetStateTool(Tool):
    name = "native_map_get_state"
    description = "Read native map identity, voxel count, and current costmap coverage."
    params_model = EmptyParams
    metadata = CapabilityMetadata(
        risk_level=RiskLevel.READ_ONLY, planner_visible=False,
        tags=frozenset({"navigation", "map", "state"}),
    )

    def __init__(self, client: NativeGo2NavigationClient) -> None:
        self._client = client

    async def execute(self, params: EmptyParams, context: ToolContext) -> ToolResult:
        identity = self._client.voxel_map.identity()
        try:
            grid = await self._client.get_costmap()
            grid_data = {
                "frame_id": grid.frame_id,
                "known_free_cells": len(grid.known_free),
                "occupied_cells": len(grid.occupied),
                "resolution_m": grid.resolution_m,
            }
        except Exception as exc:
            grid_data = {"unavailable": str(exc)}
        return ToolResult(
            success=True, message="native map state",
            data={**identity.__dict__, "costmap": grid_data},
        )


class NativeMapSaveTool(Tool):
    name = "native_map_save"
    description = "Atomically save the native map to the operator-configured path."
    params_model = EmptyParams
    metadata = CapabilityMetadata(
        risk_level=RiskLevel.LOW, planner_visible=False,
        tags=frozenset({"navigation", "map", "persistence"}),
    )

    def __init__(self, client: NativeGo2NavigationClient, path: str) -> None:
        if not path:
            raise ValueError("native map save path is required")
        self._client = client
        self._path = path

    async def execute(self, params: EmptyParams, context: ToolContext) -> ToolResult:
        try:
            identity = self._client.save_map(self._path)
        except Exception as exc:
            return ToolResult(success=False, message=f"native map save failed: {exc}", data={"path": self._path})
        return ToolResult(
            success=True, message="native map saved",
            data={**identity.__dict__, "path": self._path},
        )


class NativeTerrainPlanTool(Tool):
    name = "native_terrain_plan"
    description = "Plan and inspect a bounded multi-level terrain route without moving the robot."
    params_model = NativeTerrainPlanParams
    metadata = CapabilityMetadata(
        risk_level=RiskLevel.READ_ONLY, planner_visible=False,
        tags=frozenset({"navigation", "terrain", "3d", "plan"}),
    )

    def __init__(self, client: NativeGo2NavigationClient) -> None:
        self._client = client

    async def execute(self, params: NativeTerrainPlanParams, context: ToolContext) -> ToolResult:
        try:
            path = await self._client.plan_terrain_relative(
                forward_m=params.forward_m, left_m=params.left_m, up_m=params.up_m,
                navigation_boundary_xy=params.navigation_boundary_xy,
                added_obstacles_xyz=params.added_obstacles_xyz,
                added_obstacle_radius_m=params.added_obstacle_radius_m,
            )
        except Exception as exc:
            return ToolResult(success=False, message=f"terrain plan failed: {exc}",
                              data={"stop_reason": "terrain_plan_failed"})
        return ToolResult(
            success=True, message=f"terrain path has {len(path.nodes)} surface node(s)",
            data={
                "path": [[node.x_m, node.y_m, node.z_m] for node in path.nodes],
                "length_m": path.length_m,
                "elevation_gain_m": path.elevation_gain_m,
                "minimum_clearance_m": (
                    path.minimum_clearance_m
                    if math.isfinite(path.minimum_clearance_m) else None
                ),
                "cost": path.cost,
            },
        )
