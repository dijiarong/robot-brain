"""Atomic read-only navigation tools."""
from __future__ import annotations

from robot_brain.navigation import NavigationClient
from robot_brain.tools.base import (
    CapabilityMetadata,
    EmptyParams,
    RiskLevel,
    Tool,
    ToolContext,
    ToolResult,
)


class NavigationGetStateTool(Tool):
    name = "nav_get_state"
    description = "Read navigation provider readiness, active goal, progress, and pose."
    params_model = EmptyParams
    metadata = CapabilityMetadata(
        risk_level=RiskLevel.READ_ONLY,
        planner_visible=False,
        tags=frozenset({"navigation", "state"}),
    )

    def __init__(self, client: NavigationClient) -> None:
        self._client = client

    async def execute(self, params: EmptyParams, context: ToolContext) -> ToolResult:
        state = await self._client.get_state()
        return ToolResult(
            success=state.ready,
            message=state.message or f"navigation {state.status.value}",
            data=state.model_dump(mode="json"),
        )


class LocalizationGetStateTool(Tool):
    name = "localization_get_state"
    description = "Read map identity, localization validity, confidence, and map pose."
    params_model = EmptyParams
    metadata = CapabilityMetadata(
        risk_level=RiskLevel.READ_ONLY,
        planner_visible=False,
        tags=frozenset({"navigation", "localization", "map"}),
    )

    def __init__(self, client: NavigationClient) -> None:
        self._client = client

    async def execute(self, params: EmptyParams, context: ToolContext) -> ToolResult:
        state = await self._client.get_localization_state()
        return ToolResult(
            success=state.pose is not None,
            message=state.message or f"localization {state.status.value}",
            data=state.model_dump(mode="json"),
        )
