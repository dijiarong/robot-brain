"""Planner-facing LLM contract."""
from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any, Literal

from pydantic import BaseModel, Field

from robodog_brain.core.world_state import WorldState


class ToolCall(BaseModel):
    skill_name: str
    parameters: dict[str, Any] = Field(default_factory=dict)
    reason: str = ""
    source: Literal["fast", "slow"] = "slow"


class LLMClient(ABC):
    @abstractmethod
    async def plan(
        self,
        command: str,
        world: WorldState,
        tools: list[dict[str, object]],
        memories: list[str],
    ) -> list[ToolCall]:
        """Turn an objective and world snapshot into whitelisted tool calls."""
