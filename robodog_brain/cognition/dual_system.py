"""Router that prioritizes fast reflexes before slow planning."""
from __future__ import annotations

from typing import Literal

from pydantic import BaseModel, Field

from robodog_brain.cognition.fast_reflex import FastReflex
from robodog_brain.cognition.planner import Planner
from robodog_brain.core.world_state import WorldState
from robodog_brain.llm.base import ToolCall


class Decision(BaseModel):
    source: Literal["fast", "slow"]
    tool_calls: list[ToolCall] = Field(default_factory=list)
    reason: str


class DualSystem:
    def __init__(self, reflex: FastReflex, planner: Planner) -> None:
        self.reflex = reflex
        self.planner = planner

    async def decide(self, command: str, world: WorldState) -> Decision:
        reflex_calls = self.reflex.decide(world)
        if reflex_calls:
            return Decision(source="fast", tool_calls=reflex_calls, reason=reflex_calls[0].reason or "fast reflex")
        calls = await self.planner.plan(command, world)
        return Decision(source="slow", tool_calls=calls, reason="slow planner")
