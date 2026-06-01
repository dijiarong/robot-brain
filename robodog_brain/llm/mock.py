"""Deterministic offline LLM substitute."""
from __future__ import annotations

import re
from collections import deque
from collections.abc import Iterable

from robodog_brain.core.world_state import WorldState
from robodog_brain.llm.base import LLMClient, ToolCall


class MockLLM(LLMClient):
    def __init__(self, scripted_plans: Iterable[list[ToolCall]] | None = None) -> None:
        self._scripted_plans = deque(scripted_plans or [])

    async def plan(
        self,
        command: str,
        world: WorldState,
        tools: list[dict[str, object]],
        memories: list[str],
    ) -> list[ToolCall]:
        if self._scripted_plans:
            return [call.model_copy(deep=True) for call in self._scripted_plans.popleft()]

        text = command.lower()
        if "patrol" in text:
            return [ToolCall(skill_name="patrol", parameters={"waypoints": [{"x": 4, "y": 0}, {"x": 4, "y": 3}]})]
        if "navigate" in text or "go to" in text:
            numbers = [float(value) for value in re.findall(r"-?\d+(?:\.\d+)?", text)]
            x, y = (numbers + [0.0, 0.0])[:2]
            return [ToolCall(skill_name="navigate", parameters={"target": {"x": x, "y": y}})]
        if "recognize" in text or "inspect" in text:
            calls = [ToolCall(skill_name="recognize", parameters={})]
            if world.alerts:
                calls.append(
                    ToolCall(
                        skill_name="report",
                        parameters={"message": "; ".join(world.alerts), "severity": "warning"},
                    )
                )
            return calls
        if "follow" in text:
            target = text.split("follow", maxsplit=1)[1].strip() or "person-1"
            return [ToolCall(skill_name="follow", parameters={"target_id": target})]
        if "dock" in text or "charge" in text:
            return [ToolCall(skill_name="dock", parameters={})]
        if "stop" in text:
            return [ToolCall(skill_name="stop", parameters={"reason": "operator command"})]
        message = command.strip() or "empty command"
        return [ToolCall(skill_name="report", parameters={"message": message, "severity": "info"})]
