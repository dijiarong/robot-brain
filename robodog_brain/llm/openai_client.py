"""Optional OpenAI Responses API planner adapter."""
from __future__ import annotations

import json
from typing import Any

from robodog_brain.core.world_state import WorldState
from robodog_brain.llm.base import LLMClient, ToolCall


class OpenAIClient(LLMClient):
    def __init__(self, model: str, client: Any | None = None) -> None:
        if client is None:
            try:
                from openai import AsyncOpenAI
            except ImportError as exc:
                raise RuntimeError("Install robodog-brain[openai] to use OpenAIClient") from exc
            client = AsyncOpenAI()
        self.client = client
        self.model = model

    async def plan(
        self,
        command: str,
        world: WorldState,
        tools: list[dict[str, object]],
        memories: list[str],
    ) -> list[ToolCall]:
        response = await self.client.responses.create(
            model=self.model,
            instructions=(
                "Plan robot-dog L3 cognition actions. Select only provided tools. "
                "Do not bypass safety constraints. Return tool calls for the next objective."
            ),
            input=json.dumps(
                {"command": command, "world": world.snapshot(), "recent_memories": memories},
                ensure_ascii=False,
            ),
            tools=tools,
        )
        return [
            ToolCall(
                skill_name=item.name,
                parameters=json.loads(item.arguments),
                reason="OpenAI Responses API function call",
            )
            for item in response.output
            if item.type == "function_call"
        ]
