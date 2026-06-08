"""Optional OpenAI Responses API planner adapter with fallback to MockLLM."""
from __future__ import annotations

import asyncio
import json
import logging
from typing import Any

from robot_brain.core.errors import BrainError, ErrorCode
from robot_brain.core.world_state import WorldState
from robot_brain.llm.base import LLMClient, ToolCall
from robot_brain.llm.output_validator import LLMOutputValidator
from robot_brain.skills.registry import SkillRegistry

logger = logging.getLogger(__name__)


class OpenAIClient(LLMClient):
    """OpenAI-backed planner with automatic fallback to MockLLM on failure."""

    def __init__(
        self,
        model: str,
        client: Any | None = None,
        *,
        timeout_seconds: float = 30.0,
        max_retries: int = 1,
        skills: SkillRegistry | None = None,
    ) -> None:
        if client is None:
            try:
                from openai import AsyncOpenAI
            except ImportError as exc:
                raise RuntimeError("Install robot-brain[openai] to use OpenAIClient") from exc
            client = AsyncOpenAI()
        self.client = client
        self.model = model
        self.timeout_seconds = timeout_seconds
        self.max_retries = max_retries
        self.is_degraded: bool = False
        self.last_error: BrainError | None = None
        self.validation_errors: list[BrainError] = []
        self._fallback: LLMClient | None = None
        self._validator: LLMOutputValidator | None = None
        if skills is not None:
            self._validator = LLMOutputValidator(skills)

    @property
    def fallback(self) -> LLMClient:
        if self._fallback is None:
            from robot_brain.llm.mock import MockLLM

            self._fallback = MockLLM()
        return self._fallback

    def set_skills(self, skills: SkillRegistry) -> None:
        self._validator = LLMOutputValidator(skills)

    async def plan(
        self,
        command: str,
        world: WorldState,
        tools: list[dict[str, object]],
        memories: list[str],
    ) -> list[ToolCall]:
        self.validation_errors = []
        for attempt in range(1 + self.max_retries):
            try:
                result = await self._call_openai(command, world, tools, memories)
                if self.is_degraded:
                    logger.info("OpenAI recovered, clearing degraded state")
                    self.is_degraded = False
                    self.last_error = None
                if self._validator is not None:
                    result, errors = self._validator.validate_tool_calls(result)
                    self.validation_errors.extend(errors)
                return result
            except (asyncio.TimeoutError, Exception) as exc:
                logger.warning(
                    "OpenAI plan attempt %d/%d failed: %s",
                    attempt + 1,
                    1 + self.max_retries,
                    exc,
                )
                if attempt < self.max_retries:
                    continue
                self.is_degraded = True
                if isinstance(exc, asyncio.TimeoutError):
                    self.last_error = BrainError(
                        code=ErrorCode.LLM_TIMEOUT,
                        message=f"OpenAI timed out after {self.timeout_seconds}s",
                    )
                else:
                    self.last_error = BrainError(
                        code=ErrorCode.LLM_API_ERROR,
                        message=f"OpenAI API error: {exc}",
                    )
                logger.error("OpenAI unavailable, falling back to MockLLM (degraded mode)")
                return await self.fallback.plan(command, world, tools, memories)
        return await self.fallback.plan(command, world, tools, memories)  # pragma: no cover

    async def _call_openai(
        self,
        command: str,
        world: WorldState,
        tools: list[dict[str, object]],
        memories: list[str],
    ) -> list[ToolCall]:
        response = await asyncio.wait_for(
            self.client.responses.create(
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
            ),
            timeout=self.timeout_seconds,
        )
        calls: list[ToolCall] = []
        parse_errors: list[BrainError] = []
        for item in response.output:
            if item.type != "function_call":
                continue
            try:
                parameters = json.loads(item.arguments)
            except (json.JSONDecodeError, TypeError) as exc:
                parse_errors.append(
                    BrainError(
                        code=ErrorCode.LLM_INVALID_OUTPUT,
                        message=f"Failed to parse arguments for {item.name}: {exc}",
                        details={"skill_name": item.name, "raw_arguments": item.arguments},
                    )
                )
                logger.warning("Skipping malformed function_call %r: %s", item.name, exc)
                continue
            calls.append(
                ToolCall(
                    skill_name=item.name,
                    parameters=parameters,
                    reason="OpenAI Responses API function call",
                )
            )
        self.validation_errors.extend(parse_errors)
        return calls
