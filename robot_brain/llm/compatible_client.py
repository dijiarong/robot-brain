"""OpenAI-compatible Chat Completions + tools adapter.

Works with any provider exposing the ``/v1/chat/completions`` endpoint:
OpenAI, DeepSeek, Ollama, vLLM, LM Studio, Together AI, etc.

Configure via environment variables:
    RDB_LLM=compatible
    OPENAI_BASE_URL=https://api.deepseek.com   (or http://127.0.0.1:11434/v1)
    OPENAI_API_KEY=sk-...
    RDB_OPENAI_MODEL=deepseek-chat
"""
from __future__ import annotations

import asyncio
import json
import logging
from typing import Any

from robot_brain.core.errors import BrainError, ErrorCode
from robot_brain.core.world_state import WorldState
from robot_brain.llm.base import LLMClient, ToolCall
from robot_brain.llm.output_validator import LLMOutputValidator
from robot_brain.llm.prompt_builder import PromptBuilder
from robot_brain.skills.registry import SkillRegistry

logger = logging.getLogger(__name__)


class CompatibleLLMClient(LLMClient):
    """OpenAI-compatible Chat Completions planner with automatic fallback to MockLLM."""

    def __init__(
        self,
        model: str,
        client: Any | None = None,
        *,
        timeout_seconds: float = 30.0,
        max_retries: int = 1,
        skills: SkillRegistry | None = None,
        backend: str = "mock",
        prompt_builder: PromptBuilder | None = None,
        settings: Any | None = None,
    ) -> None:
        if client is None:
            try:
                from openai import AsyncOpenAI
            except ImportError as exc:
                raise RuntimeError(
                    "Install robot-brain[openai] to use CompatibleLLMClient"
                ) from exc
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
        self._backend = backend
        self._settings = settings
        self._prompt_builder = prompt_builder or PromptBuilder(settings=settings)
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
        conversation: list[dict[str, str]] | None = None,
    ) -> list[ToolCall]:
        self.validation_errors = []
        for attempt in range(1 + self.max_retries):
            try:
                result = await self._call_chat(command, world, tools, memories, conversation)
                if self.is_degraded:
                    logger.info("Compatible LLM recovered, clearing degraded state")
                    self.is_degraded = False
                    self.last_error = None
                if self._validator is not None:
                    result, errors = self._validator.validate_tool_calls(result)
                    self.validation_errors.extend(errors)
                return result
            except (asyncio.TimeoutError, Exception) as exc:
                logger.warning(
                    "Compatible LLM attempt %d/%d failed: %s",
                    attempt + 1,
                    1 + self.max_retries,
                    exc,
                )
                if attempt == self.max_retries:
                    error_code = (
                        ErrorCode.LLM_TIMEOUT
                        if isinstance(exc, asyncio.TimeoutError)
                        else ErrorCode.LLM_API_ERROR
                    )
                    self.last_error = BrainError(
                        code=error_code,
                        message=f"Compatible LLM failed after {1 + self.max_retries} attempts: {exc}",
                    )
                    self.is_degraded = True
                    logger.warning(
                        "Degrading to MockLLM fallback: %s", self.last_error.message
                    )
                    return await self.fallback.plan(
                        command, world, tools, memories, conversation=conversation
                    )
        return []  # Unreachable, but satisfies type checker.

    async def _call_chat(
        self,
        command: str,
        world: WorldState,
        tools: list[dict[str, object]],
        memories: list[str],
        conversation: list[dict[str, str]] | None = None,
    ) -> list[ToolCall]:
        """Call the Chat Completions API with tool-calling."""
        instructions = self._prompt_builder.build_system_prompt(
            world,
            backend=self._backend,
            memories=memories,
            conversation=conversation,
        )

        # Build messages array.
        input_payload = json.dumps(
            {"command": command, "world": world.cognitive_snapshot(self._settings)},
            ensure_ascii=False,
        )
        messages: list[dict[str, str]] = [
            {"role": "system", "content": instructions},
            {"role": "user", "content": input_payload},
        ]

        # Convert tools to Chat Completions format.
        chat_tools = self._convert_tools(tools)

        response = await asyncio.wait_for(
            self.client.chat.completions.create(
                model=self.model,
                messages=messages,
                tools=chat_tools or None,
                tool_choice="auto" if chat_tools else None,
            ),
            timeout=self.timeout_seconds,
        )

        return self._parse_response(response)

    @staticmethod
    def _convert_tools(tools: list[dict[str, object]]) -> list[dict[str, object]]:
        """Convert Responses API tool format to Chat Completions format.

        Input:  {"type": "function", "name": ..., "description": ..., "parameters": ..., "strict": ...}
        Output: {"type": "function", "function": {"name": ..., "description": ..., "parameters": ..., "strict": ...}}
        """
        result: list[dict[str, object]] = []
        for t in tools:
            func_def: dict[str, object] = {
                "name": t["name"],
                "description": t["description"],
                "parameters": t["parameters"],
            }
            if "strict" in t:
                func_def["strict"] = t["strict"]
            result.append({"type": "function", "function": func_def})
        return result

    def _parse_response(self, response: Any) -> list[ToolCall]:
        """Extract ToolCall list from Chat Completions response."""
        calls: list[ToolCall] = []
        message = response.choices[0].message

        # Handle tool_calls (function calling).
        if message.tool_calls:
            for tc in message.tool_calls:
                if tc.type != "function":
                    continue
                try:
                    parameters = json.loads(tc.function.arguments) if tc.function.arguments else {}
                except (json.JSONDecodeError, TypeError) as exc:
                    logger.warning(
                        "Malformed tool arguments for %s: %s", tc.function.name, exc
                    )
                    self.validation_errors.append(
                        BrainError(
                            code=ErrorCode.LLM_INVALID_OUTPUT,
                            message=f"Invalid JSON in tool_call arguments for {tc.function.name}: {exc}",
                        )
                    )
                    continue
                calls.append(
                    ToolCall(
                        skill_name=tc.function.name,
                        parameters=parameters,
                        reason="Chat Completions tool call",
                    )
                )

        # Fallback: if no tool_calls, try parsing content as JSON (some models
        # return tool calls in the message body instead of the tool_calls field).
        if not calls and message.content:
            calls = self._try_parse_content_as_tools(message.content)

        return calls

    def _try_parse_content_as_tools(self, content: str) -> list[ToolCall]:
        """Best-effort fallback: parse message content as JSON tool calls.

        Some smaller models (especially via Ollama) may return tool invocations
        as JSON in the content field rather than using the tool_calls mechanism.
        Expected format: [{"name": "...", "arguments": {...}}, ...]
        """
        try:
            data = json.loads(content)
        except (json.JSONDecodeError, TypeError):
            return []

        if isinstance(data, dict):
            data = [data]
        if not isinstance(data, list):
            return []

        calls: list[ToolCall] = []
        for item in data:
            if not isinstance(item, dict):
                continue
            name = item.get("name") or item.get("tool") or item.get("function")
            args = item.get("arguments") or item.get("parameters") or item.get("args") or {}
            if isinstance(name, str) and name:
                calls.append(
                    ToolCall(
                        skill_name=name,
                        parameters=args if isinstance(args, dict) else {},
                        reason="Parsed from message content (fallback)",
                    )
                )
        return calls
