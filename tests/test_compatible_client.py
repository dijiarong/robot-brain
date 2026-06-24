"""Tests for CompatibleLLMClient (Chat Completions + tools adapter)."""
from __future__ import annotations

import asyncio
import json
import unittest
import unittest.mock
from unittest.mock import AsyncMock, MagicMock

from config.settings import Settings
from robot_brain.core.world_state import WorldState
from robot_brain.llm.compatible_client import CompatibleLLMClient
from robot_brain.llm.base import ToolCall
from robot_brain.runtime.loop import AgentRuntime


def _make_tool_call_response(tool_calls: list[dict]) -> MagicMock:
    """Create a mock Chat Completions response with tool_calls."""
    mock_tcs = []
    for tc in tool_calls:
        mock_tc = MagicMock()
        mock_tc.type = "function"
        mock_tc.function = MagicMock()
        mock_tc.function.name = tc["name"]
        mock_tc.function.arguments = json.dumps(tc.get("arguments", {}))
        mock_tcs.append(mock_tc)

    message = MagicMock()
    message.tool_calls = mock_tcs
    message.content = None

    choice = MagicMock()
    choice.message = message

    response = MagicMock()
    response.choices = [choice]
    return response


def _make_content_response(content: str) -> MagicMock:
    """Create a mock Chat Completions response with content only (no tool_calls)."""
    message = MagicMock()
    message.tool_calls = None
    message.content = content

    choice = MagicMock()
    choice.message = message

    response = MagicMock()
    response.choices = [choice]
    return response


class TestToolConversion(unittest.TestCase):
    """Verify tool schema format conversion."""

    def test_convert_responses_api_to_chat_completions(self) -> None:
        tools_input = [
            {
                "type": "function",
                "name": "nudge",
                "description": "Move forward",
                "parameters": {"type": "object", "properties": {}},
                "strict": True,
            }
        ]
        result = CompatibleLLMClient._convert_tools(tools_input)
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0]["type"], "function")
        self.assertIn("function", result[0])
        func = result[0]["function"]
        self.assertEqual(func["name"], "nudge")
        self.assertEqual(func["description"], "Move forward")
        self.assertEqual(func["parameters"], {"type": "object", "properties": {}})
        self.assertTrue(func["strict"])

    def test_convert_without_strict(self) -> None:
        tools_input = [
            {
                "type": "function",
                "name": "stop",
                "description": "Stop",
                "parameters": {},
            }
        ]
        result = CompatibleLLMClient._convert_tools(tools_input)
        self.assertNotIn("strict", result[0]["function"])


class TestParsing(unittest.IsolatedAsyncioTestCase):
    """Verify response parsing from Chat Completions."""

    async def test_parse_tool_calls(self) -> None:
        mock_client = MagicMock()
        mock_client.chat = MagicMock()
        mock_client.chat.completions = MagicMock()
        mock_client.chat.completions.create = AsyncMock(
            return_value=_make_tool_call_response([
                {"name": "nudge", "arguments": {"direction": "forward", "distance_cm": 30}},
                {"name": "report", "arguments": {"message": "done", "severity": "info"}},
            ])
        )

        client = CompatibleLLMClient("test-model", client=mock_client, max_retries=0)
        world = WorldState()
        tools = [
            {"type": "function", "name": "nudge", "description": "Move", "parameters": {}},
            {"type": "function", "name": "report", "description": "Report", "parameters": {}},
        ]

        result = await client.plan("move forward", world, tools, [])

        self.assertEqual(len(result), 2)
        self.assertEqual(result[0].skill_name, "nudge")
        self.assertEqual(result[0].parameters["direction"], "forward")
        self.assertEqual(result[1].skill_name, "report")

    async def test_parse_malformed_arguments_skipped(self) -> None:
        """Malformed JSON in arguments should be skipped with error."""
        mock_tc = MagicMock()
        mock_tc.type = "function"
        mock_tc.function = MagicMock()
        mock_tc.function.name = "nudge"
        mock_tc.function.arguments = "not valid json{{"

        message = MagicMock()
        message.tool_calls = [mock_tc]
        message.content = None
        choice = MagicMock()
        choice.message = message
        response = MagicMock()
        response.choices = [choice]

        mock_client = MagicMock()
        mock_client.chat.completions.create = AsyncMock(return_value=response)

        client = CompatibleLLMClient("test-model", client=mock_client, max_retries=0)
        result = await client.plan("test", WorldState(), [], [])

        self.assertEqual(len(result), 0)
        self.assertEqual(len(client.validation_errors), 1)

    async def test_content_fallback_json_array(self) -> None:
        """If no tool_calls, try parsing content as JSON."""
        content = json.dumps([
            {"name": "stop", "arguments": {}},
            {"name": "report", "arguments": {"message": "stopped"}},
        ])
        mock_client = MagicMock()
        mock_client.chat.completions.create = AsyncMock(
            return_value=_make_content_response(content)
        )

        client = CompatibleLLMClient("test-model", client=mock_client, max_retries=0)
        result = await client.plan("stop", WorldState(), [], [])

        self.assertEqual(len(result), 2)
        self.assertEqual(result[0].skill_name, "stop")
        self.assertEqual(result[1].skill_name, "report")

    async def test_content_fallback_single_dict(self) -> None:
        """Single dict in content should also work."""
        content = json.dumps({"name": "stop", "arguments": {}})
        mock_client = MagicMock()
        mock_client.chat.completions.create = AsyncMock(
            return_value=_make_content_response(content)
        )

        client = CompatibleLLMClient("test-model", client=mock_client, max_retries=0)
        result = await client.plan("stop", WorldState(), [], [])

        self.assertEqual(len(result), 1)
        self.assertEqual(result[0].skill_name, "stop")

    async def test_content_fallback_plain_text_ignored(self) -> None:
        """Plain text content (not JSON) should return empty."""
        mock_client = MagicMock()
        mock_client.chat.completions.create = AsyncMock(
            return_value=_make_content_response("I cannot do that.")
        )

        client = CompatibleLLMClient("test-model", client=mock_client, max_retries=0)
        result = await client.plan("test", WorldState(), [], [])

        self.assertEqual(len(result), 0)


class TestFallback(unittest.IsolatedAsyncioTestCase):
    """Verify degradation to MockLLM on failure."""

    async def test_timeout_triggers_fallback(self) -> None:
        mock_client = MagicMock()
        mock_client.chat.completions.create = AsyncMock(
            side_effect=asyncio.TimeoutError()
        )

        client = CompatibleLLMClient(
            "test-model", client=mock_client, timeout_seconds=1.0, max_retries=0
        )
        result = await client.plan("patrol", WorldState(), [], [])

        self.assertTrue(client.is_degraded)
        self.assertIsInstance(result, list)
        self.assertGreater(len(result), 0)

    async def test_api_error_triggers_fallback(self) -> None:
        mock_client = MagicMock()
        mock_client.chat.completions.create = AsyncMock(
            side_effect=RuntimeError("connection refused")
        )

        client = CompatibleLLMClient(
            "test-model", client=mock_client, timeout_seconds=5.0, max_retries=0
        )
        result = await client.plan("stop", WorldState(), [], [])

        self.assertTrue(client.is_degraded)
        self.assertIsInstance(result, list)

    async def test_retry_then_fallback(self) -> None:
        mock_client = MagicMock()
        mock_client.chat.completions.create = AsyncMock(
            side_effect=RuntimeError("network error")
        )

        client = CompatibleLLMClient(
            "test-model", client=mock_client, timeout_seconds=5.0, max_retries=1
        )
        result = await client.plan("stop", WorldState(), [], [])

        self.assertTrue(client.is_degraded)
        # Should have been called twice (initial + 1 retry)
        self.assertEqual(mock_client.chat.completions.create.call_count, 2)

    async def test_recovery_clears_degraded(self) -> None:
        mock_client = MagicMock()
        mock_client.chat.completions.create = AsyncMock(
            return_value=_make_tool_call_response([{"name": "stop", "arguments": {}}])
        )

        client = CompatibleLLMClient(
            "test-model", client=mock_client, timeout_seconds=5.0, max_retries=0
        )
        client.is_degraded = True  # Simulate prior failure.

        result = await client.plan("stop", WorldState(), [], [])

        self.assertFalse(client.is_degraded)
        self.assertEqual(len(result), 1)


class TestRuntimeRegistration(unittest.TestCase):
    """Verify that RDB_LLM=compatible integrates with AgentRuntime."""

    def test_compatible_client_injected_into_runtime(self) -> None:
        """CompatibleLLMClient can be injected and used as the LLM backend."""
        mock_client = MagicMock()
        llm = CompatibleLLMClient("deepseek-chat", client=mock_client)
        runtime = AgentRuntime.create(llm=llm)

        self.assertIsInstance(runtime.context.llm, CompatibleLLMClient)

    def test_compatible_client_accepts_settings(self) -> None:
        """Settings are correctly propagated through the client."""
        settings = Settings()
        mock_client = MagicMock()
        llm = CompatibleLLMClient(
            "deepseek-chat",
            client=mock_client,
            settings=settings,
            backend="unitree",
        )
        self.assertEqual(llm._backend, "unitree")
        self.assertIs(llm._settings, settings)


if __name__ == "__main__":
    unittest.main()
