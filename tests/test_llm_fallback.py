"""Tests for LLM fallback robustness."""
from __future__ import annotations

import asyncio
import unittest
from unittest.mock import AsyncMock, MagicMock

from config.settings import Settings
from robot_brain.core.world_state import WorldState
from robot_brain.llm.openai_client import OpenAIClient
from robot_brain.runtime.loop import AgentRuntime


class OpenAIClientFallbackTests(unittest.IsolatedAsyncioTestCase):
    async def test_timeout_triggers_fallback(self) -> None:
        """OpenAI timeout should trigger fallback to MockLLM."""
        mock_client = MagicMock()
        # Simulate a timeout
        mock_client.responses = MagicMock()
        mock_client.responses.create = AsyncMock(side_effect=asyncio.TimeoutError())

        client = OpenAIClient("gpt-4o-mini", client=mock_client, timeout_seconds=1.0, max_retries=0)
        world = WorldState()
        tools = [{"type": "function", "function": {"name": "patrol", "parameters": {}}}]

        result = await client.plan("patrol the lobby", world, tools, [])

        self.assertTrue(client.is_degraded)
        # Should still get a result from MockLLM fallback
        self.assertIsInstance(result, list)
        self.assertGreater(len(result), 0)

    async def test_exception_triggers_fallback(self) -> None:
        """Generic exception from OpenAI should trigger fallback."""
        mock_client = MagicMock()
        mock_client.responses = MagicMock()
        mock_client.responses.create = AsyncMock(side_effect=RuntimeError("API error"))

        client = OpenAIClient("gpt-4o-mini", client=mock_client, timeout_seconds=5.0, max_retries=0)
        world = WorldState()
        tools = []

        result = await client.plan("stop", world, tools, [])

        self.assertTrue(client.is_degraded)
        self.assertIsInstance(result, list)

    async def test_retry_then_fallback(self) -> None:
        """Client retries once before falling back."""
        mock_client = MagicMock()
        mock_client.responses = MagicMock()
        mock_client.responses.create = AsyncMock(side_effect=RuntimeError("network error"))

        client = OpenAIClient("gpt-4o-mini", client=mock_client, timeout_seconds=5.0, max_retries=1)
        world = WorldState()
        tools = []

        await client.plan("stop", world, tools, [])

        self.assertTrue(client.is_degraded)
        # Should have been called 2 times (original + 1 retry)
        self.assertEqual(2, mock_client.responses.create.call_count)

    async def test_recovery_clears_degraded(self) -> None:
        """Successful call after degradation clears the degraded flag."""
        mock_client = MagicMock()
        mock_client.responses = MagicMock()

        # First call fails
        mock_response = MagicMock()
        mock_response.output = []
        mock_client.responses.create = AsyncMock(
            side_effect=[RuntimeError("fail"), mock_response]
        )

        client = OpenAIClient("gpt-4o-mini", client=mock_client, timeout_seconds=5.0, max_retries=0)
        world = WorldState()
        tools = []

        # First call — should degrade
        await client.plan("stop", world, tools, [])
        self.assertTrue(client.is_degraded)

        # Second call — should succeed and clear degraded
        result = await client.plan("stop", world, tools, [])
        self.assertFalse(client.is_degraded)
        self.assertEqual([], result)

    async def test_fallback_does_not_crash_service(self) -> None:
        """Ensure that fallback behavior doesn't throw unhandled exceptions."""
        mock_client = MagicMock()
        mock_client.responses = MagicMock()
        mock_client.responses.create = AsyncMock(side_effect=ConnectionError("connection refused"))

        client = OpenAIClient("gpt-4o-mini", client=mock_client, timeout_seconds=1.0, max_retries=0)
        world = WorldState()
        tools = [{"type": "function", "function": {"name": "patrol", "parameters": {}}}]

        # This should NOT raise — it should gracefully fall back
        result = await client.plan("patrol the lobby", world, tools, [])
        self.assertIsInstance(result, list)
        self.assertTrue(client.is_degraded)


class RuntimeLLMFallbackIntegrationTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self) -> None:
        self.settings = Settings(memory_db_path=":memory:", llm_backend="mock")

    async def test_runtime_with_mock_llm(self) -> None:
        """Runtime works fine with MockLLM (baseline)."""
        runtime = AgentRuntime.create(settings=self.settings)
        result = await runtime.run_command("patrol the lobby")
        self.assertEqual("completed", result.status)

    async def test_degraded_state_recorded_in_decision(self) -> None:
        """When LLM is degraded, the decision context should reflect it."""
        mock_client = MagicMock()
        mock_client.responses = MagicMock()
        mock_client.responses.create = AsyncMock(side_effect=asyncio.TimeoutError())

        client = OpenAIClient("gpt-4o-mini", client=mock_client, timeout_seconds=0.1, max_retries=0)

        runtime = AgentRuntime.create(settings=self.settings, llm=client)
        result = await runtime.run_command("patrol the lobby")

        # Should still complete via fallback
        self.assertEqual("completed", result.status)
        self.assertTrue(client.is_degraded)

        # Decision context should record degraded state
        if runtime._database is not None:
            decision = runtime._database.latest_decision_context()
            self.assertIsNotNone(decision)
            self.assertTrue(decision["is_degraded"])


class EvaluationScenariosTests(unittest.IsolatedAsyncioTestCase):
    """Run the built-in evaluation scenarios as regression tests."""

    async def test_patrol_blocked_replan(self) -> None:
        from robot_brain.evaluation import EvaluationHarness, SCENARIOS

        harness = EvaluationHarness()
        scenario = next(s for s in SCENARIOS if s.name == "patrol_blocked_replan")
        result = await harness.run_scenario(scenario)
        self.assertTrue(result.passed, f"Scenario failed: {result.assertions} error={result.error}")

    async def test_low_battery_dock(self) -> None:
        from robot_brain.evaluation import EvaluationHarness, SCENARIOS

        harness = EvaluationHarness()
        scenario = next(s for s in SCENARIOS if s.name == "low_battery_dock")
        result = await harness.run_scenario(scenario)
        self.assertTrue(result.passed, f"Scenario failed: {result.assertions} error={result.error}")

    async def test_follow_confirm(self) -> None:
        from robot_brain.evaluation import EvaluationHarness, SCENARIOS

        harness = EvaluationHarness()
        scenario = next(s for s in SCENARIOS if s.name == "follow_confirm")
        result = await harness.run_scenario(scenario)
        self.assertTrue(result.passed, f"Scenario failed: {result.assertions} error={result.error}")

    async def test_run_all_scenarios(self) -> None:
        from robot_brain.evaluation import EvaluationHarness, SCENARIOS

        harness = EvaluationHarness()
        await harness.run_all(SCENARIOS)
        summary = harness.summary()
        self.assertEqual(0, summary["failed"], f"Failed scenarios: {summary['scenarios']}")
        self.assertGreaterEqual(summary["total"], 3)


if __name__ == "__main__":
    unittest.main()
