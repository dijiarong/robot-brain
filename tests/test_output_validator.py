"""Tests for LLM output validation and structured error codes."""
from __future__ import annotations

import asyncio
import unittest
from unittest.mock import AsyncMock, MagicMock

from config.settings import Settings
from robot_brain.core.errors import BrainError, ErrorCode
from robot_brain.llm.base import ToolCall
from robot_brain.llm.openai_client import OpenAIClient
from robot_brain.llm.output_validator import LLMOutputValidator
from robot_brain.core.world_state import WorldState
from robot_brain.runtime.loop import AgentRuntime
from robot_brain.safety.validator import SafetyValidator, ValidationResult
from robot_brain.skills.builtin import default_skills
from robot_brain.skills.registry import SkillRegistry


class LLMOutputValidatorTests(unittest.TestCase):
    def setUp(self) -> None:
        self.skills = SkillRegistry(default_skills())
        self.validator = LLMOutputValidator(self.skills)

    def test_valid_calls_pass_through(self) -> None:
        calls = [
            ToolCall(skill_name="navigate", parameters={"target": {"x": 1, "y": 2}}),
            ToolCall(skill_name="stop", parameters={"reason": "test"}),
        ]
        valid, errors = self.validator.validate_tool_calls(calls)
        self.assertEqual(2, len(valid))
        self.assertEqual(0, len(errors))

    def test_unknown_skill_rejected(self) -> None:
        calls = [
            ToolCall(skill_name="fly_to_moon", parameters={}),
            ToolCall(skill_name="navigate", parameters={"target": {"x": 1, "y": 2}}),
        ]
        valid, errors = self.validator.validate_tool_calls(calls)
        self.assertEqual(1, len(valid))
        self.assertEqual("navigate", valid[0].skill_name)
        self.assertEqual(1, len(errors))
        self.assertEqual(ErrorCode.LLM_UNKNOWN_SKILL, errors[0].code)
        self.assertIn("fly_to_moon", errors[0].message)

    def test_invalid_params_rejected(self) -> None:
        calls = [
            ToolCall(skill_name="navigate", parameters={"target": "not_a_position"}),
        ]
        valid, errors = self.validator.validate_tool_calls(calls)
        self.assertEqual(0, len(valid))
        self.assertEqual(1, len(errors))
        self.assertEqual(ErrorCode.LLM_PARAM_VALIDATION, errors[0].code)

    def test_empty_list_passes(self) -> None:
        valid, errors = self.validator.validate_tool_calls([])
        self.assertEqual(0, len(valid))
        self.assertEqual(0, len(errors))

    def test_multiple_errors_collected(self) -> None:
        calls = [
            ToolCall(skill_name="unknown1", parameters={}),
            ToolCall(skill_name="unknown2", parameters={}),
            ToolCall(skill_name="navigate", parameters={"target": "bad"}),
        ]
        valid, errors = self.validator.validate_tool_calls(calls)
        self.assertEqual(0, len(valid))
        self.assertEqual(3, len(errors))
        self.assertEqual(ErrorCode.LLM_UNKNOWN_SKILL, errors[0].code)
        self.assertEqual(ErrorCode.LLM_UNKNOWN_SKILL, errors[1].code)
        self.assertEqual(ErrorCode.LLM_PARAM_VALIDATION, errors[2].code)


class OpenAIClientValidationTests(unittest.IsolatedAsyncioTestCase):
    async def test_malformed_json_arguments_skipped(self) -> None:
        mock_client = MagicMock()
        mock_client.responses = MagicMock()

        good_item = MagicMock()
        good_item.type = "function_call"
        good_item.name = "stop"
        good_item.arguments = '{"reason": "test"}'

        bad_item = MagicMock()
        bad_item.type = "function_call"
        bad_item.name = "navigate"
        bad_item.arguments = "not valid json {{"

        mock_response = MagicMock()
        mock_response.output = [bad_item, good_item]
        mock_client.responses.create = AsyncMock(return_value=mock_response)

        skills = SkillRegistry(default_skills())
        client = OpenAIClient("gpt-4o-mini", client=mock_client, timeout_seconds=5.0, skills=skills)
        world = WorldState()
        tools = skills.tools()

        result = await client.plan("stop now", world, tools, [])

        self.assertEqual(1, len(result))
        self.assertEqual("stop", result[0].skill_name)
        # Parse error + possible validation errors collected
        all_errors = client.validation_errors
        parse_errors = [e for e in all_errors if e.code == ErrorCode.LLM_INVALID_OUTPUT]
        self.assertEqual(1, len(parse_errors))

    async def test_unknown_skill_filtered_by_validator(self) -> None:
        mock_client = MagicMock()
        mock_client.responses = MagicMock()

        item = MagicMock()
        item.type = "function_call"
        item.name = "hack_the_planet"
        item.arguments = '{}'

        mock_response = MagicMock()
        mock_response.output = [item]
        mock_client.responses.create = AsyncMock(return_value=mock_response)

        skills = SkillRegistry(default_skills())
        client = OpenAIClient("gpt-4o-mini", client=mock_client, timeout_seconds=5.0, skills=skills)
        world = WorldState()

        result = await client.plan("do something", world, skills.tools(), [])

        self.assertEqual(0, len(result))
        self.assertTrue(any(e.code == ErrorCode.LLM_UNKNOWN_SKILL for e in client.validation_errors))

    async def test_timeout_sets_last_error(self) -> None:
        mock_client = MagicMock()
        mock_client.responses = MagicMock()
        mock_client.responses.create = AsyncMock(side_effect=asyncio.TimeoutError())

        client = OpenAIClient("gpt-4o-mini", client=mock_client, timeout_seconds=1.0, max_retries=0)
        world = WorldState()

        await client.plan("stop", world, [], [])

        self.assertTrue(client.is_degraded)
        self.assertIsNotNone(client.last_error)
        self.assertEqual(ErrorCode.LLM_TIMEOUT, client.last_error.code)

    async def test_api_error_sets_last_error(self) -> None:
        mock_client = MagicMock()
        mock_client.responses = MagicMock()
        mock_client.responses.create = AsyncMock(side_effect=RuntimeError("rate limit"))

        client = OpenAIClient("gpt-4o-mini", client=mock_client, timeout_seconds=5.0, max_retries=0)
        world = WorldState()

        await client.plan("stop", world, [], [])

        self.assertTrue(client.is_degraded)
        self.assertIsNotNone(client.last_error)
        self.assertEqual(ErrorCode.LLM_API_ERROR, client.last_error.code)
        self.assertIn("rate limit", client.last_error.message)


class SafetyValidatorErrorCodeTests(unittest.TestCase):
    def setUp(self) -> None:
        self.settings = Settings()
        self.skills = SkillRegistry(default_skills())
        self.validator = SafetyValidator(self.settings, self.skills)

    def test_unknown_skill_error_code(self) -> None:
        call = ToolCall(skill_name="nonexistent", parameters={})
        result = self.validator.validate(call, WorldState())
        self.assertFalse(result.allowed)
        self.assertEqual(ErrorCode.SAFETY_NOT_WHITELISTED, result.error_code)

    def test_invalid_params_error_code(self) -> None:
        call = ToolCall(skill_name="navigate", parameters={"target": "bad", "speed": "fast"})
        result = self.validator.validate(call, WorldState())
        self.assertFalse(result.allowed)
        self.assertEqual(ErrorCode.SAFETY_INVALID_PARAMS, result.error_code)

    def test_estop_error_code(self) -> None:
        call = ToolCall(skill_name="navigate", parameters={"target": {"x": 1, "y": 2}})
        world = WorldState(estop_active=True)
        result = self.validator.validate(call, world)
        self.assertFalse(result.allowed)
        self.assertEqual(ErrorCode.SAFETY_ESTOP_ACTIVE, result.error_code)

    def test_battery_critical_error_code(self) -> None:
        call = ToolCall(skill_name="navigate", parameters={"target": {"x": 1, "y": 2}})
        world = WorldState(battery_level=5.0)
        result = self.validator.validate(call, world)
        self.assertFalse(result.allowed)
        self.assertEqual(ErrorCode.SAFETY_BATTERY_CRITICAL, result.error_code)

    def test_confirmation_required_error_code(self) -> None:
        from datetime import datetime, timezone
        from robot_brain.core.world_state import DetectedObject, Position

        call = ToolCall(skill_name="follow", parameters={"target_id": "person-1"})
        world = WorldState()
        world.known_objects["person-1"] = DetectedObject(
            object_id="person-1",
            kind="person",
            position=Position(x=1, y=1),
            last_seen_at=datetime.now(timezone.utc),
        )
        result = self.validator.validate(call, world)
        self.assertFalse(result.allowed)
        self.assertEqual(ErrorCode.SAFETY_CONFIRMATION_REQUIRED, result.error_code)

    def test_motion_violation_error_code(self) -> None:
        call = ToolCall(skill_name="navigate", parameters={"target": {"x": 100, "y": 100}})
        result = self.validator.validate(call, WorldState())
        self.assertFalse(result.allowed)
        self.assertEqual(ErrorCode.SAFETY_MOTION_VIOLATION, result.error_code)

    def test_allowed_has_no_error_code(self) -> None:
        call = ToolCall(skill_name="stop", parameters={"reason": "test"})
        result = self.validator.validate(call, WorldState())
        self.assertTrue(result.allowed)
        self.assertIsNone(result.error_code)


class RunResultErrorCodeTests(unittest.IsolatedAsyncioTestCase):
    async def test_error_code_propagates_to_run_result(self) -> None:
        settings = Settings(memory_db_path=":memory:", max_loop_iterations=1)
        runtime = AgentRuntime.create(settings=settings)
        result = await runtime.run_command("patrol the lobby")
        # With max_loop_iterations=1, depending on execution path it may complete or fail
        # The key test: error_code field exists and is serializable
        dumped = result.model_dump(mode="json")
        self.assertIn("error_code", dumped)

    async def test_blocked_result_has_error_code(self) -> None:
        from robot_brain.llm.mock import MockLLM

        scripted = [[ToolCall(skill_name="navigate", parameters={"target": {"x": 100, "y": 100}})]]
        llm = MockLLM(scripted_plans=scripted)
        settings = Settings(memory_db_path=":memory:")
        runtime = AgentRuntime.create(settings=settings, llm=llm)
        result = await runtime.run_command("go far away")
        self.assertEqual("blocked", result.status)
        self.assertEqual(ErrorCode.SAFETY_MOTION_VIOLATION, result.error_code)


class SkillRegistryStrictTests(unittest.TestCase):
    def test_tools_strict_by_default(self) -> None:
        skills = SkillRegistry(default_skills())
        tools = skills.tools()
        for tool in tools:
            self.assertTrue(tool["strict"])

    def test_tools_strict_can_be_disabled(self) -> None:
        skills = SkillRegistry(default_skills())
        tools = skills.tools(strict=False)
        for tool in tools:
            self.assertFalse(tool["strict"])

    def test_has_method(self) -> None:
        skills = SkillRegistry(default_skills())
        self.assertTrue(skills.has("navigate"))
        self.assertTrue(skills.has("stop"))
        self.assertFalse(skills.has("fly"))


class PlannerValidationIntegrationTests(unittest.IsolatedAsyncioTestCase):
    async def test_planner_filters_invalid_llm_output(self) -> None:
        from robot_brain.cognition.planner import Planner
        from robot_brain.llm.mock import MockLLM
        from robot_brain.memory.long_term import LongTermMemory
        from robot_brain.memory.short_term import ShortTermMemory

        scripted = [[
            ToolCall(skill_name="navigate", parameters={"target": {"x": 1, "y": 2}}),
            ToolCall(skill_name="nonexistent_skill", parameters={}),
        ]]
        llm = MockLLM(scripted_plans=scripted)
        skills = SkillRegistry(default_skills())
        short_term = ShortTermMemory()
        long_term = LongTermMemory()

        planner = Planner(llm, skills, short_term, long_term)
        world = WorldState()
        calls = await planner.plan("go to 1 2", world)

        self.assertEqual(1, len(calls))
        self.assertEqual("navigate", calls[0].skill_name)
        # Error should be logged in short-term memory
        recent = short_term.recent()
        self.assertTrue(any("LLM output rejected" in m for m in recent))


if __name__ == "__main__":
    unittest.main()
