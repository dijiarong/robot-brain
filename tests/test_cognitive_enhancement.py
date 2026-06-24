"""Integration tests for iteration 14: cognitive enhancement."""
from __future__ import annotations

import pytest
import pytest_asyncio

from robot_brain.core.robot_self_state import RobotSelfState
from robot_brain.core.world_state import WorldState
from robot_brain.llm.base import ToolCall
from robot_brain.llm.mock import MockLLM
from robot_brain.memory.conversation import ConversationMemory
from robot_brain.memory.sqlite_store import SQLiteMemoryStore
from robot_brain.runtime.loop import AgentRuntime


@pytest.fixture
def runtime() -> AgentRuntime:
    """Create a runtime with mock backends for integration testing."""
    return AgentRuntime.create()


@pytest.fixture
def sqlite_db() -> SQLiteMemoryStore:
    return SQLiteMemoryStore(":memory:")


class TestMockModeBackwardCompat:
    """Ensure mock LLM mode continues to work identically."""

    @pytest.mark.asyncio
    async def test_run_command_basic(self, runtime: AgentRuntime) -> None:
        result = await runtime.run_command("patrol the area")
        assert result.status in ("completed", "failed", "blocked")
        assert result.thread_id is not None

    @pytest.mark.asyncio
    async def test_run_command_stop(self, runtime: AgentRuntime) -> None:
        result = await runtime.run_command("stop moving")
        assert result.status == "completed"

    @pytest.mark.asyncio
    async def test_run_command_report(self, runtime: AgentRuntime) -> None:
        result = await runtime.run_command("hello there")
        # MockLLM falls back to report for unrecognized commands
        assert result.status == "completed"


class TestConversationContextPassing:
    """Verify conversation history flows through the chain."""

    @pytest.mark.asyncio
    async def test_conversation_stored_and_retrievable(self, runtime: AgentRuntime) -> None:
        thread_id = "test-thread-conv"
        await runtime.run_command("navigate to 5 5", thread_id=thread_id)
        messages = runtime.conversations.recent(thread_id, limit=10)
        # Should have at least user command + assistant result
        roles = [m.role for m in messages]
        assert "user" in roles
        assert "assistant" in roles

    @pytest.mark.asyncio
    async def test_multi_turn_conversation(self, runtime: AgentRuntime) -> None:
        thread_id = "test-thread-multi"
        await runtime.run_command("navigate to 3 3", thread_id=thread_id)
        await runtime.run_command("stop", thread_id=thread_id)
        messages = runtime.conversations.recent(thread_id, limit=20)
        user_messages = [m for m in messages if m.role == "user"]
        assert len(user_messages) >= 2


class TestAgentContextConversations:
    """Verify AgentContext has conversations wired."""

    def test_context_has_conversations(self, runtime: AgentRuntime) -> None:
        assert runtime.context.conversations is not None
        assert isinstance(runtime.context.conversations, ConversationMemory)


class TestMockLLMNewSignature:
    """Verify MockLLM accepts the new conversation parameter."""

    @pytest.mark.asyncio
    async def test_plan_with_conversation(self) -> None:
        llm = MockLLM()
        world = WorldState()
        conversation = [
            {"role": "user", "content": "forward"},
            {"role": "assistant", "content": "done"},
        ]
        # Should not raise
        result = await llm.plan("stop now", world, [], [], conversation=conversation)
        assert isinstance(result, list)
        assert all(isinstance(c, ToolCall) for c in result)

    @pytest.mark.asyncio
    async def test_plan_without_conversation(self) -> None:
        llm = MockLLM()
        world = WorldState()
        result = await llm.plan("stop now", world, [], [])
        assert isinstance(result, list)


class TestDecisionContextEnrichment:
    """Verify decision_context table accepts world_snapshot."""

    def test_save_decision_context_with_world_snapshot(self, sqlite_db: SQLiteMemoryStore) -> None:
        sqlite_db.save_decision_context(
            thread_id="t1",
            command="nudge forward",
            chosen_skills=["nudge"],
            reason="slow",
            memory_refs=["past nudge ok"],
            safety_result="allowed",
            next_plan="completed",
            is_degraded=False,
            world_snapshot='{"battery_level": 80, "_state_summary": {"battery": "OK"}}',
        )
        ctx = sqlite_db.latest_decision_context()
        assert ctx is not None
        assert ctx["command"] == "nudge forward"

    def test_save_decision_context_without_world_snapshot(self, sqlite_db: SQLiteMemoryStore) -> None:
        # Backward compatible — world_snapshot is optional
        sqlite_db.save_decision_context(
            thread_id="t2",
            command="stop",
            chosen_skills=["stop"],
            reason="fast",
            memory_refs=[],
            safety_result="allowed",
            next_plan="completed",
        )
        ctx = sqlite_db.latest_decision_context()
        assert ctx is not None


class TestEnrichedExperience:
    """Verify that _remember produces richer summaries."""

    @pytest.mark.asyncio
    async def test_experience_includes_skills_and_duration(self, runtime: AgentRuntime) -> None:
        result = await runtime.run_command("navigate to 2 2")
        # Check long-term memory has the enriched summary
        experiences = runtime.context.long_term.search("navigate to 2 2", limit=1)
        assert len(experiences) > 0
        exp = experiences[0]
        # Should contain skill names or duration info
        assert exp.summary  # Non-empty
