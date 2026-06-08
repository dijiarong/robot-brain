"""Tests for execution summary generation and storage."""
from __future__ import annotations

import unittest

from config.settings import Settings
from robot_brain.memory.execution_summary import ExecutionSummary, InMemoryExecutionSummaryStore
from robot_brain.memory.sqlite_store import SQLiteMemoryStore
from robot_brain.runtime.loop import AgentRuntime


class ExecutionSummaryModelTests(unittest.TestCase):
    def test_summary_creation(self) -> None:
        summary = ExecutionSummary(
            thread_id="test-thread-1",
            task_id="task-1",
            objective="patrol the lobby",
            outcome="completed",
            skills_executed=["navigate", "recognize"],
            duration_seconds=1.5,
            memory_refs=["previous patrol was successful"],
            decision_source="slow",
        )
        self.assertEqual("test-thread-1", summary.thread_id)
        self.assertEqual("completed", summary.outcome)
        self.assertEqual(["navigate", "recognize"], summary.skills_executed)
        self.assertIsNone(summary.failure_reason)

    def test_failed_summary(self) -> None:
        summary = ExecutionSummary(
            thread_id="test-thread-2",
            objective="navigate to exit",
            outcome="failed",
            failure_reason="path blocked by obstacle",
            decision_source="slow",
        )
        self.assertEqual("failed", summary.outcome)
        self.assertEqual("path blocked by obstacle", summary.failure_reason)


class InMemoryExecutionSummaryStoreTests(unittest.TestCase):
    def test_save_and_get(self) -> None:
        store = InMemoryExecutionSummaryStore()
        summary = ExecutionSummary(
            thread_id="thread-1",
            objective="patrol",
            outcome="completed",
        )
        store.save_summary(summary)
        retrieved = store.get_summary("thread-1")
        self.assertIsNotNone(retrieved)
        self.assertEqual("patrol", retrieved.objective)

    def test_get_nonexistent(self) -> None:
        store = InMemoryExecutionSummaryStore()
        self.assertIsNone(store.get_summary("nonexistent"))

    def test_list_summaries(self) -> None:
        store = InMemoryExecutionSummaryStore()
        for i in range(5):
            store.save_summary(ExecutionSummary(
                thread_id=f"thread-{i}",
                objective=f"task {i}",
                outcome="completed",
            ))
        summaries = store.list_summaries(limit=3)
        self.assertEqual(3, len(summaries))
        # Most recent first
        self.assertEqual("thread-4", summaries[0].thread_id)


class SQLiteExecutionSummaryTests(unittest.TestCase):
    def setUp(self) -> None:
        self.db = SQLiteMemoryStore(":memory:")

    def tearDown(self) -> None:
        self.db.close()

    def test_save_and_get_summary(self) -> None:
        summary = ExecutionSummary(
            thread_id="t-1",
            task_id="task-1",
            objective="patrol lobby",
            outcome="completed",
            skills_executed=["navigate", "recognize"],
            duration_seconds=2.3,
            memory_refs=["ref1", "ref2"],
            decision_source="slow",
        )
        self.db.save_summary(summary)
        retrieved = self.db.get_summary("t-1")
        self.assertIsNotNone(retrieved)
        self.assertEqual("patrol lobby", retrieved.objective)
        self.assertEqual(["navigate", "recognize"], retrieved.skills_executed)
        self.assertEqual(2.3, retrieved.duration_seconds)

    def test_list_summaries(self) -> None:
        for i in range(5):
            self.db.save_summary(ExecutionSummary(
                thread_id=f"thread-{i}",
                objective=f"task {i}",
                outcome="completed",
                duration_seconds=float(i),
            ))
        summaries = self.db.list_summaries(limit=3)
        self.assertEqual(3, len(summaries))

    def test_failed_summary_with_reason(self) -> None:
        summary = ExecutionSummary(
            thread_id="fail-1",
            objective="navigate to exit",
            outcome="failed",
            failure_reason="path blocked",
            skills_executed=["navigate"],
            duration_seconds=0.5,
        )
        self.db.save_summary(summary)
        retrieved = self.db.get_summary("fail-1")
        self.assertEqual("failed", retrieved.outcome)
        self.assertEqual("path blocked", retrieved.failure_reason)


class RuntimeExecutionSummaryIntegrationTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self) -> None:
        self.settings = Settings(memory_db_path=":memory:")

    async def test_run_command_generates_summary(self) -> None:
        runtime = AgentRuntime.create(settings=self.settings)
        result = await runtime.run_command("patrol the lobby")
        self.assertEqual("completed", result.status)

        # Check that summary was generated
        summary = runtime.summaries.get_summary(result.thread_id)
        self.assertIsNotNone(summary)
        self.assertEqual("patrol the lobby", summary.objective)
        self.assertEqual("completed", summary.outcome)
        self.assertGreater(summary.duration_seconds, 0)

    async def test_failed_command_generates_summary(self) -> None:
        from robot_brain.llm.base import ToolCall
        from robot_brain.llm.mock import MockLLM

        llm = MockLLM([[ToolCall(skill_name="navigate", parameters={"target": {"x": 100, "y": 0}})]])
        runtime = AgentRuntime.create(settings=self.settings, llm=llm)
        result = await runtime.run_command("go far away")
        self.assertEqual("blocked", result.status)

        summary = runtime.summaries.get_summary(result.thread_id)
        self.assertIsNotNone(summary)
        self.assertEqual("blocked", summary.outcome)
        self.assertIsNotNone(summary.failure_reason)

    async def test_multiple_commands_list_summaries(self) -> None:
        runtime = AgentRuntime.create(settings=self.settings)
        await runtime.run_command("patrol the lobby")
        await runtime.run_command("stop")

        summaries = runtime.summaries.list_summaries()
        self.assertGreaterEqual(len(summaries), 2)


if __name__ == "__main__":
    unittest.main()
