"""Tests for thread history replay functionality."""
from __future__ import annotations

import unittest

from config.settings import Settings
from robot_brain.memory.sqlite_store import SQLiteMemoryStore
from robot_brain.runtime.loop import AgentRuntime


class SQLiteThreadReplayTests(unittest.TestCase):
    def setUp(self) -> None:
        self.db = SQLiteMemoryStore(":memory:")

    def tearDown(self) -> None:
        self.db.close()

    def test_empty_thread_replay(self) -> None:
        replay = self.db.thread_replay("nonexistent-thread")
        self.assertEqual("nonexistent-thread", replay["thread_id"])
        self.assertEqual([], replay["messages"])
        self.assertEqual([], replay["tasks"])
        self.assertEqual([], replay["world_states"])
        self.assertEqual([], replay["execution_summaries"])

    def test_replay_with_messages(self) -> None:
        from robot_brain.memory.conversation import ConversationMessage

        msg = ConversationMessage(
            thread_id="t-1",
            role="user",
            content="patrol the lobby",
            message_type="command",
        )
        self.db.add_message(msg)

        replay = self.db.thread_replay("t-1")
        self.assertEqual(1, len(replay["messages"]))
        self.assertEqual("patrol the lobby", replay["messages"][0]["content"])
        self.assertEqual("user", replay["messages"][0]["role"])

    def test_replay_with_world_states(self) -> None:
        from robot_brain.core.world_state import WorldState
        from robot_brain.memory.world_state import WorldStateSnapshot

        snapshot = WorldStateSnapshot(
            state=WorldState(),
            reason="test",
            thread_id="t-1",
        )
        self.db.save_world_state(snapshot)

        replay = self.db.thread_replay("t-1")
        self.assertEqual(1, len(replay["world_states"]))
        self.assertEqual("test", replay["world_states"][0]["reason"])


class RuntimeThreadReplayIntegrationTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self) -> None:
        self.settings = Settings(memory_db_path=":memory:")

    async def test_full_thread_replay(self) -> None:
        """Run a command and verify the thread replay contains all expected data."""
        runtime = AgentRuntime.create(settings=self.settings)
        result = await runtime.run_command("patrol the lobby", thread_id="replay-test-1")

        self.assertEqual("completed", result.status)
        self.assertEqual("replay-test-1", result.thread_id)

        # Get replay data
        replay = runtime._database.thread_replay("replay-test-1")

        # Should have messages (at least user command + assistant result)
        self.assertGreater(len(replay["messages"]), 0)
        # First message should be the user command
        self.assertEqual("user", replay["messages"][0]["role"])
        self.assertEqual("patrol the lobby", replay["messages"][0]["content"])

        # Should have world state snapshots
        self.assertGreater(len(replay["world_states"]), 0)

        # Should have execution summary
        self.assertGreater(len(replay["execution_summaries"]), 0)
        self.assertEqual("completed", replay["execution_summaries"][0]["outcome"])

    async def test_multiple_commands_same_thread(self) -> None:
        """Multiple commands on the same thread produce comprehensive replay."""
        runtime = AgentRuntime.create(settings=self.settings)
        await runtime.run_command("patrol the lobby", thread_id="multi-cmd")
        await runtime.run_command("stop", thread_id="multi-cmd")

        replay = runtime._database.thread_replay("multi-cmd")

        # Should have messages from both commands
        user_messages = [m for m in replay["messages"] if m["role"] == "user"]
        self.assertGreaterEqual(len(user_messages), 2)

    async def test_replay_includes_task_state(self) -> None:
        """Replay includes scheduled task data when tasks exist for the thread."""
        runtime = AgentRuntime.create(settings=self.settings)
        from robot_brain.core.tasks import ScheduledTask

        task = ScheduledTask(
            task_id="eval-task-1",
            thread_id="task-thread",
            objective="patrol",
            priority=1,
            source="test",
        )
        runtime.tasks.store.save_task(task)

        replay = runtime._database.thread_replay("task-thread")
        self.assertEqual(1, len(replay["tasks"]))
        self.assertEqual("eval-task-1", replay["tasks"][0]["task_id"])


if __name__ == "__main__":
    unittest.main()
