from __future__ import annotations

from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from config.settings import Settings
from robot_brain.actuation.mock import MockRobot
from robot_brain.core.world_state import DetectedObject
from robot_brain.memory.conversation import ConversationMemory
from robot_brain.memory.short_term import ShortTermMemory
from robot_brain.memory.sqlite_store import SQLiteMemoryStore
from robot_brain.perception.base import Observation
from robot_brain.perception.mock import MockPerception
from robot_brain.runtime.loop import AgentRuntime


class SQLiteMemoryTests(unittest.IsolatedAsyncioTestCase):
    async def test_experiences_and_messages_survive_new_runtime(self) -> None:
        with TemporaryDirectory() as directory:
            settings = Settings(memory_db_path=str(Path(directory) / "memory.sqlite3"))
            first = AgentRuntime.create(settings=settings)

            result = await first.run_command("stop", thread_id="persistent-thread")
            first.close()

            second = AgentRuntime.create(settings=settings)
            experiences = second.context.long_term.search("stop")
            messages = second.conversations.recent("persistent-thread")

            self.assertEqual("completed", result.status)
            self.assertEqual("stop", experiences[0].objective)
            self.assertEqual(["command", "runtime_result"], [item.message_type for item in messages])
            self.assertEqual(["user", "assistant"], [item.role for item in messages])
            second.close()

    async def test_checkpoint_survives_new_runtime_and_resumes(self) -> None:
        with TemporaryDirectory() as directory:
            settings = Settings(memory_db_path=str(Path(directory) / "memory.sqlite3"))
            first_robot = MockRobot()
            first_perception = MockPerception(
                first_robot,
                [Observation(detected_objects=[DetectedObject(object_id="person-1", kind="person")])],
            )
            first = AgentRuntime.create(settings=settings, robot=first_robot, perception=first_perception)

            pending = await first.run_command("follow person-1", thread_id="follow-thread")
            first.close()

            second_robot = MockRobot()
            second_perception = MockPerception(
                second_robot,
                [Observation(detected_objects=[DetectedObject(object_id="person-1", kind="person")])],
            )
            second = AgentRuntime.create(settings=settings, robot=second_robot, perception=second_perception)
            resumed = await second.resume("follow-thread", approved=True)

            self.assertEqual("awaiting_confirmation", pending.status)
            self.assertEqual("completed", resumed.status)
            self.assertEqual("follow", second_robot.action_history[0]["action"])
            self.assertIsNone(second.checkpoints.get("follow-thread"))
            second.close()

    async def test_existing_conversation_restores_bounded_short_term_context(self) -> None:
        with TemporaryDirectory() as directory:
            database = SQLiteMemoryStore(Path(directory) / "memory.sqlite3")
            conversations = ConversationMemory(database)
            for index in range(60):
                conversations.add(
                    thread_id="long-thread",
                    role="user",
                    content=f"message {index}",
                )
            runtime = AgentRuntime.create(
                settings=Settings(memory_db_path=str(Path(directory) / "memory.sqlite3")),
                conversations=conversations,
            )

            await runtime.run_command("stop", thread_id="long-thread")

            recent = runtime.context.short_term.recent(limit=100)
            self.assertLessEqual(len(recent), ShortTermMemory().capacity)
            self.assertTrue(any("message 59" in item for item in recent))
            self.assertFalse(any("message 0" in item for item in recent))
            runtime.close()
            database.close()


if __name__ == "__main__":
    unittest.main()
