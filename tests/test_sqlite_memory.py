from __future__ import annotations

from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from config.settings import Settings
from robot_brain.actuation.mock import MockRobot
from robot_brain.core.events import Event, EventType
from robot_brain.core.world_state import DetectedObject, Position
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

    async def test_world_state_survives_new_runtime(self) -> None:
        with TemporaryDirectory() as directory:
            settings = Settings(memory_db_path=str(Path(directory) / "memory.sqlite3"))
            first_robot = MockRobot()
            observation = Observation(
                position=Position(x=3, y=4),
                battery_level=77.0,
                detected_objects=[DetectedObject(object_id="box-7", kind="package")],
            )
            first = AgentRuntime.create(
                settings=settings,
                robot=first_robot,
                perception=MockPerception(first_robot, [observation, observation]),
            )

            await first.run_command("stop", thread_id="state-thread")
            first.close()

            second = AgentRuntime.create(settings=settings)

            self.assertEqual(Position(x=3, y=4), second.context.world.position)
            self.assertEqual(77.0, second.context.world.battery_level)
            self.assertIn("box-7", second.context.world.known_objects)
            self.assertEqual("stop", second.context.world.current_task.objective)
            self.assertEqual("completed", second.context.world.current_task.status)
            second.close()

    async def test_estop_state_survives_restart_and_reset(self) -> None:
        with TemporaryDirectory() as directory:
            settings = Settings(memory_db_path=str(Path(directory) / "memory.sqlite3"))
            first = AgentRuntime.create(settings=settings)

            await first.handle_event(Event(type=EventType.INTERRUPT, message="operator stop"))
            first.close()

            second = AgentRuntime.create(settings=settings)
            self.assertTrue(second.context.world.estop_active)
            second.reset_estop()
            second.close()

            third = AgentRuntime.create(settings=settings)
            self.assertFalse(third.context.world.estop_active)
            third.close()

    async def test_checkpoint_resume_uses_fresh_perception_for_safety(self) -> None:
        with TemporaryDirectory() as directory:
            settings = Settings(memory_db_path=str(Path(directory) / "memory.sqlite3"))
            first_robot = MockRobot()
            first = AgentRuntime.create(
                settings=settings,
                robot=first_robot,
                perception=MockPerception(
                    first_robot,
                    [Observation(detected_objects=[DetectedObject(object_id="person-1", kind="person")])],
                ),
            )

            pending = await first.run_command("follow person-1", thread_id="safe-resume-thread")
            first.close()

            second_robot = MockRobot()
            second = AgentRuntime.create(
                settings=settings,
                robot=second_robot,
                perception=MockPerception(second_robot, [Observation(battery_level=5.0)]),
            )
            resumed = await second.resume("safe-resume-thread", approved=True)

            self.assertEqual("awaiting_confirmation", pending.status)
            self.assertEqual("blocked", resumed.status)
            self.assertIn("critical battery", resumed.message)
            self.assertEqual(5.0, second.context.world.battery_level)
            self.assertEqual([], second_robot.action_history)
            second.close()


if __name__ == "__main__":
    unittest.main()
