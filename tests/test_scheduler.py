from __future__ import annotations

from datetime import datetime, timedelta, timezone
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from config.settings import Settings
from robot_brain.actuation.base import RobotState
from robot_brain.actuation.mock import MockRobot
from robot_brain.core.events import Event, EventType
from robot_brain.core.tasks import TaskStatus
from robot_brain.core.world_state import DetectedObject, WorldState
from robot_brain.llm.base import ToolCall
from robot_brain.llm.mock import MockLLM
from robot_brain.perception.base import Observation
from robot_brain.perception.mock import MockPerception
from robot_brain.runtime.loop import AgentRuntime
from robot_brain.runtime.scheduler import AgentScheduler
from robot_brain.safety.validator import SafetyValidator
from robot_brain.skills.builtin import default_skills
from robot_brain.skills.registry import SkillRegistry


class ObjectLifecycleTests(unittest.TestCase):
    def test_stale_objects_expire_after_ttl(self) -> None:
        observed_at = datetime(2026, 6, 2, tzinfo=timezone.utc)
        world = WorldState()

        world.apply_observation(
            Observation(
                observed_at=observed_at,
                detected_objects=[DetectedObject(object_id="person-1", kind="person")],
            ),
            object_ttl_seconds=30,
        )
        world.apply_observation(
            Observation(observed_at=observed_at + timedelta(seconds=31)),
            object_ttl_seconds=30,
        )

        self.assertNotIn("person-1", world.known_objects)
        self.assertFalse(world.is_object_fresh("person-1", 30, now=observed_at + timedelta(seconds=31)))

    def test_follow_validation_rejects_stale_target(self) -> None:
        settings = Settings(object_ttl_seconds=30)
        world = WorldState(
            known_objects={
                "person-1": DetectedObject(
                    object_id="person-1",
                    kind="person",
                    last_seen_at=datetime.now(timezone.utc) - timedelta(seconds=31),
                )
            }
        )
        validator = SafetyValidator(settings, SkillRegistry(default_skills()))

        result = validator.validate(ToolCall(skill_name="follow", parameters={"target_id": "person-1"}), world)

        self.assertFalse(result.allowed)
        self.assertIn("recently", result.reason)


class SchedulerTests(unittest.IsolatedAsyncioTestCase):
    async def test_command_event_is_queued_and_executed(self) -> None:
        robot = MockRobot()
        runtime = AgentRuntime.create(settings=Settings(memory_db_path=":memory:"), robot=robot)
        scheduler = AgentScheduler(runtime)

        queued = await scheduler.handle_event(
            Event(type=EventType.COMMAND, message="stop", payload={"priority": 5})
        )
        completed = await scheduler.run_next()

        self.assertEqual(TaskStatus.QUEUED, queued.task.status)
        self.assertEqual(5, queued.task.priority)
        self.assertEqual(TaskStatus.COMPLETED, completed.task.status)
        self.assertEqual("stop", robot.action_history[0]["action"])
        runtime.close()

    async def test_warning_task_runs_before_normal_task(self) -> None:
        robot = MockRobot()
        runtime = AgentRuntime.create(settings=Settings(memory_db_path=":memory:"), robot=robot)
        scheduler = AgentScheduler(runtime)
        normal = scheduler.submit("patrol the lobby")
        warning = await scheduler.handle_event(Event(type=EventType.WARNING, message="inspect urgent warning"))

        result = await scheduler.run_next()

        self.assertEqual(warning.task.task_id, result.task.task_id)
        self.assertEqual(TaskStatus.COMPLETED, result.task.status)
        self.assertEqual(TaskStatus.QUEUED, scheduler.tasks.get(normal.task_id).status)
        runtime.close()

    async def test_tasks_survive_restart_and_running_tasks_are_requeued(self) -> None:
        with TemporaryDirectory() as directory:
            settings = Settings(memory_db_path=str(Path(directory) / "memory.sqlite3"))
            first = AgentRuntime.create(settings=settings)
            first_scheduler = AgentScheduler(first)
            task = first_scheduler.submit("patrol the lobby")
            first.tasks.update(task, status=TaskStatus.RUNNING, increment_attempts=True)
            first.close()

            second = AgentRuntime.create(settings=settings)
            second_scheduler = AgentScheduler(second)
            restored = second_scheduler.tasks.get(task.task_id)

            self.assertEqual(TaskStatus.QUEUED, restored.status)
            self.assertEqual(1, restored.attempts)
            self.assertIn("runtime restart", restored.last_message)
            second.close()

    async def test_cancelled_task_is_not_executed(self) -> None:
        robot = MockRobot()
        runtime = AgentRuntime.create(settings=Settings(memory_db_path=":memory:"), robot=robot)
        scheduler = AgentScheduler(runtime)
        task = scheduler.submit("patrol the lobby")

        cancelled = scheduler.cancel(task.task_id)
        result = await scheduler.run_next()

        self.assertEqual(TaskStatus.CANCELLED, cancelled.status)
        self.assertEqual("idle", result.status)
        self.assertEqual([], robot.action_history)
        runtime.close()

    async def test_failed_task_retries_until_limit(self) -> None:
        unsafe = [ToolCall(skill_name="navigate", parameters={"target": {"x": 100, "y": 0}})]
        runtime = AgentRuntime.create(
            settings=Settings(memory_db_path=":memory:"),
            llm=MockLLM([unsafe, unsafe]),
        )
        scheduler = AgentScheduler(runtime)
        scheduler.submit("go far away", max_attempts=2)

        first = await scheduler.run_next()
        second = await scheduler.run_next()

        self.assertEqual(TaskStatus.QUEUED, first.task.status)
        self.assertEqual(TaskStatus.FAILED, second.task.status)
        self.assertEqual(2, second.task.attempts)
        runtime.close()

    async def test_low_battery_recharges_before_consuming_task(self) -> None:
        robot = MockRobot(RobotState(battery_level=20.0))
        runtime = AgentRuntime.create(settings=Settings(memory_db_path=":memory:"), robot=robot)
        scheduler = AgentScheduler(runtime)
        task = scheduler.submit("patrol the lobby")

        recharge = await scheduler.run_next()
        retained = scheduler.tasks.get(task.task_id)
        patrol = await scheduler.run_next()

        self.assertEqual("auto_recharge", recharge.status)
        self.assertEqual(TaskStatus.QUEUED, retained.status)
        self.assertEqual(TaskStatus.COMPLETED, patrol.task.status)
        self.assertEqual("dock", robot.action_history[0]["action"])
        self.assertEqual(["move_to", "move_to"], [item["action"] for item in robot.action_history[1:]])
        runtime.close()

    async def test_interrupt_blocks_dispatch_until_reset(self) -> None:
        robot = MockRobot()
        runtime = AgentRuntime.create(settings=Settings(memory_db_path=":memory:"), robot=robot)
        scheduler = AgentScheduler(runtime)
        task = scheduler.submit("patrol the lobby")

        interrupted = await scheduler.handle_event(Event(type=EventType.INTERRUPT, message="operator stop"))
        paused = await scheduler.run_next()
        scheduler.reset_estop()
        completed = await scheduler.run_next()

        self.assertEqual("interrupted", interrupted.status)
        self.assertEqual("paused", paused.status)
        self.assertEqual(TaskStatus.COMPLETED, completed.task.status)
        self.assertEqual(TaskStatus.COMPLETED, scheduler.tasks.get(task.task_id).status)
        self.assertEqual("stop", robot.action_history[0]["action"])
        runtime.close()

    async def test_scheduler_resumes_confirmation_task(self) -> None:
        robot = MockRobot()
        runtime = AgentRuntime.create(
            settings=Settings(memory_db_path=":memory:"),
            robot=robot,
            perception=MockPerception(
                robot,
                [Observation(detected_objects=[DetectedObject(object_id="person-1", kind="person")])],
            ),
        )
        scheduler = AgentScheduler(runtime)
        task = scheduler.submit("follow person-1")

        pending = await scheduler.run_next()
        resumed = await scheduler.resume_task(task.task_id, approved=True)

        self.assertEqual(TaskStatus.AWAITING_CONFIRMATION, pending.task.status)
        self.assertEqual(TaskStatus.COMPLETED, resumed.task.status)
        self.assertEqual("follow", robot.action_history[0]["action"])
        runtime.close()


if __name__ == "__main__":
    unittest.main()
