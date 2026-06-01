from __future__ import annotations

import unittest

from config.settings import Settings
from robot_brain.actuation.mock import MockRobot
from robot_brain.core.events import Event, EventType
from robot_brain.core.world_state import DetectedObject
from robot_brain.llm.base import ToolCall
from robot_brain.llm.mock import MockLLM
from robot_brain.memory.long_term import Experience, LongTermMemory
from robot_brain.perception.base import Observation
from robot_brain.perception.mock import MockPerception
from robot_brain.runtime.loop import AgentRuntime
from robot_brain.skills.base import Skill, SkillResult


class AgentRuntimeTests(unittest.IsolatedAsyncioTestCase):
    async def test_patrol_uses_slow_planner(self) -> None:
        robot = MockRobot()
        runtime = AgentRuntime.create(robot=robot)

        result = await runtime.run_command("patrol the lobby")

        self.assertEqual("completed", result.status)
        self.assertEqual("slow", result.decision_source)
        self.assertEqual(["move_to", "move_to"], [item["action"] for item in robot.action_history])

    async def test_low_battery_reflex_docks_before_planning(self) -> None:
        robot = MockRobot()
        perception = MockPerception(robot, [Observation(battery_level=20.0)])
        runtime = AgentRuntime.create(robot=robot, perception=perception)

        result = await runtime.run_command("patrol the lobby")

        self.assertEqual("completed", result.status)
        self.assertEqual("fast", result.decision_source)
        self.assertEqual("dock", robot.action_history[0]["action"])

    async def test_unsafe_navigation_is_blocked(self) -> None:
        robot = MockRobot()
        llm = MockLLM([[ToolCall(skill_name="navigate", parameters={"target": {"x": 100, "y": 0}})]])
        runtime = AgentRuntime.create(robot=robot, llm=llm)

        result = await runtime.run_command("go far away")

        self.assertEqual("blocked", result.status)
        self.assertIn("max_step_distance", result.message)
        self.assertEqual([], robot.action_history)

    async def test_follow_waits_for_confirmation_and_resumes(self) -> None:
        robot = MockRobot()
        perception = MockPerception(
            robot,
            [Observation(detected_objects=[DetectedObject(object_id="person-1", kind="person")])],
        )
        runtime = AgentRuntime.create(robot=robot, perception=perception)

        pending = await runtime.run_command("follow person-1", thread_id="follow-thread")
        resumed = await runtime.resume("follow-thread", approved=True)

        self.assertEqual("awaiting_confirmation", pending.status)
        self.assertEqual("completed", resumed.status)
        self.assertEqual("follow", robot.action_history[0]["action"])

    async def test_interrupt_activates_estop(self) -> None:
        robot = MockRobot()
        runtime = AgentRuntime.create(robot=robot)

        result = await runtime.handle_event(Event(type=EventType.INTERRUPT, message="operator stop"))

        self.assertEqual("interrupted", result.status)
        self.assertTrue(runtime.context.world.estop_active)
        self.assertEqual("stop", robot.action_history[0]["action"])

    async def test_failed_skill_replans_after_fresh_perception(self) -> None:
        robot = MockRobot()
        llm = MockLLM(
            [
                [ToolCall(skill_name="recognize", parameters={"kind": "missing"})],
                [ToolCall(skill_name="report", parameters={"message": "inspection inconclusive", "severity": "warning"})],
            ]
        )
        runtime = AgentRuntime.create(robot=robot, llm=llm)

        result = await runtime.run_command("inspect the area")

        self.assertEqual("completed", result.status)
        self.assertEqual([False, True], [item.success for item in result.results])
        self.assertEqual("report", robot.action_history[0]["action"])
        self.assertIn("reflection requested replanning", runtime.context.short_term.recent())

    async def test_warning_alert_uses_slow_investigation_then_reports(self) -> None:
        robot = MockRobot()
        perception = MockPerception(
            robot,
            [
                Observation(
                    detected_objects=[DetectedObject(object_id="box-7", kind="unattended_box")],
                    alerts=["unattended box detected"],
                )
            ],
        )
        runtime = AgentRuntime.create(robot=robot, perception=perception)

        result = await runtime.run_command("inspect the anomaly")

        self.assertEqual("completed", result.status)
        self.assertEqual("slow", result.decision_source)
        self.assertEqual(["recognize", "report"], runtime.context.world.current_task.completed_skills)
        self.assertEqual("report", robot.action_history[0]["action"])

    async def test_unfinished_skill_replans_until_done(self) -> None:
        class IncompleteOnceSkill(Skill):
            name = "incomplete_once"
            description = "Require a second planning cycle before completion."

            def __init__(self) -> None:
                self.attempts = 0

            async def execute(self, params, robot, world) -> SkillResult:
                self.attempts += 1
                return SkillResult(success=True, message=f"attempt {self.attempts}")

            def is_done(self, world) -> bool:
                return self.attempts >= 2

        skill = IncompleteOnceSkill()
        llm = MockLLM(
            [
                [ToolCall(skill_name=skill.name)],
                [ToolCall(skill_name=skill.name)],
            ]
        )
        runtime = AgentRuntime.create(llm=llm)
        runtime.context.skills.register(skill)

        result = await runtime.run_command("run iterative skill")

        self.assertEqual("completed", result.status)
        self.assertEqual(2, skill.attempts)
        self.assertEqual([skill.name], runtime.context.world.current_task.completed_skills)

    async def test_critical_alert_uses_fast_report(self) -> None:
        robot = MockRobot()
        perception = MockPerception(robot, [Observation(alerts=["critical: smoke detected"])])
        runtime = AgentRuntime.create(robot=robot, perception=perception)

        result = await runtime.run_command("patrol the lobby")

        self.assertEqual("completed", result.status)
        self.assertEqual("fast", result.decision_source)
        self.assertEqual("critical", robot.action_history[0]["severity"])

    async def test_long_term_memory_accepts_replaceable_store(self) -> None:
        class RecordingStore:
            def __init__(self) -> None:
                self.added: list[Experience] = []

            def add(self, experience: Experience) -> None:
                self.added.append(experience)

            def search(self, query: str, limit: int = 5) -> list[Experience]:
                return self.added[-limit:]

        store = RecordingStore()
        runtime = AgentRuntime.create(long_term=LongTermMemory(store))

        result = await runtime.run_command("stop")

        self.assertEqual("completed", result.status)
        self.assertEqual("stop", store.added[0].objective)


if __name__ == "__main__":
    unittest.main()
