"""Tests for the bounded explore composite skill."""
from __future__ import annotations

import unittest

from config.settings import Settings
from robot_brain.actuation.mock import MockRobot
from robot_brain.core.robot_self_state import RobotSelfState, UltrasonicData
from robot_brain.core.world_state import WorldState
from robot_brain.llm.base import ToolCall
from robot_brain.safety.validator import SafetyValidator
from robot_brain.skills.builtin.explore import ExploreParams, ExploreSkill
from robot_brain.skills.registry import SkillRegistry, UNITREE_LLM_SKILLS


class TestExploreNormalMock(unittest.IsolatedAsyncioTestCase):
    """Normal explore cycle on mock backend."""

    async def test_completes_max_steps(self) -> None:
        settings = Settings()
        skill = ExploreSkill(settings)
        robot = MockRobot()
        world = WorldState(
            battery_level=80.0,
            robot_self_state=RobotSelfState(
                source="test",
                ultrasonic=UltrasonicData(front_m=1.0, rear_m=1.0, left_m=1.0, right_m=1.0),
            ),
        )

        params = ExploreParams(max_steps=3, step_distance_cm=20.0, scan_degrees=45.0)
        result = await skill.execute(params, robot, world)

        self.assertTrue(result.success)
        self.assertEqual(result.data["steps_completed"], 3)
        self.assertEqual(result.data["stop_reason"], "max_steps")
        # Position should have changed
        self.assertNotEqual(world.position.x, 0.0)
        # Actions should contain scan + nudge for each step
        self.assertIn("scan", result.data["actions"])
        self.assertIn("nudge", result.data["actions"])

    async def test_heading_changes(self) -> None:
        settings = Settings()
        skill = ExploreSkill(settings)
        robot = MockRobot()
        world = WorldState(
            robot_self_state=RobotSelfState(
                source="test",
                ultrasonic=UltrasonicData(front_m=1.0, rear_m=1.0),
            ),
        )

        params = ExploreParams(max_steps=2, scan_degrees=30.0)
        await skill.execute(params, robot, world)

        # Heading should have rotated by scan_degrees * steps
        self.assertNotEqual(world.heading_degrees, 0.0)


class TestExploreNoUltrasonic(unittest.IsolatedAsyncioTestCase):
    """When ultrasonic data is absent, explore should NOT forward nudge."""

    async def test_no_ultrasonic_does_not_nudge_forward(self) -> None:
        settings = Settings()
        skill = ExploreSkill(settings)
        robot = MockRobot()
        # No robot_self_state → no ultrasonic
        world = WorldState(battery_level=80.0)

        params = ExploreParams(max_steps=2)
        result = await skill.execute(params, robot, world)

        self.assertTrue(result.success)
        # Should NOT contain "nudge" (forward) — only scan + retreat
        self.assertNotIn("nudge", result.data["actions"])


class TestExploreObstacles(unittest.IsolatedAsyncioTestCase):
    """Obstacle handling during explore."""

    async def test_front_obstacle_triggers_retreat(self) -> None:
        settings = Settings()
        skill = ExploreSkill(settings)
        robot = MockRobot()
        world = WorldState(
            battery_level=80.0,
            robot_self_state=RobotSelfState(
                source="test",
                ultrasonic=UltrasonicData(front_m=0.15, rear_m=1.0),
            ),
        )

        params = ExploreParams(max_steps=2, step_distance_cm=20.0)
        result = await skill.execute(params, robot, world)

        self.assertTrue(result.success)
        self.assertIn("retreat", result.data["actions"])

    async def test_all_blocked_stops_early(self) -> None:
        settings = Settings()
        skill = ExploreSkill(settings)
        robot = MockRobot()
        world = WorldState(
            battery_level=80.0,
            robot_self_state=RobotSelfState(
                source="test",
                ultrasonic=UltrasonicData(
                    front_m=0.1, rear_m=0.1, left_m=0.1, right_m=0.1,
                ),
            ),
        )

        params = ExploreParams(max_steps=5)
        result = await skill.execute(params, robot, world)

        self.assertTrue(result.success)
        self.assertEqual(result.data["stop_reason"], "blocked")
        self.assertLess(result.data["steps_completed"], 5)


class TestExploreStopConditions(unittest.IsolatedAsyncioTestCase):
    """Hard stop conditions."""

    async def test_low_battery_stops(self) -> None:
        settings = Settings()
        skill = ExploreSkill(settings)
        robot = MockRobot()
        world = WorldState(battery_level=20.0)  # Below low_battery_threshold=25

        params = ExploreParams(max_steps=5)
        result = await skill.execute(params, robot, world)

        self.assertFalse(result.success)
        self.assertEqual(result.data["stop_reason"], "low_battery")
        self.assertEqual(result.data["steps_completed"], 0)

    async def test_estop_stops(self) -> None:
        settings = Settings()
        skill = ExploreSkill(settings)
        robot = MockRobot()
        world = WorldState(battery_level=80.0, estop_active=True)

        params = ExploreParams(max_steps=5)
        result = await skill.execute(params, robot, world)

        self.assertFalse(result.success)
        self.assertEqual(result.data["stop_reason"], "estop")

    async def test_robot_error_stops(self) -> None:
        settings = Settings()
        skill = ExploreSkill(settings)
        robot = MockRobot()
        world = WorldState(
            battery_level=80.0,
            robot_self_state=RobotSelfState(source="test", error_code=42),
        )

        params = ExploreParams(max_steps=5)
        result = await skill.execute(params, robot, world)

        self.assertFalse(result.success)
        self.assertEqual(result.data["stop_reason"], "robot_error")


class TestExploreValidator(unittest.TestCase):
    """SafetyValidator rejects out-of-bounds explore params."""

    def setUp(self) -> None:
        self.settings = Settings()
        # Register explore skill
        skill = ExploreSkill(self.settings)
        self.skills = SkillRegistry([skill])
        self.validator = SafetyValidator(self.settings, self.skills)
        self.world = WorldState(battery_level=80.0)

    def test_max_steps_exceeds_settings(self) -> None:
        # settings.explore_max_steps = 5 by default
        call = ToolCall(skill_name="explore", parameters={"max_steps": 10})
        result = self.validator.validate(call, self.world, confirmation_granted=True)
        self.assertFalse(result.allowed)
        self.assertIn("exceeds settings limit", result.reason)

    def test_valid_params_allowed(self) -> None:
        call = ToolCall(skill_name="explore", parameters={"max_steps": 3})
        result = self.validator.validate(call, self.world, confirmation_granted=True)
        self.assertTrue(result.allowed)

    def test_requires_confirmation(self) -> None:
        call = ToolCall(skill_name="explore", parameters={"max_steps": 3})
        result = self.validator.validate(call, self.world, confirmation_granted=False)
        self.assertFalse(result.allowed)
        self.assertTrue(result.requires_confirmation)


class TestExploreRegistry(unittest.TestCase):
    """Explore is properly registered."""

    def test_explore_in_unitree_whitelist(self) -> None:
        self.assertIn("explore", UNITREE_LLM_SKILLS)

    def test_explore_registered_in_runtime(self) -> None:
        from robot_brain.runtime.loop import AgentRuntime

        rt = AgentRuntime.create()
        self.assertTrue(rt.context.skills.has("explore"))


class TestExploreMockLLM(unittest.IsolatedAsyncioTestCase):
    """MockLLM recognizes explore intent."""

    async def test_explore_intent(self) -> None:
        from robot_brain.llm.mock import MockLLM

        llm = MockLLM()
        world = WorldState()
        result = await llm.plan("explore the area", world, [], [])
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0].skill_name, "explore")

    async def test_look_around_intent(self) -> None:
        from robot_brain.llm.mock import MockLLM

        llm = MockLLM()
        world = WorldState()
        result = await llm.plan("look around", world, [], [])
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0].skill_name, "explore")


class TestExploreGo2Fake(unittest.IsolatedAsyncioTestCase):
    """Go2 fake transport explore tests."""

    async def test_go2_explore_with_self_state(self) -> None:
        """Fake UnitreeRobot with standing self_state should run explore."""
        from robot_brain.actuation.unitree import FakeUnitreeTransport, UnitreeRobot

        settings = Settings()
        transport = FakeUnitreeTransport()
        robot = UnitreeRobot(transport, settings)
        skill = ExploreSkill(settings)
        world = WorldState(
            battery_level=80.0,
            robot_self_state=RobotSelfState(
                source="test",
                is_standing=True,
                state_age_seconds=0.5,
                ultrasonic=UltrasonicData(front_m=1.0, rear_m=1.0),
            ),
        )

        params = ExploreParams(max_steps=2, step_distance_cm=15.0)
        result = await skill.execute(params, robot, world)

        self.assertTrue(result.success)
        self.assertEqual(result.data["stop_reason"], "max_steps")
        self.assertGreater(result.data["segments_total"], 0)

    async def test_go2_explore_no_self_state_fails(self) -> None:
        """Without self_state, precondition should fail immediately."""
        from robot_brain.actuation.unitree import FakeUnitreeTransport, UnitreeRobot

        settings = Settings()
        transport = FakeUnitreeTransport()
        robot = UnitreeRobot(transport, settings)
        skill = ExploreSkill(settings)
        world = WorldState(battery_level=80.0)  # No robot_self_state

        params = ExploreParams(max_steps=3)
        result = await skill.execute(params, robot, world)

        self.assertFalse(result.success)
        self.assertIn("precondition:", result.data["stop_reason"])


class TestExplorePerceptionPoll(unittest.IsolatedAsyncioTestCase):
    """Verify perception polling changes exploration decisions."""

    async def test_perception_poll_changes_decision(self) -> None:
        """Injected perception that adds obstacle mid-loop should change behavior."""
        from unittest.mock import MagicMock

        settings = Settings()
        skill = ExploreSkill(settings)
        robot = MockRobot()

        # Start with clear path
        world = WorldState(
            battery_level=80.0,
            robot_self_state=RobotSelfState(
                source="test",
                ultrasonic=UltrasonicData(front_m=1.0, rear_m=1.0),
            ),
        )

        # Mock perception that injects obstacle after first poll
        call_count = 0

        async def mock_observe():
            nonlocal call_count
            call_count += 1
            if call_count >= 3:  # After step 1's scan poll
                # Inject front obstacle
                world.robot_self_state = RobotSelfState(
                    source="test",
                    ultrasonic=UltrasonicData(front_m=0.15, rear_m=1.0),
                )
            return None  # Don't apply_observation, we mutate world directly

        mock_perception = MagicMock()
        mock_perception.observe = mock_observe
        skill._perception = mock_perception

        params = ExploreParams(max_steps=3)
        result = await skill.execute(params, robot, world)

        # Should have adapted: first step nudge, later steps scan_alt/retreat
        actions = result.data["actions"]
        self.assertIn("nudge", actions)  # First step went forward
        # After obstacle injected, should see scan_alt (turning attempt)
        self.assertTrue(
            "scan_alt" in actions or "retreat" in actions,
            f"Expected obstacle avoidance in actions: {actions}",
        )


if __name__ == "__main__":
    unittest.main()
