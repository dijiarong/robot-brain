"""Navigation capability contract, fake provider, tools, skills, and wiring."""
from __future__ import annotations

import unittest

from config.settings import Settings
from robot_brain.actuation.mock import MockRobot
from robot_brain.core.world_state import WorldState
from robot_brain.llm.base import ToolCall
from robot_brain.navigation import (
    AbsoluteNavigationGoal,
    LocalizationStatus,
    MapIdentity,
    FakeNavigationClient,
    NavigationPose,
    NavigationStatus,
    RelativeNavigationGoal,
)
from robot_brain.runtime.loop import AgentRuntime
from robot_brain.skills.builtin.navigation import (
    CancelNavigationParams,
    CancelNavigationSkill,
    NavigateAbsoluteParams,
    NavigateAbsoluteSkill,
    NavigateRelativeParams,
    NavigateRelativeSkill,
)
from robot_brain.tools.base import ToolContext
from robot_brain.tools.builtin.navigation import NavigationGetStateTool


class FakeNavigationClientTests(unittest.IsolatedAsyncioTestCase):
    async def test_success_updates_pose_and_records_goal(self):
        client = FakeNavigationClient()
        handle = await client.set_relative_goal(
            RelativeNavigationGoal(forward_m=0.4, left_m=0.1, yaw_degrees=15)
        )
        state = await client.get_state()

        self.assertTrue(handle.accepted)
        self.assertEqual(NavigationStatus.SUCCEEDED, state.status)
        self.assertAlmostEqual(0.4, state.pose.x_m)  # type: ignore[union-attr]
        self.assertAlmostEqual(0.1, state.pose.y_m)  # type: ignore[union-attr]
        self.assertEqual("set_relative_goal", client.command_history[0]["action"])

    async def test_unavailable_provider_rejects_goal(self):
        client = FakeNavigationClient(ready=False)
        with self.assertRaises(Exception):
            await client.set_relative_goal(RelativeNavigationGoal(forward_m=0.2))

    async def test_relative_goal_is_transformed_from_robot_frame(self):
        client = FakeNavigationClient(pose=NavigationPose(yaw_degrees=90.0))
        await client.set_relative_goal(RelativeNavigationGoal(forward_m=0.4))
        state = await client.get_state()
        self.assertAlmostEqual(0.0, state.pose.x_m, places=6)  # type: ignore[union-attr]
        self.assertAlmostEqual(0.4, state.pose.y_m, places=6)  # type: ignore[union-attr]

    async def test_cancel_active_goal(self):
        client = FakeNavigationClient([NavigationStatus.ACTIVE])
        handle = await client.set_relative_goal(RelativeNavigationGoal(forward_m=0.2))
        await client.get_state()
        state = await client.cancel(handle.goal_id)
        self.assertEqual(NavigationStatus.CANCELED, state.status)

    async def test_persistent_map_absolute_goal_updates_pose(self):
        identity = MapIdentity(map_id="office", version="v1", frame_id="map")
        client = FakeNavigationClient(
            pose=NavigationPose(frame_id="map"), map_identity=identity
        )
        localization = await client.get_localization_state()
        self.assertEqual(LocalizationStatus.LOCALIZED, localization.status)
        self.assertTrue(localization.usable_for_persistent_memory)

        await client.set_absolute_goal(AbsoluteNavigationGoal(
            pose=NavigationPose(x_m=2.0, y_m=3.0, yaw_degrees=45.0, frame_id="map"),
            map_id="office",
            map_version="v1",
        ))
        state = await client.get_state()
        self.assertEqual((2.0, 3.0), (state.pose.x_m, state.pose.y_m))  # type: ignore[union-attr]

    async def test_session_local_fake_is_not_persistent_memory_safe(self):
        localization = await FakeNavigationClient().get_localization_state()
        self.assertEqual(LocalizationStatus.LOCAL, localization.status)
        self.assertFalse(localization.usable_for_persistent_memory)


class NavigationSkillTests(unittest.IsolatedAsyncioTestCase):
    async def _run_outcome(self, outcome: NavigationStatus):
        client = FakeNavigationClient([outcome])
        skill = NavigateRelativeSkill(client)
        return await skill.execute(
            NavigateRelativeParams(forward_m=0.3), MockRobot(), WorldState()
        )

    async def test_success(self):
        result = await self._run_outcome(NavigationStatus.SUCCEEDED)
        self.assertTrue(result.success)
        self.assertEqual("succeeded", result.data["stop_reason"])

    async def test_failure(self):
        result = await self._run_outcome(NavigationStatus.FAILED)
        self.assertFalse(result.success)
        self.assertEqual("failed", result.data["stop_reason"])

    async def test_timeout(self):
        result = await self._run_outcome(NavigationStatus.TIMED_OUT)
        self.assertFalse(result.success)
        self.assertEqual("timed_out", result.data["stop_reason"])

    async def test_no_progress(self):
        result = await self._run_outcome(NavigationStatus.NO_PROGRESS)
        self.assertFalse(result.success)
        self.assertEqual("no_progress", result.data["stop_reason"])

    async def test_waits_for_active_provider_to_reach_terminal_state(self):
        class DelayedSuccessClient(FakeNavigationClient):
            reads = 0

            async def get_state(self):
                self.reads += 1
                if self.reads == 1:
                    return self._state.model_copy(deep=True)
                return await super().get_state()

        client = DelayedSuccessClient()
        skill = NavigateRelativeSkill(client, poll_interval_s=0.0)
        result = await skill.execute(
            NavigateRelativeParams(forward_m=0.2), MockRobot(), WorldState()
        )
        self.assertTrue(result.success)
        self.assertEqual(2, client.reads)

    async def test_cancel_skill_stops_active_goal(self):
        client = FakeNavigationClient([NavigationStatus.ACTIVE])
        handle = await client.set_relative_goal(RelativeNavigationGoal(forward_m=0.3))
        await client.get_state()
        result = await CancelNavigationSkill(client).execute(
            CancelNavigationParams(goal_id=handle.goal_id), MockRobot(), WorldState()
        )
        self.assertTrue(result.success)
        self.assertEqual("canceled", result.data["stop_reason"])

    async def test_absolute_skill_executes_only_on_persistent_map_provider(self):
        identity = MapIdentity(map_id="office", version="v1", frame_id="map")
        client = FakeNavigationClient(
            pose=NavigationPose(frame_id="map"), map_identity=identity
        )
        skill = NavigateAbsoluteSkill(client, poll_interval_s=0.0)
        result = await skill.execute(
            NavigateAbsoluteParams(
                pose=NavigationPose(x_m=1.0, y_m=2.0, frame_id="map"),
                map_id="office",
                map_version="v1",
            ),
            MockRobot(),
            WorldState(),
        )
        self.assertTrue(result.success)
        self.assertEqual("succeeded", result.data["stop_reason"])


class NavigationToolTests(unittest.IsolatedAsyncioTestCase):
    async def test_get_state_is_read_only(self):
        client = FakeNavigationClient()
        tool = NavigationGetStateTool(client)
        result = await tool.execute(
            tool.params_model(),
            ToolContext(settings=None, world=WorldState(), robot=MockRobot()),
        )
        self.assertTrue(result.success)
        self.assertEqual("idle", result.data["status"])
        self.assertEqual([], client.command_history)


class NavigationRuntimeTests(unittest.TestCase):
    def test_mock_runtime_registers_fake_navigation_capabilities(self):
        runtime = AgentRuntime.create(
            settings=Settings(robot_backend="mock", memory_db_path=":memory:")
        )
        self.assertIsInstance(runtime.context.navigation, FakeNavigationClient)
        self.assertIsNotNone(runtime.context.skills.get("nav_go_relative"))
        self.assertIsNotNone(runtime.context.skills.get("nav_cancel"))
        self.assertIsNotNone(runtime.context.tools.get("nav_get_state"))

    def test_unitree_runtime_requires_explicit_navigation_provider(self):
        runtime = AgentRuntime.create(
            settings=Settings(
                robot_backend="unitree",
                unitree_transport="fake",
                perception_backend="mock",
                memory_db_path=":memory:",
            )
        )
        self.assertIsNone(runtime.context.navigation)
        self.assertIsNone(runtime.context.skills.get("nav_go_relative"))
        self.assertIsNone(runtime.context.tools.get("nav_get_state"))

    def test_injected_provider_is_visible_on_unitree(self):
        runtime = AgentRuntime.create(
            settings=Settings(
                robot_backend="unitree",
                unitree_transport="fake",
                perception_backend="mock",
                memory_db_path=":memory:",
            ),
            navigation=FakeNavigationClient(),
        )
        names = {tool["name"] for tool in runtime.context.skills.tools_for_backend("unitree")}
        self.assertIn("nav_go_relative", names)
        self.assertIn("nav_cancel", names)

    def test_persistent_provider_exposes_absolute_navigation_skill(self):
        provider = FakeNavigationClient(
            pose=NavigationPose(frame_id="map"),
            map_identity=MapIdentity(map_id="office", frame_id="map"),
        )
        runtime = AgentRuntime.create(
            settings=Settings(robot_backend="mock", memory_db_path=":memory:"),
            navigation=provider,
        )
        self.assertIsNotNone(runtime.context.skills.get("nav_go_to_pose"))


class NavigationSafetyTests(unittest.TestCase):
    def setUp(self):
        self.runtime = AgentRuntime.create(
            settings=Settings(robot_backend="mock", memory_db_path=":memory:")
        )

    def test_relative_navigation_requires_confirmation(self):
        result = self.runtime.context.validator.validate(
            ToolCall(skill_name="nav_go_relative", parameters={"forward_m": 0.3}),
            self.runtime.context.world,
        )
        self.assertFalse(result.allowed)
        self.assertTrue(result.requires_confirmation)

    def test_cancel_is_allowed_during_estop(self):
        self.runtime.context.world.estop_active = True
        result = self.runtime.context.validator.validate(
            ToolCall(skill_name="nav_cancel", parameters={}),
            self.runtime.context.world,
        )
        self.assertTrue(result.allowed, result.reason)


if __name__ == "__main__":
    unittest.main()
