"""End-to-end tests for the capability vertical slices (stop + nudge).

Proves the migrated path: a skill delegates to a low-level tool, the tool's
metadata drives SafetyPolicy, and estop / low-battery / confirmation behavior
does not regress.
"""
from __future__ import annotations

import unittest

from config.settings import Settings
from robot_brain.actuation.mock import MockRobot
from robot_brain.actuation.unitree import FakeUnitreeTransport, UnitreeRobot, UnitreeState
from robot_brain.core.robot_self_state import RobotSelfState
from robot_brain.core.world_state import WorldState
from robot_brain.llm.base import ToolCall
from robot_brain.safety.validator import SafetyValidator
from robot_brain.skills.base import SkillResult
from robot_brain.skills.builtin.catalog import StopParams, StopSkill
from robot_brain.skills.builtin.go2_catalog import NudgeParams, NudgeSkill
from robot_brain.skills.builtin.go2_catalog import go2_skills
from robot_brain.skills.registry import SkillRegistry
from robot_brain.planning.catalog import PlannerCatalog
from robot_brain.tools.base import MotionKind, ToolContext
from robot_brain.tools.builtin.control import (
    Go2DriveSegmentParams,
    Go2DriveSegmentTool,
    StopMotionTool,
)
from robot_brain.tools.registry import ToolRegistry
from robot_brain.tools.builtin import default_tools


# ---------------------------------------------------------------------------
# Go2 fake-robot helpers (mirror tests/test_go2_skills.py fixtures)
# ---------------------------------------------------------------------------

_GO2_SETTINGS = dict(
    robot_backend="unitree",
    unitree_transport="fake",
    unitree_dry_run=True,
    unitree_enable_motion=False,
    memory_db_path=":memory:",
)


def _go2_settings(**overrides) -> Settings:
    return Settings(**{**_GO2_SETTINGS, **overrides})


def _standing_world() -> WorldState:
    ws = WorldState()
    ws.robot_self_state = RobotSelfState(
        source="unitree_go2",
        is_standing=True,
        is_moving=False,
        error_code=0,
        state_age_seconds=0.1,
    )
    return ws


async def _new_go2_robot() -> UnitreeRobot:
    s = _go2_settings()
    st = UnitreeState(connected=True, is_standing=True)
    transport = FakeUnitreeTransport(initial_state=st)
    robot = UnitreeRobot(transport, s)
    await transport.connect()
    return robot


class StopSkillVerticalTests(unittest.IsolatedAsyncioTestCase):
    async def test_stop_skill_delegates_to_tool(self):
        robot = MockRobot()
        skill = StopSkill()
        result: SkillResult = await skill.execute(
            StopParams(reason="halt"), robot, WorldState()
        )
        self.assertTrue(result.success)
        self.assertIn("halt", result.message)
        # Underlying robot.stop was invoked.
        self.assertEqual(robot.action_history[-1]["action"], "stop")
        self.assertEqual(robot.action_history[-1]["reason"], "halt")
        # Tool identity recorded in result data.
        self.assertEqual(result.data.get("tool"), "stop_motion")

    async def test_stop_skill_uses_injected_tool(self):
        tool = StopMotionTool()
        skill = StopSkill(stop_tool=tool)
        self.assertIs(skill.capability_metadata, tool.metadata)

    def test_stop_skill_metadata_is_stop_motion(self):
        skill = StopSkill()
        md = skill.capability_metadata
        self.assertEqual(md.motion_kind, MotionKind.STOP)
        self.assertFalse(md.requires_confirmation)
        # Low-level tool is not planner-visible; the skill is the planner unit.
        self.assertFalse(md.planner_visible)


class StopValidatorVerticalTests(unittest.TestCase):
    def setUp(self) -> None:
        self.settings = Settings(memory_db_path=":memory:")
        self.registry = SkillRegistry([StopSkill()])
        self.validator = SafetyValidator(self.settings, self.registry)

    def test_stop_allowed_during_estop_via_policy(self):
        # stop carries metadata -> SafetyPolicy (motion_kind=stop) allows it.
        world = WorldState(estop_active=True)
        res = self.validator.validate(
            ToolCall(skill_name="stop", parameters={"reason": "e"}), world
        )
        self.assertTrue(res.allowed, res.reason)

    def test_stop_allowed_on_critical_battery_via_policy(self):
        world = WorldState(battery_level=4.0)
        res = self.validator.validate(
            ToolCall(skill_name="stop", parameters={"reason": "b"}), world
        )
        self.assertTrue(res.allowed, res.reason)

    def test_stop_no_confirmation_required(self):
        res = self.validator.validate(
            ToolCall(skill_name="stop", parameters={}), WorldState(),
            confirmation_granted=False,
        )
        self.assertTrue(res.allowed, res.reason)
        self.assertFalse(res.requires_confirmation)

    def test_stop_normalized_params_round_trip(self):
        res = self.validator.validate(
            ToolCall(skill_name="stop", parameters={"reason": "manual"}),
            WorldState(),
        )
        self.assertTrue(res.allowed)
        self.assertEqual(res.normalized_parameters["reason"], "manual")


# ---------------------------------------------------------------------------
# PlannerCatalog layering: runtime-internal tool != planner-visible skill
# ---------------------------------------------------------------------------

class PlannerCatalogLayeringTests(unittest.TestCase):
    def test_low_level_tool_not_in_catalog(self):
        # stop_motion is registered as a runtime tool but planner_visible=False,
        # so the catalog (which surfaces skills today) never exposes it.
        tools = ToolRegistry(default_tools())
        self.assertTrue(tools.has("stop_motion"))
        self.assertFalse(tools.get("stop_motion").metadata.planner_visible)

        skills = SkillRegistry([StopSkill()])
        for backend in ("mock", "unitree"):
            catalog = PlannerCatalog(skills, backend)
            names = {t["name"] for t in catalog.planner_tools()}
            self.assertNotIn("stop_motion", names)
            # The skill is the planner-facing unit, not the tool.
            self.assertIn("stop", names)

    def test_catalog_equivalent_to_legacy_filter(self):
        # tools_for_backend now delegates to the catalog; behavior unchanged.
        skills = SkillRegistry([StopSkill()])
        via_catalog = PlannerCatalog(skills, "mock").planner_tools()
        via_registry = skills.tools_for_backend("mock")
        self.assertEqual(
            [t["name"] for t in via_catalog],
            [t["name"] for t in via_registry],
        )


# ---------------------------------------------------------------------------
# Go2DriveSegmentTool (low-level motion primitive)
# ---------------------------------------------------------------------------

class Go2DriveSegmentToolTests(unittest.IsolatedAsyncioTestCase):
    async def test_execute_single_segment(self):
        robot = await _new_go2_robot()
        ctx = ToolContext(settings=_go2_settings(), world=_standing_world(), robot=robot)
        result = await Go2DriveSegmentTool().execute(
            Go2DriveSegmentParams(vx=0.15, vy=0.0, vyaw=0.0, duration=0.5), ctx
        )
        self.assertTrue(result.success)
        self.assertEqual(result.data["vx"], 0.15)
        self.assertEqual(result.data["vyaw"], 0.0)
        self.assertIn("end_reason", result.data)

    async def test_wrong_robot_type_returns_error_segment(self):
        ctx = ToolContext(settings=_go2_settings(), world=_standing_world(), robot=MockRobot())
        result = await Go2DriveSegmentTool().execute(
            Go2DriveSegmentParams(vx=0.15, duration=0.5), ctx
        )
        self.assertFalse(result.success)
        self.assertEqual(result.data["end_reason"], "error")
        self.assertIn("UnitreeRobot", result.message)

    def test_metadata_is_linear_unitree_confirmed(self):
        md = Go2DriveSegmentTool().metadata
        self.assertEqual(md.motion_kind, MotionKind.LINEAR)
        self.assertEqual(md.backend_allowlist, ("unitree",))
        self.assertTrue(md.requires_confirmation)
        self.assertFalse(md.planner_visible)


# ---------------------------------------------------------------------------
# Nudge vertical slice (NudgeSkill -> Go2DriveSegmentTool)
# ---------------------------------------------------------------------------

class NudgeVerticalTests(unittest.IsolatedAsyncioTestCase):
    async def test_nudge_delegates_to_drive_tool(self):
        robot = await _new_go2_robot()
        skill = NudgeSkill(_go2_settings())
        result: SkillResult = await skill.execute(
            NudgeParams(direction="forward", distance_cm=30), robot, _standing_world()
        )
        self.assertTrue(result.success)
        self.assertEqual(result.data["skill"], "nudge")
        self.assertGreater(result.data["segment_count"], 0)
        # Segment audit shape unchanged from the run_go2_drive_segments era.
        seg = result.data["segments"][0]
        self.assertEqual(seg["index"], 0)
        for key in ("vx", "vy", "vyaw", "duration", "end_reason"):
            self.assertIn(key, seg)

    async def test_nudge_injected_tool_is_used(self):
        tool = Go2DriveSegmentTool()
        skill = NudgeSkill(_go2_settings(), drive_tool=tool)
        self.assertIs(skill.capability_metadata, tool.metadata)

    def test_nudge_metadata_drives_policy(self):
        skill = NudgeSkill(_go2_settings())
        md = skill.capability_metadata
        self.assertEqual(md.motion_kind, MotionKind.LINEAR)
        self.assertEqual(md.backend_allowlist, ("unitree",))
        self.assertTrue(md.requires_confirmation)


class NudgeValidatorVerticalTests(unittest.TestCase):
    def setUp(self) -> None:
        self.settings = _go2_settings()
        self.registry = SkillRegistry(go2_skills(self.settings))
        self.validator = SafetyValidator(self.settings, self.registry)
        self.world = _standing_world()

    def test_nudge_rejected_during_estop_via_policy(self):
        world = _standing_world()
        world.estop_active = True
        res = self.validator.validate(
            ToolCall(skill_name="nudge", parameters={"direction": "forward", "distance_cm": 20}),
            world,
            confirmation_granted=True,
        )
        self.assertFalse(res.allowed)
        self.assertEqual(res.error_code and res.error_code.value, "safety_estop_active")

    def test_nudge_rejected_on_critical_battery_via_policy(self):
        world = _standing_world()
        world.battery_level = 4.0
        res = self.validator.validate(
            ToolCall(skill_name="nudge", parameters={"direction": "forward", "distance_cm": 20}),
            world,
            confirmation_granted=True,
        )
        self.assertFalse(res.allowed)
        self.assertEqual(res.error_code and res.error_code.value, "safety_battery_critical")

    def test_nudge_requires_confirmation(self):
        res = self.validator.validate(
            ToolCall(skill_name="nudge", parameters={"direction": "forward", "distance_cm": 20}),
            self.world,
            confirmation_granted=False,
        )
        self.assertFalse(res.allowed)
        self.assertTrue(res.requires_confirmation)

    def test_nudge_allowed_with_confirmation(self):
        res = self.validator.validate(
            ToolCall(skill_name="nudge", parameters={"direction": "forward", "distance_cm": 20}),
            self.world,
            confirmation_granted=True,
        )
        self.assertTrue(res.allowed, res.reason)

    def test_nudge_out_of_range_is_motion_violation_not_confirmation(self):
        """P1 regression: an illegal distance must be rejected as a motion
        violation, NOT surface as a confirmation request. Asking an operator
        to confirm an action that can never be legal is a safety/UX bug."""
        res = self.validator.validate(
            ToolCall(skill_name="nudge", parameters={"direction": "forward", "distance_cm": 999}),
            self.world,
            confirmation_granted=False,
        )
        self.assertFalse(res.allowed)
        self.assertEqual(
            res.error_code and res.error_code.value, "safety_motion_violation"
        )
        self.assertIn("10–50", res.reason)
        self.assertFalse(res.requires_confirmation)


if __name__ == "__main__":
    unittest.main()
