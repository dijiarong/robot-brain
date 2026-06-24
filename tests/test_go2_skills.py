"""Tests for Go2-native skills: nudge, scan, retreat and shared motion helpers."""
from __future__ import annotations

import asyncio
import math
import unittest
from unittest.mock import patch

from config.settings import Settings
from robot_brain.actuation.base import RobotInterface, RobotState
from robot_brain.actuation.unitree import (
    FakeUnitreeTransport,
    UnitreeRobot,
    UnitreeState,
)
from robot_brain.core.robot_self_state import RobotSelfState
from robot_brain.core.world_state import Position, WorldState
from robot_brain.safety.validator import SafetyValidator
from robot_brain.skills.base import SkillResult
from robot_brain.skills.builtin.go2_catalog import (
    NudgeParams,
    NudgeSkill,
    RetreatParams,
    RetreatSkill,
    ScanParams,
    ScanSkill,
    go2_skills,
)
from robot_brain.skills.builtin.go2_motion import (
    LINEAR_SPEED,
    YAW_SPEED,
    _chop,
    check_robot_self_state,
    plan_linear_segments,
    plan_yaw_segments,
    run_go2_drive_segments,
)
from robot_brain.skills.registry import SkillRegistry


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------

_SETTINGS_BASE = dict(
    robot_backend="unitree",
    unitree_transport="fake",
    unitree_dry_run=True,
    unitree_enable_motion=False,
    memory_db_path=":memory:",
)


def _settings(**overrides) -> Settings:
    merged = {**_SETTINGS_BASE, **overrides}
    return Settings(**merged)


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


async def _new_robot(**state_kw) -> UnitreeRobot:
    s = _settings()
    defaults: dict[str, object] = {"connected": True, "is_standing": True}
    defaults.update(state_kw)
    st = UnitreeState(**defaults)  # type: ignore[arg-type]
    t = FakeUnitreeTransport(initial_state=st)
    robot = UnitreeRobot(t, s)
    await t.connect()
    return robot


# ---------------------------------------------------------------------------
# check_robot_self_state
# ---------------------------------------------------------------------------

class CheckSelfStateTests(unittest.TestCase):
    def setUp(self):
        self.settings = _settings()

    def test_none_self_state_rejected(self):
        ws = WorldState()
        self.assertIsNotNone(check_robot_self_state(ws, self.settings))

    def test_not_standing_rejected(self):
        ws = WorldState()
        ws.robot_self_state = RobotSelfState(source="u", is_standing=False)
        self.assertIn("not standing", check_robot_self_state(ws, self.settings) or "")

    def test_error_code_rejected(self):
        ws = WorldState()
        ws.robot_self_state = RobotSelfState(source="u", error_code=7004)
        self.assertIn("7004", check_robot_self_state(ws, self.settings) or "")

    def test_stale_state_rejected(self):
        ws = WorldState()
        ws.robot_self_state = RobotSelfState(
            source="u", state_age_seconds=5.0
        )
        self.assertIn("stale", check_robot_self_state(ws, self.settings) or "")

    def test_healthy_state_passes(self):
        ws = _standing_world()
        self.assertIsNone(check_robot_self_state(ws, self.settings))


# ---------------------------------------------------------------------------
# segment planning
# ---------------------------------------------------------------------------

class SegmentPlanningTests(unittest.TestCase):
    SEG = 0.5

    def test_linear_50cm_is_7_segments(self):
        # 0.5m / 0.15 m/s = 3.33s → 6 full + 1 remainder
        segs = plan_linear_segments(0.5, self.SEG)
        self.assertEqual(7, len(segs))
        self.assertAlmostEqual(0.5, segs[0], places=2)
        self.assertLess(segs[-1], self.SEG + 0.01)

    def test_linear_short_single_segment(self):
        segs = plan_linear_segments(0.07, self.SEG)  # 0.47s total
        self.assertEqual(1, len(segs))
        self.assertLess(segs[0], self.SEG + 0.01)

    def test_linear_exact_multiple(self):
        # 0.075m / 0.15 = 0.5s exactly → 1 segment of 0.5
        segs = plan_linear_segments(0.075, self.SEG)
        self.assertEqual(1, len(segs))
        self.assertAlmostEqual(0.5, segs[0], places=2)

    def test_yaw_90deg_is_11_segments(self):
        # 90° = 1.57 rad / 0.3 rad/s = 5.24s → 10 full + 1 remainder
        segs = plan_yaw_segments(math.radians(90), self.SEG)
        self.assertEqual(11, len(segs))

    def test_yaw_8deg_single_segment(self):
        # 8° = 0.14 rad / 0.3 = 0.47s → 1 segment
        segs = plan_yaw_segments(math.radians(8), self.SEG)
        self.assertEqual(1, len(segs))

    def test_tiny_linear_returns_one_minimal_segment(self):
        # Very short distance → one segment under SEG duration.
        segs = plan_linear_segments(0.01, self.SEG)  # 0.01m at 0.15m/s = 0.067s
        self.assertEqual(1, len(segs))
        self.assertLess(segs[0], self.SEG + 0.01)

    def test_chop_empty(self):
        self.assertEqual([], _chop(0.0, self.SEG))


# ---------------------------------------------------------------------------
# run_go2_drive_segments
# ---------------------------------------------------------------------------

class RunGo2DriveSegmentsTests(unittest.IsolatedAsyncioTestCase):
    async def test_successful_segments_audit(self):
        robot = await _new_robot()
        result = await run_go2_drive_segments(
            robot, vx=0.15, durations=[0.5, 0.3],
        )
        self.assertTrue(result["success"])
        self.assertEqual(2, result["segment_count"])
        self.assertEqual("completed", result["segments"][0]["end_reason"])
        self.assertEqual("completed", result["segments"][1]["end_reason"])

    async def test_segment_failure_stops_and_returns(self):
        s = _settings(unitree_dry_run=False, unitree_enable_motion=True)
        robot = await _new_robot()
        robot._settings = s  # override for this test
        robot.transport.fail_next = True
        result = await run_go2_drive_segments(
            robot, vx=0.15, durations=[0.5, 0.5],
        )
        self.assertFalse(result["success"])
        self.assertEqual(1, result["segment_count"])
        self.assertEqual("error", result["segments"][0]["end_reason"])


# ---------------------------------------------------------------------------
# NudgeSkill
# ---------------------------------------------------------------------------

class NudgeSkillTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.settings = _settings()
        self.skill = NudgeSkill(self.settings)

    async def test_successful_nudge_forward(self):
        robot = await _new_robot()
        world = _standing_world()
        result = await self.skill.execute(
            NudgeParams(direction="forward", distance_cm=30), robot, world,
        )
        self.assertTrue(result.success)
        self.assertIn("forward", result.message)
        self.assertEqual("nudge", result.data["skill"])
        self.assertGreater(result.data["segment_count"], 0)

    async def test_clamp_distance_above_max(self):
        robot = await _new_robot()
        world = _standing_world()
        result = await self.skill.execute(
            NudgeParams(direction="forward", distance_cm=80), robot, world,
        )
        self.assertTrue(result.success)
        self.assertEqual(80.0, result.data["requested"]["distance_cm"])
        self.assertEqual(50.0, result.data["clamped"]["distance_cm"])

    async def test_direction_mapping(self):
        robot = await _new_robot()
        world = _standing_world()

        for direction, expected_vx, expected_vy in [
            ("forward", LINEAR_SPEED, 0.0),
            ("back", -LINEAR_SPEED, 0.0),
            ("left", 0.0, LINEAR_SPEED),
            ("right", 0.0, -LINEAR_SPEED),
        ]:
            result = await self.skill.execute(
                NudgeParams(direction=direction, distance_cm=10), robot, world,
            )
            self.assertTrue(result.success, f"nudge {direction} failed")
            seg = result.data["segments"][0]
            self.assertAlmostEqual(expected_vx, seg["vx"], places=4)
            self.assertAlmostEqual(expected_vy, seg["vy"], places=4)

    async def test_not_standing_rejected(self):
        robot = await _new_robot(is_standing=False)
        world = WorldState()
        world.robot_self_state = RobotSelfState(source="u", is_standing=False)
        result = await self.skill.execute(
            NudgeParams(direction="forward"), robot, world,
        )
        self.assertFalse(result.success)
        self.assertIn("not standing", result.message)

    async def test_error_code_rejected(self):
        robot = await _new_robot()
        world = WorldState()
        world.robot_self_state = RobotSelfState(source="u", error_code=7004)
        result = await self.skill.execute(
            NudgeParams(direction="forward"), robot, world,
        )
        self.assertFalse(result.success)
        self.assertIn("7004", result.message)

    async def test_no_self_state_rejected(self):
        robot = await _new_robot()
        world = WorldState()
        result = await self.skill.execute(
            NudgeParams(direction="forward"), robot, world,
        )
        self.assertFalse(result.success)
        self.assertIn("self-state", result.message)

    async def test_wrong_robot_type(self):
        from robot_brain.actuation.mock import MockRobot
        result = await self.skill.execute(
            NudgeParams(direction="forward"), MockRobot(), WorldState(),
        )
        self.assertFalse(result.success)
        self.assertIn("UnitreeRobot", result.message)


# ---------------------------------------------------------------------------
# ScanSkill
# ---------------------------------------------------------------------------

class ScanSkillTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.settings = _settings()
        self.skill = ScanSkill(self.settings)

    async def test_successful_scan(self):
        robot = await _new_robot()
        world = _standing_world()
        result = await self.skill.execute(
            ScanParams(yaw_degrees=45), robot, world,
        )
        self.assertTrue(result.success)
        self.assertIn("45", result.message)
        self.assertEqual("scan", result.data["skill"])
        # Positive yaw → positive vyaw
        self.assertGreater(result.data["segments"][0]["vyaw"], 0)

    async def test_negative_scan(self):
        robot = await _new_robot()
        world = _standing_world()
        result = await self.skill.execute(
            ScanParams(yaw_degrees=-45), robot, world,
        )
        self.assertTrue(result.success)
        self.assertLess(result.data["segments"][0]["vyaw"], 0)

    async def test_clamp_angle(self):
        robot = await _new_robot()
        world = _standing_world()
        result = await self.skill.execute(
            ScanParams(yaw_degrees=120), robot, world,
        )
        self.assertTrue(result.success)
        self.assertEqual(120.0, result.data["requested"]["yaw_degrees"])
        self.assertEqual(90.0, result.data["clamped"]["yaw_degrees"])

    async def test_not_standing_rejected(self):
        robot = await _new_robot(is_standing=False)
        world = WorldState()
        world.robot_self_state = RobotSelfState(source="u", is_standing=False)
        result = await self.skill.execute(ScanParams(), robot, world)
        self.assertFalse(result.success)


# ---------------------------------------------------------------------------
# RetreatSkill
# ---------------------------------------------------------------------------

class RetreatSkillTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.settings = _settings()
        self.skill = RetreatSkill(self.settings)

    async def test_successful_retreat(self):
        robot = await _new_robot()
        world = _standing_world()
        result = await self.skill.execute(
            RetreatParams(distance_cm=50), robot, world,
        )
        self.assertTrue(result.success)
        self.assertEqual("retreat", result.data["skill"])
        # Always negative vx
        for seg in result.data["segments"]:
            self.assertLess(seg["vx"], 0)

    async def test_clamp_distance(self):
        robot = await _new_robot()
        world = _standing_world()
        result = await self.skill.execute(
            RetreatParams(distance_cm=200), robot, world,
        )
        self.assertTrue(result.success)
        self.assertEqual(200.0, result.data["requested"]["distance_cm"])
        self.assertEqual(100.0, result.data["clamped"]["distance_cm"])

    async def test_not_standing_rejected(self):
        robot = await _new_robot(is_standing=False)
        world = WorldState()
        world.robot_self_state = RobotSelfState(source="u", is_standing=False)
        result = await self.skill.execute(RetreatParams(), robot, world)
        self.assertFalse(result.success)


# ---------------------------------------------------------------------------
# SafetyValidator
# ---------------------------------------------------------------------------

class SafetyValidatorGo2Tests(unittest.TestCase):
    def setUp(self):
        from robot_brain.llm.base import ToolCall
        self.ToolCall = ToolCall
        self.settings = _settings()
        self.registry = SkillRegistry(go2_skills(self.settings))
        self.validator = SafetyValidator(self.settings, self.registry)
        self.world = _standing_world()

    def test_nudge_distance_out_of_range_rejected(self):
        result = self.validator.validate(
            self.ToolCall(skill_name="nudge", parameters={"direction": "forward", "distance_cm": 60}),
            self.world,
            confirmation_granted=True,
        )
        self.assertFalse(result.allowed)
        self.assertIn("10–50", result.reason)

    def test_nudge_distance_in_range_allowed(self):
        result = self.validator.validate(
            self.ToolCall(skill_name="nudge", parameters={"direction": "forward", "distance_cm": 30}),
            self.world,
            confirmation_granted=True,
        )
        self.assertTrue(result.allowed, result.reason)

    def test_scan_angle_out_of_range_rejected(self):
        result = self.validator.validate(
            self.ToolCall(skill_name="scan", parameters={"yaw_degrees": 100}),
            self.world,
            confirmation_granted=True,
        )
        self.assertFalse(result.allowed)
        self.assertIn("±90", result.reason)

    def test_scan_angle_in_range_allowed(self):
        result = self.validator.validate(
            self.ToolCall(skill_name="scan", parameters={"yaw_degrees": 90}),
            self.world,
            confirmation_granted=True,
        )
        self.assertTrue(result.allowed, result.reason)

    def test_retreat_distance_out_of_range_rejected(self):
        result = self.validator.validate(
            self.ToolCall(skill_name="retreat", parameters={"distance_cm": 150}),
            self.world,
            confirmation_granted=True,
        )
        self.assertFalse(result.allowed)
        self.assertIn("10–100", result.reason)

    def test_retreat_distance_in_range_allowed(self):
        result = self.validator.validate(
            self.ToolCall(skill_name="retreat", parameters={"distance_cm": 50}),
            self.world,
            confirmation_granted=True,
        )
        self.assertTrue(result.allowed, result.reason)

    def test_nudge_confirmation_required(self):
        """nudge ∈ require_confirmation_for → blocked without confirmation."""
        result = self.validator.validate(
            self.ToolCall(skill_name="nudge", parameters={"direction": "forward", "distance_cm": 20}),
            self.world,
            confirmation_granted=False,
        )
        self.assertFalse(result.allowed)
        self.assertTrue(result.requires_confirmation)

    def test_nudge_confirmation_granted(self):
        result = self.validator.validate(
            self.ToolCall(skill_name="nudge", parameters={"direction": "forward", "distance_cm": 20}),
            self.world,
            confirmation_granted=True,
        )
        self.assertTrue(result.allowed)


# ---------------------------------------------------------------------------
# LLM tool schema
# ---------------------------------------------------------------------------

class ToolSchemaTests(unittest.TestCase):
    def test_nudge_schema_includes_direction_and_distance(self):
        skill = NudgeSkill(_settings())
        schema = skill.params_schema()
        props = schema["properties"]
        self.assertIn("direction", props)
        self.assertIn("distance_cm", props)

    def test_scan_schema_includes_yaw_degrees(self):
        skill = ScanSkill(_settings())
        schema = skill.params_schema()
        self.assertIn("yaw_degrees", schema["properties"])

    def test_retreat_schema_includes_distance_cm(self):
        skill = RetreatSkill(_settings())
        schema = skill.params_schema()
        self.assertIn("distance_cm", schema["properties"])


# ---------------------------------------------------------------------------
# AgentRuntime integration
# ---------------------------------------------------------------------------

class RuntimeGo2SkillsTests(unittest.IsolatedAsyncioTestCase):
    async def test_unitree_backend_registers_go2_skills(self):
        from robot_brain.runtime.loop import AgentRuntime
        rt = AgentRuntime.create(
            settings=_settings(robot_backend="unitree", unitree_dry_run=True)
        )
        self.assertIsNotNone(rt.context.skills.get("nudge"))
        self.assertIsNotNone(rt.context.skills.get("scan"))
        self.assertIsNotNone(rt.context.skills.get("retreat"))

    async def test_mock_backend_does_not_register_go2_skills(self):
        from robot_brain.runtime.loop import AgentRuntime
        rt = AgentRuntime.create(
            settings=Settings(robot_backend="mock", memory_db_path=":memory:")
        )
        self.assertIsNone(rt.context.skills.get("nudge"))
        self.assertIsNone(rt.context.skills.get("scan"))
        self.assertIsNone(rt.context.skills.get("retreat"))

    async def test_go2_skills_via_runtime_execute(self):
        """Execute nudge through the runtime's skill execution path."""
        from robot_brain.runtime.loop import AgentRuntime
        rt = AgentRuntime.create(
            settings=_settings(robot_backend="unitree", unitree_dry_run=True)
        )
        await rt.context.robot.transport.connect()
        # Set standing self-state on world
        rt.context.world.robot_self_state = RobotSelfState(
            source="unitree_go2",
            is_standing=True,
            error_code=0,
            state_age_seconds=0.1,
        )
        skill = rt.context.skills.get("nudge")
        self.assertIsNotNone(skill)
        result = await skill.execute(
            NudgeParams(direction="forward", distance_cm=20),
            rt.context.robot,
            rt.context.world,
        )
        self.assertTrue(result.success)


# ---------------------------------------------------------------------------
# Generic skills compatibility on Go2
# ---------------------------------------------------------------------------

class GenericSkillsOnGo2Tests(unittest.IsolatedAsyncioTestCase):
    async def test_navigate_within_step_limit_succeeds_on_fake(self):
        """navigate calls robot.move_to() — UnitreeRobot rejects targets beyond
        max_step_distance (default 2.0m)."""
        from robot_brain.skills.builtin.catalog import NavigateSkill
        robot = await _new_robot()
        world = _standing_world()
        skill = NavigateSkill()
        result = await skill.execute(
            skill.params_model(target={"x": 0.5, "y": 0.0}, speed=0.8),
            robot, world,
        )
        # Within step limit — succeeds on fake transport
        self.assertTrue(result.success, result.message)

    async def test_follow_raises_not_implemented(self):
        """follow calls robot.follow() — UnitreeRobot raises NotImplementedError."""
        from robot_brain.skills.builtin.catalog import FollowSkill
        robot = await _new_robot()
        world = _standing_world()
        world.known_objects["obj1"] = type(
            "DetectedObject", (),
            {"object_id": "obj1", "kind": "person", "last_seen_at": None},
        )()
        skill = FollowSkill()
        with self.assertRaises(NotImplementedError):
            await skill.execute(
                skill.params_model(target_id="obj1", distance=2.0),
                robot, world,
            )

    async def test_scan_zero_degrees_no_motion(self):
        """scan(0°) succeeds without sending any drive segments."""
        skill = ScanSkill(_settings())
        robot = await _new_robot()
        world = _standing_world()
        result = await skill.execute(ScanParams(yaw_degrees=0), robot, world)
        self.assertTrue(result.success)
        self.assertIn("no rotation", result.message)
        self.assertEqual(0, result.data["segment_count"])


# ---------------------------------------------------------------------------
# go2_skills factory
# ---------------------------------------------------------------------------

class Go2SkillsFactoryTests(unittest.TestCase):
    def test_factory_returns_skills(self):
        skills = go2_skills(_settings())
        self.assertEqual(4, len(skills))
        names = {s.name for s in skills}
        self.assertEqual({"nudge", "scan", "retreat", "explore"}, names)


if __name__ == "__main__":
    unittest.main()
