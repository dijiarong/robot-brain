"""Tests for the explore structured step trace (iteration 18)."""
from __future__ import annotations

import unittest

from config.settings import Settings
from robot_brain.actuation.mock import MockRobot
from robot_brain.core.passability import PassabilityHint
from robot_brain.core.robot_self_state import RobotSelfState, UltrasonicData
from robot_brain.core.world_state import WorldState
from robot_brain.skills.builtin.explore import ExploreParams, ExploreSkill


class _StubPassability:
    """Minimal stand-in returning a fixed hint each call."""

    def __init__(self, hint: PassabilityHint | None) -> None:
        self._hint = hint

    async def analyze_if_due(self, world: WorldState) -> PassabilityHint | None:
        if self._hint is not None:
            world.passability_hint = self._hint
        return self._hint


def _hint(direction: str, confidence: float = 0.8) -> PassabilityHint:
    return PassabilityHint(recommended_direction=direction, confidence=confidence, reason="stub")  # type: ignore[arg-type]


def _world(front: float = 1.0, rear: float = 1.0) -> WorldState:
    return WorldState(
        battery_level=80.0,
        robot_self_state=RobotSelfState(
            source="test", ultrasonic=UltrasonicData(front_m=front, rear_m=rear),
        ),
    )


class ExploreTraceTests(unittest.IsolatedAsyncioTestCase):
    async def test_trace_one_entry_per_step(self):
        skill = ExploreSkill(Settings(memory_db_path=":memory:"))
        result = await skill.execute(
            ExploreParams(max_steps=2), MockRobot(), _world(front=1.0)
        )
        trace = result.data["trace"]
        self.assertEqual(len(trace), 2)
        self.assertEqual(result.data["steps_completed"], 2)
        # actions compat list still present.
        self.assertIn("scan", result.data["actions"])
        self.assertIn("nudge", result.data["actions"])

    async def test_trace_fields_populated(self):
        skill = ExploreSkill(Settings(memory_db_path=":memory:"))
        result = await skill.execute(
            ExploreParams(max_steps=1), MockRobot(), _world(front=1.0)
        )
        t = result.data["trace"][0]
        self.assertEqual(t["step_index"], 0)
        self.assertEqual(t["chosen_action"], "nudge")
        self.assertEqual(t["fallback_reason"], "")
        self.assertIsNone(t["stop_reason"])
        self.assertIn("front_m", t["ultrasonic"])
        self.assertIsNotNone(t["heading_before"])
        self.assertIsNotNone(t["duration_ms"])
        # No analyzer -> no hint recorded.
        self.assertIsNone(t["passability_hint"])

    async def test_trace_records_vlm_hint_and_reason(self):
        skill = ExploreSkill(
            Settings(memory_db_path=":memory:"),
            passability=_StubPassability(_hint("left")),
        )
        result = await skill.execute(
            ExploreParams(max_steps=1), MockRobot(), _world(front=0.15)
        )
        t = result.data["trace"][0]
        # VLM hint surfaced in trace; alt direction came from VLM.
        self.assertEqual(t["passability_hint"]["recommended_direction"], "left")
        self.assertEqual(t["fallback_reason"], "vlm")
        # Front obstacle + still blocked after alt turn -> retreat.
        self.assertEqual(t["chosen_action"], "retreat")

    async def test_trace_records_vlm_stop_hold(self):
        skill = ExploreSkill(
            Settings(memory_db_path=":memory:"),
            passability=_StubPassability(_hint("stop")),
        )
        result = await skill.execute(
            ExploreParams(max_steps=1), MockRobot(), _world(front=1.0)
        )
        t = result.data["trace"][0]
        self.assertEqual(t["chosen_action"], "vlm_hold")
        self.assertEqual(t["fallback_reason"], "vlm_stop")

    async def test_diagnostics_updated(self):
        skill = ExploreSkill(Settings(memory_db_path=":memory:"))
        await skill.execute(ExploreParams(max_steps=2), MockRobot(), _world(front=1.0))
        diag = skill.diagnostics()
        self.assertEqual(diag["last_stop_reason"], "max_steps")
        self.assertEqual(diag["last_steps_completed"], 2)
        self.assertEqual(diag["last_trace_count"], 2)


class ExploreGo2TraceSegmentsTests(unittest.IsolatedAsyncioTestCase):
    """Go2 trace must record scan + alt_scan + move segments per step (review P2)."""

    async def test_go2_trace_records_all_segment_phases(self):
        from robot_brain.actuation.unitree import FakeUnitreeTransport, UnitreeRobot
        from robot_brain.core.robot_self_state import RobotSelfState, UltrasonicData

        settings = Settings(memory_db_path=":memory:")
        transport = FakeUnitreeTransport()
        robot = UnitreeRobot(transport, settings)
        skill = ExploreSkill(settings)
        world = WorldState(
            battery_level=80.0,
            robot_self_state=RobotSelfState(
                source="test",
                is_standing=True,
                state_age_seconds=0.1,
                ultrasonic=UltrasonicData(front_m=0.15, rear_m=1.0),
            ),
        )
        result = await skill.execute(ExploreParams(max_steps=1, step_distance_cm=15.0), robot, world)
        t = result.data["trace"][0]
        # Scan, alt-scan, and retreat all produced drive segments this step.
        self.assertGreater(len(t["scan_segments"]), 0)
        self.assertGreater(len(t["alt_scan_segments"]), 0)
        self.assertGreater(len(t["move_segments"]), 0)
        # segments_total counts every phase (scan + alt + move).
        expected = len(t["scan_segments"]) + len(t["alt_scan_segments"]) + len(t["move_segments"])
        self.assertEqual(result.data["segments_total"], expected)

    async def test_go2_trace_records_odom_delta_when_perception_available(self):
        from robot_brain.actuation.unitree import FakeUnitreeTransport, UnitreeRobot, UnitreeState
        from robot_brain.core.robot_self_state import UltrasonicData
        from robot_brain.perception.unitree import UnitreePerceptionAdapter

        settings = Settings(
            robot_backend="unitree",
            unitree_transport="fake",
            unitree_dry_run=False,
            memory_db_path=":memory:",
        )
        transport = FakeUnitreeTransport(
            UnitreeState(
                connected=True,
                is_standing=True,
                ultrasonic=(1.0, 1.0, 1.0, 1.0),
            )
        )
        await transport.connect()
        robot = UnitreeRobot(transport, settings)
        skill = ExploreSkill(settings, perception=UnitreePerceptionAdapter(robot))
        world = WorldState(
            battery_level=80.0,
            robot_self_state=RobotSelfState(
                source="test",
                is_standing=True,
                state_age_seconds=0.1,
                ultrasonic=UltrasonicData(front_m=1.0, rear_m=1.0),
            ),
        )

        result = await skill.execute(ExploreParams(max_steps=1, step_distance_cm=15.0), robot, world)
        t = result.data["trace"][0]
        self.assertEqual("odom", t["progress_source"])
        self.assertIsNotNone(t["pose_before"])
        self.assertIsNotNone(t["pose_after"])
        self.assertIsNotNone(t["motion_delta"])
        self.assertGreaterEqual(t["motion_delta"]["delta_m"], settings.odom_progress_min_m)


if __name__ == "__main__":
    unittest.main()
