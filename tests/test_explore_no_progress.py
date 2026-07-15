"""Tests for explore stop protection: no_progress / semantic_hold / ping_pong."""
from __future__ import annotations

import unittest

from config.settings import Settings
from robot_brain.actuation.mock import MockRobot
from robot_brain.core.passability import PassabilityHint
from robot_brain.core.robot_self_state import RobotSelfState, UltrasonicData
from robot_brain.core.world_state import WorldState
from robot_brain.skills.builtin.explore import ExploreParams, ExploreSkill


class _StubPassability:
    """Returns a scripted sequence of hints (cycling)."""

    def __init__(self, hints: list[PassabilityHint | None]) -> None:
        self._hints = list(hints)
        self._i = 0

    async def analyze_if_due(self, world: WorldState) -> PassabilityHint | None:
        h = self._hints[self._i % len(self._hints)]
        self._i += 1
        if h is not None:
            world.passability_hint = h
        return h


def _hint(direction: str, confidence: float = 0.8) -> PassabilityHint:
    return PassabilityHint(recommended_direction=direction, confidence=confidence, reason="stub")  # type: ignore[arg-type]


def _world(front: float = 1.0, rear: float = 1.0) -> WorldState:
    return WorldState(
        battery_level=80.0,
        robot_self_state=RobotSelfState(
            source="test", ultrasonic=UltrasonicData(front_m=front, rear_m=rear),
        ),
    )


def _settings(**kw) -> Settings:
    base = dict(memory_db_path=":memory:")
    base.update(kw)
    return Settings(**base)


class NoProgressTests(unittest.IsolatedAsyncioTestCase):
    async def test_consecutive_retreats_stop_no_progress(self):
        # Front blocked, rear clear -> scan_alt + retreat every step (no nudge).
        skill = ExploreSkill(_settings(explore_no_progress_steps=3, explore_ping_pong_steps=10))
        result = await skill.execute(
            ExploreParams(max_steps=5), MockRobot(), _world(front=0.15, rear=1.0)
        )
        self.assertEqual(result.data["stop_reason"], "no_progress")
        self.assertLess(result.data["steps_completed"], 5)
        self.assertNotIn("nudge", result.data["actions"])

    async def test_nudge_resets_no_progress(self):
        # Clear front -> nudge every step -> never triggers no_progress.
        skill = ExploreSkill(_settings(explore_no_progress_steps=3))
        result = await skill.execute(
            ExploreParams(max_steps=3), MockRobot(), _world(front=1.0)
        )
        self.assertEqual(result.data["stop_reason"], "max_steps")
        self.assertEqual(result.data["steps_completed"], 3)

    async def test_odom_nudge_without_translation_stops_no_progress(self):
        from robot_brain.actuation.unitree import FakeUnitreeTransport, UnitreeCommand, UnitreeRobot, UnitreeState
        from robot_brain.perception.unitree import UnitreePerceptionAdapter

        class NoMotionTransport(FakeUnitreeTransport):
            def _apply_command(self, command: UnitreeCommand) -> None:
                if command.action == "drive":
                    self._state.is_moving = False
                    self._state.velocity = (0.0, 0.0, 0.0)
                    return
                super()._apply_command(command)

        settings = _settings(
            robot_backend="unitree",
            unitree_transport="fake",
            unitree_dry_run=False,
            explore_no_progress_steps=2,
            explore_ping_pong_steps=10,
        )
        transport = NoMotionTransport(
            UnitreeState(
                connected=True,
                is_standing=True,
                ultrasonic=(1.0, 1.0, 1.0, 1.0),
            )
        )
        await transport.connect()
        robot = UnitreeRobot(transport, settings)
        skill = ExploreSkill(settings, perception=UnitreePerceptionAdapter(robot))
        result = await skill.execute(ExploreParams(max_steps=4), robot, WorldState(battery_level=80.0))

        self.assertEqual(result.data["stop_reason"], "no_progress")
        self.assertEqual(result.data["steps_completed"], 2)
        self.assertTrue(all(t["progress_source"] == "odom" for t in result.data["trace"]))
        self.assertTrue(all(t["chosen_action"] == "nudge" for t in result.data["trace"]))


class SemanticHoldTests(unittest.IsolatedAsyncioTestCase):
    async def test_consecutive_vlm_hold_stops(self):
        # VLM says stop, front clear -> vlm_hold every step.
        skill = ExploreSkill(
            _settings(explore_max_holds=2, explore_no_progress_steps=10),
            passability=_StubPassability([_hint("stop")]),
        )
        result = await skill.execute(
            ExploreParams(max_steps=5), MockRobot(), _world(front=1.0)
        )
        self.assertEqual(result.data["stop_reason"], "semantic_hold")
        self.assertLess(result.data["steps_completed"], 5)
        self.assertIn("vlm_hold", result.data["actions"])


class PingPongTests(unittest.IsolatedAsyncioTestCase):
    async def test_alternating_alt_dirs_stops(self):
        # VLM alternates left/right; front blocked -> scan_alt_left/right + retreat.
        skill = ExploreSkill(
            _settings(explore_ping_pong_steps=4, explore_no_progress_steps=10),
            passability=_StubPassability([_hint("left"), _hint("right")]),
        )
        result = await skill.execute(
            ExploreParams(max_steps=6), MockRobot(), _world(front=0.15, rear=1.0)
        )
        self.assertEqual(result.data["stop_reason"], "ping_pong")
        self.assertLess(result.data["steps_completed"], 6)
        # Alternating alt tags present.
        actions = result.data["actions"]
        self.assertIn("scan_alt_left", actions)
        self.assertIn("scan_alt_right", actions)


if __name__ == "__main__":
    unittest.main()
