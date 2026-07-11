"""Tests for explore + VLM passability hint integration (iteration 17).

Covers the doc's verification matrix: VLM soft direction, forward gating on
``stop``, confidence threshold, ultrasonic hard gate, and full regression when
no analyzer is wired (iteration 16 behavior).
"""
from __future__ import annotations

import unittest

from config.settings import Settings
from robot_brain.actuation.mock import MockRobot
from robot_brain.core.passability import PassabilityHint
from robot_brain.core.robot_self_state import RobotSelfState, UltrasonicData
from robot_brain.core.world_state import WorldState
from robot_brain.skills.builtin.explore import ExploreParams, ExploreSkill


class _StubPassability:
    """Minimal stand-in for PassabilityAnalyzer returning a fixed hint."""

    def __init__(self, hint: PassabilityHint | None) -> None:
        self._hint = hint

    async def analyze_if_due(self, world: WorldState) -> PassabilityHint | None:
        if self._hint is not None:
            world.passability_hint = self._hint
        return self._hint


def _settings(**kw) -> Settings:
    base = dict(memory_db_path=":memory:", vlm_confidence_min=0.5)
    base.update(kw)
    return Settings(**base)


def _hint(direction: str, confidence: float = 0.8) -> PassabilityHint:
    return PassabilityHint(
        recommended_direction=direction,  # type: ignore[arg-type]
        confidence=confidence,
        reason="stub",
    )


def _world(front: float = 1.0, rear: float = 1.0, left=None, right=None) -> WorldState:
    return WorldState(
        battery_level=80.0,
        robot_self_state=RobotSelfState(
            source="test",
            ultrasonic=UltrasonicData(front_m=front, rear_m=rear, left_m=left, right_m=right),
        ),
    )


async def _run(skill_hint, world, *, max_steps=2) -> tuple[list[str], PassabilityHint | None]:
    settings = _settings()
    skill = ExploreSkill(settings, passability=_StubPassability(skill_hint))
    result = await skill.execute(ExploreParams(max_steps=max_steps), MockRobot(), world)
    return result.data["actions"], world.passability_hint


class ExploreVlmDirectionTests(unittest.IsolatedAsyncioTestCase):
    async def test_hint_right_uses_scan_alt_right(self):
        actions, hint = await _run(_hint("right"), _world(front=0.15))
        self.assertIn("scan_alt_right", actions)
        self.assertEqual(hint.recommended_direction, "right")

    async def test_hint_left_uses_scan_alt_left(self):
        actions, _ = await _run(_hint("left"), _world(front=0.15))
        self.assertIn("scan_alt_left", actions)

    async def test_low_confidence_falls_back_to_scan_alt(self):
        actions, _ = await _run(_hint("right", confidence=0.3), _world(front=0.15))
        self.assertIn("scan_alt", actions)
        self.assertNotIn("scan_alt_right", actions)


class ExploreVlmForwardGatingTests(unittest.IsolatedAsyncioTestCase):
    async def test_hint_stop_with_front_clear_holds(self):
        actions, _ = await _run(_hint("stop"), _world(front=1.0))
        self.assertIn("vlm_hold", actions)
        self.assertNotIn("nudge", actions)

    async def test_hint_forward_with_front_clear_nudges(self):
        actions, _ = await _run(_hint("forward"), _world(front=1.0))
        self.assertIn("nudge", actions)
        self.assertNotIn("vlm_hold", actions)

    async def test_front_near_blocks_forward_regardless_of_hint(self):
        # VLM says forward, but ultrasonic front is near -> never forward nudge.
        actions, _ = await _run(_hint("forward"), _world(front=0.15))
        self.assertNotIn("nudge", actions)


class ExploreVlmRegressionTests(unittest.IsolatedAsyncioTestCase):
    async def test_no_analyzer_matches_iteration_16(self):
        settings = _settings()
        skill = ExploreSkill(settings, passability=None)
        result = await skill.execute(
            ExploreParams(max_steps=2), MockRobot(), _world(front=0.15)
        )
        # No VLM, no left/right ultrasonic -> default +90 with legacy "scan_alt" tag.
        self.assertIn("scan_alt", result.data["actions"])
        self.assertNotIn("scan_alt_left", result.data["actions"])
        self.assertNotIn("scan_alt_right", result.data["actions"])

    async def test_ultrasonic_fallback_picks_clearer_side(self):
        # No usable hint (None); left clearer than right -> scan_alt_left.
        actions, _ = await _run(None, _world(front=0.15, left=2.0, right=0.2))
        self.assertIn("scan_alt_left", actions)


if __name__ == "__main__":
    unittest.main()
