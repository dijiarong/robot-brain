"""Tests for SafetyPolicy and its integration into SafetyValidator."""
from __future__ import annotations

import unittest

from config.settings import Settings
from robot_brain.core.world_state import WorldState
from robot_brain.llm.base import ToolCall
from robot_brain.safety.policy import SafetyPolicy
from robot_brain.safety.validator import SafetyValidator
from robot_brain.skills.builtin import default_skills
from robot_brain.skills.registry import SkillRegistry
from robot_brain.tools.base import (
    CapabilityMetadata,
    MotionKind,
    RiskLevel,
)

_SETTINGS = dict(memory_db_path=":memory:")


def _md(**kw) -> CapabilityMetadata:
    base = dict(risk_level=RiskLevel.LOW, motion_kind=MotionKind.NONE)
    base.update(kw)
    return CapabilityMetadata(**base)


# ---------------------------------------------------------------------------
# SafetyPolicy unit tests
# ---------------------------------------------------------------------------

class SafetyPolicyTests(unittest.TestCase):
    def setUp(self) -> None:
        self.settings = Settings(**_SETTINGS)
        self.policy = SafetyPolicy(self.settings)

    def test_backend_allowlist_denies(self):
        md = _md(backend_allowlist=("unitree",))
        world = WorldState()
        d = self.policy.evaluate(md, world, backend="mock")
        self.assertFalse(d.allowed)
        self.assertEqual(d.error_code and d.error_code.value, "safety_not_whitelisted")

    def test_backend_no_allowlist_allows_all(self):
        md = _md(backend_allowlist=None)
        world = WorldState()
        for backend in ("mock", "unitree"):
            self.assertTrue(self.policy.evaluate(md, world, backend=backend).allowed)

    def test_estop_allows_stop_denies_linear(self):
        world = WorldState(estop_active=True)
        stop = _md(motion_kind=MotionKind.STOP)
        linear = _md(motion_kind=MotionKind.LINEAR)
        self.assertTrue(self.policy.evaluate(stop, world, backend="mock").allowed)
        d = self.policy.evaluate(linear, world, backend="mock")
        self.assertFalse(d.allowed)
        self.assertEqual(d.error_code and d.error_code.value, "safety_estop_active")

    def test_estop_allows_read_only(self):
        world = WorldState(estop_active=True)
        ro = _md(risk_level=RiskLevel.READ_ONLY, motion_kind=MotionKind.NONE)
        self.assertTrue(self.policy.evaluate(ro, world, backend="mock").allowed)

    def test_critical_battery_allows_stop_denies_linear(self):
        world = WorldState(battery_level=5.0)
        stop = _md(motion_kind=MotionKind.STOP)
        linear = _md(motion_kind=MotionKind.LINEAR)
        self.assertTrue(self.policy.evaluate(stop, world, backend="mock").allowed)
        d = self.policy.evaluate(linear, world, backend="mock")
        self.assertFalse(d.allowed)
        self.assertEqual(d.error_code and d.error_code.value, "safety_battery_critical")

    def test_critical_battery_allows_read_only(self):
        world = WorldState(battery_level=5.0)
        ro = _md(risk_level=RiskLevel.READ_ONLY)
        self.assertTrue(self.policy.evaluate(ro, world, backend="mock").allowed)

    def test_requires_confirmation_without_grant(self):
        md = _md(requires_confirmation=True)
        world = WorldState()
        d = self.policy.evaluate(md, world, backend="mock", confirmation_granted=False)
        self.assertFalse(d.allowed)
        self.assertTrue(d.requires_confirmation)
        self.assertEqual(d.error_code and d.error_code.value, "safety_confirmation_required")

    def test_requires_confirmation_with_grant_allows(self):
        md = _md(requires_confirmation=True)
        world = WorldState()
        d = self.policy.evaluate(md, world, backend="mock", confirmation_granted=True)
        self.assertTrue(d.allowed)

    def test_allows_clean_state(self):
        md = _md(motion_kind=MotionKind.LINEAR, risk_level=RiskLevel.MEDIUM)
        d = self.policy.evaluate(md, WorldState(), backend="mock", confirmation_granted=True)
        self.assertTrue(d.allowed)


class SafetyPolicyGranularTests(unittest.TestCase):
    """The validator relies on the three checks being independent and ordered."""

    def setUp(self) -> None:
        self.policy = SafetyPolicy(Settings(**_SETTINGS))

    def test_check_confirmation_independent_of_state(self):
        # Estop active, but check_confirmation only looks at confirmation_grant.
        md = _md(motion_kind=MotionKind.LINEAR, requires_confirmation=True)
        d = self.policy.check_confirmation(md, confirmation_granted=False)
        self.assertFalse(d.allowed)
        self.assertTrue(d.requires_confirmation)
        # Granted -> allowed regardless of world state (state is a separate check).
        self.assertTrue(self.policy.check_confirmation(md, confirmation_granted=True).allowed)

    def test_check_state_independent_of_confirmation(self):
        md = _md(motion_kind=MotionKind.LINEAR)
        world = WorldState(estop_active=True)
        # check_state ignores confirmation; estop denies a linear capability.
        d = self.policy.check_state(md, world)
        self.assertFalse(d.allowed)
        self.assertEqual(d.error_code and d.error_code.value, "safety_estop_active")

    def test_check_backend_independent(self):
        md = _md(backend_allowlist=("unitree",))
        self.assertFalse(self.policy.check_backend(md, "mock").allowed)
        self.assertTrue(self.policy.check_backend(md, "unitree").allowed)

    def test_evaluate_runs_backend_state_confirmation_in_order(self):
        # backend denied -> evaluate returns backend denial first.
        md = _md(backend_allowlist=("unitree",), motion_kind=MotionKind.LINEAR, requires_confirmation=True)
        d = self.policy.evaluate(md, WorldState(estop_active=True), backend="mock", confirmation_granted=False)
        self.assertEqual(d.error_code and d.error_code.value, "safety_not_whitelisted")


# ---------------------------------------------------------------------------
# SafetyValidator integration (stop is the migrated vertical slice)
# ---------------------------------------------------------------------------

class ValidatorPolicyIntegrationTests(unittest.TestCase):
    def setUp(self) -> None:
        self.settings = Settings(**_SETTINGS)
        self.registry = SkillRegistry(default_skills())
        self.validator = SafetyValidator(self.settings, self.registry)

    def test_stop_allowed_during_estop(self):
        world = WorldState(estop_active=True)
        res = self.validator.validate(
            ToolCall(skill_name="stop", parameters={"reason": "halt"}), world
        )
        self.assertTrue(res.allowed, res.reason)

    def test_stop_allowed_on_critical_battery(self):
        world = WorldState(battery_level=5.0)
        res = self.validator.validate(
            ToolCall(skill_name="stop", parameters={"reason": "low"}), world
        )
        self.assertTrue(res.allowed, res.reason)

    def test_stop_allowed_without_confirmation(self):
        # stop metadata.requires_confirmation is False.
        res = self.validator.validate(
            ToolCall(skill_name="stop", parameters={}), WorldState(),
            confirmation_granted=False,
        )
        self.assertTrue(res.allowed, res.reason)


if __name__ == "__main__":
    unittest.main()
