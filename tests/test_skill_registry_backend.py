"""Tests for backend-aware tool filtering and Validator backend checks."""
from __future__ import annotations

import unittest

from config.settings import Settings
from robot_brain.core.world_state import WorldState
from robot_brain.llm.base import ToolCall
from robot_brain.safety.validator import SafetyValidator
from robot_brain.skills.builtin import default_skills, go2_skills
from robot_brain.skills.registry import UNITREE_LLM_SKILLS, SkillRegistry


def _make_registry(backend: str) -> SkillRegistry:
    s = Settings(robot_backend=backend, memory_db_path=":memory:")
    if backend == "unitree":
        return SkillRegistry(default_skills() + go2_skills(s))
    return SkillRegistry(default_skills())


def _tool_names(registry: SkillRegistry, backend: str) -> set[str]:
    return {t["name"] for t in registry.tools_for_backend(backend)}


# ---------------------------------------------------------------------------
# tools_for_backend
# ---------------------------------------------------------------------------

class ToolsForBackendTests(unittest.TestCase):
    def test_unitree_excludes_navigate(self):
        reg = _make_registry("unitree")
        names = _tool_names(reg, "unitree")
        self.assertNotIn("navigate", names)
        self.assertNotIn("patrol", names)
        self.assertNotIn("follow", names)
        self.assertNotIn("dock", names)

    def test_unitree_includes_go2_skills(self):
        reg = _make_registry("unitree")
        names = _tool_names(reg, "unitree")
        for sk in UNITREE_LLM_SKILLS:
            self.assertIn(sk, names, f"{sk} should be in unitree tools")

    def test_mock_includes_all(self):
        reg = _make_registry("mock")
        names = _tool_names(reg, "mock")
        self.assertIn("navigate", names)
        self.assertIn("patrol", names)
        self.assertIn("follow", names)
        self.assertIn("dock", names)

    def test_unitree_tools_strict_passthrough(self):
        reg = _make_registry("unitree")
        tools = reg.tools_for_backend("unitree", strict=False)
        for t in tools:
            self.assertFalse(t["strict"])

    def test_tools_unchanged_for_mock(self):
        reg = _make_registry("mock")
        raw = reg.tools()
        filtered = reg.tools_for_backend("mock")
        self.assertEqual(len(raw), len(filtered))


# ---------------------------------------------------------------------------
# SafetyValidator backend checks
# ---------------------------------------------------------------------------

class ValidatorBackendTests(unittest.TestCase):
    def setUp(self):
        self.unitree_settings = Settings(
            robot_backend="unitree",
            memory_db_path=":memory:",
        )
        self.mock_settings = Settings(
            robot_backend="mock",
            memory_db_path=":memory:",
        )
        self.unitree_reg = _make_registry("unitree")
        self.mock_reg = _make_registry("mock")

    def test_rejects_navigate_on_unitree(self):
        v = SafetyValidator(self.unitree_settings, self.unitree_reg)
        res = v.validate(
            ToolCall(skill_name="navigate", parameters={"target": {"x": 1, "y": 0}, "speed": 0.5}),
            WorldState(),
            confirmation_granted=True,
        )
        self.assertFalse(res.allowed)
        self.assertIn("unsupported on unitree", res.reason)

    def test_rejects_patrol_on_unitree(self):
        v = SafetyValidator(self.unitree_settings, self.unitree_reg)
        res = v.validate(
            ToolCall(skill_name="patrol", parameters={"waypoints": [{"x": 1, "y": 0}]}),
            WorldState(),
            confirmation_granted=True,
        )
        self.assertFalse(res.allowed)

    def test_rejects_follow_on_unitree(self):
        v = SafetyValidator(self.unitree_settings, self.unitree_reg)
        res = v.validate(
            ToolCall(skill_name="follow", parameters={"target_id": "x", "distance": 2.0}),
            WorldState(),
            confirmation_granted=True,
        )
        self.assertFalse(res.allowed)

    def test_allows_nudge_on_unitree(self):
        v = SafetyValidator(self.unitree_settings, self.unitree_reg)
        res = v.validate(
            ToolCall(skill_name="nudge", parameters={"direction": "forward", "distance_cm": 20}),
            WorldState(),
            confirmation_granted=True,
        )
        self.assertTrue(res.allowed, res.reason)

    def test_allows_navigate_on_mock(self):
        v = SafetyValidator(self.mock_settings, self.mock_reg)
        res = v.validate(
            ToolCall(skill_name="navigate", parameters={"target": {"x": 1, "y": 0}, "speed": 0.5}),
            WorldState(),
        )
        # navigate is allowed on mock (follow requires confirmation, but navigate doesn't)
        self.assertTrue(res.allowed, res.reason)


if __name__ == "__main__":
    unittest.main()
