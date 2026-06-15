"""Tests for Go2 FastReflex rules and integration with DualSystem."""
from __future__ import annotations

import unittest

from config.settings import Settings
from robot_brain.cognition.dual_system import DualSystem
from robot_brain.cognition.fast_reflex import FastReflex
from robot_brain.cognition.go2_reflex_rules import decide_go2_reflex
from robot_brain.cognition.planner import Planner
from robot_brain.core.robot_self_state import RobotSelfState
from robot_brain.core.world_state import TaskProgress, WorldState
from robot_brain.llm.base import ToolCall


def _settings(**overrides) -> Settings:
    return Settings(memory_db_path=":memory:", **overrides)


def _go2_world(**ss_overrides) -> WorldState:
    w = WorldState()
    defaults: dict[str, object] = {
        "source": "unitree_go2",
        "is_standing": True,
        "is_moving": False,
        "error_code": 0,
        "state_age_seconds": 0.1,
    }
    defaults.update(ss_overrides)
    w.robot_self_state = RobotSelfState(**defaults)  # type: ignore[arg-type]
    return w


# ---------------------------------------------------------------------------
# decide_go2_reflex unit tests
# ---------------------------------------------------------------------------

class Go2ReflexRulesTests(unittest.TestCase):
    def setUp(self):
        self.settings = _settings()

    def test_none_self_state_returns_none(self):
        w = WorldState()
        self.assertIsNone(decide_go2_reflex(w, self.settings))

    def test_error_code_triggers_stop_and_report(self):
        w = _go2_world(error_code=7004)
        calls = decide_go2_reflex(w, self.settings)
        self.assertIsNotNone(calls)
        names = [c.skill_name for c in calls]  # type: ignore[union-attr]
        self.assertIn("stop", names)
        self.assertIn("report", names)
        self.assertEqual("critical", calls[1].parameters["severity"])  # type: ignore[index, union-attr]

    def test_stale_state_triggers_stop_and_warning(self):
        w = _go2_world(state_age_seconds=5.0)
        calls = decide_go2_reflex(w, self.settings)
        self.assertIsNotNone(calls)
        names = [c.skill_name for c in calls]  # type: ignore[union-attr]
        self.assertIn("stop", names)
        self.assertIn("report", names)
        self.assertEqual("warning", calls[1].parameters["severity"])  # type: ignore[index, union-attr]

    def test_not_standing_triggers_report_only(self):
        w = _go2_world(is_standing=False)
        calls = decide_go2_reflex(w, self.settings)
        self.assertIsNotNone(calls)
        names = [c.skill_name for c in calls]  # type: ignore[union-attr]
        self.assertEqual(["report"], names)  # no stop for not-standing
        self.assertEqual("warning", calls[0].parameters["severity"])  # type: ignore[index, union-attr]

    def test_moving_without_running_task_triggers_stop(self):
        w = _go2_world(is_moving=True)
        w.current_task = TaskProgress(objective="test", status="idle")
        calls = decide_go2_reflex(w, self.settings)
        self.assertIsNotNone(calls)
        names = [c.skill_name for c in calls]  # type: ignore[union-attr]
        self.assertIn("stop", names)

    def test_moving_with_running_task_no_trigger(self):
        w = _go2_world(is_moving=True)
        w.current_task = TaskProgress(objective="test", status="running")
        calls = decide_go2_reflex(w, self.settings)
        # Should not trigger rule 5 while task is running,
        # but battery is 100 so no other rule fires.
        self.assertIsNone(calls)

    def test_critical_battery_stop_report_no_dock(self):
        w = _go2_world()
        w.battery_level = 5.0
        calls = decide_go2_reflex(w, self.settings)
        self.assertIsNotNone(calls)
        names = [c.skill_name for c in calls]  # type: ignore[union-attr]
        self.assertIn("stop", names)
        self.assertIn("report", names)
        self.assertNotIn("dock", names)

    def test_low_battery_report_no_dock(self):
        w = _go2_world()
        w.battery_level = 20.0
        calls = decide_go2_reflex(w, self.settings)
        self.assertIsNotNone(calls)
        names = [c.skill_name for c in calls]  # type: ignore[union-attr]
        self.assertIn("report", names)
        self.assertNotIn("dock", names)
        self.assertNotIn("stop", names)  # low ≠ critical → no stop

    def test_healthy_no_rules_fire(self):
        w = _go2_world()
        self.assertIsNone(decide_go2_reflex(w, self.settings))

    def test_skip_error_check_suppresses_rule_2(self):
        w = _go2_world(error_code=42)
        calls = decide_go2_reflex(w, self.settings, skip_error_check=True)
        # Error rule suppressed; battery=100, standing=True, fresh state →
        # no rules fire
        self.assertIsNone(calls)


# ---------------------------------------------------------------------------
# FastReflex integration
# ---------------------------------------------------------------------------

class FastReflexGo2Tests(unittest.TestCase):
    def test_go2_rules_preempt_generic_battery(self):
        """On Go2 (self_state present), critical battery → stop+report, not dock."""
        reflex = FastReflex(_settings())
        w = _go2_world()
        w.battery_level = 5.0
        calls = reflex.decide(w)
        names = [c.skill_name for c in calls]
        self.assertNotIn("dock", names, "Go2 path must NOT suggest dock")
        self.assertIn("stop", names)

    def test_mock_low_battery_still_dock(self):
        """Without self_state, generic rules still dock."""
        reflex = FastReflex(_settings())
        w = WorldState()
        w.battery_level = 10.0
        calls = reflex.decide(w)
        names = [c.skill_name for c in calls]
        self.assertIn("dock", names)

    def test_estop_still_priority_one_with_go2(self):
        reflex = FastReflex(_settings())
        w = _go2_world(error_code=7004)
        w.estop_active = True
        calls = reflex.decide(w)
        self.assertEqual(1, len(calls))
        self.assertEqual("stop", calls[0].skill_name)

    def test_no_go2_rules_when_self_state_none_mock_still_docks(self):
        reflex = FastReflex(_settings(robot_backend="mock"))
        w = WorldState()  # no robot_self_state
        w.battery_level = 5.0
        calls = reflex.decide(w)
        names = [c.skill_name for c in calls]
        self.assertIn("dock", names)

    def test_unitree_without_self_state_no_dock(self):
        """unitree + RDB_PERCEPTION=mock: low battery → stop+report, not dock."""
        reflex = FastReflex(_settings(robot_backend="unitree"))
        w = WorldState()
        w.battery_level = 5.0
        calls = reflex.decide(w)
        names = [c.skill_name for c in calls]
        self.assertNotIn("dock", names)
        self.assertIn("stop", names)
        self.assertIn("report", names)

    def test_go2_healthy_critical_alert_still_fires(self):
        reflex = FastReflex(_settings())
        w = _go2_world()
        w.alerts = ["critical: smoke detected"]
        calls = reflex.decide(w)
        names = [c.skill_name for c in calls]
        self.assertEqual(["report"], names)
        self.assertEqual("critical", calls[0].parameters["severity"])


# ---------------------------------------------------------------------------
# Error debounce
# ---------------------------------------------------------------------------

class ErrorDebounceTests(unittest.TestCase):
    def test_debounce_default_triggers_immediately(self):
        reflex = FastReflex(_settings())
        w = _go2_world(error_code=7004)
        calls = reflex.decide(w)
        names = [c.skill_name for c in calls]
        self.assertIn("stop", names)

    def test_debounce_2_single_error_no_trigger(self):
        s = _settings(go2_reflex_error_debounce=2)
        reflex = FastReflex(s)
        w = _go2_world(error_code=7004)
        # First call: streak → 1, < 2, skip_error=True → no rules fire
        calls = reflex.decide(w)
        self.assertEqual([], calls)
        # Second call with same error: streak → 2, ≥ 2 → fire
        calls = reflex.decide(w)
        names = [c.skill_name for c in calls]
        self.assertIn("stop", names)

    def test_debounce_resets_on_recovery(self):
        s = _settings(go2_reflex_error_debounce=3)
        reflex = FastReflex(s)
        w_err = _go2_world(error_code=7004)
        w_ok = _go2_world(error_code=0)

        reflex.decide(w_err)  # streak 1
        reflex.decide(w_err)  # streak 2
        reflex.decide(w_ok)   # streak → 0
        calls = reflex.decide(w_err)  # streak 1 < 3
        self.assertEqual([], calls)


# ---------------------------------------------------------------------------
# DualSystem integration
# ---------------------------------------------------------------------------

class DualSystemGo2Tests(unittest.TestCase):
    def setUp(self):
        self.settings = _settings()

    def test_reflex_fast_when_error(self):
        w = _go2_world(error_code=42)
        reflex = FastReflex(self.settings)
        # Use a dummy planner that would otherwise return results
        class _DummyPlanner:
            async def plan(self, cmd, world):
                return [ToolCall(skill_name="nudge", parameters={}, source="slow")]
        dual = DualSystem(reflex, _DummyPlanner())
        import asyncio
        decision = asyncio.run(dual.decide("test", w))
        self.assertEqual("fast", decision.source)

    def test_planner_slow_when_healthy(self):
        w = _go2_world()
        reflex = FastReflex(self.settings)
        class _DummyPlanner:
            async def plan(self, cmd, world):
                return [ToolCall(skill_name="report", parameters={"message": "ok", "severity": "info"}, source="slow")]
        dual = DualSystem(reflex, _DummyPlanner())
        import asyncio
        decision = asyncio.run(dual.decide("test", w))
        self.assertEqual("slow", decision.source)


if __name__ == "__main__":
    unittest.main()
