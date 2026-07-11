"""Unit tests for StateInterpreter — verifies thresholds are read from Settings."""
from __future__ import annotations

import unittest

from config.settings import Settings
from robot_brain.core.robot_self_state import RobotSelfState, UltrasonicData
from robot_brain.core.state_interpreter import StateInterpreter
from robot_brain.core.world_state import WorldState


class TestStateInterpreterBattery(unittest.TestCase):
    """Battery thresholds should come from Settings, not hardcoded values."""

    def setUp(self) -> None:
        self.settings = Settings()
        self.interpreter = StateInterpreter(self.settings)

    def test_critical_battery_uses_setting(self) -> None:
        world = WorldState(battery_level=9.0)
        result = self.interpreter.interpret(world)
        assert "CRITICAL" in result.summary["battery"]
        assert any("CRITICAL" in p for p in result.active_policies)

    def test_low_battery_uses_setting(self) -> None:
        world = WorldState(battery_level=20.0)
        result = self.interpreter.interpret(world)
        assert "LOW" in result.summary["battery"]
        assert any("LOW" in p for p in result.active_policies)

    def test_ok_battery(self) -> None:
        world = WorldState(battery_level=80.0)
        result = self.interpreter.interpret(world)
        assert "OK" in result.summary["battery"]

    def test_custom_thresholds(self) -> None:
        """Changing settings should change interpretation boundaries."""
        custom = Settings()
        # Override thresholds
        object.__setattr__(custom, "critical_battery_threshold", 20.0)
        object.__setattr__(custom, "low_battery_threshold", 50.0)
        interp = StateInterpreter(custom)

        # 15% is below new critical (20%)
        result = interp.interpret(WorldState(battery_level=15.0))
        assert "CRITICAL" in result.summary["battery"]

        # 30% is below new low (50%) but above new critical (20%)
        result = interp.interpret(WorldState(battery_level=30.0))
        assert "LOW" in result.summary["battery"]

        # 60% is OK
        result = interp.interpret(WorldState(battery_level=60.0))
        assert "OK" in result.summary["battery"]


class TestStateInterpreterEstop(unittest.TestCase):
    def test_estop_active(self) -> None:
        interp = StateInterpreter(Settings())
        world = WorldState(estop_active=True)
        result = interp.interpret(world)
        assert "estop" in result.summary
        assert any("E-stop" in p for p in result.active_policies)


class TestStateInterpreterStaleness(unittest.TestCase):
    def test_stale_state_uses_setting(self) -> None:
        settings = Settings()
        # Default: unitree_state_max_age_seconds = 2.0
        interp = StateInterpreter(settings)
        ss = RobotSelfState(source="test", state_age_seconds=3.0)
        world = WorldState(robot_self_state=ss)
        result = interp.interpret(world)
        assert "STALE" in result.summary["freshness"]
        assert any("STALE" in p for p in result.active_policies)

    def test_fresh_state(self) -> None:
        interp = StateInterpreter(Settings())
        ss = RobotSelfState(source="test", state_age_seconds=1.0)
        world = WorldState(robot_self_state=ss)
        result = interp.interpret(world)
        assert "FRESH" in result.summary["freshness"]

    def test_custom_stale_threshold(self) -> None:
        custom = Settings()
        object.__setattr__(custom, "unitree_state_max_age_seconds", 5.0)
        interp = StateInterpreter(custom)
        # 3s is now fresh (threshold=5)
        ss = RobotSelfState(source="test", state_age_seconds=3.0)
        world = WorldState(robot_self_state=ss)
        result = interp.interpret(world)
        assert "FRESH" in result.summary["freshness"]


class TestStateInterpreterProximity(unittest.TestCase):
    def test_obstacle_front_uses_setting(self) -> None:
        interp = StateInterpreter(Settings())
        ss = RobotSelfState(
            source="test",
            ultrasonic=UltrasonicData(front_m=0.2),
        )
        world = WorldState(robot_self_state=ss)
        result = interp.interpret(world)
        assert "OBSTACLE" in result.summary.get("proximity", "")
        assert any("FRONT" in p for p in result.active_policies)

    def test_clear_proximity(self) -> None:
        interp = StateInterpreter(Settings())
        ss = RobotSelfState(
            source="test",
            ultrasonic=UltrasonicData(front_m=1.0, rear_m=0.8),
        )
        world = WorldState(robot_self_state=ss)
        result = interp.interpret(world)
        assert "CLEAR" in result.summary.get("proximity", "")

    def test_custom_proximity_threshold(self) -> None:
        custom = Settings()
        object.__setattr__(custom, "obstacle_proximity_threshold", 0.5)
        interp = StateInterpreter(custom)
        # 0.4m is now an obstacle (threshold=0.5)
        ss = RobotSelfState(
            source="test",
            ultrasonic=UltrasonicData(front_m=0.4),
        )
        world = WorldState(robot_self_state=ss)
        result = interp.interpret(world)
        assert "OBSTACLE" in result.summary.get("proximity", "")


class TestStateInterpreterNominal(unittest.TestCase):
    def test_nominal_state_has_default_policy(self) -> None:
        interp = StateInterpreter(Settings())
        world = WorldState(battery_level=90.0)
        result = interp.interpret(world)
        assert any("nominal" in p for p in result.active_policies)


class TestWorldStateSummaryPublicAPI(unittest.TestCase):
    """Verify the public state_summary() method on WorldState."""

    def test_state_summary_without_settings(self) -> None:
        world = WorldState(battery_level=5.0)
        summary = world.state_summary()
        assert "CRITICAL" in summary["battery"]

    def test_state_summary_with_settings(self) -> None:
        settings = Settings()
        world = WorldState(battery_level=5.0)
        summary = world.state_summary(settings)
        assert "CRITICAL" in summary["battery"]

    def test_backward_compat_private_method(self) -> None:
        """_build_state_summary still works for backward compat."""
        world = WorldState(battery_level=20.0)
        summary = world._build_state_summary()
        assert "LOW" in summary["battery"]


class TestStateInterpreterPassability(unittest.TestCase):
    """VLM passability hint surfaces in the summary and cognitive_snapshot."""

    def setUp(self) -> None:
        self.settings = Settings()
        self.interpreter = StateInterpreter(self.settings)

    def test_no_hint_omits_passability(self) -> None:
        result = self.interpreter.interpret(WorldState(battery_level=80.0))
        assert "passability" not in result.summary

    def test_hint_appears_in_summary(self) -> None:
        from robot_brain.core.passability import PassabilityHint

        world = WorldState(
            battery_level=80.0,
            passability_hint=PassabilityHint(
                recommended_direction="left", confidence=0.82, reason="left open"
            ),
        )
        result = self.interpreter.interpret(world)
        assert "left" in result.summary["passability"]
        assert "0.82" in result.summary["passability"]
        assert any("SOFT" in p for p in result.active_policies)

    def test_hint_flows_into_cognitive_snapshot(self) -> None:
        from robot_brain.core.passability import PassabilityHint

        world = WorldState(
            passability_hint=PassabilityHint(recommended_direction="stop", confidence=0.9),
        )
        snap = world.cognitive_snapshot(self.settings)
        assert "left" not in snap["_state_summary"].get("passability", "")
        assert "stop" in snap["_state_summary"]["passability"]


if __name__ == "__main__":
    unittest.main()
