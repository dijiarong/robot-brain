"""Deterministic fast-system rules for urgent or obvious situations."""
from __future__ import annotations

from config.settings import Settings
from robot_brain.cognition.go2_reflex_rules import decide_go2_reflex
from robot_brain.core.world_state import WorldState
from robot_brain.llm.base import ToolCall


def _critical_alert_calls(world: WorldState) -> list[ToolCall]:
    """Rule 8: critical perception alerts → report."""
    critical_alerts = [
        alert for alert in world.alerts if alert.lower().startswith("critical:")
    ]
    if not critical_alerts:
        return []
    return [
        ToolCall(
            skill_name="report",
            parameters={"message": "; ".join(critical_alerts), "severity": "critical"},
            reason="critical perception alert",
            source="fast",
        )
    ]


def _unitree_battery_calls(world: WorldState, settings: Settings) -> list[ToolCall]:
    """Battery reflex for unitree when dock is unavailable (no self_state path)."""
    if world.battery_level <= settings.critical_battery_threshold:
        return [
            ToolCall(
                skill_name="stop",
                parameters={"reason": "critical battery"},
                reason="critical battery",
                source="fast",
            ),
            ToolCall(
                skill_name="report",
                parameters={"message": "Go2 critical battery", "severity": "critical"},
                reason="critical battery",
                source="fast",
            ),
        ]
    if world.battery_level <= settings.low_battery_threshold:
        return [
            ToolCall(
                skill_name="report",
                parameters={
                    "message": f"Go2 low battery ({world.battery_level:.0f}%)",
                    "severity": "warning",
                },
                reason="low battery",
                source="fast",
            )
        ]
    return []


class FastReflex:
    def __init__(self, settings: Settings) -> None:
        self.settings = settings
        self._go2_error_streak: int = 0

    def decide(self, world: WorldState) -> list[ToolCall]:
        # Priority 1: estop always wins (regardless of backend)
        if world.estop_active:
            return [ToolCall(skill_name="stop", parameters={"reason": "emergency stop active"}, source="fast")]

        # ---- Go2 path (robot_self_state is available) ----
        ss = world.robot_self_state
        if ss is not None:
            if ss.error_code is not None and ss.error_code != 0:
                self._go2_error_streak += 1
            else:
                self._go2_error_streak = 0

            skip_error = self._go2_error_streak < self.settings.go2_reflex_error_debounce
            go2_calls = decide_go2_reflex(world, self.settings, skip_error_check=skip_error)
            if go2_calls:
                return go2_calls
            alert_calls = _critical_alert_calls(world)
            if alert_calls:
                return alert_calls
            return []

        # ---- Unitree backend without self_state (e.g. RDB_PERCEPTION=mock) ----
        if self.settings.robot_backend == "unitree":
            battery_calls = _unitree_battery_calls(world, self.settings)
            if battery_calls:
                return battery_calls
            alert_calls = _critical_alert_calls(world)
            if alert_calls:
                return alert_calls
            return []

        # ---- Mock / generic path ----
        if world.battery_level <= self.settings.critical_battery_threshold:
            return [ToolCall(skill_name="dock", parameters={}, reason="critical battery", source="fast")]
        if world.battery_level <= self.settings.low_battery_threshold:
            return [ToolCall(skill_name="dock", parameters={}, reason="low battery", source="fast")]
        alert_calls = _critical_alert_calls(world)
        if alert_calls:
            return alert_calls
        return []
