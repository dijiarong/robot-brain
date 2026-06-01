"""Deterministic fast-system rules for urgent or obvious situations."""
from __future__ import annotations

from config.settings import Settings
from robodog_brain.core.world_state import WorldState
from robodog_brain.llm.base import ToolCall


class FastReflex:
    def __init__(self, settings: Settings) -> None:
        self.settings = settings

    def decide(self, world: WorldState) -> list[ToolCall]:
        if world.estop_active:
            return [ToolCall(skill_name="stop", parameters={"reason": "emergency stop active"}, source="fast")]
        if world.battery_level <= self.settings.critical_battery_threshold:
            return [ToolCall(skill_name="dock", parameters={}, reason="critical battery", source="fast")]
        if world.battery_level <= self.settings.low_battery_threshold:
            return [ToolCall(skill_name="dock", parameters={}, reason="low battery", source="fast")]
        critical_alerts = [alert for alert in world.alerts if alert.lower().startswith("critical:")]
        if critical_alerts:
            return [
                ToolCall(
                    skill_name="report",
                    parameters={"message": "; ".join(critical_alerts), "severity": "critical"},
                    reason="critical perception alert",
                    source="fast",
                )
            ]
        return []
