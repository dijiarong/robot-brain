"""Centralised state interpretation — single source of truth for thresholds.

Reads ALL safety/policy thresholds from Settings so that prompt policies,
state summaries, and fast-reflex rules never drift out of sync.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from config.settings import Settings
    from robot_brain.core.world_state import WorldState


@dataclass
class StateInterpretation:
    """Result of interpreting the current world state against configured thresholds."""

    summary: dict[str, str] = field(default_factory=dict)
    active_policies: list[str] = field(default_factory=list)


class StateInterpreter:
    """Interprets WorldState using thresholds from Settings.

    Eliminates hardcoded magic numbers in both the LLM prompt layer
    (PromptBuilder) and the world-state summary builder.
    """

    def __init__(self, settings: "Settings") -> None:
        self._settings = settings

    def interpret(self, world: "WorldState") -> StateInterpretation:
        """Produce a unified summary + policy list from the current state."""
        summary: dict[str, str] = {}
        policies: list[str] = []

        s = self._settings
        critical_bat = s.critical_battery_threshold
        low_bat = s.low_battery_threshold
        stale_seconds = s.unitree_state_max_age_seconds
        obstacle_m = s.obstacle_proximity_threshold

        # --- Battery ---
        if world.battery_level <= critical_bat:
            summary["battery"] = (
                f"CRITICAL ({world.battery_level:.0f}%) — must stop and report immediately"
            )
            policies.append(
                f"- Battery CRITICAL (≤{critical_bat:.0f}%): ONLY use `stop` or `report`. No motion."
            )
        elif world.battery_level <= low_bat:
            summary["battery"] = (
                f"LOW ({world.battery_level:.0f}%) — conservative actions only, avoid long motion"
            )
            policies.append(
                f"- Battery LOW (≤{low_bat:.0f}%): Avoid long-distance motion (nudge ≤20cm). "
                "Prefer `report` to inform operator."
            )
        else:
            summary["battery"] = f"OK ({world.battery_level:.0f}%)"

        # --- E-stop ---
        if world.estop_active:
            summary["estop"] = "ACTIVE — only stop command is permitted"
            policies.append("- E-stop ACTIVE: ONLY `stop` is permitted. No other actions.")

        # --- Robot self state ---
        ss = world.robot_self_state
        if ss is not None:
            # Posture
            if ss.is_standing is False:
                summary["posture"] = "NOT STANDING — motion commands forbidden until robot stands"
                policies.append(
                    "- Robot NOT STANDING: Do NOT issue nudge/scan/retreat. "
                    "Use `report` to inform operator."
                )
            elif ss.is_standing is True:
                summary["posture"] = "STANDING — ready for motion"

            # Error code
            if ss.error_code is not None and ss.error_code != 0:
                summary["error"] = f"FAULT (code={ss.error_code}) — stop and report"
                policies.append(
                    "- Hardware FAULT detected: Use `stop` then "
                    "`report(severity=critical)` immediately."
                )
            else:
                summary["error"] = "NORMAL"

            # State freshness
            if ss.state_age_seconds is not None:
                if ss.state_age_seconds > stale_seconds:
                    summary["freshness"] = (
                        f"STALE ({ss.state_age_seconds:.1f}s) — data outdated, be cautious"
                    )
                    policies.append(
                        f"- State data STALE (>{stale_seconds:.0f}s old): Be cautious. "
                        "Prefer `stop` + `report(severity=warning)` if motion was planned."
                    )
                else:
                    summary["freshness"] = f"FRESH ({ss.state_age_seconds:.1f}s)"

            # Motion
            if ss.is_moving:
                vel = ""
                if ss.velocity:
                    vel = f" vx={ss.velocity.vx:.2f} vy={ss.velocity.vy:.2f}"
                summary["motion"] = f"MOVING{vel}"
            else:
                summary["motion"] = "STATIONARY"

            # Proximity (ultrasonic)
            if ss.ultrasonic:
                obstacles: list[str] = []
                u = ss.ultrasonic
                if u.front_m is not None and u.front_m < obstacle_m:
                    obstacles.append(f"front={u.front_m:.2f}m")
                    policies.append(
                        f"- Obstacle CLOSE in FRONT (<{obstacle_m}m): "
                        "Do NOT nudge forward. Consider `retreat` or `scan` to find clear path."
                    )
                if u.rear_m is not None and u.rear_m < obstacle_m:
                    obstacles.append(f"rear={u.rear_m:.2f}m")
                    policies.append(
                        f"- Obstacle CLOSE in REAR (<{obstacle_m}m): "
                        "Do NOT retreat. Consider `nudge` forward or `scan`."
                    )
                if u.left_m is not None and u.left_m < obstacle_m:
                    obstacles.append(f"left={u.left_m:.2f}m")
                if u.right_m is not None and u.right_m < obstacle_m:
                    obstacles.append(f"right={u.right_m:.2f}m")
                if obstacles:
                    summary["proximity"] = (
                        f"OBSTACLE CLOSE: {', '.join(obstacles)} — avoid motion toward obstacle"
                    )
                else:
                    summary["proximity"] = "CLEAR"

        # --- Alerts ---
        critical = [a for a in world.alerts if a.startswith("critical:")]
        if critical:
            summary["alerts"] = f"CRITICAL ALERTS: {'; '.join(critical)}"

        # --- VLM passability hint (soft suggestion; ultrasonic is the hard gate) ---
        hint = world.passability_hint
        if hint is not None:
            summary["passability"] = (
                f"{hint.recommended_direction} (conf={hint.confidence:.2f})"
                + (f": {hint.reason}" if hint.reason else "")
            )
            policies.append(
                "- VLM passability hint is a SOFT suggestion only; ultrasonic "
                "proximity is the hard safety gate and may override it."
            )

        # --- Default policy ---
        if not policies:
            policies.append(
                "- All systems nominal. Proceed with the objective using available tools."
            )

        return StateInterpretation(summary=summary, active_policies=policies)
