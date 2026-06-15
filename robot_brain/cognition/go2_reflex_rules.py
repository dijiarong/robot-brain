"""Go2-specific reflex rules that read ``WorldState.robot_self_state``.

These rules run **before** the LLM planner and can preempt it with
deterministic ``stop`` / ``report`` tool calls.  All rules are pure
functions — FastReflex owns debounce state.
"""
from __future__ import annotations

from config.settings import Settings
from robot_brain.core.world_state import WorldState
from robot_brain.llm.base import ToolCall


def decide_go2_reflex(
    world: WorldState,
    settings: Settings,
    *,
    skip_error_check: bool = False,
) -> list[ToolCall] | None:
    """Return a list of ToolCalls if a Go2 reflex rule fires, or ``None``.

    Only activates when ``world.robot_self_state`` is populated
    (i.e. ``RDB_PERCEPTION=unitree`` and at least one perceive has run).

    Set *skip_error_check* to ``True`` to suppress the error-code rule
    (used by FastReflex for error debounce).
    """
    ss = world.robot_self_state
    if ss is None:
        return None

    calls: list[ToolCall] = []

    # Priority 2: non-zero error_code → stop + critical report
    if not skip_error_check and ss.error_code is not None and ss.error_code != 0:
        calls.append(ToolCall(skill_name="stop", parameters={"reason": f"error_code={ss.error_code}"}, source="fast"))
        calls.append(ToolCall(skill_name="report", parameters={"message": f"Go2 error_code={ss.error_code}", "severity": "critical"}, source="fast"))
        return calls

    # Priority 3: stale state → stop + warning
    if (
        ss.state_age_seconds is not None
        and ss.state_age_seconds > settings.unitree_state_max_age_seconds
    ):
        calls.append(ToolCall(skill_name="stop", parameters={"reason": f"state stale ({ss.state_age_seconds:.1f}s)"}, source="fast"))
        calls.append(ToolCall(skill_name="report", parameters={"message": f"Go2 state stale: {ss.state_age_seconds:.1f}s", "severity": "warning"}, source="fast"))
        return calls

    # Priority 4: not standing → warning report only (no stop, no stand_up)
    if ss.is_standing is not None and not ss.is_standing:
        calls.append(ToolCall(skill_name="report", parameters={"message": "Go2 is not standing", "severity": "warning"}, source="fast"))
        return calls

    # Priority 5: moving without an active task → stop
    if (
        ss.is_moving
        and (world.current_task is None or world.current_task.status != "running")
    ):
        calls.append(ToolCall(skill_name="stop", parameters={"reason": "moving without active task"}, source="fast"))
        return calls

    # Priority 6: critical battery → stop + critical report (no dock on Go2)
    if world.battery_level <= settings.critical_battery_threshold:
        calls.append(ToolCall(skill_name="stop", parameters={"reason": "critical battery"}, source="fast"))
        calls.append(ToolCall(skill_name="report", parameters={"message": "Go2 critical battery", "severity": "critical"}, source="fast"))
        return calls

    # Priority 7: low battery → warning report (no dock on Go2)
    if world.battery_level <= settings.low_battery_threshold:
        calls.append(ToolCall(skill_name="report", parameters={"message": f"Go2 low battery ({world.battery_level:.0f}%)", "severity": "warning"}, source="fast"))
        return calls

    return None  # No Go2 rule fired — allow slow path
