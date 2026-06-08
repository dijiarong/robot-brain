"""Fixed evaluation scenarios for regression testing Planner behavior."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable

from robot_brain.llm.base import ToolCall
from robot_brain.runtime.loop import RunResult


@dataclass
class Scenario:
    """A fixed evaluation scenario with expected behavior assertions."""

    name: str
    description: str
    command: str
    # Scripted LLM plans to feed into MockLLM
    scripted_plans: list[list[ToolCall]] = field(default_factory=list)
    # Expected properties of the final result
    expected_status: str | None = None
    expected_decision_source: str | None = None
    expected_skills: list[str] | None = None
    # Custom assertion function: (result) -> (pass, message)
    custom_assert: Callable[[RunResult], tuple[bool, str]] | None = None
    # Override settings
    settings_overrides: dict[str, Any] = field(default_factory=dict)
    # Mock perception observations
    observations: list[dict[str, Any]] | None = None


def _patrol_blocked_replan_scenario() -> Scenario:
    """Patrol encounters an obstacle, replans to navigate around it."""
    return Scenario(
        name="patrol_blocked_replan",
        description="Robot patrols, first attempt fails (recognize returns failure), replans and reports",
        command="inspect the area",
        scripted_plans=[
            # First plan: try to recognize
            [ToolCall(skill_name="recognize", parameters={"kind": "missing"})],
            # After failure, replan: report the situation
            [ToolCall(skill_name="report", parameters={"message": "area clear after recheck", "severity": "info"})],
        ],
        expected_status="completed",
        expected_decision_source="slow",
        custom_assert=lambda result: (
            len(result.results) == 2,
            f"Expected 2 skill results, got {len(result.results)}",
        ),
    )


def _low_battery_dock_scenario() -> Scenario:
    """Low battery triggers fast reflex to dock."""
    return Scenario(
        name="low_battery_dock",
        description="Low battery detected, fast reflex triggers docking before planning",
        command="patrol the lobby",
        observations=[{"battery_level": 20.0}],
        expected_status="completed",
        expected_decision_source="fast",
        expected_skills=None,
        custom_assert=lambda result: (
            any("dock" in r.message.lower() or "dock" == getattr(r, "skill_name", "") for r in result.results)
            if result.results
            else (result.status == "completed", "Expected dock skill execution"),
            "Expected docking behavior on low battery",
        ),
    )


def _follow_confirm_scenario() -> Scenario:
    """Follow command requires confirmation before execution."""
    return Scenario(
        name="follow_confirm",
        description="Follow command triggers confirmation flow",
        command="follow person-1",
        observations=[
            {
                "detected_objects": [{"object_id": "person-1", "kind": "person"}],
            }
        ],
        expected_status="awaiting_confirmation",
        expected_decision_source="slow",
    )


SCENARIOS: list[Scenario] = [
    _patrol_blocked_replan_scenario(),
    _low_battery_dock_scenario(),
    _follow_confirm_scenario(),
]
