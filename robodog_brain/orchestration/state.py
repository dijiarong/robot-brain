"""State payload passed between graph nodes."""
from __future__ import annotations

from typing import Any, TypedDict

from robodog_brain.llm.base import ToolCall
from robodog_brain.safety.validator import ValidationResult
from robodog_brain.skills.base import SkillResult


class GraphState(TypedDict, total=False):
    command: str
    thread_id: str
    queue: list[ToolCall]
    current_call: ToolCall | None
    validation: ValidationResult | None
    last_result: SkillResult | None
    results: list[SkillResult]
    status: str
    error: str
    decision_source: str
    confirmation_granted: bool
    iterations: int
    plan_cycles: int
    observation: dict[str, Any]
