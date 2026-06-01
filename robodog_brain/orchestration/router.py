"""Conditional-edge routing for the decision graph."""
from __future__ import annotations

from robodog_brain.orchestration.state import GraphState


def after_select(state: GraphState) -> str:
    return "validate" if state.get("current_call") is not None else "finish"


def after_validate(state: GraphState) -> str:
    validation = state.get("validation")
    return "execute" if validation is not None and validation.allowed else "finish"


def after_reflect(state: GraphState) -> str:
    if state.get("status") == "replan":
        return "perceive"
    return "select_action" if state.get("queue") else "finish"
