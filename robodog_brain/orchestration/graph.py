"""LangGraph assembly with an offline-compatible fallback runner."""
from __future__ import annotations

from typing import Any

from robodog_brain.cognition.dual_system import DualSystem
from robodog_brain.cognition.fast_reflex import FastReflex
from robodog_brain.cognition.planner import Planner
from robodog_brain.core.context import AgentContext
from robodog_brain.orchestration.nodes import OrchestrationNodes
from robodog_brain.orchestration.router import after_reflect, after_select, after_validate
from robodog_brain.orchestration.state import GraphState

try:
    from langgraph.graph import END, START, StateGraph
except ImportError:  # pragma: no cover - exercised when optional runtime dependency is absent.
    END = START = StateGraph = None


class BrainGraph:
    def __init__(self, nodes: OrchestrationNodes, compiled: Any | None) -> None:
        self.nodes = nodes
        self.compiled = compiled

    async def ainvoke(self, initial: GraphState) -> GraphState:
        if self.compiled is not None:
            return await self.compiled.ainvoke(initial)
        return await self._fallback_ainvoke(initial)

    async def _fallback_ainvoke(self, initial: GraphState) -> GraphState:
        state: GraphState = dict(initial)
        state.update(await self.nodes.perceive(state))
        state.update(await self.nodes.decide(state))
        while True:
            state.update(await self.nodes.select_action(state))
            if after_select(state) == "finish":
                state.update(await self.nodes.finish(state))
                return state
            state.update(await self.nodes.validate(state))
            if after_validate(state) == "finish":
                state.update(await self.nodes.finish(state))
                return state
            state.update(await self.nodes.execute(state))
            state.update(await self.nodes.observe(state))
            state.update(await self.nodes.reflect(state))
            reflection_route = after_reflect(state)
            if reflection_route == "perceive":
                state.update(await self.nodes.perceive(state))
                state.update(await self.nodes.decide(state))
            elif reflection_route == "finish":
                state.update(await self.nodes.finish(state))
                return state


def build_graph(context: AgentContext) -> BrainGraph:
    planner = Planner(context.llm, context.skills, context.short_term, context.long_term)
    dual_system = DualSystem(FastReflex(context.settings), planner)
    nodes = OrchestrationNodes(context, dual_system)
    if StateGraph is None:
        return BrainGraph(nodes, compiled=None)

    builder = StateGraph(GraphState)
    builder.add_node("perceive", nodes.perceive)
    builder.add_node("decide", nodes.decide)
    builder.add_node("select_action", nodes.select_action)
    builder.add_node("validate", nodes.validate)
    builder.add_node("execute", nodes.execute)
    builder.add_node("observe", nodes.observe)
    builder.add_node("reflect", nodes.reflect)
    builder.add_node("finish", nodes.finish)
    builder.add_edge(START, "perceive")
    builder.add_edge("perceive", "decide")
    builder.add_edge("decide", "select_action")
    builder.add_conditional_edges("select_action", after_select, {"validate": "validate", "finish": "finish"})
    builder.add_conditional_edges("validate", after_validate, {"execute": "execute", "finish": "finish"})
    builder.add_edge("execute", "observe")
    builder.add_edge("observe", "reflect")
    builder.add_conditional_edges(
        "reflect",
        after_reflect,
        {"perceive": "perceive", "select_action": "select_action", "finish": "finish"},
    )
    builder.add_edge("finish", END)
    return BrainGraph(nodes, compiled=builder.compile())
