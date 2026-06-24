"""Node implementations for the cognition graph."""
from __future__ import annotations

from robot_brain.cognition.dual_system import DualSystem
from robot_brain.core.context import AgentContext
from robot_brain.core.errors import ErrorCode
from robot_brain.core.world_state import TaskProgress
from robot_brain.orchestration.state import GraphState


class OrchestrationNodes:
    def __init__(self, context: AgentContext, dual_system: DualSystem) -> None:
        self.context = context
        self.dual_system = dual_system

    async def perceive(self, state: GraphState) -> GraphState:
        observation = await self.context.perception.observe()
        self.context.world.apply_observation(
            observation,
            object_ttl_seconds=self.context.settings.object_ttl_seconds,
        )
        self.context.short_term.add(f"observation: {observation.model_dump(mode='json')}")
        self._save_world(state, "perceive")
        return {"observation": observation.model_dump(mode="json"), "status": "perceived"}

    async def decide(self, state: GraphState) -> GraphState:
        command = state["command"]
        conversation = self._get_conversation_context(state.get("thread_id", ""))
        decision = await self.dual_system.decide(command, self.context.world, conversation=conversation)
        task = self.context.world.current_task
        if task is None or task.objective != command:
            self.context.world.current_task = TaskProgress(objective=command, status="running")
        else:
            task.status = "running"
        self.context.short_term.add(f"{decision.source} decision: {[call.skill_name for call in decision.tool_calls]}")
        self._save_world(state, "decide")
        return {
            "queue": decision.tool_calls,
            "decision_source": decision.source,
            "plan_cycles": state.get("plan_cycles", 0) + 1,
            "status": "decided",
        }

    def _get_conversation_context(self, thread_id: str) -> list[dict[str, str]] | None:
        """Extract recent dialogue turns for LLM context."""
        if not thread_id:
            return None
        messages = self.context.conversations.recent(thread_id, limit=10)
        turns = [
            {"role": msg.role, "content": msg.content}
            for msg in messages
            if msg.role in ("user", "assistant")
        ]
        return turns or None

    async def select_action(self, state: GraphState) -> GraphState:
        iterations = state.get("iterations", 0) + 1
        if iterations > self.context.settings.max_loop_iterations:
            return {
                "current_call": None,
                "error": "max_loop_iterations exceeded",
                "error_code": ErrorCode.RUNTIME_MAX_ITERATIONS,
                "status": "failed",
            }
        queue = list(state.get("queue", []))
        current_call = queue.pop(0) if queue else None
        status = state.get("status", "")
        if current_call is None and status not in {"blocked", "failed", "awaiting_confirmation"}:
            status = "completed"
        return {"current_call": current_call, "queue": queue, "iterations": iterations, "status": status}

    async def validate(self, state: GraphState) -> GraphState:
        call = state.get("current_call")
        if call is None:
            return {"status": "completed"}
        validation = self.context.validator.validate(
            call,
            self.context.world,
            confirmation_granted=state.get("confirmation_granted", False),
        )
        if validation.requires_confirmation:
            status = "awaiting_confirmation"
        elif validation.allowed:
            status = "validated"
        else:
            status = "blocked"
        self.context.short_term.add(f"validation for {call.skill_name}: {status} ({validation.reason})")
        return {
            "validation": validation,
            "status": status,
            "error": validation.reason,
            "error_code": validation.error_code,
        }

    async def execute(self, state: GraphState) -> GraphState:
        call = state["current_call"]
        validation = state["validation"]
        if call is None or validation is None or not validation.allowed:
            return {"status": "blocked"}
        skill = self.context.skills.get(call.skill_name)
        if skill is None:
            return {
                "status": "blocked",
                "error": f"unknown skill: {call.skill_name}",
                "error_code": ErrorCode.RUNTIME_SKILL_NOT_FOUND,
            }
        params = skill.parse_params(validation.normalized_parameters)
        result = await skill.execute(params, self.context.robot, self.context.world)
        self._save_world(state, f"execute:{call.skill_name}")
        return {"last_result": result, "status": "executed"}

    async def observe(self, state: GraphState) -> GraphState:
        return await self.perceive(state)

    async def reflect(self, state: GraphState) -> GraphState:
        call = state.get("current_call")
        result = state.get("last_result")
        results = list(state.get("results", []))
        if result is None:
            return {"status": "failed", "error": "execution produced no result", "error_code": ErrorCode.RUNTIME_NO_RESULT}
        results.append(result)
        self.context.short_term.add(f"result for {call.skill_name if call else 'unknown'}: {result.message}")
        skill = self.context.skills.get(call.skill_name) if call is not None else None
        needs_replan = not result.success or (skill is not None and not skill.is_done(self.context.world))
        task = self.context.world.current_task
        if task is not None:
            if call is not None and not needs_replan:
                task.completed_skills.append(call.skill_name)
            task.last_message = result.message
            task.status = "running" if needs_replan or state.get("queue") else "completed"
        if needs_replan:
            self.context.short_term.add("reflection requested replanning")
            self._save_world(state, "reflect:replan")
            return {"results": results, "queue": [], "status": "replan"}
        self._save_world(state, "reflect")
        return {"results": results, "status": "ready" if state.get("queue") else "completed"}

    async def finish(self, state: GraphState) -> GraphState:
        status = state.get("status", "completed")
        task = self.context.world.current_task
        if task is not None and status in {"blocked", "failed", "awaiting_confirmation"}:
            task.status = "paused" if status == "awaiting_confirmation" else "failed"
            task.last_message = state.get("error", "")
        self._save_world(state, f"finish:{status}")
        return {"status": status}

    def _save_world(self, state: GraphState, reason: str) -> None:
        self.context.world_states.save(self.context.world, reason=reason, thread_id=state.get("thread_id"))
