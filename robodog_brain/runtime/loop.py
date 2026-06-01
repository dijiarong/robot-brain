"""High-level runtime entry point for commands, interrupts, and resumes."""
from __future__ import annotations

from typing import Any
from uuid import uuid4

from pydantic import BaseModel, Field

from config.settings import SETTINGS, Settings
from robodog_brain.actuation.base import RobotInterface
from robodog_brain.actuation.mock import MockRobot
from robodog_brain.core.context import AgentContext
from robodog_brain.core.events import Event, EventType
from robodog_brain.core.world_state import WorldState
from robodog_brain.llm.base import LLMClient
from robodog_brain.llm.mock import MockLLM
from robodog_brain.memory.long_term import Experience, LongTermMemory
from robodog_brain.memory.short_term import ShortTermMemory
from robodog_brain.orchestration.graph import BrainGraph, build_graph
from robodog_brain.orchestration.state import GraphState
from robodog_brain.perception.base import PerceptionAdapter
from robodog_brain.perception.mock import MockPerception
from robodog_brain.runtime.checkpoint import CheckpointStore, PendingCheckpoint
from robodog_brain.safety.estop import EmergencyStop
from robodog_brain.safety.validator import SafetyValidator
from robodog_brain.skills.base import SkillResult
from robodog_brain.skills.builtin import default_skills
from robodog_brain.skills.registry import SkillRegistry


class RunResult(BaseModel):
    status: str
    message: str = ""
    thread_id: str | None = None
    decision_source: str = ""
    results: list[SkillResult] = Field(default_factory=list)
    world: dict[str, Any] = Field(default_factory=dict)


class AgentRuntime:
    def __init__(self, context: AgentContext, checkpoints: CheckpointStore | None = None) -> None:
        self.context = context
        self.checkpoints = checkpoints or CheckpointStore()
        self.graph: BrainGraph = build_graph(context)

    @classmethod
    def create(
        cls,
        *,
        settings: Settings | None = None,
        world: WorldState | None = None,
        robot: RobotInterface | None = None,
        perception: PerceptionAdapter | None = None,
        llm: LLMClient | None = None,
        long_term: LongTermMemory | None = None,
    ) -> "AgentRuntime":
        settings = settings or SETTINGS
        if robot is None:
            if settings.robot_backend != "mock":
                raise ValueError(f"unsupported robot backend: {settings.robot_backend}")
            robot = MockRobot()
        if perception is None:
            if settings.perception_backend != "mock":
                raise ValueError(f"unsupported perception backend: {settings.perception_backend}")
            perception = MockPerception(robot)
        if llm is None:
            if settings.llm_backend == "mock":
                llm = MockLLM()
            elif settings.llm_backend == "openai":
                from robodog_brain.llm.openai_client import OpenAIClient

                llm = OpenAIClient(settings.openai_model)
            else:
                raise ValueError(f"unsupported LLM backend: {settings.llm_backend}")
        skills = SkillRegistry(default_skills())
        context = AgentContext(
            settings=settings,
            world=world or WorldState(),
            robot=robot,
            perception=perception,
            llm=llm,
            skills=skills,
            validator=SafetyValidator(settings, skills),
            estop=EmergencyStop(),
            short_term=ShortTermMemory(),
            long_term=long_term or LongTermMemory(),
        )
        return cls(context)

    async def run_command(self, command: str, *, thread_id: str | None = None) -> RunResult:
        thread_id = thread_id or str(uuid4())
        initial: GraphState = {
            "command": command,
            "thread_id": thread_id,
            "queue": [],
            "results": [],
            "iterations": 0,
            "plan_cycles": 0,
            "confirmation_granted": False,
        }
        final = await self.graph.ainvoke(initial)
        if final.get("status") == "awaiting_confirmation" and final.get("current_call") is not None:
            self.checkpoints.save(
                PendingCheckpoint(thread_id=thread_id, command=command, tool_call=final["current_call"])
            )
        result = self._to_result(final)
        self._remember(command, result)
        return result

    async def resume(self, thread_id: str, *, approved: bool) -> RunResult:
        checkpoint = self.checkpoints.pop(thread_id)
        if checkpoint is None:
            return self._simple_result("missing_checkpoint", "no pending checkpoint", thread_id)
        if not approved:
            task = self.context.world.current_task
            if task is not None:
                task.status = "failed"
                task.last_message = "operator rejected pending action"
            result = self._simple_result("rejected", "operator rejected pending action", thread_id)
            self._remember(checkpoint.command, result)
            return result

        validation = self.context.validator.validate(
            checkpoint.tool_call,
            self.context.world,
            confirmation_granted=True,
        )
        if not validation.allowed:
            return self._simple_result("blocked", validation.reason, thread_id)
        skill = self.context.skills.get(checkpoint.tool_call.skill_name)
        if skill is None:
            return self._simple_result("blocked", "skill disappeared from registry", thread_id)
        params = skill.parse_params(validation.normalized_parameters)
        skill_result = await skill.execute(params, self.context.robot, self.context.world)
        observation = await self.context.perception.observe()
        self.context.world.apply_observation(observation)
        task = self.context.world.current_task
        if task is not None:
            task.completed_skills.append(checkpoint.tool_call.skill_name)
            task.status = "completed" if skill_result.success else "failed"
            task.last_message = skill_result.message
        result = RunResult(
            status="completed" if skill_result.success else "failed",
            message=skill_result.message,
            thread_id=thread_id,
            results=[skill_result],
            world=self.context.world.snapshot(),
        )
        self._remember(checkpoint.command, result)
        return result

    async def handle_event(self, event: Event) -> RunResult:
        if event.type == EventType.INTERRUPT:
            await self.context.estop.activate(event.message, self.context.robot, self.context.world)
            return self._simple_result("interrupted", event.message)
        if event.type == EventType.COMMAND:
            return await self.run_command(event.message)
        return self._simple_result("ignored", f"event type is not actionable: {event.type}")

    def reset_estop(self) -> None:
        self.context.estop.reset(self.context.world)

    def _to_result(self, state: GraphState) -> RunResult:
        results = state.get("results", [])
        message = results[-1].message if results else state.get("error", state.get("status", ""))
        return RunResult(
            status=state.get("status", "completed"),
            message=message,
            thread_id=state.get("thread_id"),
            decision_source=state.get("decision_source", ""),
            results=results,
            world=self.context.world.snapshot(),
        )

    def _simple_result(self, status: str, message: str, thread_id: str | None = None) -> RunResult:
        return RunResult(status=status, message=message, thread_id=thread_id, world=self.context.world.snapshot())

    def _remember(self, command: str, result: RunResult) -> None:
        self.context.short_term.add(f"runtime result for {command!r}: {result.status} ({result.message})")
        self.context.long_term.add(Experience(objective=command, outcome=result.status, summary=result.message))
