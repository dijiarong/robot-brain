"""High-level runtime entry point for commands, interrupts, and resumes."""
from __future__ import annotations

import time
from typing import Any
from uuid import uuid4

from pydantic import BaseModel, Field

from config.settings import SETTINGS, Settings
from robot_brain.actuation.base import RobotInterface
from robot_brain.actuation.mock import MockRobot
from robot_brain.core.context import AgentContext
from robot_brain.core.errors import ErrorCode
from robot_brain.core.events import Event, EventType
from robot_brain.core.world_state import WorldState
from robot_brain.llm.base import LLMClient
from robot_brain.llm.mock import MockLLM
from robot_brain.memory.conversation import ConversationMemory
from robot_brain.memory.execution_summary import ExecutionSummary, ExecutionSummaryStore, InMemoryExecutionSummaryStore
from robot_brain.memory.long_term import Experience, LongTermMemory
from robot_brain.memory.semantic_store import SemanticExperienceStore, SQLiteSemanticStore
from robot_brain.memory.short_term import ShortTermMemory
from robot_brain.memory.sqlite_store import SQLiteMemoryStore
from robot_brain.memory.task_queue import TaskQueue
from robot_brain.memory.world_state import WorldStateMemory
from robot_brain.orchestration.graph import BrainGraph, build_graph
from robot_brain.orchestration.state import GraphState
from robot_brain.perception.base import PerceptionAdapter
from robot_brain.perception.mock import MockPerception
from robot_brain.runtime.checkpoint import (
    CheckpointRepository,
    CheckpointStore,
    PendingCheckpoint,
    SQLiteCheckpointStore,
)
from robot_brain.safety.estop import EmergencyStop
from robot_brain.safety.validator import SafetyValidator
from robot_brain.skills.base import SkillResult
from robot_brain.skills.builtin import default_skills
from robot_brain.skills.registry import SkillRegistry
from robot_brain.tools.builtin import default_tools
from robot_brain.tools.registry import ToolRegistry


class RunResult(BaseModel):
    status: str
    message: str = ""
    error_code: ErrorCode | None = None
    thread_id: str | None = None
    decision_source: str = ""
    results: list[SkillResult] = Field(default_factory=list)
    world: dict[str, Any] = Field(default_factory=dict)


class AgentRuntime:
    def __init__(
        self,
        context: AgentContext,
        checkpoints: CheckpointRepository | None = None,
        conversations: ConversationMemory | None = None,
        database: SQLiteMemoryStore | None = None,
        tasks: TaskQueue | None = None,
        summaries: ExecutionSummaryStore | None = None,
    ) -> None:
        self.context = context
        self.checkpoints = checkpoints or CheckpointStore()
        self.conversations = conversations or ConversationMemory()
        self.tasks = tasks or TaskQueue()
        self.summaries = summaries or InMemoryExecutionSummaryStore()
        self._database = database
        self._restored_threads: set[str] = set()
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
        conversations: ConversationMemory | None = None,
        checkpoints: CheckpointRepository | None = None,
        world_states: WorldStateMemory | None = None,
        tasks: TaskQueue | None = None,
        summaries: ExecutionSummaryStore | None = None,
    ) -> "AgentRuntime":
        settings = settings or SETTINGS
        database: SQLiteMemoryStore | None = None

        def sqlite_database() -> SQLiteMemoryStore:
            nonlocal database
            if database is None:
                database = SQLiteMemoryStore(settings.memory_db_path)
            return database

        if robot is None:
            if settings.robot_backend == "mock":
                robot = MockRobot()
            elif settings.robot_backend == "unitree":
                from robot_brain.actuation.unitree import FakeUnitreeTransport, UnitreeRobot

                if settings.unitree_transport == "sdk":
                    from robot_brain.actuation.unitree_sdk import create_sdk_transport

                    transport = create_sdk_transport(settings)
                elif settings.unitree_transport == "webrtc":
                    from robot_brain.actuation.unitree_webrtc import create_webrtc_transport

                    transport = create_webrtc_transport(settings)
                else:
                    transport = FakeUnitreeTransport()
                robot = UnitreeRobot(transport, settings)
            else:
                raise ValueError(f"unsupported robot backend: {settings.robot_backend}")
        if perception is None:
            if settings.perception_backend == "mock":
                perception = MockPerception(robot)
            elif settings.perception_backend == "unitree":
                from robot_brain.actuation.unitree import UnitreeRobot
                from robot_brain.perception.unitree import UnitreePerceptionAdapter

                if not isinstance(robot, UnitreeRobot):
                    raise ValueError(
                        "unitree perception backend requires a UnitreeRobot; "
                        f"got {type(robot).__name__}"
                    )
                perception = UnitreePerceptionAdapter(robot)
            else:
                raise ValueError(f"unsupported perception backend: {settings.perception_backend}")
        if settings.robot_backend == "unitree":
            from robot_brain.skills.builtin.go2_catalog import go2_skills
            from robot_brain.tools.builtin import go2_tools

            tool_registry = ToolRegistry(default_tools() + go2_tools())
            stop_tool = tool_registry.get("stop_motion")
            drive_tool = tool_registry.get("go2_drive_segment")
            skills = SkillRegistry(
                default_skills(stop_tool=stop_tool)
                + go2_skills(settings, perception=perception, drive_tool=drive_tool)
            )
        else:
            from robot_brain.skills.builtin.explore import ExploreSkill

            tool_registry = ToolRegistry(default_tools())
            stop_tool = tool_registry.get("stop_motion")
            skills = SkillRegistry(
                default_skills(stop_tool=stop_tool)
                + [ExploreSkill(settings, perception=perception)]
            )
        if llm is None:
            if settings.llm_backend == "mock":
                llm = MockLLM()
            elif settings.llm_backend == "openai":
                from robot_brain.llm.openai_client import OpenAIClient
                from robot_brain.llm.prompt_builder import PromptBuilder

                llm = OpenAIClient(
                    settings.openai_model,
                    skills=skills,
                    backend=settings.robot_backend,
                    prompt_builder=PromptBuilder(settings=settings),
                    settings=settings,
                )
            elif settings.llm_backend == "compatible":
                from robot_brain.llm.compatible_client import CompatibleLLMClient
                from robot_brain.llm.prompt_builder import PromptBuilder

                llm = CompatibleLLMClient(
                    settings.openai_model,
                    skills=skills,
                    backend=settings.robot_backend,
                    prompt_builder=PromptBuilder(settings=settings),
                    settings=settings,
                )
            else:
                raise ValueError(f"unsupported LLM backend: {settings.llm_backend}")
        elif hasattr(llm, "set_skills"):
            llm.set_skills(skills)
        if long_term is None:
            long_term = LongTermMemory(SQLiteSemanticStore(sqlite_database()))
        if conversations is None:
            conversations = ConversationMemory(sqlite_database())
        if checkpoints is None:
            checkpoints = SQLiteCheckpointStore(sqlite_database())
        if world_states is None:
            world_states = WorldStateMemory(sqlite_database())
        if tasks is None:
            tasks = TaskQueue(sqlite_database())
        if summaries is None:
            summaries = sqlite_database()
        if world is None:
            snapshot = world_states.latest()
            world = snapshot.state.model_copy(deep=True) if snapshot is not None else WorldState()
        context = AgentContext(
            settings=settings,
            world=world,
            robot=robot,
            perception=perception,
            llm=llm,
            skills=skills,
            validator=SafetyValidator(settings, skills),
            estop=EmergencyStop(),
            short_term=ShortTermMemory(),
            long_term=long_term,
            world_states=world_states,
            conversations=conversations,
            tools=tool_registry,
        )
        return cls(
            context,
            checkpoints=checkpoints,
            conversations=conversations,
            database=database,
            tasks=tasks,
            summaries=summaries,
        )

    async def run_command(self, command: str, *, thread_id: str | None = None) -> RunResult:
        thread_id = thread_id or str(uuid4())
        start_time = time.monotonic()
        self._restore_thread_context(thread_id)
        self.conversations.add(thread_id=thread_id, role="user", content=command, message_type="command")
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
        duration = time.monotonic() - start_time
        self._remember(command, result, duration=duration)
        self._save_execution_summary(command, result, duration)
        self._save_decision_context(command, result, final)
        return result

    async def resume(self, thread_id: str, *, approved: bool) -> RunResult:
        self._restore_thread_context(thread_id)
        self.conversations.add(
            thread_id=thread_id,
            role="user",
            content="operator approved pending action" if approved else "operator rejected pending action",
            message_type="confirmation",
            metadata={"approved": approved},
        )
        checkpoint = self.checkpoints.pop(thread_id)
        if checkpoint is None:
            result = self._simple_result(
                "missing_checkpoint", "no pending checkpoint", thread_id,
                error_code=ErrorCode.RUNTIME_MISSING_CHECKPOINT,
            )
            self._record_result(result)
            return result
        if not approved:
            task = self.context.world.current_task
            if task is not None:
                task.status = "failed"
                task.last_message = "operator rejected pending action"
            self._save_world("resume:rejected", thread_id)
            result = self._simple_result("rejected", "operator rejected pending action", thread_id)
            self._remember(checkpoint.command, result)
            return result

        observation = await self.context.perception.observe()
        self.context.world.apply_observation(
            observation,
            object_ttl_seconds=self.context.settings.object_ttl_seconds,
        )
        self.context.short_term.add(f"resume observation: {observation.model_dump(mode='json')}")
        self._save_world("resume:perceive", thread_id)
        validation = self.context.validator.validate(
            checkpoint.tool_call,
            self.context.world,
            confirmation_granted=True,
        )
        if not validation.allowed:
            result = self._simple_result(
                "blocked", validation.reason, thread_id,
                error_code=validation.error_code,
            )
            self._remember(checkpoint.command, result)
            return result
        skill = self.context.skills.get(checkpoint.tool_call.skill_name)
        if skill is None:
            result = self._simple_result(
                "blocked", "skill disappeared from registry", thread_id,
                error_code=ErrorCode.RUNTIME_SKILL_NOT_FOUND,
            )
            self._remember(checkpoint.command, result)
            return result
        params = skill.parse_params(validation.normalized_parameters)
        skill_result = await skill.execute(params, self.context.robot, self.context.world)
        self._save_world(f"resume:execute:{checkpoint.tool_call.skill_name}", thread_id)
        observation = await self.context.perception.observe()
        self.context.world.apply_observation(
            observation,
            object_ttl_seconds=self.context.settings.object_ttl_seconds,
        )
        task = self.context.world.current_task
        if task is not None:
            task.completed_skills.append(checkpoint.tool_call.skill_name)
            task.status = "completed" if skill_result.success else "failed"
            task.last_message = skill_result.message
        self._save_world("resume:complete", thread_id)
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
        if event.type == EventType.COMMAND:
            return await self.run_command(event.message, thread_id=event.payload.get("thread_id"))
        thread_id = str(event.payload.get("thread_id") or uuid4())
        self._restore_thread_context(thread_id)
        self.conversations.add(
            thread_id=thread_id,
            role="system",
            content=event.message,
            message_type=f"event:{event.type}",
            metadata=event.payload,
        )
        if event.type == EventType.INTERRUPT:
            await self.context.estop.activate(event.message, self.context.robot, self.context.world)
            self._save_world("event:interrupt", thread_id)
            result = self._simple_result("interrupted", event.message, thread_id)
            self._remember(event.message, result)
            return result
        result = self._simple_result("ignored", f"event type is not actionable: {event.type}", thread_id)
        self._record_result(result)
        return result

    async def refresh_world(self, *, reason: str = "runtime:refresh", thread_id: str | None = None) -> None:
        observation = await self.context.perception.observe()
        self.context.world.apply_observation(
            observation,
            object_ttl_seconds=self.context.settings.object_ttl_seconds,
        )
        self.context.short_term.add(f"{reason} observation: {observation.model_dump(mode='json')}")
        self._save_world(reason, thread_id)

    def reset_estop(self) -> None:
        self.context.estop.reset(self.context.world)
        self._save_world("estop:reset")

    def close(self) -> None:
        if self._database is not None:
            self._database.close()

    def _to_result(self, state: GraphState) -> RunResult:
        results = state.get("results", [])
        message = results[-1].message if results else state.get("error", state.get("status", ""))
        return RunResult(
            status=state.get("status", "completed"),
            message=message,
            error_code=state.get("error_code"),
            thread_id=state.get("thread_id"),
            decision_source=state.get("decision_source", ""),
            results=results,
            world=self.context.world.snapshot(),
        )

    def _simple_result(
        self, status: str, message: str, thread_id: str | None = None, *, error_code: ErrorCode | None = None,
    ) -> RunResult:
        return RunResult(
            status=status, message=message, error_code=error_code,
            thread_id=thread_id, world=self.context.world.snapshot(),
        )

    def _remember(self, command: str, result: RunResult, *, duration: float = 0.0) -> None:
        # Build a richer summary that includes executed skills and duration
        task = self.context.world.current_task
        skills_used = list(task.completed_skills) if task is not None else []
        if skills_used or duration > 0:
            skills_str = ", ".join(skills_used) or "none"
            summary = f"{result.message} [skills: {skills_str}, {duration:.1f}s]"
        else:
            summary = result.message
        self.context.short_term.add(f"runtime result for {command!r}: {result.status} ({result.message})")
        self.context.long_term.add(Experience(objective=command, outcome=result.status, summary=summary))
        self._record_result(result)

    def _record_result(self, result: RunResult) -> None:
        if result.thread_id is None:
            return
        self.conversations.add(
            thread_id=result.thread_id,
            role="assistant",
            content=result.message,
            message_type="runtime_result",
            metadata=result.model_dump(mode="json"),
        )

    def _restore_thread_context(self, thread_id: str) -> None:
        if thread_id in self._restored_threads:
            return
        for message in self.conversations.recent(thread_id, limit=self.context.short_term.capacity):
            self.context.short_term.add(
                f"conversation {message.role}/{message.message_type}: {message.content}"
            )
        self._restored_threads.add(thread_id)

    def _save_execution_summary(self, command: str, result: RunResult, duration: float) -> None:
        if result.thread_id is None:
            return
        # Gather executed skill names from the world state's current task
        task = self.context.world.current_task
        skills_executed = list(task.completed_skills) if task is not None else []

        # Gather memory refs from long-term search
        experiences = self.context.long_term.search(command, limit=3)
        memory_refs = [exp.summary for exp in experiences]

        summary = ExecutionSummary(
            thread_id=result.thread_id,
            task_id=None,
            objective=command,
            outcome=result.status,
            skills_executed=skills_executed,
            duration_seconds=duration,
            failure_reason=result.message if result.status in ("failed", "blocked") else None,
            memory_refs=memory_refs,
            decision_source=result.decision_source,
        )
        self.summaries.save_summary(summary)

    def _save_decision_context(self, command: str, result: RunResult, final: GraphState) -> None:
        if result.thread_id is None or self._database is None:
            return
        import json as _json

        experiences = self.context.long_term.search(command, limit=3)
        memory_refs = [exp.summary for exp in experiences]

        # Gather executed skill names from world state task progress
        task = self.context.world.current_task
        executed_skills = list(task.completed_skills) if task is not None else []

        safety_result = "allowed"
        if result.status == "blocked":
            safety_result = f"blocked: {result.message}"
        elif result.status == "awaiting_confirmation":
            safety_result = "awaiting_confirmation"

        # Persist the cognitive snapshot for full audit trail
        world_snapshot = _json.dumps(
            self.context.world.cognitive_snapshot(self.context.settings), ensure_ascii=False
        )

        self._database.save_decision_context(
            thread_id=result.thread_id,
            command=command,
            chosen_skills=executed_skills,
            reason=result.decision_source or "planner",
            memory_refs=memory_refs,
            safety_result=safety_result,
            next_plan="completed" if result.status == "completed" else result.message,
            is_degraded=getattr(self.context.llm, "is_degraded", False),
            world_snapshot=world_snapshot,
        )

    def _save_world(self, reason: str, thread_id: str | None = None) -> None:
        self.context.world_states.save(self.context.world, reason=reason, thread_id=thread_id)
