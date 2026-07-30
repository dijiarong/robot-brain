"""High-level runtime entry point for commands, interrupts, and resumes."""
from __future__ import annotations

import logging
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
from robot_brain.memory.semantic_store import SQLiteSemanticStore
from robot_brain.memory.short_term import ShortTermMemory
from robot_brain.memory.sqlite_store import SQLiteMemoryStore
from robot_brain.memory.task_queue import TaskQueue
from robot_brain.memory.world_state import WorldStateMemory
from robot_brain.navigation import FakeNavigationClient, NavigationClient
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

logger = logging.getLogger(__name__)


def _build_passability(settings: Settings) -> tuple[Any | None, Any | None]:
    """Build a VLM PassabilityAnalyzer when ``RDB_VLM_ENABLED`` is on.

    Returns ``(analyzer, frame_source)`` - both ``None`` when VLM is disabled
    (the default), so explore behaves exactly as in iteration 16. The frame
    source is selected by ``RDB_VLM_FRAME_SOURCE`` (auto/file/go2_tap/none);
    ``RDB_VLM_VIDEO_PRIORITY=relay`` suppresses the Go2 tap so the RTP relay
    keeps the video track. The frame_source is returned separately so a service
    can register the Go2 tap after ``await conn.connect()``.
    """
    if not settings.vlm_enabled:
        return None, None
    from robot_brain.vlm.client import VLMClient
    from robot_brain.vlm.passability import PassabilityAnalyzer

    frame_source = _select_frame_source(settings)
    client = VLMClient(settings)
    return PassabilityAnalyzer(client, frame_source, settings), frame_source


def _select_frame_source(settings: Settings) -> Any:
    """Pick the VLM frame source per ``RDB_VLM_FRAME_SOURCE`` / video priority."""
    from robot_brain.vlm.frame_source import FileFrameSource, NullFrameSource

    choice = settings.vlm_frame_source
    # relay priority: keep the RTP relay as the sole video consumer -> no tap.
    can_go2_tap = settings.vlm_video_priority != "relay"

    if choice == "file":
        return FileFrameSource(settings.vlm_frame_path)
    if choice == "go2_tap":
        if can_go2_tap:
            from robot_brain.vlm.frame_source import Go2VideoFrameSource

            return Go2VideoFrameSource()
        return NullFrameSource()
    if choice == "none":
        return NullFrameSource()
    # auto
    if settings.vlm_frame_path:
        return FileFrameSource(settings.vlm_frame_path)
    if (
        can_go2_tap
        and settings.robot_backend == "unitree"
        and settings.unitree_transport == "webrtc"
    ):
        from robot_brain.vlm.frame_source import Go2VideoFrameSource

        return Go2VideoFrameSource()
    return NullFrameSource()


def _vlm_video_warning(settings: Settings, frame_source_kind: str) -> str:
    """Warn when the VLM tap and RTP relay would compete for the same track."""
    if frame_source_kind != "go2_tap":
        return ""
    if settings.vlm_video_priority != "vlm":
        return ""
    if getattr(settings, "unitree_video_relay", False):
        return (
            "VLM tap and RTP relay compete for the same Go2 video track; "
            "set RDB_VLM_VIDEO_PRIORITY=relay or implement a tee."
        )
    return ""


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
        passability_frame_source: Any | None = None,
        passability: Any | None = None,
    ) -> None:
        self.context = context
        self.checkpoints = checkpoints or CheckpointStore()
        self.conversations = conversations or ConversationMemory()
        self.tasks = tasks or TaskQueue()
        self.summaries = summaries or InMemoryExecutionSummaryStore()
        #: Go2 VLM frame source (None unless unitree+webrtc+VLM); a service
        #: registers the WebRTC tap via attach_passability_tap() after connect.
        self.passability_frame_source = passability_frame_source
        #: VLM PassabilityAnalyzer (None unless VLM enabled). Owns the client
        #: and frame source lifecycle + diagnostics.
        self.passability = passability
        self._database = database
        self._restored_threads: set[str] = set()
        self._closed = False
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
        navigation: NavigationClient | None = None,
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
        passability, passability_frame_source = _build_passability(settings)
        spatial_skill_set = []
        if navigation is None:
            if settings.navigation_backend == "nav2":
                from robot_brain.navigation.nav2 import create_nav2_navigation_client

                navigation = create_nav2_navigation_client(settings)
            elif settings.navigation_backend == "direct_go2":
                from robot_brain.actuation.unitree import UnitreeRobot
                from robot_brain.navigation.direct_go2 import DirectGo2NavigationClient
                from robot_brain.navigation.sensors import UnitreeNavigationSensorProvider

                if not isinstance(robot, UnitreeRobot):
                    raise ValueError("direct_go2 navigation requires a UnitreeRobot")
                transport = robot.transport
                if not all(
                    hasattr(transport, name)
                    for name in ("read_lidar_snapshot", "lidar_age_seconds")
                ):
                    raise ValueError(
                        "direct_go2 navigation requires a transport with built-in LiDAR"
                    )
                sensors = UnitreeNavigationSensorProvider(
                    transport,
                    max_pose_age_s=settings.odom_max_age_seconds,
                    max_pointcloud_age_s=settings.direct_nav_pointcloud_max_age_s,
                    require_authoritative_odom=settings.direct_nav_require_robotodom,
                )
                navigation = DirectGo2NavigationClient(
                    robot,
                    sensors,
                    linear_speed_mps=settings.unitree_max_speed,
                    yaw_speed_rps=settings.unitree_max_yaw_speed,
                    segment_duration_s=settings.direct_nav_segment_duration_s,
                    obstacle_stop_m=settings.direct_nav_obstacle_stop_m,
                    obstacle_half_width_m=settings.direct_nav_obstacle_half_width_m,
                    min_progress_m=settings.odom_progress_min_m,
                    min_progress_yaw_deg=settings.odom_progress_min_yaw_deg,
                    max_no_progress_segments=settings.direct_nav_no_progress_segments,
                    odom_settle_s=settings.direct_nav_odom_settle_s,
                    reach_tolerance_m=settings.direct_nav_reach_tolerance_m,
                    reach_tolerance_yaw_deg=settings.direct_nav_reach_tolerance_yaw_deg,
                )
            elif settings.robot_backend == "mock" and settings.navigation_backend in {"auto", "fake"}:
                navigation = FakeNavigationClient()

        if (
            navigation is not None
            and passability is not None
            and passability_frame_source is not None
        ):
            from robot_brain.memory.spatial import SpatialMemoryStore
            from robot_brain.skills.builtin.spatial_memory import (
                FindObjectSkill,
                RememberRoomSkill,
            )
            from robot_brain.vlm.object_recognition import ObjectRecognizer

            spatial_store = SpatialMemoryStore(settings.memory_db_path)
            recognizer = ObjectRecognizer(passability._client)
            spatial_skill_set = [
                RememberRoomSkill(
                    spatial_store, passability_frame_source, recognizer, navigation
                ),
                FindObjectSkill(
                    spatial_store, passability_frame_source, recognizer, navigation
                ),
            ]

        navigation_tools = []
        navigation_skill_set = []
        if navigation is not None:
            from robot_brain.skills.builtin.navigation import navigation_skills
            from robot_brain.tools.builtin.navigation import (
                LocalizationGetStateTool,
                NavigationGetStateTool,
            )

            navigation_tools = [
                NavigationGetStateTool(navigation),
                LocalizationGetStateTool(navigation),
            ]
            navigation_skill_set = navigation_skills(navigation)
        if settings.robot_backend == "unitree":
            from robot_brain.skills.builtin.go2_catalog import go2_skills
            from robot_brain.tools.builtin import go2_tools

            tool_registry = ToolRegistry(default_tools() + go2_tools() + navigation_tools)
            stop_tool = tool_registry.get("stop_motion")
            drive_tool = tool_registry.get("go2_drive_segment")
            skills = SkillRegistry(
                default_skills(stop_tool=stop_tool)
                + go2_skills(
                    settings,
                    perception=perception,
                    drive_tool=drive_tool,
                    passability=passability,
                )
                + navigation_skill_set
                + spatial_skill_set
            )
        else:
            from robot_brain.skills.builtin.explore import ExploreSkill

            tool_registry = ToolRegistry(default_tools() + navigation_tools)
            stop_tool = tool_registry.get("stop_motion")
            skills = SkillRegistry(
                default_skills(stop_tool=stop_tool)
                + [ExploreSkill(settings, perception=perception, passability=passability)]
                + navigation_skill_set
                + spatial_skill_set
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
            navigation=navigation,
        )
        return cls(
            context,
            checkpoints=checkpoints,
            conversations=conversations,
            database=database,
            tasks=tasks,
            summaries=summaries,
            passability_frame_source=passability_frame_source,
            passability=passability,
        )

    def attach_passability_tap(self, conn: Any) -> bool:
        """Register the Go2 WebRTC VLM frame tap on *conn* (call after connect).

        Returns True if a tap was registered, False if VLM is off or the frame
        source is not a Go2 video tap (e.g. file/null source).
        """
        fs = self.passability_frame_source
        if fs is None:
            return False
        from robot_brain.vlm.frame_source import Go2VideoFrameSource

        if not isinstance(fs, Go2VideoFrameSource):
            return False
        from robot_brain.vlm.go2_video_tap import prime_go2_video_for_passability

        prime_go2_video_for_passability(conn, fs)
        return True

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
        """Synchronous best-effort cleanup.

        Closes the DB and stops the frame source synchronously. The VLM client
        (async httpx) is only closable from an event loop, so when there is no
        running loop this delegates to :meth:`aclose` via ``asyncio.run``;
        inside a running loop callers should use ``await aclose()`` instead.
        Idempotent.
        """
        if self._closed:
            return
        self._closed = True
        # Sync parts: frame source + DB.
        try:
            if self.passability_frame_source is not None:
                self.passability_frame_source.stop()
        except Exception as exc:  # noqa: BLE001
            logger.warning("frame source stop failed: %s", exc)
        if self._database is not None:
            self._database.close()
        navigation_close = getattr(self.context.navigation, "close", None)
        if callable(navigation_close):
            try:
                navigation_close()
            except Exception as exc:  # noqa: BLE001
                logger.warning("navigation close failed: %s", exc)
        # Async part (VLM client): only if we can run a loop.
        if self.passability is not None:
            import asyncio

            try:
                asyncio.get_running_loop()
                running = True
            except RuntimeError:
                running = False
            if not running:
                try:
                    asyncio.run(self.passability.aclose())
                except Exception as exc:  # noqa: BLE001
                    logger.warning("passability aclose failed: %s", exc)
            else:
                logger.warning(
                    "AgentRuntime.close() called inside a running loop; "
                    "VLM client not closed - use await aclose() instead"
                )

    async def aclose(self) -> None:
        """Full async cleanup: VLM client + frame source + DB. Idempotent."""
        if self._closed:
            return
        self._closed = True
        if self.passability is not None:
            try:
                await self.passability.aclose()
            except Exception as exc:  # noqa: BLE001
                logger.warning("passability aclose failed: %s", exc)
        elif self.passability_frame_source is not None:
            try:
                self.passability_frame_source.stop()
            except Exception as exc:  # noqa: BLE001
                logger.warning("frame source stop failed: %s", exc)
        navigation_aclose = getattr(self.context.navigation, "aclose", None)
        if callable(navigation_aclose):
            try:
                await navigation_aclose()
            except Exception as exc:  # noqa: BLE001
                logger.warning("navigation aclose failed: %s", exc)
        else:
            navigation_close = getattr(self.context.navigation, "close", None)
            if callable(navigation_close):
                try:
                    navigation_close()
                except Exception as exc:  # noqa: BLE001
                    logger.warning("navigation close failed: %s", exc)
        if self._database is not None:
            self._database.close()

    def diagnostics(self) -> dict[str, Any]:
        """VLM + explore diagnostics for the service status API."""
        settings = self.context.settings
        vlm: dict[str, Any]
        if self.passability is not None:
            vlm = self.passability.diagnostics()
            vlm["video_priority"] = settings.vlm_video_priority
            vlm["video_warning"] = _vlm_video_warning(settings, str(vlm.get("frame_source", "")))
        else:
            vlm = {
                "enabled": False,
                "video_priority": settings.vlm_video_priority,
                "video_warning": "",
            }
        explore_skill = self.context.skills.get("explore")
        explore = explore_skill.diagnostics() if explore_skill is not None else {}
        navigation = self.context.navigation
        nav = {
            "backend": settings.navigation_backend,
            "configured": navigation is not None,
            "provider": type(navigation).__name__ if navigation is not None else None,
        }
        return {"vlm": vlm, "explore": explore, "navigation": nav}

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
