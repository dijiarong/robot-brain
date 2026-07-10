"""Tool contract: atomic runtime capabilities.

The capability foundation splits robot abilities into four layers:

| Layer   | Responsibility                                  | Exposed to LLM |
|---------|-------------------------------------------------|----------------|
| ``Tool``    | Atomic machine capability (stop, drive segment) | Default no     |
| ``Skill``   | Behavioral orchestration over one or more tools | Optional       |
| ``Policy``  | Independent safety bounds keyed off metadata    | No             |
| ``Catalog`` | Planner-visible view, filters/hides/describes   | Yes            |

Principle: *tool is machine capability, skill is behavioral semantics,
policy is the safety boundary, catalog is the planner view.*

A ``Tool`` is a runtime-internal atomic capability. It is **not** the same
thing as an OpenAI function tool: low-level tools default to
``planner_visible=False`` and only reach the LLM through a ``PlannerCatalog``
(if at all). Skills remain the primary planner-facing unit for now.
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import StrEnum
from typing import Any

from pydantic import BaseModel, Field

from config.settings import Settings
from robot_brain.actuation.base import RobotInterface
from robot_brain.core.world_state import WorldState
from robot_brain.memory.long_term import LongTermMemory
from robot_brain.memory.short_term import ShortTermMemory
from robot_brain.perception.base import PerceptionAdapter


class EmptyParams(BaseModel):
    pass


class RiskLevel(StrEnum):
    """Safety grading for a capability."""

    READ_ONLY = "read_only"
    LOW = "low"
    MEDIUM = "medium"
    HIGH = "high"
    CRITICAL = "critical"


class MotionKind(StrEnum):
    """Motion constraint declared by a capability.

    ``Policy`` keys off this instead of skill names so safety rules survive
    refactors and apply uniformly to tools and skills alike.
    """

    NONE = "none"
    STOP = "stop"
    LINEAR = "linear"
    YAW = "yaw"
    POSTURE = "posture"


class CapabilityMetadata(BaseModel):
    """Capability risk and backend declaration.

    This replaces part of the hardcoded knowledge that used to live in
    ``SafetyValidator`` (per-skill-name estop/battery/confirmation rules) and
    ``SkillRegistry`` (static backend whitelists).
    """

    risk_level: RiskLevel = RiskLevel.LOW
    motion_kind: MotionKind = MotionKind.NONE
    requires_confirmation: bool = False
    #: ``None`` means the capability is available on every backend.
    backend_allowlist: tuple[str, ...] | None = None
    planner_visible: bool = False
    tags: frozenset[str] = Field(default_factory=frozenset)

    model_config = {"frozen": True}


class ToolResult(BaseModel):
    success: bool
    message: str
    data: dict[str, Any] = Field(default_factory=dict)


@dataclass
class ToolContext:
    """Execution context shared by all tools.

    Avoids growing every tool's ``execute`` signature. Initially carries only
    ``settings``, ``world`` and ``robot``; ``perception`` / ``short_term`` /
    ``long_term`` are reserved for later expansion (perception tools, audit).
    """

    settings: Settings | None
    world: WorldState
    robot: RobotInterface
    perception: PerceptionAdapter | None = None
    short_term: ShortTermMemory | None = None
    long_term: LongTermMemory | None = None


class Tool(ABC):
    """Atomic runtime capability.

    Subclasses set ``name``, ``description``, ``params_model`` and
    ``metadata`` and implement :meth:`execute`.
    """

    name: str
    description: str
    params_model: type[BaseModel] = EmptyParams
    metadata: CapabilityMetadata = CapabilityMetadata()

    def params_schema(self) -> dict[str, Any]:
        return self.params_model.model_json_schema()

    def parse_params(self, params: dict[str, Any]) -> BaseModel:
        return self.params_model.model_validate(params)

    @abstractmethod
    async def execute(self, params: BaseModel, context: ToolContext) -> ToolResult: ...
