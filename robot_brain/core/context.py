"""Runtime dependency injection container."""
from __future__ import annotations

from dataclasses import dataclass, field

from config.settings import Settings
from robot_brain.actuation.base import RobotInterface
from robot_brain.core.world_state import WorldState
from robot_brain.llm.base import LLMClient
from robot_brain.memory.conversation import ConversationMemory
from robot_brain.memory.long_term import LongTermMemory
from robot_brain.memory.short_term import ShortTermMemory
from robot_brain.memory.world_state import WorldStateMemory
from robot_brain.perception.base import PerceptionAdapter
from robot_brain.safety.estop import EmergencyStop
from robot_brain.safety.validator import SafetyValidator
from robot_brain.skills.registry import SkillRegistry
from robot_brain.tools.registry import ToolRegistry


@dataclass
class AgentContext:
    settings: Settings
    world: WorldState
    robot: RobotInterface
    perception: PerceptionAdapter
    llm: LLMClient
    skills: SkillRegistry
    validator: SafetyValidator
    estop: EmergencyStop
    short_term: ShortTermMemory
    long_term: LongTermMemory
    world_states: WorldStateMemory
    conversations: ConversationMemory = field(default_factory=ConversationMemory)
    #: Runtime-internal atomic tools. Skills delegate to these; the
    #: PlannerCatalog may expose a safe subset to the LLM in future iterations.
    tools: ToolRegistry | None = None
