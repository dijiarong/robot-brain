"""Runtime dependency injection container."""
from __future__ import annotations

from dataclasses import dataclass

from config.settings import Settings
from robodog_brain.actuation.base import RobotInterface
from robodog_brain.core.world_state import WorldState
from robodog_brain.llm.base import LLMClient
from robodog_brain.memory.long_term import LongTermMemory
from robodog_brain.memory.short_term import ShortTermMemory
from robodog_brain.perception.base import PerceptionAdapter
from robodog_brain.safety.estop import EmergencyStop
from robodog_brain.safety.validator import SafetyValidator
from robodog_brain.skills.registry import SkillRegistry


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
