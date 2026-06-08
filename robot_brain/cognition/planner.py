"""Slow-system planner backed by an LLM adapter."""
from __future__ import annotations

from robot_brain.core.world_state import WorldState
from robot_brain.llm.base import LLMClient, ToolCall
from robot_brain.llm.output_validator import LLMOutputValidator
from robot_brain.memory.long_term import LongTermMemory
from robot_brain.memory.short_term import ShortTermMemory
from robot_brain.skills.registry import SkillRegistry


class Planner:
    def __init__(
        self,
        llm: LLMClient,
        skills: SkillRegistry,
        short_term: ShortTermMemory,
        long_term: LongTermMemory,
    ) -> None:
        self.llm = llm
        self.skills = skills
        self.short_term = short_term
        self.long_term = long_term
        self._validator = LLMOutputValidator(skills)

    async def plan(self, command: str, world: WorldState) -> list[ToolCall]:
        experiences = self.long_term.search(command)
        memories = self.short_term.recent() + [item.summary for item in experiences]
        calls = await self.llm.plan(command, world, self.skills.tools(), memories)
        calls, errors = self._validator.validate_tool_calls(calls)
        for error in errors:
            self.short_term.add(f"LLM output rejected: [{error.code}] {error.message}")
        self.short_term.add(f"slow plan for {command!r}: {[call.skill_name for call in calls]}")
        return calls
