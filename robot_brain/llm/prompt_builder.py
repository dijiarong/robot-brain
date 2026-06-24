"""Composable system prompt builder for the LLM planner.

Assembles a context-aware system instruction from the current world state,
decision policies, tool guidance, memories, and conversation history.
"""
from __future__ import annotations

from typing import TYPE_CHECKING

from robot_brain.core.state_interpreter import StateInterpreter
from robot_brain.core.world_state import WorldState
from robot_brain.llm.prompts.templates import (
    CONVERSATION_TEMPLATE,
    MEMORY_TEMPLATE,
    POLICIES_TEMPLATE,
    ROBOT_STATE_BLOCK_TEMPLATE,
    ROLE_TEMPLATE,
    ROLE_TEMPLATE_GENERIC,
    STATE_TEMPLATE,
    TOOLS_GUIDANCE_TEMPLATE,
)

if TYPE_CHECKING:
    from config.settings import Settings


class PromptBuilder:
    """Builds a structured, state-aware system prompt for the LLM planner."""

    def __init__(
        self,
        *,
        settings: "Settings | None" = None,
        max_conversation_turns: int = 5,
        max_memories: int = 5,
    ) -> None:
        self._settings = settings
        self.max_conversation_turns = max_conversation_turns
        self.max_memories = max_memories

    def build_system_prompt(
        self,
        world: WorldState,
        backend: str = "mock",
        memories: list[str] | None = None,
        conversation: list[dict[str, str]] | None = None,
    ) -> str:
        """Assemble the full system instruction for the LLM."""
        # Build unified interpretation once (P1+P2: single source of truth).
        interpreter = self._get_interpreter()
        interpretation = interpreter.interpret(world)

        sections = [
            self._role_section(backend),
            self._state_section(world, interpretation.summary),
            self._policies_section(interpretation.active_policies),
            self._tools_guidance_section(backend),
        ]
        if memories:
            sections.append(self._memory_section(memories))
        if conversation:
            sections.append(self._conversation_section(conversation))
        return "\n\n".join(sections)

    def _get_interpreter(self) -> StateInterpreter:
        if self._settings is not None:
            return StateInterpreter(self._settings)
        from config.settings import Settings
        return StateInterpreter(Settings())

    def _role_section(self, backend: str) -> str:
        if backend == "unitree":
            return ROLE_TEMPLATE
        return ROLE_TEMPLATE_GENERIC

    def _state_section(self, world: WorldState, summary: dict[str, str]) -> str:
        state_summary_lines = (
            "\n".join(f"  {k}: {v}" for k, v in summary.items()) or "  No issues detected."
        )

        robot_state_block = ""
        ss = world.robot_self_state
        if ss is not None:
            robot_state_block = "\n" + ROBOT_STATE_BLOCK_TEMPLATE.format(
                posture=summary.get("posture", "UNKNOWN"),
                motion=summary.get("motion", "UNKNOWN"),
                error=summary.get("error", "UNKNOWN"),
                freshness=summary.get("freshness", "UNKNOWN"),
                proximity=summary.get("proximity", "UNKNOWN"),
            )

        return STATE_TEMPLATE.format(
            state_summary=state_summary_lines,
            battery=world.battery_level,
            pos_x=world.position.x,
            pos_y=world.position.y,
            heading=world.heading_degrees,
            estop="ACTIVE" if world.estop_active else "inactive",
            robot_state_block=robot_state_block,
        )

    def _policies_section(self, active_policies: list[str]) -> str:
        return POLICIES_TEMPLATE.format(active_policies="\n".join(active_policies))

    def _tools_guidance_section(self, backend: str) -> str:
        if backend == "unitree":
            return TOOLS_GUIDANCE_TEMPLATE
        # For mock/other backends, provide a generic note
        return "[Available Tools]\nUse the provided function tools to achieve the objective."

    def _memory_section(self, memories: list[str]) -> str:
        truncated = memories[: self.max_memories]
        if not truncated:
            return ""
        formatted = "\n".join(f"- {m}" for m in truncated)
        return MEMORY_TEMPLATE.format(memories=formatted)

    def _conversation_section(self, conversation: list[dict[str, str]]) -> str:
        truncated = conversation[-self.max_conversation_turns :]
        if not truncated:
            return ""
        lines: list[str] = []
        for turn in truncated:
            role = turn.get("role", "user")
            content = turn.get("content", "")
            label = "User" if role == "user" else "Robot"
            lines.append(f"{label}: {content}")
        return CONVERSATION_TEMPLATE.format(conversation="\n".join(lines))
