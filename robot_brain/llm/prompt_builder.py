"""Composable system prompt builder for the LLM planner.

Assembles a context-aware system instruction from the current world state,
decision policies, tool guidance, memories, and conversation history.
"""
from __future__ import annotations

from robot_brain.core.world_state import WorldState
from robot_brain.llm.prompts.templates import (
    CONVERSATION_TEMPLATE,
    MEMORY_TEMPLATE,
    POLICIES_TEMPLATE,
    POLICY_CRITICAL_BATTERY,
    POLICY_ERROR,
    POLICY_ESTOP,
    POLICY_LOW_BATTERY,
    POLICY_NORMAL,
    POLICY_NOT_STANDING,
    POLICY_OBSTACLE_FRONT,
    POLICY_OBSTACLE_REAR,
    POLICY_STALE,
    ROBOT_STATE_BLOCK_TEMPLATE,
    ROLE_TEMPLATE,
    STATE_TEMPLATE,
    TOOLS_GUIDANCE_TEMPLATE,
)


class PromptBuilder:
    """Builds a structured, state-aware system prompt for the LLM planner."""

    def __init__(self, *, max_conversation_turns: int = 5, max_memories: int = 5) -> None:
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
        sections = [
            self._role_section(),
            self._state_section(world),
            self._policies_section(world),
            self._tools_guidance_section(backend),
        ]
        if memories:
            sections.append(self._memory_section(memories))
        if conversation:
            sections.append(self._conversation_section(conversation))
        return "\n\n".join(sections)

    def _role_section(self) -> str:
        return ROLE_TEMPLATE

    def _state_section(self, world: WorldState) -> str:
        summary = world._build_state_summary()
        state_summary_lines = "\n".join(f"  {k}: {v}" for k, v in summary.items()) or "  No issues detected."

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

    def _policies_section(self, world: WorldState) -> str:
        policies: list[str] = []

        # Priority-ordered policy selection based on current state
        if world.estop_active:
            policies.append(POLICY_ESTOP)

        if world.battery_level <= 10:
            policies.append(POLICY_CRITICAL_BATTERY)
        elif world.battery_level <= 25:
            policies.append(POLICY_LOW_BATTERY)

        ss = world.robot_self_state
        if ss is not None:
            if ss.error_code is not None and ss.error_code != 0:
                policies.append(POLICY_ERROR)
            if ss.is_standing is False:
                policies.append(POLICY_NOT_STANDING)
            if ss.state_age_seconds is not None and ss.state_age_seconds > 2.0:
                policies.append(POLICY_STALE)
            if ss.ultrasonic:
                if ss.ultrasonic.front_m is not None and ss.ultrasonic.front_m < 0.3:
                    policies.append(POLICY_OBSTACLE_FRONT)
                if ss.ultrasonic.rear_m is not None and ss.ultrasonic.rear_m < 0.3:
                    policies.append(POLICY_OBSTACLE_REAR)

        if not policies:
            policies.append(POLICY_NORMAL)

        return POLICIES_TEMPLATE.format(active_policies="\n".join(policies))

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
