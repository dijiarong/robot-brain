"""Validates LLM output before it enters the orchestration graph."""
from __future__ import annotations

import logging

from pydantic import ValidationError

from robot_brain.core.errors import BrainError, ErrorCode
from robot_brain.llm.base import ToolCall
from robot_brain.skills.registry import SkillRegistry

logger = logging.getLogger(__name__)


class LLMOutputValidator:
    def __init__(self, skills: SkillRegistry) -> None:
        self.skills = skills

    def validate_tool_calls(
        self, raw_calls: list[ToolCall]
    ) -> tuple[list[ToolCall], list[BrainError]]:
        """Filter invalid tool calls, returning (valid_calls, errors)."""
        valid: list[ToolCall] = []
        errors: list[BrainError] = []

        for call in raw_calls:
            skill = self.skills.get(call.skill_name)
            if skill is None:
                errors.append(
                    BrainError(
                        code=ErrorCode.LLM_UNKNOWN_SKILL,
                        message=f"LLM requested unknown skill: {call.skill_name}",
                        details={"skill_name": call.skill_name},
                    )
                )
                logger.warning("LLM output rejected: unknown skill %r", call.skill_name)
                continue

            try:
                skill.parse_params(call.parameters)
            except ValidationError as exc:
                errors.append(
                    BrainError(
                        code=ErrorCode.LLM_PARAM_VALIDATION,
                        message=f"LLM provided invalid params for {call.skill_name}",
                        details={
                            "skill_name": call.skill_name,
                            "parameters": call.parameters,
                            "validation_errors": exc.errors(),
                        },
                    )
                )
                logger.warning(
                    "LLM output rejected: invalid params for %r: %s",
                    call.skill_name,
                    exc.errors(),
                )
                continue

            valid.append(call)

        return valid, errors
