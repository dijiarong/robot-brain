"""Structured error codes for the robot brain system."""
from __future__ import annotations

from enum import StrEnum
from typing import Any

from pydantic import BaseModel, Field


class ErrorCode(StrEnum):
    # LLM layer
    LLM_TIMEOUT = "llm_timeout"
    LLM_API_ERROR = "llm_api_error"
    LLM_INVALID_OUTPUT = "llm_invalid_output"
    LLM_UNKNOWN_SKILL = "llm_unknown_skill"
    LLM_PARAM_VALIDATION = "llm_param_validation"
    LLM_DEGRADED = "llm_degraded"

    # Safety layer
    SAFETY_NOT_WHITELISTED = "safety_not_whitelisted"
    SAFETY_INVALID_PARAMS = "safety_invalid_params"
    SAFETY_ESTOP_ACTIVE = "safety_estop_active"
    SAFETY_BATTERY_CRITICAL = "safety_battery_critical"
    SAFETY_PRECONDITION_FAILED = "safety_precondition_failed"
    SAFETY_MOTION_VIOLATION = "safety_motion_violation"
    SAFETY_CONFIRMATION_REQUIRED = "safety_confirmation_required"

    # Runtime layer
    RUNTIME_MAX_ITERATIONS = "runtime_max_iterations"
    RUNTIME_MISSING_CHECKPOINT = "runtime_missing_checkpoint"
    RUNTIME_SKILL_NOT_FOUND = "runtime_skill_not_found"
    RUNTIME_NO_RESULT = "runtime_no_result"


class BrainError(BaseModel):
    code: ErrorCode
    message: str
    details: dict[str, Any] = Field(default_factory=dict)
