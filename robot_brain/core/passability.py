"""Passability hint - structured direction suggestion from a VLM.

A :class:`PassabilityHint` is a **read-only, soft** suggestion produced by a
vision model (e.g. local Qwen3-VL). It advises which way the robot could move
next, but it is never authoritative: ultrasonic proximity is the hard safety
gate, and the explore loop falls back to rules when the hint is missing, stale,
low-confidence, or disagrees with sensors.

Mounted on :attr:`robot_brain.core.world_state.WorldState.passability_hint`.
"""
from __future__ import annotations

from datetime import datetime
from typing import Literal

from pydantic import BaseModel, Field

#: Allowed direction values. ``stop`` means "do not move forward even if
#: ultrasonic front is clear" (e.g. stairs / glass / people / drop-off).
PassabilityDirection = Literal["forward", "left", "right", "stop"]


class PassabilityHint(BaseModel):
    recommended_direction: PassabilityDirection
    confidence: float = Field(default=0.0, ge=0.0, le=1.0)
    reason: str = ""
    source: str = "qwen3-vl"
    frame_timestamp: datetime | None = None
    latency_ms: float | None = None
    #: Model identifier as configured (audit trail); may be truncated.
    raw_model: str = ""
