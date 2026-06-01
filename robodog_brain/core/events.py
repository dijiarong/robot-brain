"""Events accepted by the runtime."""
from __future__ import annotations

from datetime import datetime, timezone
from enum import StrEnum
from typing import Any

from pydantic import BaseModel, Field


class EventType(StrEnum):
    COMMAND = "command"
    WARNING = "warning"
    INTERRUPT = "interrupt"
    CONFIRMATION = "confirmation"


class Event(BaseModel):
    type: EventType
    message: str
    payload: dict[str, Any] = Field(default_factory=dict)
    created_at: datetime = Field(default_factory=lambda: datetime.now(timezone.utc))
