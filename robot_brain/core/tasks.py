"""Persistent scheduler task models."""
from __future__ import annotations

from datetime import datetime, timezone
from enum import StrEnum
from uuid import uuid4

from pydantic import BaseModel, Field


class TaskStatus(StrEnum):
    QUEUED = "queued"
    RUNNING = "running"
    AWAITING_CONFIRMATION = "awaiting_confirmation"
    PAUSED = "paused"
    COMPLETED = "completed"
    FAILED = "failed"
    CANCELLED = "cancelled"


class ScheduledTask(BaseModel):
    task_id: str = Field(default_factory=lambda: str(uuid4()))
    thread_id: str
    objective: str
    priority: int = 0
    status: TaskStatus = TaskStatus.QUEUED
    attempts: int = Field(default=0, ge=0)
    max_attempts: int = Field(default=1, ge=1)
    source: str = "command"
    last_message: str = ""
    created_at: datetime = Field(default_factory=lambda: datetime.now(timezone.utc))
    updated_at: datetime = Field(default_factory=lambda: datetime.now(timezone.utc))
