"""Structured execution summaries generated after task completion or failure."""
from __future__ import annotations

from datetime import datetime, timezone
from typing import Any, Protocol

from pydantic import BaseModel, Field


class ExecutionSummary(BaseModel):
    """Structured record of a completed or failed task execution."""

    thread_id: str
    task_id: str | None = None
    objective: str
    outcome: str  # "completed" | "failed" | "blocked" | "interrupted"
    skills_executed: list[str] = Field(default_factory=list)
    duration_seconds: float = 0.0
    failure_reason: str | None = None
    memory_refs: list[str] = Field(default_factory=list)
    decision_source: str = ""
    metadata: dict[str, Any] = Field(default_factory=dict)
    created_at: datetime = Field(default_factory=lambda: datetime.now(timezone.utc))


class ExecutionSummaryStore(Protocol):
    def save_summary(self, summary: ExecutionSummary) -> None: ...

    def get_summary(self, thread_id: str) -> ExecutionSummary | None: ...

    def list_summaries(self, limit: int = 20) -> list[ExecutionSummary]: ...


class InMemoryExecutionSummaryStore:
    def __init__(self) -> None:
        self._summaries: list[ExecutionSummary] = []

    def save_summary(self, summary: ExecutionSummary) -> None:
        self._summaries.append(summary)

    def get_summary(self, thread_id: str) -> ExecutionSummary | None:
        for s in reversed(self._summaries):
            if s.thread_id == thread_id:
                return s
        return None

    def list_summaries(self, limit: int = 20) -> list[ExecutionSummary]:
        return list(reversed(self._summaries[-limit:]))
