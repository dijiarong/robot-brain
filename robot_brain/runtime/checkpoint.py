"""In-memory checkpoints for confirmation-gated actions."""
from __future__ import annotations

from dataclasses import dataclass

from robot_brain.llm.base import ToolCall


@dataclass
class PendingCheckpoint:
    thread_id: str
    command: str
    tool_call: ToolCall


class CheckpointStore:
    def __init__(self) -> None:
        self._pending: dict[str, PendingCheckpoint] = {}

    def save(self, checkpoint: PendingCheckpoint) -> None:
        self._pending[checkpoint.thread_id] = checkpoint

    def get(self, thread_id: str) -> PendingCheckpoint | None:
        return self._pending.get(thread_id)

    def pop(self, thread_id: str) -> PendingCheckpoint | None:
        return self._pending.pop(thread_id, None)
