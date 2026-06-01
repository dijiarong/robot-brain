"""In-memory checkpoints for confirmation-gated actions."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

from robot_brain.llm.base import ToolCall
from robot_brain.memory.sqlite_store import SQLiteMemoryStore


@dataclass
class PendingCheckpoint:
    thread_id: str
    command: str
    tool_call: ToolCall


class CheckpointRepository(Protocol):
    def save(self, checkpoint: PendingCheckpoint) -> None: ...

    def get(self, thread_id: str) -> PendingCheckpoint | None: ...

    def pop(self, thread_id: str) -> PendingCheckpoint | None: ...


class InMemoryCheckpointStore:
    def __init__(self) -> None:
        self._pending: dict[str, PendingCheckpoint] = {}

    def save(self, checkpoint: PendingCheckpoint) -> None:
        self._pending[checkpoint.thread_id] = checkpoint

    def get(self, thread_id: str) -> PendingCheckpoint | None:
        return self._pending.get(thread_id)

    def pop(self, thread_id: str) -> PendingCheckpoint | None:
        return self._pending.pop(thread_id, None)


class SQLiteCheckpointStore:
    def __init__(self, store: SQLiteMemoryStore) -> None:
        self.store = store

    def save(self, checkpoint: PendingCheckpoint) -> None:
        self.store.save_checkpoint(
            checkpoint.thread_id,
            checkpoint.command,
            checkpoint.tool_call.model_dump(mode="json"),
        )

    def get(self, thread_id: str) -> PendingCheckpoint | None:
        checkpoint = self.store.get_checkpoint(thread_id)
        return self._to_model(thread_id, checkpoint)

    def pop(self, thread_id: str) -> PendingCheckpoint | None:
        checkpoint = self.store.pop_checkpoint(thread_id)
        return self._to_model(thread_id, checkpoint)

    @staticmethod
    def _to_model(
        thread_id: str,
        checkpoint: tuple[str, dict[str, object]] | None,
    ) -> PendingCheckpoint | None:
        if checkpoint is None:
            return None
        command, tool_call = checkpoint
        return PendingCheckpoint(thread_id=thread_id, command=command, tool_call=ToolCall.model_validate(tool_call))


# Backward-compatible name for callers that explicitly want ephemeral checkpoints.
CheckpointStore = InMemoryCheckpointStore
