"""Replaceable persistence for world-state snapshots."""
from __future__ import annotations

from datetime import datetime, timezone
from typing import Protocol

from pydantic import BaseModel, Field

from robot_brain.core.world_state import WorldState


class WorldStateSnapshot(BaseModel):
    state: WorldState
    reason: str
    thread_id: str | None = None
    created_at: datetime = Field(default_factory=lambda: datetime.now(timezone.utc))


class WorldStateStore(Protocol):
    def save_world_state(self, snapshot: WorldStateSnapshot) -> None: ...

    def latest_world_state(self) -> WorldStateSnapshot | None: ...


class InMemoryWorldStateStore:
    def __init__(self) -> None:
        self._snapshots: list[WorldStateSnapshot] = []

    def save_world_state(self, snapshot: WorldStateSnapshot) -> None:
        self._snapshots.append(snapshot)

    def latest_world_state(self) -> WorldStateSnapshot | None:
        if not self._snapshots:
            return None
        return self._snapshots[-1]


class WorldStateMemory:
    def __init__(self, store: WorldStateStore | None = None) -> None:
        self.store = store or InMemoryWorldStateStore()

    def save(self, world: WorldState, *, reason: str, thread_id: str | None = None) -> None:
        self.store.save_world_state(
            WorldStateSnapshot(
                state=world.model_copy(deep=True),
                reason=reason,
                thread_id=thread_id,
            )
        )

    def latest(self) -> WorldStateSnapshot | None:
        return self.store.latest_world_state()
