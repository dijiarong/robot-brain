"""Replaceable long-term experience store with an in-memory default."""
from __future__ import annotations

from datetime import datetime, timezone
from typing import Protocol

from pydantic import BaseModel, Field


class Experience(BaseModel):
    objective: str
    outcome: str
    summary: str
    created_at: datetime = Field(default_factory=lambda: datetime.now(timezone.utc))


class ExperienceStore(Protocol):
    def add(self, experience: Experience) -> None: ...

    def search(self, query: str, limit: int = 5) -> list[Experience]: ...


class InMemoryExperienceStore:
    def __init__(self) -> None:
        self._experiences: list[Experience] = []

    def add(self, experience: Experience) -> None:
        self._experiences.append(experience)

    def search(self, query: str, limit: int = 5) -> list[Experience]:
        terms = set(query.lower().split())
        ranked = sorted(
            self._experiences,
            key=lambda item: len(terms & set(f"{item.objective} {item.summary}".lower().split())),
            reverse=True,
        )
        return ranked[:limit]


class LongTermMemory:
    def __init__(self, store: ExperienceStore | None = None) -> None:
        self.store = store or InMemoryExperienceStore()

    def add(self, experience: Experience) -> None:
        self.store.add(experience)

    def search(self, query: str, limit: int = 5) -> list[Experience]:
        return self.store.search(query, limit)
