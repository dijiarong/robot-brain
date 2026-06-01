"""Replaceable long-term experience store with an in-memory default."""
from __future__ import annotations

from typing import Protocol

from pydantic import BaseModel


class Experience(BaseModel):
    objective: str
    outcome: str
    summary: str


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
