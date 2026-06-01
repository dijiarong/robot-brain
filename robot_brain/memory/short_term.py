"""Bounded working memory for recent events and decisions."""
from __future__ import annotations

from collections import deque


class ShortTermMemory:
    def __init__(self, capacity: int = 50) -> None:
        self._entries: deque[str] = deque(maxlen=capacity)

    @property
    def capacity(self) -> int:
        return self._entries.maxlen or 0

    def add(self, entry: str) -> None:
        self._entries.append(entry)

    def recent(self, limit: int = 10) -> list[str]:
        return list(self._entries)[-limit:]
