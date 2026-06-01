"""Conversation messages stored per runtime thread."""
from __future__ import annotations

from datetime import datetime, timezone
from typing import Any, Literal, Protocol

from pydantic import BaseModel, Field


class ConversationMessage(BaseModel):
    thread_id: str
    role: Literal["user", "assistant", "system"]
    content: str
    message_type: str = "message"
    metadata: dict[str, Any] = Field(default_factory=dict)
    created_at: datetime = Field(default_factory=lambda: datetime.now(timezone.utc))


class ConversationStore(Protocol):
    def add_message(self, message: ConversationMessage) -> None: ...

    def recent_messages(self, thread_id: str, limit: int = 20) -> list[ConversationMessage]: ...


class InMemoryConversationStore:
    def __init__(self) -> None:
        self._messages: list[ConversationMessage] = []

    def add_message(self, message: ConversationMessage) -> None:
        self._messages.append(message)

    def recent_messages(self, thread_id: str, limit: int = 20) -> list[ConversationMessage]:
        return [item for item in self._messages if item.thread_id == thread_id][-limit:]


class ConversationMemory:
    def __init__(self, store: ConversationStore | None = None) -> None:
        self.store = store or InMemoryConversationStore()

    def add(
        self,
        *,
        thread_id: str,
        role: Literal["user", "assistant", "system"],
        content: str,
        message_type: str = "message",
        metadata: dict[str, Any] | None = None,
    ) -> None:
        self.store.add_message(
            ConversationMessage(
                thread_id=thread_id,
                role=role,
                content=content,
                message_type=message_type,
                metadata=metadata or {},
            )
        )

    def recent(self, thread_id: str, limit: int = 20) -> list[ConversationMessage]:
        return self.store.recent_messages(thread_id, limit)
