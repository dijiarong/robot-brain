"""SQLite persistence shared by conversations, experiences, and checkpoints."""
from __future__ import annotations

from datetime import datetime, timezone
import json
from pathlib import Path
import sqlite3
from threading import RLock
from typing import Any

from robot_brain.core.tasks import ScheduledTask
from robot_brain.memory.conversation import ConversationMessage
from robot_brain.memory.execution_summary import ExecutionSummary
from robot_brain.memory.long_term import Experience
from robot_brain.memory.world_state import WorldStateSnapshot


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


class SQLiteMemoryStore:
    def __init__(self, database_path: str | Path) -> None:
        self.database_path = str(database_path)
        if self.database_path != ":memory:":
            Path(self.database_path).expanduser().parent.mkdir(parents=True, exist_ok=True)
        self._lock = RLock()
        self._connection = sqlite3.connect(self.database_path, check_same_thread=False)
        self._closed = False
        self._connection.row_factory = sqlite3.Row
        self._connection.execute("PRAGMA foreign_keys = ON")
        self._initialize()

    def _initialize(self) -> None:
        with self._lock, self._connection:
            self._connection.executescript(
                """
                CREATE TABLE IF NOT EXISTS threads (
                    thread_id TEXT PRIMARY KEY,
                    created_at TEXT NOT NULL,
                    updated_at TEXT NOT NULL
                );

                CREATE TABLE IF NOT EXISTS messages (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    thread_id TEXT NOT NULL,
                    role TEXT NOT NULL,
                    message_type TEXT NOT NULL,
                    content TEXT NOT NULL,
                    metadata_json TEXT NOT NULL,
                    created_at TEXT NOT NULL,
                    FOREIGN KEY (thread_id) REFERENCES threads(thread_id) ON DELETE CASCADE
                );

                CREATE INDEX IF NOT EXISTS idx_messages_thread_created
                    ON messages(thread_id, created_at, id);

                CREATE TABLE IF NOT EXISTS experiences (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    objective TEXT NOT NULL,
                    outcome TEXT NOT NULL,
                    summary TEXT NOT NULL,
                    created_at TEXT NOT NULL
                );

                CREATE TABLE IF NOT EXISTS checkpoints (
                    thread_id TEXT PRIMARY KEY,
                    command TEXT NOT NULL,
                    tool_call_json TEXT NOT NULL,
                    created_at TEXT NOT NULL,
                    FOREIGN KEY (thread_id) REFERENCES threads(thread_id) ON DELETE CASCADE
                );

                CREATE TABLE IF NOT EXISTS world_state_snapshots (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    thread_id TEXT,
                    reason TEXT NOT NULL,
                    state_json TEXT NOT NULL,
                    created_at TEXT NOT NULL
                );

                CREATE TABLE IF NOT EXISTS scheduled_tasks (
                    task_id TEXT PRIMARY KEY,
                    thread_id TEXT NOT NULL,
                    objective TEXT NOT NULL,
                    priority INTEGER NOT NULL,
                    status TEXT NOT NULL,
                    attempts INTEGER NOT NULL,
                    max_attempts INTEGER NOT NULL,
                    source TEXT NOT NULL,
                    last_message TEXT NOT NULL,
                    created_at TEXT NOT NULL,
                    updated_at TEXT NOT NULL
                );

                CREATE INDEX IF NOT EXISTS idx_scheduled_tasks_status_priority
                    ON scheduled_tasks(status, priority DESC, created_at, task_id);

                CREATE TABLE IF NOT EXISTS execution_summaries (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    thread_id TEXT NOT NULL,
                    task_id TEXT,
                    objective TEXT NOT NULL,
                    outcome TEXT NOT NULL,
                    skills_executed TEXT NOT NULL,
                    duration_seconds REAL NOT NULL,
                    failure_reason TEXT,
                    memory_refs TEXT NOT NULL,
                    decision_source TEXT NOT NULL,
                    metadata_json TEXT NOT NULL,
                    created_at TEXT NOT NULL
                );

                CREATE INDEX IF NOT EXISTS idx_execution_summaries_thread
                    ON execution_summaries(thread_id, created_at DESC);

                CREATE TABLE IF NOT EXISTS decision_context (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    thread_id TEXT NOT NULL,
                    command TEXT NOT NULL,
                    chosen_skills TEXT NOT NULL,
                    reason TEXT NOT NULL,
                    memory_refs TEXT NOT NULL,
                    safety_result TEXT NOT NULL,
                    next_plan TEXT NOT NULL,
                    is_degraded INTEGER NOT NULL DEFAULT 0,
                    created_at TEXT NOT NULL
                );

                CREATE INDEX IF NOT EXISTS idx_decision_context_created
                    ON decision_context(created_at DESC);
                """
            )

    def _ensure_thread(self, thread_id: str, timestamp: str | None = None) -> None:
        timestamp = timestamp or _utc_now()
        self._connection.execute(
            """
            INSERT INTO threads(thread_id, created_at, updated_at)
            VALUES (?, ?, ?)
            ON CONFLICT(thread_id) DO UPDATE SET updated_at = excluded.updated_at
            """,
            (thread_id, timestamp, timestamp),
        )

    def add_message(self, message: ConversationMessage) -> None:
        created_at = message.created_at.isoformat()
        with self._lock, self._connection:
            self._ensure_thread(message.thread_id, created_at)
            self._connection.execute(
                """
                INSERT INTO messages(thread_id, role, message_type, content, metadata_json, created_at)
                VALUES (?, ?, ?, ?, ?, ?)
                """,
                (
                    message.thread_id,
                    message.role,
                    message.message_type,
                    message.content,
                    json.dumps(message.metadata, ensure_ascii=False, sort_keys=True, default=str),
                    created_at,
                ),
            )

    def recent_messages(self, thread_id: str, limit: int = 20) -> list[ConversationMessage]:
        with self._lock:
            rows = self._connection.execute(
                """
                SELECT thread_id, role, message_type, content, metadata_json, created_at
                FROM messages
                WHERE thread_id = ?
                ORDER BY created_at DESC, id DESC
                LIMIT ?
                """,
                (thread_id, limit),
            ).fetchall()
        return [
            ConversationMessage(
                thread_id=row["thread_id"],
                role=row["role"],
                message_type=row["message_type"],
                content=row["content"],
                metadata=json.loads(row["metadata_json"]),
                created_at=datetime.fromisoformat(row["created_at"]),
            )
            for row in reversed(rows)
        ]

    def add(self, experience: Experience) -> None:
        with self._lock, self._connection:
            self._connection.execute(
                """
                INSERT INTO experiences(objective, outcome, summary, created_at)
                VALUES (?, ?, ?, ?)
                """,
                (
                    experience.objective,
                    experience.outcome,
                    experience.summary,
                    experience.created_at.isoformat(),
                ),
            )

    def search(self, query: str, limit: int = 5) -> list[Experience]:
        with self._lock:
            rows = self._connection.execute(
                """
                SELECT objective, outcome, summary, created_at
                FROM experiences
                ORDER BY created_at DESC, id DESC
                """
            ).fetchall()
        terms = set(query.lower().split())
        experiences = [
            Experience(
                objective=row["objective"],
                outcome=row["outcome"],
                summary=row["summary"],
                created_at=datetime.fromisoformat(row["created_at"]),
            )
            for row in rows
        ]
        return sorted(
            experiences,
            key=lambda item: len(terms & set(f"{item.objective} {item.summary}".lower().split())),
            reverse=True,
        )[:limit]

    def save_checkpoint(self, thread_id: str, command: str, tool_call: dict[str, Any]) -> None:
        created_at = _utc_now()
        with self._lock, self._connection:
            self._ensure_thread(thread_id, created_at)
            self._connection.execute(
                """
                INSERT INTO checkpoints(thread_id, command, tool_call_json, created_at)
                VALUES (?, ?, ?, ?)
                ON CONFLICT(thread_id) DO UPDATE SET
                    command = excluded.command,
                    tool_call_json = excluded.tool_call_json,
                    created_at = excluded.created_at
                """,
                (thread_id, command, json.dumps(tool_call, ensure_ascii=False, sort_keys=True), created_at),
            )

    def get_checkpoint(self, thread_id: str) -> tuple[str, dict[str, Any]] | None:
        with self._lock:
            row = self._connection.execute(
                "SELECT command, tool_call_json FROM checkpoints WHERE thread_id = ?",
                (thread_id,),
            ).fetchone()
        if row is None:
            return None
        return row["command"], json.loads(row["tool_call_json"])

    def pop_checkpoint(self, thread_id: str) -> tuple[str, dict[str, Any]] | None:
        with self._lock, self._connection:
            checkpoint = self.get_checkpoint(thread_id)
            if checkpoint is not None:
                self._connection.execute("DELETE FROM checkpoints WHERE thread_id = ?", (thread_id,))
        return checkpoint

    def save_world_state(self, snapshot: WorldStateSnapshot) -> None:
        with self._lock, self._connection:
            self._connection.execute(
                """
                INSERT INTO world_state_snapshots(thread_id, reason, state_json, created_at)
                VALUES (?, ?, ?, ?)
                """,
                (
                    snapshot.thread_id,
                    snapshot.reason,
                    snapshot.state.model_dump_json(),
                    snapshot.created_at.isoformat(),
                ),
            )

    def latest_world_state(self) -> WorldStateSnapshot | None:
        with self._lock:
            row = self._connection.execute(
                """
                SELECT thread_id, reason, state_json, created_at
                FROM world_state_snapshots
                ORDER BY created_at DESC, id DESC
                LIMIT 1
                """
            ).fetchone()
        if row is None:
            return None
        return WorldStateSnapshot(
            thread_id=row["thread_id"],
            reason=row["reason"],
            state=json.loads(row["state_json"]),
            created_at=datetime.fromisoformat(row["created_at"]),
        )

    def save_task(self, task: ScheduledTask) -> None:
        with self._lock, self._connection:
            self._connection.execute(
                """
                INSERT INTO scheduled_tasks(
                    task_id, thread_id, objective, priority, status, attempts,
                    max_attempts, source, last_message, created_at, updated_at
                )
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                ON CONFLICT(task_id) DO UPDATE SET
                    thread_id = excluded.thread_id,
                    objective = excluded.objective,
                    priority = excluded.priority,
                    status = excluded.status,
                    attempts = excluded.attempts,
                    max_attempts = excluded.max_attempts,
                    source = excluded.source,
                    last_message = excluded.last_message,
                    updated_at = excluded.updated_at
                """,
                (
                    task.task_id,
                    task.thread_id,
                    task.objective,
                    task.priority,
                    task.status,
                    task.attempts,
                    task.max_attempts,
                    task.source,
                    task.last_message,
                    task.created_at.isoformat(),
                    task.updated_at.isoformat(),
                ),
            )

    def get_task(self, task_id: str) -> ScheduledTask | None:
        with self._lock:
            row = self._connection.execute(
                "SELECT * FROM scheduled_tasks WHERE task_id = ?",
                (task_id,),
            ).fetchone()
        return self._task_from_row(row) if row is not None else None

    def list_tasks(self) -> list[ScheduledTask]:
        with self._lock:
            rows = self._connection.execute(
                "SELECT * FROM scheduled_tasks ORDER BY priority DESC, created_at, task_id"
            ).fetchall()
        return [self._task_from_row(row) for row in rows]

    @staticmethod
    def _task_from_row(row: sqlite3.Row) -> ScheduledTask:
        return ScheduledTask(
            task_id=row["task_id"],
            thread_id=row["thread_id"],
            objective=row["objective"],
            priority=row["priority"],
            status=row["status"],
            attempts=row["attempts"],
            max_attempts=row["max_attempts"],
            source=row["source"],
            last_message=row["last_message"],
            created_at=datetime.fromisoformat(row["created_at"]),
            updated_at=datetime.fromisoformat(row["updated_at"]),
        )

    # --- Execution Summaries ---

    def save_summary(self, summary: ExecutionSummary) -> None:
        with self._lock, self._connection:
            self._connection.execute(
                """
                INSERT INTO execution_summaries(
                    thread_id, task_id, objective, outcome, skills_executed,
                    duration_seconds, failure_reason, memory_refs, decision_source,
                    metadata_json, created_at
                )
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                """,
                (
                    summary.thread_id,
                    summary.task_id,
                    summary.objective,
                    summary.outcome,
                    json.dumps(summary.skills_executed),
                    summary.duration_seconds,
                    summary.failure_reason,
                    json.dumps(summary.memory_refs),
                    summary.decision_source,
                    json.dumps(summary.metadata, ensure_ascii=False, default=str),
                    summary.created_at.isoformat(),
                ),
            )

    def get_summary(self, thread_id: str) -> ExecutionSummary | None:
        with self._lock:
            row = self._connection.execute(
                """
                SELECT * FROM execution_summaries
                WHERE thread_id = ?
                ORDER BY created_at DESC, id DESC
                LIMIT 1
                """,
                (thread_id,),
            ).fetchone()
        return self._summary_from_row(row) if row is not None else None

    def list_summaries(self, limit: int = 20) -> list[ExecutionSummary]:
        with self._lock:
            rows = self._connection.execute(
                """
                SELECT * FROM execution_summaries
                ORDER BY created_at DESC, id DESC
                LIMIT ?
                """,
                (limit,),
            ).fetchall()
        return [self._summary_from_row(row) for row in rows]

    @staticmethod
    def _summary_from_row(row: sqlite3.Row) -> ExecutionSummary:
        return ExecutionSummary(
            thread_id=row["thread_id"],
            task_id=row["task_id"],
            objective=row["objective"],
            outcome=row["outcome"],
            skills_executed=json.loads(row["skills_executed"]),
            duration_seconds=row["duration_seconds"],
            failure_reason=row["failure_reason"],
            memory_refs=json.loads(row["memory_refs"]),
            decision_source=row["decision_source"],
            metadata=json.loads(row["metadata_json"]),
            created_at=datetime.fromisoformat(row["created_at"]),
        )

    # --- Decision Context ---

    def save_decision_context(
        self,
        *,
        thread_id: str,
        command: str,
        chosen_skills: list[str],
        reason: str,
        memory_refs: list[str],
        safety_result: str,
        next_plan: str,
        is_degraded: bool = False,
    ) -> None:
        with self._lock, self._connection:
            self._connection.execute(
                """
                INSERT INTO decision_context(
                    thread_id, command, chosen_skills, reason, memory_refs,
                    safety_result, next_plan, is_degraded, created_at
                )
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
                """,
                (
                    thread_id,
                    command,
                    json.dumps(chosen_skills),
                    reason,
                    json.dumps(memory_refs),
                    safety_result,
                    next_plan,
                    1 if is_degraded else 0,
                    _utc_now(),
                ),
            )

    def latest_decision_context(self) -> dict[str, Any] | None:
        with self._lock:
            row = self._connection.execute(
                """
                SELECT * FROM decision_context
                ORDER BY created_at DESC, id DESC
                LIMIT 1
                """
            ).fetchone()
        if row is None:
            return None
        return {
            "thread_id": row["thread_id"],
            "command": row["command"],
            "chosen_skills": json.loads(row["chosen_skills"]),
            "reason": row["reason"],
            "memory_refs": json.loads(row["memory_refs"]),
            "safety_result": row["safety_result"],
            "next_plan": row["next_plan"],
            "is_degraded": bool(row["is_degraded"]),
            "created_at": row["created_at"],
        }

    # --- Thread Replay ---

    def thread_replay(self, thread_id: str) -> dict[str, Any]:
        """Return complete replay data for a thread: messages, tasks, world states."""
        with self._lock:
            messages = self._connection.execute(
                """
                SELECT thread_id, role, message_type, content, metadata_json, created_at
                FROM messages WHERE thread_id = ? ORDER BY created_at, id
                """,
                (thread_id,),
            ).fetchall()

            tasks = self._connection.execute(
                "SELECT * FROM scheduled_tasks WHERE thread_id = ? ORDER BY created_at",
                (thread_id,),
            ).fetchall()

            world_states = self._connection.execute(
                """
                SELECT thread_id, reason, state_json, created_at
                FROM world_state_snapshots WHERE thread_id = ? ORDER BY created_at, id
                """,
                (thread_id,),
            ).fetchall()

            summaries = self._connection.execute(
                """
                SELECT * FROM execution_summaries
                WHERE thread_id = ? ORDER BY created_at, id
                """,
                (thread_id,),
            ).fetchall()

        return {
            "thread_id": thread_id,
            "messages": [
                {
                    "role": row["role"],
                    "message_type": row["message_type"],
                    "content": row["content"],
                    "metadata": json.loads(row["metadata_json"]),
                    "created_at": row["created_at"],
                }
                for row in messages
            ],
            "tasks": [
                self._task_from_row(row).model_dump(mode="json")
                for row in tasks
            ],
            "world_states": [
                {
                    "reason": row["reason"],
                    "state": json.loads(row["state_json"]),
                    "created_at": row["created_at"],
                }
                for row in world_states
            ],
            "execution_summaries": [
                self._summary_from_row(row).model_dump(mode="json")
                for row in summaries
            ],
        }

    def close(self) -> None:
        with self._lock:
            if self._closed:
                return
            self._connection.close()
            self._closed = True
