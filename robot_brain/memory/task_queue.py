"""Replaceable persistent queue for scheduler tasks."""
from __future__ import annotations

from datetime import datetime, timezone
from typing import Protocol
from uuid import uuid4

from robot_brain.core.tasks import ScheduledTask, TaskStatus


TERMINAL_TASK_STATUSES = {TaskStatus.COMPLETED, TaskStatus.FAILED, TaskStatus.CANCELLED}


class TaskStore(Protocol):
    def save_task(self, task: ScheduledTask) -> None: ...

    def get_task(self, task_id: str) -> ScheduledTask | None: ...

    def list_tasks(self) -> list[ScheduledTask]: ...


class InMemoryTaskStore:
    def __init__(self) -> None:
        self._tasks: dict[str, ScheduledTask] = {}

    def save_task(self, task: ScheduledTask) -> None:
        self._tasks[task.task_id] = task.model_copy(deep=True)

    def get_task(self, task_id: str) -> ScheduledTask | None:
        task = self._tasks.get(task_id)
        return task.model_copy(deep=True) if task is not None else None

    def list_tasks(self) -> list[ScheduledTask]:
        return [task.model_copy(deep=True) for task in self._tasks.values()]


class TaskQueue:
    def __init__(self, store: TaskStore | None = None) -> None:
        self.store = store or InMemoryTaskStore()

    def enqueue(
        self,
        objective: str,
        *,
        priority: int = 0,
        max_attempts: int = 1,
        source: str = "command",
        thread_id: str | None = None,
    ) -> ScheduledTask:
        task_id = str(uuid4())
        task = ScheduledTask(
            task_id=task_id,
            thread_id=thread_id or f"task-{task_id}",
            objective=objective,
            priority=priority,
            max_attempts=max_attempts,
            source=source,
        )
        self.store.save_task(task)
        return task

    def get(self, task_id: str) -> ScheduledTask | None:
        return self.store.get_task(task_id)

    def list(self, *, statuses: set[TaskStatus] | None = None) -> list[ScheduledTask]:
        tasks = self.store.list_tasks()
        if statuses is not None:
            tasks = [task for task in tasks if task.status in statuses]
        return sorted(tasks, key=lambda item: (-item.priority, item.created_at, item.task_id))

    def next_queued(self) -> ScheduledTask | None:
        tasks = self.list(statuses={TaskStatus.QUEUED})
        return tasks[0] if tasks else None

    def update(
        self,
        task: ScheduledTask,
        *,
        status: TaskStatus | None = None,
        last_message: str | None = None,
        increment_attempts: bool = False,
    ) -> ScheduledTask:
        updated = task.model_copy(deep=True)
        if status is not None:
            updated.status = status
        if last_message is not None:
            updated.last_message = last_message
        if increment_attempts:
            updated.attempts += 1
        updated.updated_at = datetime.now(timezone.utc)
        self.store.save_task(updated)
        return updated

    def cancel(self, task_id: str) -> ScheduledTask | None:
        task = self.get(task_id)
        if task is None or task.status in TERMINAL_TASK_STATUSES:
            return task
        return self.update(task, status=TaskStatus.CANCELLED, last_message="cancelled by operator")

    def recover_running(self) -> list[ScheduledTask]:
        recovered = []
        for task in self.list(statuses={TaskStatus.RUNNING}):
            recovered.append(
                self.update(
                    task,
                    status=TaskStatus.QUEUED,
                    last_message="requeued after runtime restart",
                )
            )
        return recovered

    def pause_running(self, reason: str) -> list[ScheduledTask]:
        paused = []
        for task in self.list(statuses={TaskStatus.RUNNING}):
            paused.append(self.update(task, status=TaskStatus.PAUSED, last_message=reason))
        return paused

    def resume_paused(self) -> list[ScheduledTask]:
        resumed = []
        for task in self.list(statuses={TaskStatus.PAUSED}):
            resumed.append(self.update(task, status=TaskStatus.QUEUED, last_message="resumed after operator reset"))
        return resumed
