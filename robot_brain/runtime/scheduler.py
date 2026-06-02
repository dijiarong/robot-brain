"""Persistent priority scheduler layered over the single-command runtime."""
from __future__ import annotations

from typing import Any
from uuid import uuid4

from pydantic import BaseModel

from robot_brain.core.events import Event, EventType
from robot_brain.core.tasks import ScheduledTask, TaskStatus
from robot_brain.memory.task_queue import TaskQueue
from robot_brain.runtime.loop import AgentRuntime, RunResult


class SchedulerResult(BaseModel):
    status: str
    message: str = ""
    task: ScheduledTask | None = None
    run_result: RunResult | None = None


class AgentScheduler:
    WARNING_PRIORITY = 100

    def __init__(self, runtime: AgentRuntime, tasks: TaskQueue | None = None) -> None:
        self.runtime = runtime
        self.tasks = tasks or runtime.tasks
        self.tasks.recover_running()

    def submit(
        self,
        objective: str,
        *,
        priority: int = 0,
        max_attempts: int | None = None,
        source: str = "command",
        thread_id: str | None = None,
    ) -> ScheduledTask:
        return self.tasks.enqueue(
            objective,
            priority=priority,
            max_attempts=max_attempts or self.runtime.context.settings.default_task_max_attempts,
            source=source,
            thread_id=thread_id,
        )

    def cancel(self, task_id: str) -> ScheduledTask | None:
        task = self.tasks.get(task_id)
        if task is not None and task.status == TaskStatus.AWAITING_CONFIRMATION:
            self.runtime.checkpoints.pop(task.thread_id)
        return self.tasks.cancel(task_id)

    def list_tasks(self, *, statuses: set[TaskStatus] | None = None) -> list[ScheduledTask]:
        return self.tasks.list(statuses=statuses)

    async def run_next(self) -> SchedulerResult:
        await self.runtime.refresh_world(reason="scheduler:poll")
        world = self.runtime.context.world
        settings = self.runtime.context.settings
        if world.estop_active:
            return SchedulerResult(status="paused", message="emergency stop is active")
        if world.battery_level <= settings.low_battery_threshold:
            result = await self.runtime.run_command(
                "dock at home for automatic recharge",
                thread_id=f"auto-recharge-{uuid4()}",
            )
            return SchedulerResult(
                status="auto_recharge",
                message=result.message,
                run_result=result,
            )
        task = self.tasks.next_queued()
        if task is None:
            return SchedulerResult(status="idle", message="no queued tasks")
        task = self.tasks.update(task, status=TaskStatus.RUNNING, increment_attempts=True)
        result = await self.runtime.run_command(task.objective, thread_id=task.thread_id)
        task = self._settle_task(task, result)
        return SchedulerResult(status=task.status, message=task.last_message, task=task, run_result=result)

    async def run_until_idle(self, *, max_cycles: int = 100) -> list[SchedulerResult]:
        results = []
        for _ in range(max_cycles):
            result = await self.run_next()
            results.append(result)
            if result.status in {"idle", "paused", TaskStatus.AWAITING_CONFIRMATION}:
                break
        return results

    async def resume_task(self, task_id: str, *, approved: bool) -> SchedulerResult:
        task = self.tasks.get(task_id)
        if task is None:
            return SchedulerResult(status="missing_task", message="task does not exist")
        if task.status != TaskStatus.AWAITING_CONFIRMATION:
            return SchedulerResult(status="invalid_task_state", message=f"task is {task.status}", task=task)
        result = await self.runtime.resume(task.thread_id, approved=approved)
        task = self._settle_task(task, result)
        return SchedulerResult(status=task.status, message=task.last_message, task=task, run_result=result)

    async def handle_event(self, event: Event) -> SchedulerResult:
        if event.type == EventType.INTERRUPT:
            result = await self.runtime.handle_event(event)
            self.tasks.pause_running(event.message)
            return SchedulerResult(status=result.status, message=result.message, run_result=result)
        if event.type == EventType.CONFIRMATION:
            task_id = str(event.payload.get("task_id", ""))
            return await self.resume_task(task_id, approved=bool(event.payload.get("approved")))
        if event.type == EventType.WARNING:
            task = self.submit(
                f"report warning: {event.message}",
                priority=int(event.payload.get("priority", self.WARNING_PRIORITY)),
                source="warning",
                thread_id=event.payload.get("thread_id"),
            )
            return SchedulerResult(status=task.status, message="warning task queued", task=task)
        if event.type == EventType.COMMAND:
            task = self.submit(
                event.message,
                priority=int(event.payload.get("priority", 0)),
                source="command",
                thread_id=event.payload.get("thread_id"),
            )
            return SchedulerResult(status=task.status, message="command task queued", task=task)
        return SchedulerResult(status="ignored", message=f"event type is not actionable: {event.type}")

    def reset_estop(self) -> list[ScheduledTask]:
        self.runtime.reset_estop()
        return self.tasks.resume_paused()

    def _settle_task(self, task: ScheduledTask, result: RunResult) -> ScheduledTask:
        if result.status == "completed":
            status = TaskStatus.COMPLETED
        elif result.status == "awaiting_confirmation":
            status = TaskStatus.AWAITING_CONFIRMATION
        elif task.attempts < task.max_attempts:
            status = TaskStatus.QUEUED
        else:
            status = TaskStatus.FAILED
        return self.tasks.update(task, status=status, last_message=result.message)
