"""Background scheduler loop with graceful shutdown and event subscriptions."""
from __future__ import annotations

import asyncio
from contextlib import asynccontextmanager
from datetime import datetime, timezone
from typing import Any, AsyncIterator

from robot_brain.core.events import Event
from robot_brain.core.tasks import ScheduledTask
from robot_brain.runtime.scheduler import AgentScheduler, SchedulerResult


class AgentService:
    def __init__(
        self,
        scheduler: AgentScheduler,
        *,
        poll_interval: float = 0.5,
        close_runtime_on_stop: bool = True,
    ) -> None:
        self.scheduler = scheduler
        self.poll_interval = poll_interval
        self.close_runtime_on_stop = close_runtime_on_stop
        self._runner: asyncio.Task[None] | None = None
        self._stop_requested = asyncio.Event()
        self._scheduler_lock = asyncio.Lock()
        self._subscribers: set[asyncio.Queue[dict[str, Any]]] = set()
        self._last_result: SchedulerResult | None = None

    @property
    def running(self) -> bool:
        return self._runner is not None and not self._runner.done()

    async def start(self) -> None:
        if self.running:
            return
        self._stop_requested.clear()
        self._runner = asyncio.create_task(self._run_loop(), name="robot-brain-scheduler")
        await self.publish("service_started")

    async def stop(self) -> None:
        runner = self._runner
        if runner is None:
            if self.close_runtime_on_stop:
                self.scheduler.runtime.close()
            return
        self._stop_requested.set()
        await runner
        self._runner = None
        await self.publish("service_stopped")
        if self.close_runtime_on_stop:
            self.scheduler.runtime.close()

    async def dispatch_once(self) -> SchedulerResult:
        async with self._scheduler_lock:
            result = await self.scheduler.run_next()
        self._last_result = result
        if result.status != "idle":
            await self.publish("scheduler_result", result=result.model_dump(mode="json"))
        return result

    async def handle_event(self, event: Event) -> SchedulerResult:
        async with self._scheduler_lock:
            result = await self.scheduler.handle_event(event)
        self._last_result = result
        await self.publish(
            "event_handled",
            event=event.model_dump(mode="json"),
            result=result.model_dump(mode="json"),
        )
        return result

    async def submit_task(
        self,
        objective: str,
        *,
        priority: int = 0,
        max_attempts: int | None = None,
        source: str = "api",
        thread_id: str | None = None,
    ) -> ScheduledTask:
        async with self._scheduler_lock:
            task = self.scheduler.submit(
                objective,
                priority=priority,
                max_attempts=max_attempts,
                source=source,
                thread_id=thread_id,
            )
        await self.publish("task_queued", task=task.model_dump(mode="json"))
        return task

    async def cancel_task(self, task_id: str) -> ScheduledTask | None:
        async with self._scheduler_lock:
            task = self.scheduler.cancel(task_id)
        if task is not None:
            await self.publish("task_cancelled", task=task.model_dump(mode="json"))
        return task

    async def resume_task(self, task_id: str, *, approved: bool) -> SchedulerResult:
        async with self._scheduler_lock:
            result = await self.scheduler.resume_task(task_id, approved=approved)
        await self.publish("task_confirmation", result=result.model_dump(mode="json"))
        return result

    async def reset_estop(self) -> list[dict[str, Any]]:
        async with self._scheduler_lock:
            tasks = self.scheduler.reset_estop()
        serialized = [task.model_dump(mode="json") for task in tasks]
        await self.publish("estop_reset", tasks=serialized)
        return serialized

    def status(self) -> dict[str, Any]:
        runtime = self.scheduler.runtime
        return {
            "service": {
                "running": self.running,
                "poll_interval": self.poll_interval,
                "updated_at": datetime.now(timezone.utc).isoformat(),
            },
            "world": runtime.context.world.snapshot(),
            "tasks": [task.model_dump(mode="json") for task in self.scheduler.list_tasks()],
            "last_result": self._last_result.model_dump(mode="json") if self._last_result is not None else None,
        }

    @asynccontextmanager
    async def subscribe(self) -> AsyncIterator[asyncio.Queue[dict[str, Any]]]:
        queue: asyncio.Queue[dict[str, Any]] = asyncio.Queue(maxsize=100)
        self._subscribers.add(queue)
        try:
            yield queue
        finally:
            self._subscribers.discard(queue)

    async def publish(self, event_type: str, **payload: Any) -> None:
        message = {
            "type": event_type,
            "created_at": datetime.now(timezone.utc).isoformat(),
            **payload,
        }
        for queue in tuple(self._subscribers):
            if queue.full():
                queue.get_nowait()
            queue.put_nowait(message)

    async def _run_loop(self) -> None:
        while not self._stop_requested.is_set():
            try:
                await self.dispatch_once()
            except Exception as exc:  # pragma: no cover - defensive loop protection.
                await self.publish("scheduler_error", message=str(exc))
            try:
                await asyncio.wait_for(self._stop_requested.wait(), timeout=self.poll_interval)
            except TimeoutError:
                pass
