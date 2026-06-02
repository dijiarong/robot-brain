"""FastAPI control surface for the background robot-brain service."""
from __future__ import annotations

from contextlib import asynccontextmanager
from typing import AsyncIterator

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from pydantic import BaseModel, Field

from config.settings import Settings
from robot_brain.core.events import Event
from robot_brain.runtime.loop import AgentRuntime
from robot_brain.runtime.scheduler import AgentScheduler
from robot_brain.service.dashboard import DASHBOARD_HTML
from robot_brain.service.runner import AgentService


class TaskCreateRequest(BaseModel):
    objective: str = Field(min_length=1)
    priority: int = 0
    max_attempts: int | None = Field(default=None, ge=1)
    source: str = "api"
    thread_id: str | None = None


class ConfirmationRequest(BaseModel):
    approved: bool


def create_service(*, settings: Settings | None = None, poll_interval: float = 0.5) -> AgentService:
    runtime = AgentRuntime.create(settings=settings)
    return AgentService(AgentScheduler(runtime), poll_interval=poll_interval)


def create_app(service: AgentService | None = None) -> FastAPI:
    service = service or create_service()

    @asynccontextmanager
    async def lifespan(_: FastAPI) -> AsyncIterator[None]:
        await service.start()
        try:
            yield
        finally:
            await service.stop()

    app = FastAPI(title="Robot Brain Service", version="0.1.0", lifespan=lifespan)
    app.state.agent_service = service

    @app.get("/", response_class=HTMLResponse)
    async def dashboard() -> str:
        return DASHBOARD_HTML

    @app.get("/health")
    async def health() -> dict[str, object]:
        return {"status": "ok", "service_running": service.running}

    @app.get("/api/status")
    async def status() -> dict[str, object]:
        return service.status()

    @app.get("/api/tasks")
    async def list_tasks() -> list[dict[str, object]]:
        return [task.model_dump(mode="json") for task in service.scheduler.list_tasks()]

    @app.get("/api/tasks/{task_id}")
    async def get_task(task_id: str) -> dict[str, object]:
        task = service.scheduler.tasks.get(task_id)
        if task is None:
            raise HTTPException(status_code=404, detail="task does not exist")
        return task.model_dump(mode="json")

    @app.post("/api/tasks", status_code=201)
    async def create_task(request: TaskCreateRequest) -> dict[str, object]:
        task = await service.submit_task(
            request.objective,
            priority=request.priority,
            max_attempts=request.max_attempts,
            source=request.source,
            thread_id=request.thread_id,
        )
        return task.model_dump(mode="json")

    @app.delete("/api/tasks/{task_id}")
    async def cancel_task(task_id: str) -> dict[str, object]:
        task = await service.cancel_task(task_id)
        if task is None:
            raise HTTPException(status_code=404, detail="task does not exist")
        return task.model_dump(mode="json")

    @app.post("/api/tasks/{task_id}/confirm")
    async def confirm_task(task_id: str, request: ConfirmationRequest) -> dict[str, object]:
        result = await service.resume_task(task_id, approved=request.approved)
        return result.model_dump(mode="json")

    @app.post("/api/events")
    async def handle_event(event: Event) -> dict[str, object]:
        result = await service.handle_event(event)
        return result.model_dump(mode="json")

    @app.post("/api/estop/reset")
    async def reset_estop() -> dict[str, object]:
        tasks = await service.reset_estop()
        return {"status": "reset", "resumed_tasks": tasks}

    @app.websocket("/ws")
    async def websocket_status(websocket: WebSocket) -> None:
        await websocket.accept()
        await websocket.send_json({"type": "status", "status": service.status()})
        try:
            async with service.subscribe() as queue:
                while True:
                    message = await queue.get()
                    await websocket.send_json({**message, "status": service.status()})
        except WebSocketDisconnect:
            return

    return app


def create_default_app() -> FastAPI:
    return create_app()
