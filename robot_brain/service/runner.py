"""Background scheduler loop with graceful shutdown and event subscriptions."""
from __future__ import annotations

import asyncio
from contextlib import asynccontextmanager
from datetime import datetime, timezone
import logging
from typing import Any, AsyncIterator

from robot_brain.core.events import Event
from robot_brain.core.tasks import ScheduledTask
from robot_brain.runtime.scheduler import AgentScheduler, SchedulerResult

logger = logging.getLogger(__name__)


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
        self._owns_robot_connection = False
        self._owns_dashboard_frame_source = False
        self._camera_bridge = None
        self._web_teleop = None
        self._web_teleop_lease_id = ""
        self._stop_lock = asyncio.Lock()

    @property
    def running(self) -> bool:
        return self._runner is not None and not self._runner.done()

    async def start(self) -> None:
        if self.running:
            return
        robot = self.scheduler.runtime.context.robot
        transport = getattr(robot, "transport", None)
        if transport is not None and not transport.is_connected:
            await transport.connect()
            self._owns_robot_connection = True

        # Wake the Go2 so teleop and mapping work even when the robot booted
        # lying down (mirrors the --live gateway's prep sequence).
        if self._owns_robot_connection and not robot.dry_run:
            from robot_brain.actuation.unitree import UnitreeRobot, prepare_locomotion

            settings = self.scheduler.runtime.context.settings
            if (
                isinstance(robot, UnitreeRobot)
                and settings.unitree_transport in {"webrtc", "sdk"}
                and settings.unitree_enable_motion
                and settings.unitree_auto_stand
            ):
                try:
                    await prepare_locomotion(robot)
                except Exception as exc:  # noqa: BLE001 - startup is best effort
                    logger.warning(
                        "automatic stand-up failed (robot may be lying down): %s", exc
                    )

        runtime = self.scheduler.runtime
        settings = runtime.context.settings
        if (
            runtime.passability_frame_source is None
            and settings.robot_backend == "unitree"
            and settings.unitree_transport == "webrtc"
            and not settings.unitree_video_relay
            and transport is not None
        ):
            from robot_brain.vlm.frame_source import Go2VideoFrameSource

            runtime.passability_frame_source = Go2VideoFrameSource()
            runtime.attach_passability_tap(transport.connection)
            self._owns_dashboard_frame_source = True
        self._stop_requested.clear()
        self._runner = asyncio.create_task(self._run_loop(), name="robot-brain-scheduler")
        await self.publish("service_started")

    async def stop(self) -> None:
        async with self._stop_lock:
            runtime = self.scheduler.runtime

            async def cleanup(name: str, action, timeout: float) -> None:
                try:
                    await asyncio.wait_for(action(), timeout=timeout)
                except Exception as exc:  # noqa: BLE001 - shutdown is best effort
                    logger.warning("shutdown step %s failed: %s", name, exc)

            runner = self._runner
            self._stop_requested.set()
            if runner is not None:
                await cleanup("scheduler", lambda: runner, 5.0)
                self._runner = None

            # Motion producers must stop before navigation/media/transport teardown.
            await cleanup("keyboard teleop", self.stop_web_teleop, 2.0)
            navigation = runtime.context.navigation
            if navigation is not None:
                await cleanup("navigation", navigation.cancel, 3.0)

            if self._camera_bridge is not None:
                await cleanup("dashboard video", self._camera_bridge.close_all, 5.0)
                self._camera_bridge = None

            if self._owns_dashboard_frame_source:
                source = runtime.passability_frame_source
                if source is not None:
                    frame_aclose = getattr(source, "aclose", None)
                    if callable(frame_aclose):
                        transport = getattr(runtime.context.robot, "transport", None)
                        if transport is not None and hasattr(transport, "run_on_conn_loop"):
                            await cleanup(
                                "dashboard camera",
                                lambda: transport.run_on_conn_loop(
                                    frame_aclose(), timeout=2.0
                                ),
                                3.0,
                            )
                        else:
                            await cleanup("dashboard camera", frame_aclose, 2.0)
                    else:
                        source.stop()
                runtime.passability_frame_source = None
                self._owns_dashboard_frame_source = False

            if self.close_runtime_on_stop:
                await cleanup("runtime", runtime.aclose, 5.0)

            if self._owns_robot_connection:
                transport = getattr(runtime.context.robot, "transport", None)
                if transport is not None:
                    await cleanup("robot transport", transport.disconnect, 8.0)
                self._owns_robot_connection = False

            self._subscribers.clear()
            await self.publish("service_stopped")

    async def set_web_teleop(self, vx: float, vy: float, vyaw: float) -> dict[str, Any]:
        """Apply a dashboard deadman setpoint through the shared teleop core."""
        from robot_brain.actuation.unitree import UnitreeRobot

        robot = self.scheduler.runtime.context.robot
        if not isinstance(robot, UnitreeRobot):
            raise RuntimeError("keyboard teleop requires the Unitree backend")
        if robot.dry_run:
            return {
                "accepted": False,
                "reason": "service is in dry-run mode; set RDB_UNITREE_DRY_RUN=false",
            }
        if self._web_teleop is None:
            from robot_brain.control.authority import session_or_create

            self._web_teleop = session_or_create(
                robot,
                self.scheduler.runtime.context.settings,
                self.scheduler.runtime.context.navigation,
            )
        if not self._web_teleop_lease_id:
            lease = await self._web_teleop.acquire_lease("dashboard-keyboard")
            if not lease.granted:
                return {"accepted": False, "reason": lease.reason}
            self._web_teleop_lease_id = lease.lease_id
        result = await self._web_teleop.set_velocity(
            self._web_teleop_lease_id, vx, vy, vyaw
        )
        return {"accepted": result.accepted, "reason": result.reason}

    async def stop_web_teleop(self) -> dict[str, Any]:
        """Release dashboard motion immediately; safe to call repeatedly."""
        lease_id = self._web_teleop_lease_id
        self._web_teleop_lease_id = ""
        if self._web_teleop is None or not lease_id:
            return {"accepted": True, "reason": "already stopped"}
        released = await self._web_teleop.release_lease(lease_id)
        return {"accepted": released, "reason": "released" if released else "stale lease"}

    async def start_camera_webrtc(self, sdp: str, sdp_type: str) -> dict[str, str]:
        """Answer a dashboard SDP offer with the live Go2 camera track."""
        from robot_brain.media.dashboard_video import DashboardVideoBridge

        transport = getattr(self.scheduler.runtime.context.robot, "transport", None)
        if transport is None or not hasattr(transport, "run_on_conn_loop"):
            raise RuntimeError("live camera requires the Unitree WebRTC transport")
        if self._camera_bridge is None:
            self._camera_bridge = DashboardVideoBridge(transport)
        answer = await self._camera_bridge.offer(sdp, sdp_type)
        self._suspend_dashboard_camera_tap()
        return answer

    def _suspend_dashboard_camera_tap(self) -> None:
        """Release the JPEG tap so it stops splitting frames with the browser.

        A Go2 video track hands each frame to a single ``recv()`` caller, so the
        snapshot tap and the WebRTC relay would otherwise get roughly half the
        frames each. The tap only exists as the dashboard's fallback here, so it
        yields to the live relay and is re-armed on the next snapshot request.
        A VLM-owned frame source is left alone: that one has its own priority.
        """
        if not self._owns_dashboard_frame_source:
            return
        source = self.scheduler.runtime.passability_frame_source
        if source is not None:
            source.stop()

    def ensure_dashboard_camera_tap(self) -> None:
        """Re-attach the JPEG fallback tap after the WebRTC relay released it."""
        if not self._owns_dashboard_frame_source:
            return
        runtime = self.scheduler.runtime
        transport = getattr(runtime.context.robot, "transport", None)
        if runtime.passability_frame_source is None or transport is None:
            return
        runtime.attach_passability_tap(transport.connection)

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
        diagnostics = runtime.diagnostics()
        return {
            "service": {
                "running": self.running,
                "poll_interval": self.poll_interval,
                "updated_at": datetime.now(timezone.utc).isoformat(),
            },
            "world": runtime.context.world.snapshot(),
            "tasks": [task.model_dump(mode="json") for task in self.scheduler.list_tasks()],
            "last_result": self._last_result.model_dump(mode="json") if self._last_result is not None else None,
            "vlm": diagnostics.get("vlm", {}),
            "explore": diagnostics.get("explore", {}),
            "navigation": diagnostics.get("navigation", {}),
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
