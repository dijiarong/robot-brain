"""FastAPI control surface for the background robot-brain service."""
from __future__ import annotations

import asyncio
from contextlib import asynccontextmanager
import logging
import math
from pathlib import Path
import time
from typing import Any, AsyncIterator

from fastapi import FastAPI, HTTPException, Response, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field

from config.settings import Settings
from robot_brain.core.events import Event
from robot_brain.navigation.base import (
    NavigationStatus,
    NavigationUnavailableError,
    RelativeNavigationGoal,
)
from robot_brain.runtime.loop import AgentRuntime
from robot_brain.runtime.scheduler import AgentScheduler
from robot_brain.service.dashboard import load_dashboard_html
from robot_brain.service.runner import AgentService

_STATIC_DIR = Path(__file__).resolve().parent / "static"
logger = logging.getLogger(__name__)

# Throttles for the dashboard map snapshot.  The navigation WebSocket polls at
# 4 Hz, but the voxel overview / costmap / LiDAR fusion are CPU-bound and would
# otherwise stall the single event loop (making teleop and camera laggy).
_VIEWER_SENSOR_INTERVAL_S = 0.5
_VIEWER_INTEGRATE_INTERVAL_S = 0.5
_VIEWER_RENDER_INTERVAL_S = 1.0
_VIEWER_VOXEL_MAX_POINTS = 1_800
_VIEWER_KNOWN_FREE_CAP = 1_600


class TaskCreateRequest(BaseModel):
    objective: str = Field(min_length=1)
    priority: int = 0
    max_attempts: int | None = Field(default=None, ge=1)
    source: str = "api"
    thread_id: str | None = None


class ConfirmationRequest(BaseModel):
    approved: bool


class MapNavigationRequest(BaseModel):
    x_m: float
    y_m: float
    confirm: str


class KeyboardTeleopRequest(BaseModel):
    active: bool = True
    vx: float = Field(default=0.0, ge=-0.5, le=0.5)
    vy: float = Field(default=0.0, ge=-0.5, le=0.5)
    vyaw: float = Field(default=0.0, ge=-1.2, le=1.2)


class PostureRequest(BaseModel):
    posture: str = Field(min_length=1)


class CameraOfferRequest(BaseModel):
    sdp: str = Field(min_length=1)
    type: str = "offer"


def _sensor_payload(sensor: Any) -> dict[str, object]:
    """Compact sensor snapshot payload (``sensor`` is a NavigationSensorSnapshot)."""
    return {
        "ready": sensor.ready,
        "reason": sensor.reason,
        "pose_age_seconds": sensor.pose_age_seconds,
        "pointcloud_age_seconds": sensor.pointcloud_age_seconds,
        "point_count": sensor.pointcloud.point_count if sensor.pointcloud else 0,
        "obstacle_frame": sensor.obstacle_frame,
        "pose_source": sensor.pose_source,
    }


def _voxel_payload(
    voxel_map: Any,
    pose: Any,
    sensor_snapshot: Any | None,
) -> tuple[dict[str, object] | None, str | None]:
    """Voxel overview payload, or ``(None, reason)`` when unavailable."""
    if voxel_map is None or pose is None:
        return None, None
    try:
        points = voxel_map.viewer_overview_points(
            center_x_m=pose.x_m,
            center_y_m=pose.y_m,
            z_min_m=-0.35,
            z_max_m=1.80,
            max_points=_VIEWER_VOXEL_MAX_POINTS,
        )
        cloud = sensor_snapshot.pointcloud if sensor_snapshot is not None else None
        return {
            "frame_id": pose.frame_id,
            "resolution_m": voxel_map.resolution_m,
            "generation": voxel_map.generation,
            "live_generation": round(cloud.received_monotonic, 2) if cloud else None,
            "points_xyz": [
                [round(x, 3), round(y, 3), round(z, 3), round(size, 3)]
                for x, y, z, size in points
            ],
        }, None
    except Exception as exc:
        return None, str(exc)


async def _costmap_payload(navigation: Any) -> tuple[dict[str, object] | None, str | None]:
    """Viewer costmap payload, or ``(None, reason)`` when unavailable."""
    get_costmap = getattr(navigation, "get_viewer_costmap", None)
    if not callable(get_costmap):
        get_costmap = getattr(navigation, "get_costmap", None)
    if not callable(get_costmap):
        return None, None
    try:
        grid = await get_costmap()
        # Cap free-space cells so the 4 Hz dashboard websocket stays light.
        known_free = sorted(grid.known_free)
        if len(known_free) > _VIEWER_KNOWN_FREE_CAP:
            stride = max(2, math.ceil(len(known_free) / _VIEWER_KNOWN_FREE_CAP))
            known_free = known_free[::stride]
        costs = list(grid.traversal_cost_values)
        if len(costs) > grid.width * grid.height:
            costs = []
        return {
            "frame_id": grid.frame_id,
            "resolution_m": grid.resolution_m,
            "width": grid.width,
            "height": grid.height,
            "origin_x_m": grid.origin_x_m,
            "origin_y_m": grid.origin_y_m,
            "occupied": [list(cell) for cell in sorted(grid.occupied)],
            "raw_occupied": [list(cell) for cell in sorted(grid.raw_occupied)],
            "known_free": [list(cell) for cell in known_free],
            "traversal_cost_values": costs,
        }, None
    except Exception as exc:
        return None, str(exc)


class _NavigationViewer:
    """Caches the expensive read-only pieces of the dashboard map snapshot.

    The dashboard polls ``/ws/navigation`` at 4 Hz; recomputing the voxel
    overview, costmap and LiDAR fusion on every tick stalls the single asyncio
    event loop and makes teleop + camera laggy.  These pieces are throttled and
    cached so the loop stays responsive while the map view stays live.
    """

    def __init__(self, service: AgentService) -> None:
        self._service = service
        self._last_sensor_monotonic = 0.0
        self._last_integrate_monotonic = 0.0
        self._last_render_monotonic = 0.0
        self._sensor_payload: dict[str, object] | None = None
        self._sensor_snapshot: Any | None = None
        self._voxels_payload: dict[str, object] | None = None
        self._costmap_payload: dict[str, object] | None = None
        self._voxel_reason: str | None = None
        self._costmap_reason: str | None = None

    @property
    def _navigation(self) -> Any:
        return self._service.scheduler.runtime.context.navigation

    async def sensor(self) -> tuple[dict[str, object] | None, Any | None]:
        """Throttled sensor payload + raw snapshot (``(None, None)`` when absent)."""
        now = time.monotonic()
        if (
            self._sensor_payload is not None
            and now - self._last_sensor_monotonic < _VIEWER_SENSOR_INTERVAL_S
        ):
            return self._sensor_payload, self._sensor_snapshot
        navigation = self._navigation
        payload: dict[str, object] | None = None
        snapshot: Any | None = None
        get_sensor_snapshot = getattr(navigation, "get_sensor_snapshot", None)
        if callable(get_sensor_snapshot):
            try:
                sensor = snapshot = await get_sensor_snapshot()
                payload = _sensor_payload(sensor)
            except Exception as exc:
                payload = {"ready": False, "reason": str(exc)}
        self._sensor_payload = payload
        self._sensor_snapshot = snapshot
        self._last_sensor_monotonic = now
        return payload, snapshot

    async def integrate(self, snapshot: Any | None) -> None:
        """Throttled LiDAR fusion into the voxel map for live visualization."""
        if snapshot is None:
            return
        now = time.monotonic()
        if now - self._last_integrate_monotonic < _VIEWER_INTEGRATE_INTERVAL_S:
            return
        navigation = self._navigation
        integrate = getattr(navigation, "integrate_viewer_snapshot", None)
        if not callable(integrate):
            return
        try:
            integrate(snapshot)
            self._last_integrate_monotonic = now
        except Exception as exc:
            logger.warning("viewer LiDAR fusion failed: %s", exc)

    async def render(
        self, *, pose: Any, sensor_snapshot: Any | None
    ) -> tuple[
        dict[str, object] | None,
        dict[str, object] | None,
        str | None,
        str | None,
    ]:
        """Throttled voxel overview + costmap payloads (cached between ticks)."""
        now = time.monotonic()
        if (
            self._voxels_payload is not None
            and now - self._last_render_monotonic < _VIEWER_RENDER_INTERVAL_S
        ):
            return (
                self._voxels_payload,
                self._costmap_payload,
                self._voxel_reason,
                self._costmap_reason,
            )
        navigation = self._navigation
        voxels, voxel_reason = _voxel_payload(
            getattr(navigation, "voxel_map", None), pose, sensor_snapshot
        )
        costmap, map_reason = await _costmap_payload(navigation)
        self._voxels_payload = voxels
        self._costmap_payload = costmap
        self._voxel_reason = voxel_reason
        self._costmap_reason = map_reason
        self._last_render_monotonic = now
        return voxels, costmap, voxel_reason, map_reason


async def _navigation_viewer_snapshot(
    service: AgentService, *, viewer: _NavigationViewer | None = None,
) -> dict[str, object]:
    """Return a compact, read-only snapshot for the dashboard map."""
    navigation = service.scheduler.runtime.context.navigation
    if navigation is None:
        return {"available": False, "reason": "navigation is not configured"}
    result: dict[str, object] = {
        "available": True,
        "provider": type(navigation).__name__,
    }
    try:
        state = await navigation.get_state()
        result["state"] = state.model_dump(mode="json")
        result["path"] = [pose.model_dump(mode="json") for pose in state.path]
    except Exception as exc:
        result.update(available=False, reason=f"navigation state unavailable: {exc}")
        return result

    # The in-memory trace can grow to tens of thousands of rows during a long
    # session; copying all of it at 4 Hz stalls the loop.  The dashboard only
    # needs the tail (trajectory keeps at most 500 samples, events 80).
    raw_trace = getattr(navigation, "_trace", ()) or ()
    trace = tuple(dict(row) for row in raw_trace[-2000:])
    result["trajectory"] = [
        {"x_m": row["x_m"], "y_m": row["y_m"],
         "yaw_degrees": row.get("yaw_degrees")}
        for row in trace if row.get("event") == "motion_sample"
        and "x_m" in row and "y_m" in row
    ][-500:]
    result["events"] = list(trace[-80:])
    commands = [row for row in trace if row.get("event") == "command"]
    result["command"] = commands[-1] if commands else None
    goals = [row for row in trace if row.get("event") == "goal_accepted"]
    result["goal"] = goals[-1] if goals else None
    safe_goals = [row for row in trace if row.get("event") == "safe_goal_adjusted"]
    result["safe_goal"] = safe_goals[-1] if safe_goals else None
    world = service.scheduler.runtime.context.world
    robot_self_state = world.robot_self_state
    attitude = robot_self_state.imu_rpy if robot_self_state is not None else None
    result["attitude"] = attitude.model_dump(mode="json") if attitude is not None else None

    if viewer is not None:
        sensor_payload, sensor_snapshot = await viewer.sensor()
        await viewer.integrate(sensor_snapshot)
        if sensor_payload is not None:
            result["sensor"] = sensor_payload
        voxels, costmap, voxel_reason, map_reason = await viewer.render(
            pose=state.pose, sensor_snapshot=sensor_snapshot
        )
        result["voxels"] = voxels
        result["costmap"] = costmap
        if voxel_reason is not None:
            result["voxel_reason"] = voxel_reason
        if map_reason is not None:
            result["map_reason"] = map_reason
        return result

    get_sensor_snapshot = getattr(navigation, "get_sensor_snapshot", None)
    sensor_snapshot = None
    if callable(get_sensor_snapshot):
        try:
            sensor = sensor_snapshot = await get_sensor_snapshot()
            result["sensor"] = _sensor_payload(sensor)
        except Exception as exc:
            result["sensor"] = {"ready": False, "reason": str(exc)}

    integrate_viewer_snapshot = getattr(navigation, "integrate_viewer_snapshot", None)
    if sensor_snapshot is not None and callable(integrate_viewer_snapshot):
        try:
            integrate_viewer_snapshot(sensor_snapshot)
        except Exception as exc:
            result["viewer_mapping_reason"] = str(exc)

    voxel_map = getattr(navigation, "voxel_map", None)
    pose = state.pose
    voxels, voxel_reason = _voxel_payload(voxel_map, pose, sensor_snapshot)
    result["voxels"] = voxels
    if voxel_reason is not None:
        result["voxel_reason"] = voxel_reason
    costmap, map_reason = await _costmap_payload(navigation)
    result["costmap"] = costmap
    if map_reason is not None:
        result["map_reason"] = map_reason
    return result


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

    # Mount static files (CSS, JS, fonts can live alongside index.html)
    if _STATIC_DIR.exists():
        app.mount("/static", StaticFiles(directory=str(_STATIC_DIR)), name="static")

    @app.get("/", response_class=HTMLResponse)
    async def dashboard() -> str:
        return load_dashboard_html()

    @app.get("/health")
    async def health() -> dict[str, object]:
        return {"status": "ok", "service_running": service.running}

    @app.get("/api/status")
    async def status() -> dict[str, object]:
        return service.status()

    @app.get("/api/navigation/snapshot")
    async def navigation_snapshot() -> dict[str, object]:
        return await _navigation_viewer_snapshot(service)

    @app.get("/api/camera/frame")
    async def camera_frame() -> Response:
        frame_source = service.scheduler.runtime.passability_frame_source
        if frame_source is None:
            raise HTTPException(status_code=503, detail="camera frame source is not enabled")
        # Snapshots are the fallback path, so the tap may have been released to
        # the WebRTC relay; re-arm it whenever the dashboard polls again.
        service.ensure_dashboard_camera_tap()
        frame = await frame_source.get_frame()
        if frame is None:
            raise HTTPException(status_code=503, detail="camera frame is not ready")
        return Response(
            content=frame,
            media_type="image/jpeg",
            headers={"Cache-Control": "no-store"},
        )

    @app.post("/api/camera/webrtc/offer")
    async def camera_webrtc_offer(request: CameraOfferRequest) -> dict[str, str]:
        """Relay the Go2 camera track to the dashboard over WebRTC (no transcode)."""
        try:
            return await service.start_camera_webrtc(request.sdp, request.type)
        except RuntimeError as exc:
            raise HTTPException(status_code=503, detail=str(exc)) from exc
        except Exception as exc:  # noqa: BLE001 - surface aiortc/media failures as 503
            logger.warning("dashboard WebRTC offer failed: %s", exc)
            raise HTTPException(status_code=503, detail=str(exc)) from exc

    @app.post("/api/navigation/map-goal")
    async def navigation_map_goal(request: MapNavigationRequest) -> dict[str, object]:
        if request.confirm != "I_UNDERSTAND_MAP_NAVIGATION":
            raise HTTPException(status_code=400, detail="explicit navigation confirmation required")
        navigation = service.scheduler.runtime.context.navigation
        if navigation is None:
            raise HTTPException(status_code=503, detail="navigation is not configured")
        state = await navigation.get_state()
        if state.pose is None:
            raise HTTPException(status_code=503, detail="navigation pose is unavailable")
        # Planning needs a fresh LiDAR frame and odometry; when either is stale
        # the operator gets the sensor reason instead of a server error.
        try:
            get_costmap = getattr(navigation, "get_costmap", None)
            if callable(get_costmap):
                grid = await get_costmap()
                cell = grid.world_to_cell(request.x_m, request.y_m)
                if cell is None or cell not in grid.known_free or cell in grid.occupied:
                    raise HTTPException(
                        status_code=400, detail="target is not known free space"
                    )
            dx, dy = request.x_m-state.pose.x_m, request.y_m-state.pose.y_m
            yaw = math.radians(state.pose.yaw_degrees)
            forward = dx*math.cos(yaw)+dy*math.sin(yaw)
            left = -dx*math.sin(yaw)+dy*math.cos(yaw)
            if math.hypot(forward, left) > 3.0:
                raise HTTPException(
                    status_code=400, detail="target exceeds the 3 metre local limit"
                )
            handle = await navigation.set_relative_goal(RelativeNavigationGoal(
                forward_m=forward,
                left_m=left,
                require_final_yaw=False,
                max_duration_s=60.0,
            ))
        except NavigationUnavailableError as exc:
            raise HTTPException(
                status_code=503, detail=f"navigation sensors are not ready: {exc}"
            ) from exc
        return handle.model_dump(mode="json")

    @app.post("/api/navigation/cancel")
    async def navigation_cancel() -> dict[str, object]:
        """Cancel active Nav2/native goal; idempotent when idle.

        Operators must always be able to stop navigation (network recovery /
        re-goal also starts from a clean cancel).
        """
        navigation = service.scheduler.runtime.context.navigation
        if navigation is None:
            raise HTTPException(status_code=503, detail="navigation is not configured")
        try:
            state = await navigation.get_state()
            canceled = await navigation.cancel(state.goal_id)
        except NavigationUnavailableError as exc:
            raise HTTPException(status_code=503, detail=str(exc)) from exc
        return canceled.model_dump(mode="json")

    @app.post("/api/media/relays/start")
    async def start_media_relays() -> dict[str, object]:
        """Start deferred ffmpeg video/audio relays (on-demand)."""
        transport = getattr(service.scheduler.runtime.context.robot, "transport", None)
        ensure = getattr(transport, "ensure_media_relays", None)
        if not callable(ensure):
            raise HTTPException(
                status_code=503,
                detail="media relays require Unitree WebRTC transport",
            )
        return ensure()

    @app.post("/api/teleop/keyboard")
    async def keyboard_teleop(request: KeyboardTeleopRequest) -> dict[str, object]:
        try:
            if not request.active:
                return await service.stop_web_teleop()
            return await service.set_web_teleop(request.vx, request.vy, request.vyaw)
        except RuntimeError as exc:
            raise HTTPException(status_code=503, detail=str(exc)) from exc

    @app.post("/api/teleop/posture")
    async def teleop_posture(request: PostureRequest) -> dict[str, object]:
        from robot_brain.actuation.unitree import UnitreeRobot

        # Release any keyboard teleop lease so posture and driving never fight.
        await service.stop_web_teleop()
        robot = service.scheduler.runtime.context.robot
        if not isinstance(robot, UnitreeRobot):
            raise HTTPException(
                status_code=503, detail="posture control requires the Unitree backend"
            )
        if request.posture not in robot.ALLOWED_POSTURES:
            raise HTTPException(
                status_code=400,
                detail=f"unsupported posture; allowed: {sorted(robot.ALLOWED_POSTURES)}",
            )
        navigation = service.scheduler.runtime.context.navigation
        if navigation is not None:
            try:
                state = await navigation.get_state()
                if state.status == NavigationStatus.ACTIVE:
                    await navigation.cancel(state.goal_id)
            except Exception as exc:
                raise HTTPException(
                    status_code=503, detail=f"cannot preempt navigation: {exc}"
                ) from exc
        try:
            await robot.set_posture(request.posture)
        except RuntimeError as exc:
            raise HTTPException(status_code=503, detail=str(exc)) from exc
        return {"accepted": True, "posture": request.posture}

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

    @app.get("/api/threads/{thread_id}/replay")
    async def thread_replay(thread_id: str) -> dict[str, object]:
        runtime = service.scheduler.runtime
        if runtime._database is None:
            raise HTTPException(status_code=503, detail="no database configured")
        replay = runtime._database.thread_replay(thread_id)
        if not replay["messages"] and not replay["tasks"] and not replay["world_states"]:
            raise HTTPException(status_code=404, detail="thread not found or empty")
        return replay

    @app.get("/api/decision/latest")
    async def latest_decision() -> dict[str, object]:
        runtime = service.scheduler.runtime
        if runtime._database is None:
            raise HTTPException(status_code=503, detail="no database configured")
        decision = runtime._database.latest_decision_context()
        if decision is None:
            raise HTTPException(status_code=404, detail="no decision recorded yet")
        return decision

    @app.get("/api/summaries")
    async def list_summaries(limit: int = 20) -> list[dict[str, object]]:
        summaries = service.scheduler.runtime.summaries.list_summaries(limit)
        return [s.model_dump(mode="json") for s in summaries]

    @app.get("/api/summaries/{thread_id}")
    async def get_summary(thread_id: str) -> dict[str, object]:
        summary = service.scheduler.runtime.summaries.get_summary(thread_id)
        if summary is None:
            raise HTTPException(status_code=404, detail="no summary for thread")
        return summary.model_dump(mode="json")

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

    @app.websocket("/ws/navigation")
    async def websocket_navigation(websocket: WebSocket) -> None:
        await websocket.accept()
        viewer = getattr(service, "_navigation_viewer", None)
        if viewer is None:
            viewer = _NavigationViewer(service)
            service._navigation_viewer = viewer

        # The client tells us whether the navigation view is actually shown.
        # While hidden, skip the expensive snapshot work entirely.
        view_active = True
        disconnected = asyncio.Event()

        async def _reader() -> None:
            nonlocal view_active
            try:
                while True:
                    message = await websocket.receive_json()
                    if message.get("type") == "view":
                        view_active = bool(message.get("active"))
            except (WebSocketDisconnect, RuntimeError, ValueError):
                pass
            finally:
                disconnected.set()

        reader_task = asyncio.create_task(_reader(), name="navigation-ws-reader")
        previous_voxels: dict[tuple[float, float, float], list[float]] = {}
        previous_costmap_id: int | None = None
        try:
            while True:
                if view_active:
                    snapshot = await _navigation_viewer_snapshot(service, viewer=viewer)
                    voxels = snapshot.get("voxels")
                    if isinstance(voxels, dict):
                        # Copy so we never mutate the shared viewer cache.
                        voxels = dict(voxels)
                        snapshot["voxels"] = voxels
                        points = voxels.get("points_xyz")
                        if isinstance(points, list):
                            current = {
                                (point[0], point[1], point[2]): point
                                for point in points
                                if isinstance(point, list) and len(point) >= 4
                            }
                            if previous_voxels:
                                added = [
                                    point for key, point in current.items()
                                    if previous_voxels.get(key) != point
                                ]
                                removed = [
                                    list(key) for key in previous_voxels.keys()-current.keys()
                                ]
                                voxels["points_xyz"] = None
                                voxels["delta"] = {"add": added, "remove": removed}
                            previous_voxels = current
                    costmap = snapshot.get("costmap")
                    if isinstance(costmap, dict):
                        # Cached viewer payloads keep object identity between ticks.
                        costmap_id = id(costmap)
                        if previous_costmap_id == costmap_id:
                            snapshot["costmap"] = None
                            snapshot["costmap_unchanged"] = True
                        else:
                            previous_costmap_id = costmap_id
                            snapshot["costmap_unchanged"] = False
                    try:
                        await websocket.send_json(snapshot)
                    except (WebSocketDisconnect, RuntimeError):
                        return
                try:
                    timeout = 0.3 if view_active else 0.5
                    await asyncio.wait_for(disconnected.wait(), timeout=timeout)
                except asyncio.TimeoutError:
                    pass
                if disconnected.is_set():
                    break
        finally:
            reader_task.cancel()

    return app


def create_default_app() -> FastAPI:
    return create_app()
