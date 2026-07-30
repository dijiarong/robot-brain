"""Safety-bounded local navigation using Go2 odometry and built-in LiDAR."""
from __future__ import annotations

import asyncio
from datetime import datetime, timezone
import math
import time
from uuid import uuid4

from robot_brain.actuation.unitree import UnitreeRobot
from robot_brain.navigation.base import (
    NavigationClient,
    LocalizationState,
    LocalizationStatus,
    MapIdentity,
    NavigationGoalHandle,
    NavigationPose,
    NavigationState,
    NavigationStatus,
    NavigationUnavailableError,
    RelativeNavigationGoal,
)
from robot_brain.navigation.sensors import NavigationSensorProvider, NavigationSensorSnapshot


class DirectGo2NavigationClient(NavigationClient):
    """Execute short relative goals as checked velocity segments.

    The client intentionally fails closed: every segment requires fresh odometry
    and a fresh point cloud in a trusted robot-relative frame.  It does not
    pretend to be a global planner and never consumes ``world`` point clouds as
    body-relative obstacles.
    """

    def __init__(
        self,
        robot: UnitreeRobot,
        sensors: NavigationSensorProvider,
        *,
        linear_speed_mps: float = 0.15,
        yaw_speed_rps: float = 0.3,
        segment_duration_s: float = 0.25,
        obstacle_stop_m: float = 0.45,
        obstacle_half_width_m: float = 0.28,
        min_progress_m: float = 0.02,
        min_progress_yaw_deg: float = 2.0,
        max_no_progress_segments: int = 4,
        odom_settle_s: float = 0.35,
        reach_tolerance_m: float = 0.015,
        reach_tolerance_yaw_deg: float = 2.0,
    ) -> None:
        self._robot = robot
        self._sensors = sensors
        self._linear_speed = linear_speed_mps
        self._yaw_speed = yaw_speed_rps
        self._segment_duration = segment_duration_s
        self._obstacle_stop = obstacle_stop_m
        self._obstacle_half_width = obstacle_half_width_m
        self._min_progress_m = min_progress_m
        self._min_progress_yaw = min_progress_yaw_deg
        self._max_no_progress = max_no_progress_segments
        self._odom_settle_s = odom_settle_s
        self._reach_tol_m = reach_tolerance_m
        self._reach_tol_yaw = reach_tolerance_yaw_deg
        self._state = NavigationState(
            provider="direct_go2",
            ready=False,
            status=NavigationStatus.UNAVAILABLE,
            message="Go2 navigation sensors have not been queried",
        )
        self._task: asyncio.Task[None] | None = None
        self._cancel_event = asyncio.Event()
        self._map_identity = MapIdentity(
            map_id=f"go2-session-{uuid4().hex}",
            frame_id="odom",
            persistent=False,
        )

    async def get_state(self) -> NavigationState:
        if self._task is None or self._task.done():
            snapshot = await self._sensors.get_snapshot()
            if self._state.status == NavigationStatus.ACTIVE and self._task is not None:
                try:
                    self._task.result()
                except (asyncio.CancelledError, Exception):
                    pass
            if self._state.goal_id is None or self._state.status == NavigationStatus.UNAVAILABLE:
                self._state = self._state.model_copy(
                    update={
                        "ready": snapshot.ready,
                        "status": NavigationStatus.IDLE if snapshot.ready else NavigationStatus.UNAVAILABLE,
                        "pose": _navigation_pose(snapshot),
                        "message": "Go2 local navigation ready" if snapshot.ready else snapshot.reason or "sensors unavailable",
                        "error_code": None if snapshot.ready else snapshot.reason,
                        "updated_at": datetime.now(timezone.utc),
                    }
                )
        return self._state.model_copy(deep=True)

    async def set_relative_goal(
        self, goal: RelativeNavigationGoal
    ) -> NavigationGoalHandle:
        if self._task is not None and not self._task.done():
            return NavigationGoalHandle(
                goal_id=self._state.goal_id or "",
                accepted=False,
                message="another navigation goal is active",
            )
        snapshot = await self._sensors.get_snapshot()
        if not snapshot.ready:
            raise NavigationUnavailableError(snapshot.reason or "Go2 navigation sensors unavailable")
        if _translation_blocked(
            snapshot,
            forward_m=goal.forward_m,
            left_m=goal.left_m,
            stop_distance_m=self._obstacle_stop,
            half_width_m=self._obstacle_half_width,
        ):
            raise NavigationUnavailableError("obstacle blocks the requested local direction")

        goal_id = f"go2-{uuid4().hex}"
        self._cancel_event = asyncio.Event()
        self._state = NavigationState(
            provider="direct_go2",
            ready=True,
            status=NavigationStatus.ACTIVE,
            goal_id=goal_id,
            pose=_navigation_pose(snapshot),
            progress=0.0,
            message="Go2 relative goal accepted",
        )
        self._task = asyncio.create_task(self._execute(goal_id, goal, snapshot))
        return NavigationGoalHandle(goal_id=goal_id, message=self._state.message)

    async def get_localization_state(self) -> LocalizationState:
        snapshot = await self._sensors.get_snapshot()
        if snapshot.pose is not None and snapshot.pose.frame_id != self._map_identity.frame_id:
            self._map_identity = self._map_identity.model_copy(
                update={"frame_id": snapshot.pose.frame_id}
            )
        return LocalizationState(
            status=LocalizationStatus.LOCAL if snapshot.pose_ready else LocalizationStatus.STALE,
            map_identity=self._map_identity,
            pose=_navigation_pose(snapshot),
            confidence=1.0 if snapshot.pose_ready else 0.0,
            message=(
                "session-local Go2 odometry; not valid across restarts"
                if snapshot.pose_ready
                else snapshot.reason or "Go2 odometry unavailable"
            ),
        )

    async def cancel(self, goal_id: str | None = None) -> NavigationState:
        if self._state.status != NavigationStatus.ACTIVE:
            return self._state.model_copy(deep=True)
        if goal_id is not None and goal_id != self._state.goal_id:
            return self._state.model_copy(deep=True)
        self._cancel_event.set()
        await self._robot.stop("local navigation canceled")
        task = self._task
        if task is not None:
            try:
                await asyncio.wait_for(asyncio.shield(task), timeout=1.0)
            except TimeoutError:
                task.cancel()
            except asyncio.CancelledError:
                pass
        if self._state.status == NavigationStatus.ACTIVE:
            self._finish(NavigationStatus.CANCELED, "navigation canceled", "canceled")
        return self._state.model_copy(deep=True)

    async def _execute(
        self,
        goal_id: str,
        goal: RelativeNavigationGoal,
        initial: NavigationSensorSnapshot,
    ) -> None:
        started = time.monotonic()
        previous = initial
        no_progress = 0
        translation_total = math.hypot(goal.forward_m, goal.left_m)
        yaw_total = abs(goal.yaw_degrees)
        translation_done = 0.0
        yaw_done = 0.0
        # Cumulative progress from the goal-start pose.  Per-segment deltas are
        # unreliable on 4G Remote: motion often arrives in odometry one segment late.
        progress_anchor_m = 0.0
        progress_anchor_yaw = 0.0
        try:
            if translation_total > 1e-3:
                ux = goal.forward_m / translation_total
                uy = goal.left_m / translation_total
                vx = self._linear_speed * ux
                vy = self._linear_speed * uy
                while True:
                    current = await self._sensors.get_snapshot()
                    failure = self._guard(current, goal, started)
                    if failure:
                        await self._robot.stop(failure[1])
                        self._finish(*failure)
                        return
                    translation_done = _goal_axis_progress_m(
                        initial, current, goal.forward_m, goal.left_m
                    )
                    if translation_done >= translation_total - self._reach_tol_m:
                        previous = current
                        self._set_progress(
                            translation_done, translation_total, yaw_done, yaw_total, current
                        )
                        break
                    remaining = max(0.0, translation_total - translation_done)
                    # Prefer a duration that matches remaining distance, but keep a
                    # short floor so Go2 can start stepping after prep.
                    ideal = remaining / max(self._linear_speed, 1e-3)
                    gait_floor = 0.45 if translation_done < 1e-3 else 0.25
                    segment_cap = max(self._segment_duration, gait_floor)
                    duration = min(segment_cap, max(gait_floor, ideal))
                    await self._robot.drive(vx=vx, vy=vy, duration=duration)
                    after = await self._snapshot_after_motion(previous, duration)
                    cumulative = _goal_axis_progress_m(
                        initial, after, goal.forward_m, goal.left_m
                    )
                    gained = cumulative - progress_anchor_m
                    if gained < _segment_min_progress_m(
                        self._min_progress_m, self._linear_speed, duration
                    ):
                        no_progress += 1
                    else:
                        no_progress = 0
                        progress_anchor_m = cumulative
                    if no_progress >= self._max_no_progress:
                        await self._robot.stop("local navigation made no progress")
                        self._finish(
                            NavigationStatus.NO_PROGRESS,
                            "local navigation made no progress",
                            "no_progress",
                            after,
                        )
                        return
                    translation_done = cumulative
                    previous = after
                    self._set_progress(
                        translation_done, translation_total, yaw_done, yaw_total, after
                    )

            no_progress = 0
            yaw_initial = previous
            if yaw_total >= 1.0:
                vyaw = self._yaw_speed if goal.yaw_degrees > 0 else -self._yaw_speed
                while True:
                    current = await self._sensors.get_snapshot()
                    failure = self._guard(current, goal, started, check_obstacle=False)
                    if failure:
                        await self._robot.stop(failure[1])
                        self._finish(*failure)
                        return
                    yaw_done = _yaw_delta(yaw_initial, current)
                    if yaw_done >= yaw_total - self._reach_tol_yaw:
                        previous = current
                        self._set_progress(
                            translation_done, translation_total, yaw_done, yaw_total, current
                        )
                        break
                    remaining_yaw = max(0.0, yaw_total - yaw_done)
                    duration = min(
                        self._segment_duration,
                        max(0.12, math.radians(remaining_yaw) / max(self._yaw_speed, 1e-3)),
                    )
                    await self._robot.drive(vyaw=vyaw, duration=duration)
                    after = await self._snapshot_after_motion(previous, duration)
                    cumulative_yaw = _yaw_delta(yaw_initial, after)
                    gained_yaw = cumulative_yaw - progress_anchor_yaw
                    if gained_yaw < self._min_progress_yaw:
                        no_progress += 1
                    else:
                        no_progress = 0
                        progress_anchor_yaw = cumulative_yaw
                    if no_progress >= self._max_no_progress:
                        await self._robot.stop("local rotation made no progress")
                        self._finish(
                            NavigationStatus.NO_PROGRESS,
                            "local rotation made no progress",
                            "no_progress",
                            after,
                        )
                        return
                    yaw_done = cumulative_yaw
                    previous = after
                    self._set_progress(
                        translation_done, translation_total, yaw_done, yaw_total, after
                    )

            translation_ok = (
                translation_total <= 1e-3
                or translation_done >= translation_total - self._reach_tol_m
            )
            yaw_ok = yaw_total < 1.0 or yaw_done >= yaw_total - self._reach_tol_yaw
            if translation_ok and yaw_ok:
                self._finish(
                    NavigationStatus.SUCCEEDED,
                    "relative goal reached",
                    None,
                    previous,
                    progress=1.0,
                )
            else:
                await self._robot.stop("relative goal not reached within tolerance")
                self._finish(
                    NavigationStatus.TIMED_OUT,
                    "relative goal not reached within tolerance",
                    "not_reached",
                    previous,
                )
        except asyncio.CancelledError:
            self._finish(NavigationStatus.CANCELED, "navigation canceled", "canceled")
            raise
        except Exception as exc:
            await self._robot.stop("local navigation provider error")
            self._finish(NavigationStatus.FAILED, f"local navigation failed: {exc}", "provider_error")

    async def _snapshot_after_motion(
        self,
        before: NavigationSensorSnapshot,
        duration: float,
    ) -> NavigationSensorSnapshot:
        """Poll sensors after a drive so delayed Remote odometry can catch up."""
        settle = min(self._odom_settle_s, max(0.12, duration))
        deadline = time.monotonic() + settle
        best = await self._sensors.get_snapshot()
        best_delta = _translation_delta(before, best) + math.radians(
            _yaw_delta(before, best)
        )
        while time.monotonic() < deadline:
            await asyncio.sleep(0.05)
            snap = await self._sensors.get_snapshot()
            score = _translation_delta(before, snap) + math.radians(
                _yaw_delta(before, snap)
            )
            if score >= best_delta:
                best = snap
                best_delta = score
            if (
                _translation_delta(before, snap) >= self._min_progress_m
                or _yaw_delta(before, snap) >= self._min_progress_yaw
            ):
                return snap
        return best

    def _guard(
        self,
        snapshot: NavigationSensorSnapshot,
        goal: RelativeNavigationGoal,
        started: float,
        *,
        check_obstacle: bool = True,
    ) -> tuple[NavigationStatus, str, str] | None:
        if self._cancel_event.is_set():
            return NavigationStatus.CANCELED, "navigation canceled", "canceled"
        if time.monotonic() - started >= goal.max_duration_s:
            return NavigationStatus.TIMED_OUT, "navigation timed out", "timed_out"
        if not snapshot.ready:
            return NavigationStatus.UNAVAILABLE, snapshot.reason or "navigation sensors unavailable", snapshot.reason or "sensors_unavailable"
        if check_obstacle and _translation_blocked(
            snapshot,
            forward_m=goal.forward_m,
            left_m=goal.left_m,
            stop_distance_m=self._obstacle_stop,
            half_width_m=self._obstacle_half_width,
        ):
            return NavigationStatus.FAILED, "dynamic obstacle blocks local path", "obstacle"
        return None

    def _set_progress(self, td: float, tt: float, yd: float, yt: float, snapshot: NavigationSensorSnapshot) -> None:
        total = tt + math.radians(yt)
        done = td + math.radians(yd)
        progress = 1.0 if total <= 1e-6 else min(0.99, done / total)
        self._state = self._state.model_copy(update={
            "pose": _navigation_pose(snapshot),
            "progress": progress,
            "updated_at": datetime.now(timezone.utc),
        })

    def _finish(
        self,
        status: NavigationStatus,
        message: str,
        error_code: str | None,
        snapshot: NavigationSensorSnapshot | None = None,
        *,
        progress: float | None = None,
    ) -> None:
        self._state = self._state.model_copy(update={
            "ready": status != NavigationStatus.UNAVAILABLE,
            "status": status,
            "pose": _navigation_pose(snapshot) if snapshot is not None else self._state.pose,
            "progress": progress if progress is not None else self._state.progress,
            "message": message,
            "error_code": error_code,
            "updated_at": datetime.now(timezone.utc),
        })


def _navigation_pose(snapshot: NavigationSensorSnapshot | None) -> NavigationPose | None:
    if snapshot is None or snapshot.pose is None:
        return None
    pose = snapshot.pose
    return NavigationPose(x_m=pose.x_m, y_m=pose.y_m, yaw_degrees=pose.yaw_deg, frame_id=pose.frame_id)


def _segment_min_progress_m(
    configured_min_m: float, linear_speed_mps: float, duration_s: float
) -> float:
    """Require a fraction of the commanded segment, never above the configured floor."""
    expected = max(0.0, linear_speed_mps) * max(0.0, duration_s)
    return min(configured_min_m, max(0.005, 0.35 * expected))


def _goal_axis_progress_m(
    start: NavigationSensorSnapshot,
    current: NavigationSensorSnapshot,
    forward_m: float,
    left_m: float,
) -> float:
    """Project odom displacement onto the original body-frame goal axis."""
    if start.pose is None or current.pose is None:
        return 0.0
    goal_len = math.hypot(forward_m, left_m)
    if goal_len <= 1e-6:
        return 0.0
    dx = current.pose.x_m - start.pose.x_m
    dy = current.pose.y_m - start.pose.y_m
    yaw0 = math.radians(start.pose.yaw_deg)
    body_forward = dx * math.cos(yaw0) + dy * math.sin(yaw0)
    body_left = -dx * math.sin(yaw0) + dy * math.cos(yaw0)
    ux, uy = forward_m / goal_len, left_m / goal_len
    return max(0.0, body_forward * ux + body_left * uy)


def _translation_delta(before: NavigationSensorSnapshot, after: NavigationSensorSnapshot) -> float:
    if before.pose is None or after.pose is None:
        return 0.0
    return math.hypot(after.pose.x_m - before.pose.x_m, after.pose.y_m - before.pose.y_m)


def _yaw_delta(before: NavigationSensorSnapshot, after: NavigationSensorSnapshot) -> float:
    if before.pose is None or after.pose is None:
        return 0.0
    return abs((after.pose.yaw_deg - before.pose.yaw_deg + 180.0) % 360.0 - 180.0)


def _translation_blocked(
    snapshot: NavigationSensorSnapshot,
    *,
    forward_m: float,
    left_m: float,
    stop_distance_m: float,
    half_width_m: float,
) -> bool:
    cloud = snapshot.pointcloud
    distance = math.hypot(forward_m, left_m)
    if cloud is None or distance <= 1e-6:
        return False
    ux, uy = forward_m / distance, left_m / distance
    corridor_length = min(distance + 0.20, stop_distance_m)
    for x, y, z in cloud.points_xyz:
        if z < 0.05 or z > 1.20:
            continue
        along = x * ux + y * uy
        lateral = abs(-x * uy + y * ux)
        if 0.0 < along <= corridor_length and lateral <= half_width_m:
            return True
    return False
