"""Adapter for the topsun-bot/Navigation Nav2 action and odometry interface."""
from __future__ import annotations

import asyncio
from dataclasses import dataclass
from datetime import datetime, timezone
import math
import threading
import time
from typing import Any, Protocol
from uuid import uuid4

from robot_brain.navigation.base import (
    AbsoluteNavigationGoal,
    LocalizationState,
    LocalizationStatus,
    MapIdentity,
    NavigationClient,
    NavigationGoalHandle,
    NavigationPose,
    NavigationState,
    NavigationStatus,
    NavigationUnavailableError,
    RelativeNavigationGoal,
)


@dataclass(frozen=True)
class Nav2GoalSubmission:
    goal_id: str
    accepted: bool
    message: str = ""


@dataclass(frozen=True)
class Nav2GoalSnapshot:
    status_code: int
    distance_remaining_m: float | None = None
    message: str = ""
    error_code: str | None = None


class Nav2Bridge(Protocol):
    """Synchronous boundary around ROS2, kept injectable for offline tests."""

    def is_ready(self, timeout_s: float) -> bool: ...

    def get_pose(self, timeout_s: float) -> NavigationPose | None: ...

    def send_goal(
        self,
        pose: NavigationPose,
        *,
        timeout_s: float,
    ) -> Nav2GoalSubmission: ...

    def get_goal(self, goal_id: str) -> Nav2GoalSnapshot: ...

    def cancel_goal(self, goal_id: str, *, timeout_s: float) -> bool: ...

    def close(self) -> None: ...


class Nav2NavigationClient(NavigationClient):
    """Translate robot-frame relative goals into Navigation's Nav2 action."""

    def __init__(
        self,
        bridge: Nav2Bridge,
        *,
        server_timeout_s: float = 2.0,
        pose_timeout_s: float = 1.0,
        cancel_timeout_s: float = 2.0,
        map_id: str = "",
        map_version: str | None = None,
        map_frame: str = "map",
    ) -> None:
        self._bridge = bridge
        self._server_timeout_s = server_timeout_s
        self._pose_timeout_s = pose_timeout_s
        self._cancel_timeout_s = cancel_timeout_s
        self._map_identity = (
            MapIdentity(map_id=map_id, version=map_version, frame_id=map_frame)
            if map_id else None
        )
        self._state = NavigationState(
            provider="nav2",
            ready=False,
            status=NavigationStatus.UNAVAILABLE,
            message="Nav2 has not been queried",
        )
        self._initial_distance_m: float | None = None

    @property
    def supports_absolute_goals(self) -> bool:
        return self._map_identity is not None

    async def get_state(self) -> NavigationState:
        ready = await asyncio.to_thread(self._bridge.is_ready, self._server_timeout_s)
        if not ready:
            self._state = self._state.model_copy(
                update={
                    "ready": False,
                    "status": NavigationStatus.UNAVAILABLE,
                    "message": "Nav2 navigate_to_pose action is unavailable",
                    "error_code": "nav2_unavailable",
                    "updated_at": datetime.now(timezone.utc),
                }
            )
            return self._state.model_copy(deep=True)

        pose = await asyncio.to_thread(self._bridge.get_pose, self._pose_timeout_s)
        if self._state.goal_id is None or self._state.status.terminal:
            status = NavigationStatus.IDLE if self._state.goal_id is None else self._state.status
            self._state = self._state.model_copy(
                update={"ready": True, "status": status, "pose": pose or self._state.pose}
            )
            return self._state.model_copy(deep=True)

        snapshot = await asyncio.to_thread(self._bridge.get_goal, self._state.goal_id)
        status = self._map_status(snapshot)
        progress = self._progress(snapshot.distance_remaining_m)
        self._state = self._state.model_copy(
            update={
                "ready": True,
                "status": status,
                "pose": pose or self._state.pose,
                "progress": progress,
                "message": snapshot.message or f"Nav2 goal {status.value}",
                "error_code": snapshot.error_code,
                "updated_at": datetime.now(timezone.utc),
            }
        )
        return self._state.model_copy(deep=True)

    async def set_relative_goal(
        self, goal: RelativeNavigationGoal
    ) -> NavigationGoalHandle:
        if not await asyncio.to_thread(self._bridge.is_ready, self._server_timeout_s):
            raise NavigationUnavailableError("Nav2 navigate_to_pose action is unavailable")
        current = await asyncio.to_thread(self._bridge.get_pose, self._pose_timeout_s)
        if current is None:
            raise NavigationUnavailableError("Nav2 odometry pose is unavailable")

        yaw_rad = math.radians(current.yaw_degrees)
        target = NavigationPose(
            x_m=current.x_m + goal.forward_m * math.cos(yaw_rad) - goal.left_m * math.sin(yaw_rad),
            y_m=current.y_m + goal.forward_m * math.sin(yaw_rad) + goal.left_m * math.cos(yaw_rad),
            yaw_degrees=_normalize_degrees(current.yaw_degrees + goal.yaw_degrees),
            frame_id=current.frame_id,
        )
        submission = await asyncio.to_thread(
            self._bridge.send_goal,
            target,
            timeout_s=self._server_timeout_s,
        )
        if not submission.accepted:
            return NavigationGoalHandle(
                goal_id=submission.goal_id,
                accepted=False,
                message=submission.message or "Nav2 rejected relative goal",
            )

        self._initial_distance_m = math.hypot(goal.forward_m, goal.left_m)
        self._state = NavigationState(
            provider="nav2",
            ready=True,
            status=NavigationStatus.ACTIVE,
            goal_id=submission.goal_id,
            pose=current,
            progress=0.0,
            message=submission.message or "Nav2 relative goal accepted",
        )
        return NavigationGoalHandle(
            goal_id=submission.goal_id,
            accepted=True,
            message=self._state.message,
        )

    async def get_localization_state(self) -> LocalizationState:
        if not await asyncio.to_thread(self._bridge.is_ready, self._server_timeout_s):
            return LocalizationState(
                status=LocalizationStatus.LOST,
                map_identity=self._map_identity,
                message="Nav2 action server unavailable",
            )
        get_in_frame = getattr(self._bridge, "get_pose_in_frame", None)
        map_pose = None
        if callable(get_in_frame) and self._map_identity is not None:
            map_pose = await asyncio.to_thread(
                get_in_frame, self._map_identity.frame_id, self._pose_timeout_s
            )
        if self._map_identity is not None and map_pose is not None:
            return LocalizationState(
                status=LocalizationStatus.LOCALIZED,
                map_identity=self._map_identity,
                pose=map_pose,
                confidence=1.0,
                message="Nav2 map localization ready",
            )
        odom_pose = await asyncio.to_thread(self._bridge.get_pose, self._pose_timeout_s)
        return LocalizationState(
            status=LocalizationStatus.LOCAL if odom_pose is not None else LocalizationStatus.LOST,
            map_identity=self._map_identity,
            pose=odom_pose,
            confidence=1.0 if odom_pose is not None else 0.0,
            message=(
                "map identity or map->base_link transform unavailable"
                if odom_pose is not None
                else "Nav2 odometry unavailable"
            ),
        )

    async def set_absolute_goal(
        self, goal: AbsoluteNavigationGoal
    ) -> NavigationGoalHandle:
        localization = await self.get_localization_state()
        if not localization.usable_for_persistent_memory:
            raise NavigationUnavailableError("Nav2 map localization is not ready")
        identity = localization.map_identity
        assert identity is not None
        if goal.map_id != identity.map_id or (
            goal.map_version is not None and goal.map_version != identity.version
        ):
            raise NavigationUnavailableError("absolute goal belongs to a different map")
        if goal.pose.frame_id != identity.frame_id:
            raise NavigationUnavailableError("absolute goal frame does not match map frame")
        submission = await asyncio.to_thread(
            self._bridge.send_goal, goal.pose, timeout_s=self._server_timeout_s
        )
        if not submission.accepted:
            return NavigationGoalHandle(
                goal_id=submission.goal_id, accepted=False,
                message=submission.message or "Nav2 rejected absolute goal",
            )
        current = localization.pose
        self._initial_distance_m = (
            math.hypot(goal.pose.x_m - current.x_m, goal.pose.y_m - current.y_m)
            if current is not None else None
        )
        self._state = NavigationState(
            provider="nav2", ready=True, status=NavigationStatus.ACTIVE,
            goal_id=submission.goal_id, pose=current, progress=0.0,
            message=submission.message or "Nav2 absolute goal accepted",
        )
        return NavigationGoalHandle(
            goal_id=submission.goal_id, accepted=True, message=self._state.message
        )

    async def cancel(self, goal_id: str | None = None) -> NavigationState:
        active_id = self._state.goal_id
        if active_id is None or self._state.status != NavigationStatus.ACTIVE:
            return self._state.model_copy(deep=True)
        if goal_id is not None and goal_id != active_id:
            return self._state.model_copy(deep=True)

        canceled = await asyncio.to_thread(
            self._bridge.cancel_goal,
            active_id,
            timeout_s=self._cancel_timeout_s,
        )
        if canceled:
            self._state = self._state.model_copy(
                update={
                    "status": NavigationStatus.CANCELED,
                    "message": "Nav2 goal canceled",
                    "error_code": "canceled",
                    "updated_at": datetime.now(timezone.utc),
                }
            )
        return self._state.model_copy(deep=True)

    async def aclose(self) -> None:
        await asyncio.to_thread(self._bridge.close)

    def close(self) -> None:
        self._bridge.close()

    def _progress(self, remaining_m: float | None) -> float | None:
        if remaining_m is None or self._initial_distance_m is None:
            return self._state.progress
        if self._initial_distance_m <= 1e-6:
            return 0.0
        return max(0.0, min(1.0, 1.0 - remaining_m / self._initial_distance_m))

    @staticmethod
    def _map_status(snapshot: Nav2GoalSnapshot) -> NavigationStatus:
        if snapshot.status_code == 4:
            return NavigationStatus.SUCCEEDED
        if snapshot.status_code == 5:
            return NavigationStatus.CANCELED
        if snapshot.status_code == 6:
            detail = f"{snapshot.error_code or ''} {snapshot.message}".lower()
            if "progress" in detail or "stuck" in detail:
                return NavigationStatus.NO_PROGRESS
            return NavigationStatus.FAILED
        return NavigationStatus.ACTIVE


class RclpyNav2Bridge:
    """Lazy ROS2 bridge matching the Navigation repository's public topics."""

    def __init__(
        self,
        *,
        action_name: str = "/navigate_to_pose",
        odom_topic: str = "/odom",
        goal_frame: str = "odom",
        node_name: str = "robot_brain_nav2",
        base_frame: str = "base_link",
    ) -> None:
        self._action_name = action_name
        self._odom_topic = odom_topic
        self._goal_frame = goal_frame
        self._node_name = node_name
        self._base_frame = base_frame
        self._lock = threading.RLock()
        self._rclpy: Any | None = None
        self._node: Any | None = None
        self._action_client: Any | None = None
        self._navigate_type: Any | None = None
        self._latest_pose: NavigationPose | None = None
        self._goal_handles: dict[str, Any] = {}
        self._result_futures: dict[str, Any] = {}
        self._feedback_distance: dict[str, float] = {}
        self._owns_rclpy = False
        self._tf_buffer: Any | None = None
        self._tf_listener: Any | None = None

    def is_ready(self, timeout_s: float) -> bool:
        with self._lock:
            if not self._ensure():
                return False
            return bool(self._action_client.wait_for_server(timeout_sec=max(0.0, timeout_s)))

    def get_pose(self, timeout_s: float) -> NavigationPose | None:
        with self._lock:
            if not self._ensure():
                return None
            deadline = time.monotonic() + timeout_s
            while self._latest_pose is None and time.monotonic() < deadline:
                self._rclpy.spin_once(self._node, timeout_sec=0.05)
            return self._latest_pose.model_copy(deep=True) if self._latest_pose else None

    def get_pose_in_frame(self, frame_id: str, timeout_s: float) -> NavigationPose | None:
        with self._lock:
            if not self._ensure() or self._tf_buffer is None:
                return None
            deadline = time.monotonic() + timeout_s
            while time.monotonic() < deadline:
                try:
                    transform = self._tf_buffer.lookup_transform(
                        frame_id, self._base_frame, self._rclpy.time.Time()
                    )
                    t = transform.transform.translation
                    q = transform.transform.rotation
                    siny = 2.0 * (q.w * q.z + q.x * q.y)
                    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                    return NavigationPose(
                        x_m=float(t.x), y_m=float(t.y),
                        yaw_degrees=math.degrees(math.atan2(siny, cosy)),
                        frame_id=frame_id,
                    )
                except Exception:
                    self._rclpy.spin_once(self._node, timeout_sec=0.05)
            return None

    def send_goal(
        self,
        pose: NavigationPose,
        *,
        timeout_s: float,
    ) -> Nav2GoalSubmission:
        with self._lock:
            if not self._ensure() or not self._action_client.wait_for_server(timeout_sec=timeout_s):
                return Nav2GoalSubmission("", False, "Nav2 action server unavailable")

            goal = self._navigate_type.Goal()
            goal.pose.header.frame_id = pose.frame_id or self._goal_frame
            goal.pose.header.stamp = self._node.get_clock().now().to_msg()
            goal.pose.pose.position.x = pose.x_m
            goal.pose.pose.position.y = pose.y_m
            yaw = math.radians(pose.yaw_degrees)
            goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
            goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

            goal_box: dict[str, str] = {}

            def feedback_callback(message: Any) -> None:
                goal_id = goal_box.get("goal_id")
                remaining = getattr(message.feedback, "distance_remaining", None)
                if goal_id and remaining is not None:
                    self._feedback_distance[goal_id] = float(remaining)

            future = self._action_client.send_goal_async(goal, feedback_callback=feedback_callback)
            self._rclpy.spin_until_future_complete(self._node, future, timeout_sec=timeout_s)
            if not future.done() or future.result() is None:
                return Nav2GoalSubmission("", False, "Nav2 goal submission timed out")
            goal_handle = future.result()
            goal_id = _goal_id(goal_handle)
            goal_box["goal_id"] = goal_id
            if not goal_handle.accepted:
                return Nav2GoalSubmission(goal_id, False, "Nav2 rejected goal")
            self._goal_handles[goal_id] = goal_handle
            self._result_futures[goal_id] = goal_handle.get_result_async()
            return Nav2GoalSubmission(goal_id, True, "Nav2 goal accepted")

    def get_goal(self, goal_id: str) -> Nav2GoalSnapshot:
        with self._lock:
            if not self._ensure():
                return Nav2GoalSnapshot(0, message="ROS2 unavailable", error_code="ros_unavailable")
            future = self._result_futures.get(goal_id)
            if future is None:
                return Nav2GoalSnapshot(0, message="unknown Nav2 goal", error_code="unknown_goal")
            self._rclpy.spin_once(self._node, timeout_sec=0.01)
            remaining = self._feedback_distance.get(goal_id)
            if not future.done():
                return Nav2GoalSnapshot(2, distance_remaining_m=remaining, message="Nav2 goal active")
            wrapper = future.result()
            result = getattr(wrapper, "result", None)
            error_code = getattr(result, "error_code", None)
            error_msg = getattr(result, "error_msg", "") or ""
            return Nav2GoalSnapshot(
                int(getattr(wrapper, "status", 0)),
                distance_remaining_m=remaining,
                message=str(error_msg),
                error_code=str(error_code) if error_code not in (None, 0) else None,
            )

    def cancel_goal(self, goal_id: str, *, timeout_s: float) -> bool:
        with self._lock:
            if not self._ensure():
                return False
            handle = self._goal_handles.get(goal_id)
            if handle is None:
                return False
            future = handle.cancel_goal_async()
            self._rclpy.spin_until_future_complete(self._node, future, timeout_sec=timeout_s)
            if not future.done() or future.result() is None:
                return False
            return bool(getattr(future.result(), "goals_canceling", []))

    def close(self) -> None:
        with self._lock:
            if self._node is not None:
                try:
                    self._node.destroy_node()
                except Exception:
                    pass
            if self._owns_rclpy and self._rclpy is not None:
                try:
                    self._rclpy.shutdown()
                except Exception:
                    pass
            self._node = None
            self._action_client = None

    def _ensure(self) -> bool:
        if self._node is not None:
            return True
        try:
            import rclpy
            from nav2_msgs.action import NavigateToPose
            from nav_msgs.msg import Odometry
            from rclpy.action import ActionClient
            from tf2_ros import Buffer, TransformListener
        except Exception:
            return False
        try:
            if not rclpy.ok():
                rclpy.init(args=None)
                self._owns_rclpy = True
            self._rclpy = rclpy
            self._node = rclpy.create_node(self._node_name)
            self._navigate_type = NavigateToPose
            self._action_client = ActionClient(self._node, NavigateToPose, self._action_name)
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self._node)
            self._node.create_subscription(Odometry, self._odom_topic, self._on_odom, 10)
            return True
        except Exception:
            self.close()
            return False

    def _on_odom(self, message: Any) -> None:
        pose = message.pose.pose
        q = pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._latest_pose = NavigationPose(
            x_m=float(pose.position.x),
            y_m=float(pose.position.y),
            yaw_degrees=math.degrees(math.atan2(siny, cosy)),
            frame_id=str(message.header.frame_id or self._goal_frame),
        )


def create_nav2_navigation_client(settings: Any) -> Nav2NavigationClient:
    bridge = RclpyNav2Bridge(
        action_name=settings.nav2_action_name,
        odom_topic=settings.nav2_odom_topic,
        goal_frame=settings.nav2_goal_frame,
        base_frame=settings.nav2_base_frame,
    )
    return Nav2NavigationClient(
        bridge,
        server_timeout_s=settings.nav2_server_timeout_s,
        pose_timeout_s=settings.nav2_pose_timeout_s,
        cancel_timeout_s=settings.nav2_cancel_timeout_s,
        map_id=settings.nav2_map_id,
        map_version=settings.nav2_map_version,
        map_frame=settings.nav2_map_frame,
    )


def _normalize_degrees(value: float) -> float:
    return (value + 180.0) % 360.0 - 180.0


def _goal_id(goal_handle: Any) -> str:
    raw = getattr(getattr(goal_handle, "goal_id", None), "uuid", None)
    if raw is None:
        return str(uuid4())
    try:
        return bytes(raw).hex()
    except Exception:
        return str(uuid4())
