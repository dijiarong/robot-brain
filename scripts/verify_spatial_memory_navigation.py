#!/usr/bin/env python3
"""Spatial-memory acceptance: offline provider loop, or live Nav2 return.

Offline (default): FakeNavigation + RememberRoom/FindObject.
Live ``--live-return``: persist room/object at current map pose, leave, return
via absolute Nav2 goal bound to ``RDB_NAV2_MAP_ID`` (phase-3 gate without VLM).
"""
from __future__ import annotations

import argparse
import asyncio
import json
import math
import time
from pathlib import Path

from robot_brain.actuation.mock import MockRobot
from robot_brain.core.world_state import Position, WorldState
from robot_brain.memory.spatial import ObjectObservation, RoomMemory, SpatialMemoryStore
from robot_brain.navigation import (
    AbsoluteNavigationGoal,
    FakeNavigationClient,
    MapIdentity,
    NavigationPose,
    NavigationStatus,
)
from robot_brain.navigation.nav2 import create_nav2_navigation_client
from robot_brain.skills.builtin.spatial_memory import (
    FindObjectParams,
    FindObjectSkill,
    RememberRoomParams,
    RememberRoomSkill,
)
from robot_brain.vlm.frame_source import FrameSource
from robot_brain.vlm.object_recognition import VisualObject


class _Frame(FrameSource):
    async def get_frame(self) -> bytes | None:
        return b"offline-frame"


class _Recognizer:
    async def recognize(
        self, image: bytes, target: str | None = None
    ) -> list[VisualObject]:
        return [VisualObject(name=target or "杯子", confidence=0.95, bbox=(0.4, 0.2, 0.6, 0.8))]


async def _wait_terminal(client, goal_id: str, timeout_s: float, poll_s: float):
    deadline = time.monotonic() + timeout_s
    state = await client.get_state()
    samples: list[dict[str, object]] = []
    while not state.status.terminal and time.monotonic() < deadline:
        samples.append(state.model_dump(mode="json"))
        await asyncio.sleep(poll_s)
        state = await client.get_state()
    if not state.status.terminal:
        await client.cancel(goal_id)
        state = await client.get_state()
    samples.append(state.model_dump(mode="json"))
    return state, samples


class _NavViz:
    """Publish map goals / memory anchors for RViz (optional)."""

    def __init__(self) -> None:
        self._node = None
        self._goal_pub = None
        self._marker_pub = None
        try:
            import rclpy
            from geometry_msgs.msg import PoseStamped
            from rclpy.node import Node
            from visualization_msgs.msg import MarkerArray

            if not rclpy.ok():
                rclpy.init(args=None)
            self._node = Node("spatial_memory_viz")
            self._goal_pub = self._node.create_publisher(PoseStamped, "/goal_pose", 10)
            self._marker_pub = self._node.create_publisher(
                MarkerArray, "/spatial_memory_markers", 10
            )
            self._PoseStamped = PoseStamped
            self._MarkerArray = MarkerArray
        except Exception:
            self._node = None

    def _yaw_to_quat(self, yaw_deg: float) -> tuple[float, float, float, float]:
        half = math.radians(yaw_deg) * 0.5
        return (0.0, 0.0, math.sin(half), math.cos(half))

    def publish_goal(self, pose: NavigationPose, *, frame_id: str) -> None:
        if self._goal_pub is None:
            return
        msg = self._PoseStamped()
        msg.header.frame_id = frame_id
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.pose.position.x = float(pose.x_m)
        msg.pose.position.y = float(pose.y_m)
        qz, qw = self._yaw_to_quat(pose.yaw_degrees)[2:]
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self._goal_pub.publish(msg)

    def publish_anchor(
        self,
        *,
        x: float,
        y: float,
        yaw_deg: float,
        frame_id: str,
        label: str,
    ) -> None:
        if self._marker_pub is None:
            return
        from visualization_msgs.msg import Marker

        arr = self._MarkerArray()
        sphere = Marker()
        sphere.header.frame_id = frame_id
        sphere.header.stamp = self._node.get_clock().now().to_msg()
        sphere.ns = "spatial_memory"
        sphere.id = 1
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.position.x = float(x)
        sphere.pose.position.y = float(y)
        sphere.pose.position.z = 0.15
        qz, qw = self._yaw_to_quat(yaw_deg)[2:]
        sphere.pose.orientation.z = qz
        sphere.pose.orientation.w = qw
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.25
        sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = 0.1, 0.8, 1.0, 0.9
        text = Marker()
        text.header = sphere.header
        text.ns = "spatial_memory"
        text.id = 2
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = float(x)
        text.pose.position.y = float(y)
        text.pose.position.z = 0.45
        text.scale.z = 0.2
        text.color.r = text.color.g = text.color.b = text.color.a = 1.0
        text.text = label
        arr.markers = [sphere, text]
        self._marker_pub.publish(arr)

    def close(self) -> None:
        if self._node is not None:
            self._node.destroy_node()


async def verify_offline() -> int:
    store = SpatialMemoryStore(":memory:")
    navigation = FakeNavigationClient(
        pose=NavigationPose(x_m=1.0, y_m=2.0, frame_id="map"),
        map_identity=MapIdentity(map_id="acceptance-office", version="v1", frame_id="map"),
    )
    robot = MockRobot()
    world = WorldState()
    frames = _Frame()
    recognizer = _Recognizer()
    remember = RememberRoomSkill(store, frames, recognizer, navigation)  # type: ignore[arg-type]
    find = FindObjectSkill(store, frames, recognizer, navigation)  # type: ignore[arg-type]
    try:
        remembered = await remember.execute(
            RememberRoomParams(room_name="验收客厅"), robot, world
        )
        found = await find.execute(FindObjectParams(object_name="杯子"), robot, world)
        forbidden_actions = [
            item for item in robot.action_history
            if item.get("action") in {"move_to", "turn", "drive"}
        ]
        nav_actions = [item["action"] for item in navigation.command_history]
        report = {
            "mode": "offline",
            "ok": bool(
                remembered.success
                and found.success
                and not forbidden_actions
                and "set_absolute_goal" in nav_actions
                and "cancel" in nav_actions
            ),
            "remember_result": remembered.model_dump(mode="json"),
            "find_result": found.model_dump(mode="json"),
            "navigation_actions": navigation.command_history,
            "robot_actions": robot.action_history,
            "forbidden_direct_motion": forbidden_actions,
            "rooms": [room.model_dump(mode="json") for room in store.rooms()],
            "observations": [
                item.model_dump(mode="json")
                for item in store.observations("杯子", map_id="acceptance-office")
            ],
        }
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 0 if report["ok"] else 1
    finally:
        store.close()


async def verify_live_return(args: argparse.Namespace) -> int:
    from config.settings import Settings

    settings = Settings(navigation_backend="nav2")
    if not settings.nav2_map_id:
        print(json.dumps({
            "mode": "live_return",
            "ok": False,
            "error": "set RDB_NAV2_MAP_ID (and preferably RDB_NAV2_MAP_VERSION)",
        }, ensure_ascii=False, indent=2))
        return 6

    db_path = Path(args.db).expanduser()
    store = SpatialMemoryStore(db_path)
    client = create_nav2_navigation_client(settings)
    viz = _NavViz()
    report: dict[str, object] = {
        "mode": "live_return",
        "db": str(db_path),
        "map_id": settings.nav2_map_id,
        "map_version": settings.nav2_map_version,
        "room_name": args.room_name,
        "object_name": args.object_name,
        "away_m": args.away_m,
        "viz_enabled": viz._node is not None,
    }
    try:
        localization = await client.get_localization_state()
        report["localization"] = localization.model_dump(mode="json")
        if (
            not localization.usable_for_persistent_memory
            or localization.pose is None
            or localization.map_identity is None
        ):
            report["ok"] = False
            report["error"] = "localization not usable for persistent memory"
            print(json.dumps(report, ensure_ascii=False, indent=2))
            return 5

        pose = localization.pose
        identity = localization.map_identity
        context = {
            "map_id": identity.map_id,
            "map_version": identity.version,
            "frame_id": identity.frame_id,
            "session_id": None if identity.persistent else identity.map_id,
            "persistent_map": identity.persistent,
        }
        room = RoomMemory(
            name=args.room_name,
            anchor=Position(x=pose.x_m, y=pose.y_m),
            anchor_heading_degrees=pose.yaw_degrees,
            **context,
        )
        observation = ObjectObservation(
            room_name=room.name,
            object_name=args.object_name,
            position=Position(x=pose.x_m, y=pose.y_m),
            heading_degrees=pose.yaw_degrees,
            confidence=1.0,
            bbox=None,
            **context,
        )
        store.save_room(room)
        store.replace_room_observations(room.name, [observation], map_id=room.map_id)
        report["remembered_room"] = room.model_dump(mode="json")
        report["remembered_observation"] = observation.model_dump(mode="json")
        viz.publish_anchor(
            x=pose.x_m,
            y=pose.y_m,
            yaw_deg=pose.yaw_degrees,
            frame_id=identity.frame_id,
            label=args.room_name,
        )

        yaw = math.radians(pose.yaw_degrees)
        c, s = math.cos(yaw), math.sin(yaw)
        away_pose = NavigationPose(
            x_m=pose.x_m + c * args.away_m,
            y_m=pose.y_m + s * args.away_m,
            yaw_degrees=pose.yaw_degrees,
            frame_id=identity.frame_id,
        )
        away_goal = AbsoluteNavigationGoal(
            map_id=identity.map_id,
            map_version=identity.version,
            pose=away_pose,
            max_duration_s=args.timeout_s,
        )
        viz.publish_goal(away_pose, frame_id=identity.frame_id)
        away_handle = await client.set_absolute_goal(away_goal)
        report["away_goal"] = away_goal.model_dump(mode="json")
        report["away_handle"] = away_handle.model_dump(mode="json")
        if not away_handle.accepted:
            report["ok"] = False
            report["error"] = "away goal rejected"
            print(json.dumps(report, ensure_ascii=False, indent=2))
            return 3
        away_state, away_samples = await _wait_terminal(
            client, away_handle.goal_id, args.timeout_s, args.poll_interval_s
        )
        report["away_final"] = away_state.model_dump(mode="json")
        report["away_samples"] = away_samples
        if away_state.status != NavigationStatus.SUCCEEDED:
            report["ok"] = False
            report["error"] = "away leg did not succeed"
            print(json.dumps(report, ensure_ascii=False, indent=2))
            return 4

        mid = await client.get_localization_state()
        report["pose_after_away"] = (
            None if mid.pose is None else mid.pose.model_dump(mode="json")
        )
        away_travel_m = None
        if mid.pose is not None:
            away_travel_m = math.hypot(mid.pose.x_m - pose.x_m, mid.pose.y_m - pose.y_m)
        min_away_m = max(0.0, args.away_m * args.min_away_ratio)
        report["away_travel_m"] = away_travel_m
        report["min_away_m"] = min_away_m
        if away_travel_m is None or away_travel_m < min_away_m:
            report["ok"] = False
            report["error"] = (
                "away Nav2 succeeded but map displacement too small "
                f"(travel={away_travel_m}, need>={min_away_m}); "
                "likely false goal success — check RViz /cmd_vel and goal tolerance"
            )
            print(json.dumps(report, ensure_ascii=False, indent=2))
            return 8

        # Re-open store to mimic a later session reading the same map-bound memory.
        store.close()
        store = SpatialMemoryStore(db_path)
        rooms = store.rooms(map_id=identity.map_id)
        if not any(r.name == args.room_name for r in rooms):
            report["ok"] = False
            report["error"] = "room missing after reopen"
            report["rooms_reloaded"] = [r.model_dump(mode="json") for r in rooms]
            print(json.dumps(report, ensure_ascii=False, indent=2))
            return 7
        saved = next(r for r in rooms if r.name == args.room_name)
        report["rooms_reloaded"] = [r.model_dump(mode="json") for r in rooms]

        return_pose = NavigationPose(
            x_m=saved.anchor.x,
            y_m=saved.anchor.y,
            yaw_degrees=saved.anchor_heading_degrees,
            frame_id=saved.frame_id,
        )
        return_goal = AbsoluteNavigationGoal(
            map_id=saved.map_id,
            map_version=saved.map_version,
            pose=return_pose,
            max_duration_s=args.timeout_s,
        )
        viz.publish_anchor(
            x=saved.anchor.x,
            y=saved.anchor.y,
            yaw_deg=saved.anchor_heading_degrees,
            frame_id=saved.frame_id,
            label=saved.name,
        )
        viz.publish_goal(return_pose, frame_id=saved.frame_id)
        return_handle = await client.set_absolute_goal(return_goal)
        report["return_goal"] = return_goal.model_dump(mode="json")
        report["return_handle"] = return_handle.model_dump(mode="json")
        if not return_handle.accepted:
            report["ok"] = False
            report["error"] = "return goal rejected"
            print(json.dumps(report, ensure_ascii=False, indent=2))
            return 3
        return_state, return_samples = await _wait_terminal(
            client, return_handle.goal_id, args.timeout_s, args.poll_interval_s
        )
        report["return_final"] = return_state.model_dump(mode="json")
        report["return_samples"] = return_samples

        after = await client.get_localization_state()
        report["pose_after_return"] = (
            None if after.pose is None else after.pose.model_dump(mode="json")
        )
        dist_m = None
        if after.pose is not None:
            dist_m = math.hypot(after.pose.x_m - saved.anchor.x, after.pose.y_m - saved.anchor.y)
        report["distance_to_anchor_m"] = dist_m
        report["reach_tolerance_m"] = args.reach_tolerance_m
        report["ok"] = bool(
            return_state.status == NavigationStatus.SUCCEEDED
            and dist_m is not None
            and dist_m <= args.reach_tolerance_m
            and away_travel_m is not None
            and away_travel_m >= min_away_m
        )
        if not report["ok"] and "error" not in report:
            if return_state.status != NavigationStatus.SUCCEEDED:
                report["error"] = "return leg did not succeed"
            else:
                report["error"] = "return succeeded but pose farther than tolerance"
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 0 if report["ok"] else 4
    finally:
        viz.close()
        store.close()
        await client.aclose()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--live-return",
        action="store_true",
        help="Nav2: remember map pose, leave, reopen DB, return (moves the robot).",
    )
    parser.add_argument(
        "--db",
        default="data/spatial_phase3.sqlite",
        help="SQLite path for --live-return persistence.",
    )
    parser.add_argument("--room-name", default="验收锚点")
    parser.add_argument("--object-name", default="锚点标记")
    parser.add_argument(
        "--away-m",
        type=float,
        default=1.0,
        help="Body-forward leave distance before returning (meters).",
    )
    parser.add_argument(
        "--min-away-ratio",
        type=float,
        default=0.5,
        help="Require map travel >= away_m * ratio after away leg (anti false-success).",
    )
    parser.add_argument("--timeout-s", type=float, default=120.0)
    parser.add_argument("--poll-interval-s", type=float, default=0.25)
    parser.add_argument(
        "--reach-tolerance-m",
        type=float,
        default=0.25,
        help="Max map distance to remembered anchor after return.",
    )
    return parser.parse_args()


async def main_async(args: argparse.Namespace) -> int:
    if args.live_return:
        return await verify_live_return(args)
    return await verify_offline()


def main() -> None:
    raise SystemExit(asyncio.run(main_async(parse_args())))


if __name__ == "__main__":
    main()
