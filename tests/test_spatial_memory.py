from __future__ import annotations

import unittest
import sqlite3
import tempfile
from pathlib import Path

from robot_brain.actuation.mock import MockRobot
from robot_brain.core.world_state import Position, WorldState
from robot_brain.memory.spatial import ObjectObservation, RoomMemory, SpatialMemoryStore
from robot_brain.navigation import FakeNavigationClient, MapIdentity, NavigationPose
from robot_brain.skills.builtin.spatial_memory import (
    FindObjectParams, FindObjectSkill, RememberRoomParams, RememberRoomSkill,
)
from robot_brain.vlm.frame_source import FrameSource
from robot_brain.vlm.object_recognition import VisualObject


class _Frames(FrameSource):
    async def get_frame(self) -> bytes | None:
        return b"frame"


class _Recognizer:
    def __init__(self, responses: list[list[VisualObject]]) -> None:
        self.responses = responses
        self.calls = 0

    async def recognize(self, image: bytes, target: str | None = None) -> list[VisualObject]:
        result = self.responses[min(self.calls, len(self.responses) - 1)]
        self.calls += 1
        return result


def _navigation(*, x: float = 0.0, y: float = 0.0, yaw: float = 0.0):
    return FakeNavigationClient(
        pose=NavigationPose(x_m=x, y_m=y, yaw_degrees=yaw, frame_id="map"),
        map_identity=MapIdentity(map_id="office", version="v1", frame_id="map"),
    )


def _room(name: str, x: float, y: float) -> RoomMemory:
    return RoomMemory(
        name=name,
        anchor=Position(x=x, y=y),
        map_id="office",
        map_version="v1",
        frame_id="map",
        persistent_map=True,
    )


class SpatialMemoryTests(unittest.IsolatedAsyncioTestCase):
    async def test_room_capture_rotates_five_times_and_persists_pose(self):
        store = SpatialMemoryStore(":memory:")
        recognizer = _Recognizer([[VisualObject(name="杯子", confidence=.9)]] * 5)
        navigation = _navigation(x=1, y=2)
        skill = RememberRoomSkill(store, _Frames(), recognizer, navigation)  # type: ignore[arg-type]
        robot, world = MockRobot(), WorldState(position=Position(x=1, y=2), heading_degrees=0)

        result = await skill.execute(RememberRoomParams(room_name="客厅"), robot, world)

        self.assertTrue(result.success)
        self.assertEqual(5, result.data["photo_count"])
        yaw_goals = [
            item["goal"]["yaw_degrees"]
            for item in navigation.command_history
            if item["action"] == "set_relative_goal"
        ]
        self.assertEqual([60.0] * 5, yaw_goals)
        self.assertEqual(5, len(store.observations("杯子", map_id="office")))
        room = store.rooms(map_id="office")[0]
        self.assertEqual(Position(x=1, y=2), room.anchor)
        self.assertTrue(room.persistent_map)
        self.assertEqual("observation_pose", store.observations("杯子", map_id="office")[0].pose_kind)

    async def test_find_uses_remembered_room_first_then_faces_and_waves(self):
        store = SpatialMemoryStore(":memory:")
        store.save_room(_room("厨房", 2, 0))
        store.save_room(_room("客厅", 5, 0))
        store.replace_room_observations("厨房", [ObjectObservation(
            room_name="厨房", object_name="杯子", position=Position(x=2, y=0),
            heading_degrees=30, confidence=.8, map_id="office", map_version="v1",
            frame_id="map", persistent_map=True)], map_id="office")
        # Two travel checks, then target at remembered heading.
        recognizer = _Recognizer([[], [], [VisualObject(name="杯子", confidence=.95,
                                                        bbox=(.6, .2, .8, .8))]])
        navigation = _navigation()
        skill = FindObjectSkill(store, _Frames(), recognizer, navigation)  # type: ignore[arg-type]
        robot, world = MockRobot(), WorldState()

        result = await skill.execute(FindObjectParams(object_name="杯子", travel_check_interval_s=2.5),
                                     robot, world)

        self.assertTrue(result.success)
        self.assertEqual("厨房", result.data["source"])
        self.assertIn("wave", [a["action"] for a in robot.action_history])
        self.assertTrue(any(
            item["action"] == "set_absolute_goal"
            for item in navigation.command_history
        ))

    async def test_travel_detection_stops_search_immediately(self):
        store = SpatialMemoryStore(":memory:")
        store.save_room(_room("卧室", 2, 0))
        recognizer = _Recognizer([[VisualObject(name="钥匙", confidence=.9)]])
        navigation = _navigation()
        skill = FindObjectSkill(store, _Frames(), recognizer, navigation)  # type: ignore[arg-type]
        robot, world = MockRobot(), WorldState()

        result = await skill.execute(FindObjectParams(object_name="钥匙"), robot, world)

        self.assertTrue(result.success)
        self.assertEqual("travel", result.data["source"])
        self.assertEqual(["wave"], [a["action"] for a in robot.action_history])
        self.assertTrue(any(
            item["action"] == "cancel" and not item.get("noop", False)
            for item in navigation.command_history
        ))

    def test_same_room_name_is_isolated_by_map_identity(self):
        store = SpatialMemoryStore(":memory:")
        store.save_room(RoomMemory(
            name="厨房", anchor=Position(x=1, y=1), map_id="office", frame_id="map",
            persistent_map=True,
        ))
        store.save_room(RoomMemory(
            name="厨房", anchor=Position(x=8, y=8), map_id="warehouse", frame_id="map",
            persistent_map=True,
        ))
        store.replace_room_observations("厨房", [ObjectObservation(
            room_name="厨房", object_name="杯子", position=Position(x=1, y=1),
            heading_degrees=0, confidence=.9, map_id="office", frame_id="map",
            persistent_map=True,
        )], map_id="office")

        self.assertEqual(1, len(store.rooms(map_id="office")))
        self.assertEqual(8, store.rooms(map_id="warehouse")[0].anchor.x)
        self.assertEqual(1, len(store.observations("杯子", map_id="office")))
        self.assertEqual([], store.observations("杯子", map_id="warehouse"))

    async def test_session_local_cross_room_travel_is_split_into_provider_goals(self):
        store = SpatialMemoryStore(":memory:")
        navigation = FakeNavigationClient()
        localization = await navigation.get_localization_state()
        identity = localization.map_identity
        self.assertIsNotNone(identity)
        store.save_room(RoomMemory(
            name="远端",
            anchor=Position(x=1.5, y=0.0),
            map_id=identity.map_id,  # type: ignore[union-attr]
            frame_id="odom",
            session_id=identity.map_id,  # type: ignore[union-attr]
            persistent_map=False,
        ))
        skill = FindObjectSkill(
            store, _Frames(), _Recognizer([[]]), navigation  # type: ignore[arg-type]
        )
        robot = MockRobot()

        result = await skill.execute(FindObjectParams(object_name="不存在"), robot, WorldState())

        self.assertFalse(result.success)
        relative = [
            item for item in navigation.command_history
            if item["action"] == "set_relative_goal"
            and abs(float(item["goal"]["forward_m"])) > 0.0
        ]
        self.assertGreaterEqual(len(relative), 2)
        self.assertFalse(any(
            item["action"] in {"move_to", "turn", "drive"}
            for item in robot.action_history
        ))

    def test_legacy_database_migrates_without_losing_rooms(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "memory.sqlite3"
            connection = sqlite3.connect(path)
            connection.executescript("""
                CREATE TABLE spatial_rooms (
                    name TEXT PRIMARY KEY, x REAL NOT NULL, y REAL NOT NULL,
                    heading REAL NOT NULL, updated_at TEXT NOT NULL
                );
                CREATE TABLE spatial_observations (
                    id INTEGER PRIMARY KEY AUTOINCREMENT, room_name TEXT NOT NULL,
                    object_name TEXT NOT NULL, x REAL NOT NULL, y REAL NOT NULL,
                    heading REAL NOT NULL, confidence REAL NOT NULL,
                    bbox_json TEXT, observed_at TEXT NOT NULL
                );
                INSERT INTO spatial_rooms VALUES(
                    '旧客厅', 1.0, 2.0, 30.0, '2026-01-01T00:00:00+00:00'
                );
            """)
            connection.close()

            store = SpatialMemoryStore(path)
            rooms = store.rooms(map_id="legacy")
            self.assertEqual("旧客厅", rooms[0].name)
            self.assertFalse(rooms[0].persistent_map)
            store.save_room(_room("旧客厅", 8.0, 8.0))
            self.assertEqual(2, len(store.rooms()))
            store.close()


if __name__ == "__main__":
    unittest.main()
