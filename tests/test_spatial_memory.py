from __future__ import annotations

import unittest

from robot_brain.actuation.mock import MockRobot
from robot_brain.core.world_state import Position, WorldState
from robot_brain.memory.spatial import ObjectObservation, RoomMemory, SpatialMemoryStore
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


class SpatialMemoryTests(unittest.IsolatedAsyncioTestCase):
    async def test_room_capture_rotates_five_times_and_persists_pose(self):
        store = SpatialMemoryStore(":memory:")
        recognizer = _Recognizer([[VisualObject(name="杯子", confidence=.9)]] * 5)
        skill = RememberRoomSkill(store, _Frames(), recognizer)  # type: ignore[arg-type]
        robot, world = MockRobot(), WorldState(position=Position(x=1, y=2), heading_degrees=0)

        result = await skill.execute(RememberRoomParams(room_name="客厅"), robot, world)

        self.assertTrue(result.success)
        self.assertEqual(5, result.data["photo_count"])
        self.assertEqual([60, 120, -180, -120, -60],
                         [a["heading_degrees"] for a in robot.action_history])
        self.assertEqual(5, len(store.observations("杯子")))
        self.assertEqual(Position(x=1, y=2), store.rooms()[0].anchor)

    async def test_find_uses_remembered_room_first_then_faces_and_waves(self):
        store = SpatialMemoryStore(":memory:")
        store.save_room(RoomMemory(name="厨房", anchor=Position(x=2, y=0)))
        store.save_room(RoomMemory(name="客厅", anchor=Position(x=5, y=0)))
        store.replace_room_observations("厨房", [ObjectObservation(
            room_name="厨房", object_name="杯子", position=Position(x=2, y=0),
            heading_degrees=30, confidence=.8)])
        # Two travel checks, then target at remembered heading.
        recognizer = _Recognizer([[], [], [VisualObject(name="杯子", confidence=.95,
                                                        bbox=(.6, .2, .8, .8))]])
        skill = FindObjectSkill(store, _Frames(), recognizer)  # type: ignore[arg-type]
        robot, world = MockRobot(), WorldState()

        result = await skill.execute(FindObjectParams(object_name="杯子", travel_check_interval_s=2.5),
                                     robot, world)

        self.assertTrue(result.success)
        self.assertEqual("厨房", result.data["source"])
        self.assertIn("wave", [a["action"] for a in robot.action_history])
        self.assertAlmostEqual(44.0, result.data["final_heading"])

    async def test_travel_detection_stops_search_immediately(self):
        store = SpatialMemoryStore(":memory:")
        store.save_room(RoomMemory(name="卧室", anchor=Position(x=2, y=0)))
        recognizer = _Recognizer([[VisualObject(name="钥匙", confidence=.9)]])
        skill = FindObjectSkill(store, _Frames(), recognizer)  # type: ignore[arg-type]
        robot, world = MockRobot(), WorldState()

        result = await skill.execute(FindObjectParams(object_name="钥匙"), robot, world)

        self.assertTrue(result.success)
        self.assertEqual("travel", result.data["source"])
        self.assertEqual(["move_to", "stop", "wave"], [a["action"] for a in robot.action_history])


if __name__ == "__main__":
    unittest.main()
