#!/usr/bin/env python3
"""Offline end-to-end acceptance for provider-backed spatial memory."""
from __future__ import annotations

import asyncio
import json

from robot_brain.actuation.mock import MockRobot
from robot_brain.core.world_state import WorldState
from robot_brain.memory.spatial import SpatialMemoryStore
from robot_brain.navigation import FakeNavigationClient, MapIdentity, NavigationPose
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


async def verify() -> int:
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


def main() -> None:
    raise SystemExit(asyncio.run(verify()))


if __name__ == "__main__":
    main()
