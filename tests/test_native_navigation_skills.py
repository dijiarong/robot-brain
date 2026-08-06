from __future__ import annotations

import unittest
import time

from robot_brain.core.world_state import WorldState
from robot_brain.navigation import FakeNavigationClient
from robot_brain.skills.builtin.native_navigation import (
    BBoxNavigateParams,
    NativeBBoxNavigateSkill,
    NativeExploreParams,
    NativeExploreSkill,
    NativeRelocalizeParams,
    NativeRelocalizeSkill,
    NativeVisualServoSkill,
    VisualServoNavigateParams,
    native_navigation_skills,
)
from robot_brain.vlm.object_recognition import VisualObject
from tests.test_native_navigation import _WorldCloudTransport, _client


class NativeNavigationSkillsTests(unittest.IsolatedAsyncioTestCase):
    async def test_bbox_skill_rejects_invalid_detection_without_motion(self) -> None:
        client = await _client(_WorldCloudTransport())
        result = await NativeBBoxNavigateSkill(client).execute(
            BBoxNavigateParams(
                bbox=(100, 100, 50, 50), fx=500, fy=500, cx=320, cy=240,
                image_width=640, image_height=480,
            ),
            client._robot,  # type: ignore[attr-defined]
            WorldState(),
        )
        self.assertFalse(result.success)
        self.assertEqual("invalid_bbox", result.data["stop_reason"])

    async def test_explore_skill_returns_structured_no_frontier_result(self) -> None:
        client = await _client(_WorldCloudTransport())
        result = await NativeExploreSkill(client).execute(
            NativeExploreParams(max_goals=1),
            client._robot,  # type: ignore[attr-defined]
            WorldState(),
        )
        self.assertFalse(result.success)
        self.assertIn("stop_reason", result.data)

    async def test_relocalize_skill_fails_closed_without_persistent_map(self) -> None:
        client = await _client(_WorldCloudTransport())
        result = await NativeRelocalizeSkill(client).execute(
            NativeRelocalizeParams(initial_x_m=0.0, initial_y_m=0.0),
            client._robot,  # type: ignore[attr-defined]
            WorldState(),
        )
        self.assertFalse(result.success)
        self.assertEqual("provider_error", result.data["stop_reason"])

    async def test_continuous_visual_skill_uses_normalized_vlm_bbox(self) -> None:
        class Frames:
            last_frame_monotonic = None

            async def get_frame(self):
                self.last_frame_monotonic = time.monotonic()
                return b"jpeg"

        class Recognizer:
            async def recognize(self, frame, target):
                self.assertions = (frame, target)
                return [VisualObject(
                    name="bottle", confidence=0.9,
                    bbox=(0.3828125, 0.2, 0.6171875, 0.8),
                )]

        skill = NativeVisualServoSkill(FakeNavigationClient(), Frames(), Recognizer())
        result = await skill.execute(
            VisualServoNavigateParams(
                object_name="bottle", fx=500, fy=500, cx=320, cy=240,
                image_width=640, image_height=480,
            ), None, WorldState(),
        )
        self.assertTrue(result.success)
        self.assertEqual("target_reached", result.data["stop_reason"])
        self.assertEqual(0, result.data["commands"])

    def test_continuous_visual_skill_is_only_registered_with_real_dependencies(self) -> None:
        client = FakeNavigationClient()
        names = {skill.name for skill in native_navigation_skills(client)}
        self.assertNotIn("nav_visual_servo", names)
        names = {
            skill.name for skill in native_navigation_skills(
                client, visual_frames=object(), visual_recognizer=object(),
            )
        }
        self.assertIn("nav_visual_servo", names)

    def test_terrain_exploration_skill_is_only_registered_for_mcf_motion(self) -> None:
        client = FakeNavigationClient()
        names = {skill.name for skill in native_navigation_skills(client)}
        self.assertNotIn("nav_terrain_explore", names)
        names = {skill.name for skill in native_navigation_skills(
            client, terrain_motion_enabled=True,
        )}
        self.assertIn("nav_terrain_explore", names)
