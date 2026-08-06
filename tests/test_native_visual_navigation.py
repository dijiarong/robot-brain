from __future__ import annotations

import unittest

from robot_brain.navigation.visual_navigation import (
    CameraIntrinsics,
    bbox_to_relative_goal,
    compute_visual_servo,
    detection_bbox_to_pixels,
    detection_label_matches,
    robust_target_from_points,
)


CAMERA = CameraIntrinsics(fx=500, fy=500, cx=320, cy=240, width=640, height=480)


class NativeVisualNavigationTests(unittest.TestCase):
    def test_bbox_center_projects_to_bounded_relative_goal(self) -> None:
        centered = bbox_to_relative_goal((270, 180, 370, 300), CAMERA)
        right = bbox_to_relative_goal((470, 180, 570, 300), CAMERA)
        self.assertAlmostEqual(0.0, centered.left_m)
        self.assertLess(right.left_m, 0.0)

    def test_servo_turns_toward_object_and_advances_when_far(self) -> None:
        command = compute_visual_servo((450, 100, 500, 300), CAMERA)
        self.assertTrue(command.valid)
        self.assertLess(command.yaw_rps, 0.0)
        self.assertGreater(command.forward_mps, 0.0)

    def test_servo_backs_up_when_object_is_too_close(self) -> None:
        command = compute_visual_servo((100, 100, 500, 400), CAMERA)
        self.assertTrue(command.valid)
        self.assertLess(command.forward_mps, 0.0)

    def test_invalid_bbox_fails_closed_to_zero_command(self) -> None:
        command = compute_visual_servo((20, 20, 10, 10), CAMERA)
        self.assertFalse(command.valid)
        self.assertEqual(0.0, command.forward_mps)
        self.assertEqual(0.0, command.yaw_rps)

    def test_3d_target_uses_front_elevated_points(self) -> None:
        target = robust_target_from_points((
            (3.0, 0.0, 0.0), (1.0, -0.1, 0.8), (1.1, 0.0, 0.9),
            (1.2, 0.1, 0.85), (4.0, 1.0, 1.0),
        ))
        self.assertIsNotNone(target)
        self.assertLess(target[0], 2.0)  # type: ignore[index]

    def test_detection_bbox_supports_fraction_and_qwen_1000_coordinates(self) -> None:
        fraction = detection_bbox_to_pixels((.25, .2, .75, .8), CAMERA)
        qwen = detection_bbox_to_pixels((250, 200, 750, 800), CAMERA)
        self.assertEqual((160.0, 96.0, 480.0, 384.0), fraction)
        self.assertEqual(fraction, qwen)

    def test_detection_bbox_supports_large_image_pixels_and_inferred_scale(self) -> None:
        large = CameraIntrinsics(1000, 1000, 960, 540, 1920, 1080)
        self.assertEqual(
            (1100.0, 200.0, 1500.0, 800.0),
            detection_bbox_to_pixels((1100, 200, 1500, 800), large),
        )
        inferred = detection_bbox_to_pixels((500, 500, 1500, 2000), CAMERA)
        self.assertEqual((160.0, 160.0, 480.0, 640.0), inferred)

    def test_detection_label_matching_is_multilingual_but_conservative(self) -> None:
        self.assertTrue(detection_label_matches("red bottle", "bottle"))
        self.assertTrue(detection_label_matches("红色水瓶", "水瓶"))
        self.assertFalse(detection_label_matches("红色杯子", "红色水瓶"))
        self.assertFalse(detection_label_matches("", "bottle"))
