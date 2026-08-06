from __future__ import annotations

import math
import time
import unittest

from robot_brain.core.robot_self_state import RobotPose
from robot_brain.navigation.map_store import SparseVoxelMap
from robot_brain.navigation.relocalization import (
    merge_local_observation,
    relocalize_global,
    relocalize_with_initial,
)
from robot_brain.perception.pointcloud import PointCloudSnapshot


def _cloud(points):
    return PointCloudSnapshot(
        points_xyz=tuple(points), frame_id="base_link", sensor_timestamp=time.time(),
        received_monotonic=time.monotonic(), source="test", timestamp_valid=True,
    )


def _asymmetric_room():
    points = []
    for index in range(31):
        value = index * 0.1
        points.extend(((value, 0.0, 0.3), (value, 2.0, 0.3)))
    for index in range(21):
        value = index * 0.1
        points.extend(((0.0, value, 0.3), (3.0, value, 0.3)))
    # An interior L removes the room's 180-degree ambiguity.
    for index in range(11):
        points.append((0.7 + index * 0.1, 0.6, 0.3))
    for index in range(8):
        points.append((0.7, 0.6 + index * 0.1, 0.3))
    return points


def _body_view(world_points, pose):
    yaw = math.radians(pose.yaw_deg)
    result = []
    for x, y, z in world_points:
        dx, dy = x - pose.x_m, y - pose.y_m
        bx = dx * math.cos(yaw) + dy * math.sin(yaw)
        by = -dx * math.sin(yaw) + dy * math.cos(yaw)
        if math.hypot(bx, by) <= 2.2:
            result.append((bx, by, z))
    return result


class NativeRelocalizationTests(unittest.TestCase):
    def setUp(self) -> None:
        self.world = _asymmetric_room()
        self.reference = SparseVoxelMap(resolution_m=0.1, map_id="room")
        self.reference.integrate(_cloud(self.world), RobotPose(frame_id="map"))
        self.true_pose = RobotPose(x_m=1.1, y_m=0.8, yaw_deg=20.0, frame_id="map")
        self.local = _cloud(_body_view(self.world, self.true_pose))

    def test_initial_pose_search_recovers_map_pose(self) -> None:
        result = relocalize_with_initial(
            self.reference, self.local,
            RobotPose(x_m=1.3, y_m=0.6, yaw_deg=10.0, frame_id="map"),
            search_radius_m=0.4, yaw_search_deg=20.0,
        )
        self.assertTrue(result.accepted, result)
        assert result.pose is not None
        self.assertAlmostEqual(self.true_pose.x_m, result.pose.x_m, delta=0.12)
        self.assertAlmostEqual(self.true_pose.y_m, result.pose.y_m, delta=0.12)
        self.assertAlmostEqual(self.true_pose.yaw_deg, result.pose.yaw_deg, delta=3.0)
        self.assertGreater(result.fitness, 0.7)

    def test_global_fallback_recovers_without_initial_pose(self) -> None:
        result = relocalize_global(
            self.reference, self.local, xy_step_m=0.4, yaw_step_deg=20.0,
            max_candidates=50_000,
        )
        self.assertTrue(result.accepted, result)
        assert result.pose is not None
        self.assertAlmostEqual(self.true_pose.x_m, result.pose.x_m, delta=0.2)
        self.assertAlmostEqual(self.true_pose.y_m, result.pose.y_m, delta=0.2)
        self.assertAlmostEqual(self.true_pose.yaw_deg, result.pose.yaw_deg, delta=6.0)

    def test_bad_cloud_is_rejected_by_quality_gate(self) -> None:
        unrelated = _cloud(tuple((index * 0.1, index * 0.1, 0.3) for index in range(20)))
        result = relocalize_with_initial(
            self.reference, unrelated, RobotPose(x_m=1.0, y_m=1.0, frame_id="map"),
            min_fitness=0.9,
        )
        self.assertFalse(result.accepted)
        self.assertIsNone(result.pose)

    def test_global_search_budget_fails_closed(self) -> None:
        result = relocalize_global(self.reference, self.local, max_candidates=2)
        self.assertFalse(result.accepted)
        self.assertEqual("global_search_budget_exceeded", result.reason)

    def test_localized_observation_merges_into_saved_map(self) -> None:
        before = self.reference.voxel_count
        added = merge_local_observation(
            self.reference, _cloud(((0.0, 0.0, 0.3),)),
            RobotPose(x_m=1.5, y_m=1.5, yaw_deg=0.0, frame_id="map"),
        )
        self.assertEqual(1, added)
        self.assertGreaterEqual(self.reference.voxel_count, before)

    def test_merge_requires_map_frame(self) -> None:
        with self.assertRaisesRegex(ValueError, "map frame"):
            merge_local_observation(
                self.reference, self.local, RobotPose(frame_id="odom")
            )
