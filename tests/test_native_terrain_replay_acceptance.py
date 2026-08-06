from __future__ import annotations

import tempfile
import time
import unittest
from pathlib import Path

from robot_brain.core.robot_self_state import RobotPose
from robot_brain.navigation import SurfaceNode, SurfacePath
from robot_brain.navigation.replay import NavigationReplayWriter
from robot_brain.navigation.sensors import NavigationSensorSnapshot
from robot_brain.perception.pointcloud import PointCloudSnapshot
from scripts.verify_native_terrain3d import _path_safety, load_input_xyz


class NativeTerrainReplayAcceptanceTests(unittest.TestCase):
    def test_native_replay_points_are_transformed_into_world_frame(self) -> None:
        snapshot = NavigationSensorSnapshot(
            pose=RobotPose(x_m=10, y_m=20, z_m=1, yaw_deg=90,
                           frame_id="odom", timestamp=time.time()),
            pointcloud=PointCloudSnapshot(
                points_xyz=((1, 0, .5),), frame_id="base_link",
                sensor_timestamp=time.time(), received_monotonic=time.monotonic(),
                source="test", timestamp_valid=True,
            ),
            pose_age_seconds=.01, pointcloud_age_seconds=.01,
            pose_ready=True, obstacle_data_ready=True, obstacle_frame="base_link",
        )
        with tempfile.TemporaryDirectory() as raw:
            path = Path(raw)/"terrain.jsonl.gz"
            NavigationReplayWriter(path).record(snapshot)
            points, kind = load_input_xyz(path)
        self.assertEqual("native_navigation_replay", kind)
        self.assertAlmostEqual(10, points[0][0])
        self.assertAlmostEqual(21, points[0][1])
        self.assertAlmostEqual(1.5, points[0][2])

    def test_path_safety_reports_step_slope_clearance_and_cost_failures(self) -> None:
        path = SurfacePath((
            SurfaceNode(0, 0, 0, (0, 0, 0), wall_clearance_m=.5),
            SurfaceNode(.2, 0, .3, (1, 0, 1), wall_clearance_m=.05,
                        traversal_cost=float("inf")),
        ), .36, .3, minimum_clearance_m=.05)
        report = _path_safety(
            path, max_step_m=.16, max_slope_degrees=25,
            required_clearance_m=.1,
        )
        self.assertFalse(report["ok"])
        self.assertEqual({"step_limit_exceeded", "slope_limit_exceeded",
                          "non_traversable_path_node",
                          "wall_clearance_not_strictly_satisfied"},
                         set(report["failures"]))
