from __future__ import annotations

import unittest
import json
import tempfile
from pathlib import Path

from robot_brain.navigation import NavigationStatus
from scripts.verify_native_go2_navigation import (
    _emit, _maximum_path_deviation, _preflight_report_path,
    _read_only_planning_probe, _scenario_passed, _trajectory_metrics,
)
from config.settings import Settings
from robot_brain.navigation.sensors import NavigationSensorSnapshot
from robot_brain.perception.pointcloud import PointCloudSnapshot
from robot_brain.core.robot_self_state import RobotPose


class NativeLiveAcceptanceLogicTests(unittest.TestCase):
    def test_read_only_planning_probe_records_exact_grid_and_path(self) -> None:
        cloud = PointCloudSnapshot(
            points_xyz=((0.5, 0.0, 0.2),), frame_id="base_link",
            sensor_timestamp=1.0, received_monotonic=1.0, source="test",
            timestamp_valid=True,
        )
        snapshot = NavigationSensorSnapshot(
            pose=None, pointcloud=cloud, pose_age_seconds=0.0,
            pointcloud_age_seconds=0.0, pose_ready=False,
            obstacle_data_ready=True, obstacle_frame="base_link",
        )
        probe = _read_only_planning_probe(
            snapshot,
            Settings(native_nav_map_size_m=3.0, native_nav_resolution_m=0.1,
                     native_nav_robot_radius_m=0.2),
            1.0, 0.0,
        )
        self.assertIsNotNone(probe)
        self.assertGreater(probe["occupied_count"], 1)
        self.assertIsNotNone(probe["path_xy"])

    def test_report_path_preflight_rejects_existing_file(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "report.json"
            path.write_text("existing", encoding="utf-8")
            with self.assertRaises(FileExistsError):
                _preflight_report_path(str(path))

    def test_report_path_preflight_checks_parent_without_creating_report(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "nested" / "report.json"
            _preflight_report_path(str(path))
            self.assertTrue(path.parent.is_dir())
            self.assertFalse(path.exists())

    def test_obstacle_requires_geometric_detour_not_just_replans(self) -> None:
        straight = [{"event": "plan_geometry", "path_xy": [[0, 0], [1, 0]]}]
        detour = [{"event": "plan_geometry", "path_xy": [[0, 0], [0.5, 0.2], [1, 0]]}]
        self.assertFalse(_scenario_passed(
            "obstacle", NavigationStatus.SUCCEEDED, straight, [],
            _maximum_path_deviation(straight, (0, 0), (1, 0)),
            stop_reason="goal_reached",
            trajectory={"forward_progress_m": 1, "maximum_lateral_deviation_m": 0},
            requested_distance_m=1,
        ))
        self.assertTrue(_scenario_passed(
            "obstacle", NavigationStatus.SUCCEEDED,
            detour + [{"event": "motion_sample", "x_m": .5, "y_m": .2}], [],
            _maximum_path_deviation(detour, (0, 0), (1, 0)),
            stop_reason="goal_reached",
            trajectory={"forward_progress_m": 1, "maximum_lateral_deviation_m": .2},
            requested_distance_m=1,
        ))

    def test_sudden_block_requires_emergency_event_and_stop_command(self) -> None:
        trace = [{"event": "emergency_stop"}]
        self.assertFalse(_scenario_passed(
            "sudden_block", NavigationStatus.FAILED, trace, [], 0.0,
        ))
        self.assertTrue(_scenario_passed(
            "sudden_block", NavigationStatus.FAILED, trace,
            [{"action": "stop", "reason": "obstacle entered emergency corridor"}], 0.0,
        ))

    def test_cancel_and_stuck_require_exact_terminal_status(self) -> None:
        self.assertTrue(_scenario_passed(
            "cancel", NavigationStatus.CANCELED, [],
            [{"action": "stop", "reason": "native navigation canceled"}], 0.0,
            stop_reason="canceled", cancel_latency_s=.1,
        ))
        self.assertTrue(_scenario_passed(
            "stuck", NavigationStatus.NO_PROGRESS, [],
            [{"action": "stop", "reason": "native navigation made no progress"}], 0.0,
            stop_reason="no_progress",
        ))
        self.assertFalse(_scenario_passed("stuck", NavigationStatus.TIMED_OUT, [], [], 0.0))

    def test_trajectory_metrics_use_observed_odometry_not_planned_path(self) -> None:
        pose = RobotPose(x_m=10, y_m=20, yaw_deg=90, frame_id="odom")
        metrics = _trajectory_metrics([
            {"event": "plan_geometry", "path_xy": [[10, 20], [9, 21]]},
            {"event": "motion_sample", "x_m": 10, "y_m": 20.4},
            {"event": "motion_sample", "x_m": 9.8, "y_m": 20.8},
        ], pose, 1, 0)
        self.assertAlmostEqual(.8, metrics["forward_progress_m"])
        self.assertAlmostEqual(.2, metrics["maximum_lateral_deviation_m"])
        self.assertEqual(2, metrics["motion_samples"])

    def test_report_is_strict_json_and_refuses_overwrite(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "report.json"
            _emit({"ready": False, "age": float("inf")}, str(path))
            raw = path.read_text()
            parsed = json.loads(raw)
            self.assertIsNone(parsed["age"])
            self.assertNotIn("Infinity", raw)
            with self.assertRaises(FileExistsError):
                _emit({"ready": True}, str(path))
