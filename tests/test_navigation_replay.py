from __future__ import annotations

import tempfile
import time
import unittest
from pathlib import Path
import math

from robot_brain.core.robot_self_state import RobotPose
from robot_brain.navigation.replay import (
    NavigationReplayFrame, NavigationReplayWriter,
    evaluate_replay_mapping, evaluate_replay_pose_graph, evaluate_replay_relocalization,
    evaluate_replay_planning,
    load_navigation_replay,
)
from robot_brain.navigation.map_store import SparseVoxelMap
from robot_brain.navigation.pose_graph import PoseGraphTrackerConfig
from robot_brain.navigation.sensors import NavigationSensorSnapshot
from robot_brain.perception.pointcloud import PointCloudSnapshot


def _snapshot(points):
    return NavigationSensorSnapshot(
        pose=RobotPose(frame_id="odom", timestamp=time.time()),
        pointcloud=PointCloudSnapshot(
            points_xyz=tuple(points), frame_id="base_link",
            sensor_timestamp=time.time(), received_monotonic=time.monotonic(),
            source="test", timestamp_valid=True,
        ),
        pose_age_seconds=0.01, pointcloud_age_seconds=0.02,
        pose_ready=True, obstacle_data_ready=True, obstacle_frame="base_link",
    )


class NavigationReplayTests(unittest.TestCase):
    def test_replay_round_trip_and_planner_evaluation(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "nav-replay.jsonl.gz"
            writer = NavigationReplayWriter(path)
            self.assertTrue(writer.record(_snapshot(((2.0, 2.0, 0.2),))))
            self.assertTrue(writer.record(_snapshot(((0.5, 0.0, 0.2),))))
            frames = load_navigation_replay(path)
            report = evaluate_replay_planning(
                frames, goal_body_xy=(1.0, 0.0), robot_radius_m=0.2,
            )
        self.assertEqual(2, len(frames))
        self.assertEqual(2, report["frames"])
        self.assertGreaterEqual(report["paths_found"], 1)
        mapping = evaluate_replay_mapping(frames, max_voxels=100)
        self.assertTrue(mapping["ok"], mapping)
        self.assertEqual(2, mapping["frames_integrated"])
        self.assertLessEqual(mapping["voxel_count"], 100)
        self.assertTrue(mapping["map_identity"]["content_revision"])

    def test_unready_snapshot_is_not_recorded(self) -> None:
        snapshot = _snapshot(((1.0, 0.0, 0.2),))
        snapshot = NavigationSensorSnapshot(
            **{**snapshot.__dict__, "obstacle_data_ready": False, "reason": "stale_pointcloud"}
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "empty.gz"
            self.assertFalse(NavigationReplayWriter(path).record(snapshot))
            self.assertFalse(path.exists())

    def test_replay_writer_refuses_to_mix_with_existing_session(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "existing.jsonl.gz"
            first = NavigationReplayWriter(path)
            self.assertTrue(first.record(_snapshot(((1, 0, .2),))))
            with self.assertRaises(FileExistsError):
                NavigationReplayWriter(path).record(_snapshot(((2, 0, .2),)))

    def test_replay_relocalization_emits_quality_and_map_identity(self) -> None:
        world = tuple((x, y, .3) for x, y in (
            (0, 0), (.2, .1), (.4, -.1), (.7, .3), (1, 0), (.1, .8),
            (.5, 1.1), (1.2, .7), (1.5, .2), (1.4, 1.3), (.3, 1.7),
            (1, 1.9), (1.8, 1.7), (2, .8), (2.2, .1),
        ))
        reference = SparseVoxelMap(resolution_m=.1, map_id="replay-room")
        reference.integrate(_snapshot(world).pointcloud, RobotPose(frame_id="map"))
        pose = RobotPose(x_m=.5, y_m=.4, yaw_deg=10, frame_id="map", timestamp=1)
        body = _body_points(world, pose)
        frame = NavigationReplayFrame(0, pose, body, .01, .01, "base_link")
        report = evaluate_replay_relocalization(
            [frame], reference,
            initial_map_pose=RobotPose(x_m=.6, y_m=.3, yaw_deg=5, frame_id="map"),
        )
        self.assertTrue(report["ok"], report)
        self.assertGreater(report["fitness"], .45)
        self.assertEqual("replay-room", report["map_identity"]["map_id"])

    def test_pose_graph_replay_requires_an_accepted_verified_loop(self) -> None:
        landmarks = tuple((x, y, .5) for x, y in (
            (0, 0), (.2, .1), (.4, -.1), (.7, .3), (1, 0), (.1, .8),
            (.5, 1.1), (1.2, .7), (1.5, .2), (1.4, 1.3), (.3, 1.7),
            (1, 1.9), (1.8, 1.7), (2, .8), (2.2, .1),
        ))
        raw_poses = ((0, 0, 0), (2, 0, 90), (2, 2, 180),
                     (0, 2, -90), (.3, .2, 4))
        frames = []
        for sequence, raw in enumerate(raw_poses):
            scan_pose = raw_poses[0] if sequence == 4 else raw
            pose = RobotPose(x_m=raw[0], y_m=raw[1], yaw_deg=raw[2],
                             frame_id="odom", timestamp=float(sequence+1))
            scan = _body_points(landmarks, RobotPose(
                x_m=scan_pose[0], y_m=scan_pose[1], yaw_deg=scan_pose[2], frame_id="odom",
            ))
            frames.append(NavigationReplayFrame(sequence, pose, scan, .01, .01, "base_link"))
        report = evaluate_replay_pose_graph(
            frames,
            config=PoseGraphTrackerConfig(
                keyframe_translation_m=.1, keyframe_yaw_degrees=5,
                loop_search_radius_m=.8, minimum_loop_age_s=3,
                minimum_keyframes_for_loop=4,
            ),
            verification_overrides={
                "translation_radius_m": .5, "yaw_radius_degrees": 10,
                "translation_step_m": .1, "yaw_step_degrees": 2,
                "minimum_score_margin": 0,
            },
        )
        self.assertTrue(report["ok"], report)
        self.assertEqual(1, report["accepted_loops"])
        accepted = [row for row in report["events"] if row["loop_accepted"]]
        self.assertLess(accepted[0]["optimized_graph_rmse"], accepted[0]["initial_graph_rmse"])


def _body_points(world, pose):
    yaw = math.radians(pose.yaw_deg)
    return tuple(
        ((x-pose.x_m)*math.cos(yaw)+(y-pose.y_m)*math.sin(yaw),
         -(x-pose.x_m)*math.sin(yaw)+(y-pose.y_m)*math.cos(yaw), z)
        for x, y, z in world
    )
