from __future__ import annotations

import json
import tempfile
import time
import unittest
from pathlib import Path

from robot_brain.core.robot_self_state import RobotPose
from robot_brain.navigation.map_store import SparseVoxelMap, _payload_hash
from robot_brain.perception.pointcloud import PointCloudSnapshot


def _cloud(points):
    return PointCloudSnapshot(
        points_xyz=tuple(points), frame_id="base_link", sensor_timestamp=time.time(),
        received_monotonic=time.monotonic(), source="test", timestamp_valid=True,
    )


class SparseVoxelMapTests(unittest.TestCase):
    def test_integrates_body_cloud_in_map_frame(self) -> None:
        voxel_map = SparseVoxelMap(resolution_m=0.1, map_id="office")
        touched = voxel_map.integrate(
            _cloud(((1.0, 0.0, 0.2),)),
            RobotPose(x_m=2.0, y_m=3.0, yaw_deg=90.0, frame_id="odom"),
        )
        self.assertEqual(1, touched)
        x, y, z = voxel_map.points()[0]
        self.assertAlmostEqual(2.05, x)
        self.assertAlmostEqual(4.05, y)
        self.assertAlmostEqual(0.25, z)

    def test_repeated_observations_increment_hits_and_filter_noise(self) -> None:
        voxel_map = SparseVoxelMap(resolution_m=0.1)
        pose = RobotPose(frame_id="odom")
        voxel_map.integrate(_cloud(((1.0, 0.0, 0.2), (2.0, 0.0, 0.2))), pose)
        voxel_map.integrate(_cloud(((1.0, 0.0, 0.2),)), pose)
        self.assertEqual(2, voxel_map.voxel_count)
        self.assertEqual(1, len(voxel_map.points(min_hits=2)))

    def test_body_cloud_round_trip(self) -> None:
        voxel_map = SparseVoxelMap(resolution_m=0.1)
        pose = RobotPose(x_m=1.0, y_m=2.0, yaw_deg=45.0, frame_id="odom")
        voxel_map.integrate(_cloud(((0.5, 0.0, 0.2),)), pose)
        cloud = voxel_map.body_cloud(pose, radius_m=1.0)
        self.assertEqual("base_link", cloud.frame_id)
        self.assertAlmostEqual(0.5, cloud.points_xyz[0][0], delta=0.08)
        self.assertAlmostEqual(0.0, cloud.points_xyz[0][1], delta=0.08)

    def test_atomic_save_load_preserves_identity(self) -> None:
        voxel_map = SparseVoxelMap(resolution_m=0.1, map_id="office")
        voxel_map.integrate(_cloud(((1.0, 0.0, 0.2),)), RobotPose(frame_id="odom"))
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "office.native-map.json"
            before = voxel_map.save(path)
            loaded = SparseVoxelMap.load(path)
            after = loaded.identity()
        self.assertEqual(before, after)
        self.assertEqual(voxel_map.points(), loaded.points())

    def test_map_version_is_stable_while_content_revision_changes(self) -> None:
        voxel_map = SparseVoxelMap(resolution_m=0.1, map_id="office")
        before = voxel_map.identity()
        voxel_map.integrate(
            _cloud(((1.0, 0.0, 0.2),)), RobotPose(frame_id="odom")
        )
        after = voxel_map.identity()
        self.assertEqual(before.map_id, after.map_id)
        self.assertEqual(before.version, after.version)
        self.assertNotEqual(before.revision, after.revision)

    def test_tampered_map_is_rejected(self) -> None:
        voxel_map = SparseVoxelMap(map_id="office")
        voxel_map.integrate(_cloud(((1.0, 0.0, 0.2),)), RobotPose(frame_id="odom"))
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "map.json"
            voxel_map.save(path)
            payload = json.loads(path.read_text())
            payload["voxels"][0][3] = 999
            path.write_text(json.dumps(payload))
            with self.assertRaisesRegex(ValueError, "hash mismatch"):
                SparseVoxelMap.load(path)

    def test_valid_schema_v1_map_loads_with_stable_legacy_version(self) -> None:
        payload = {
            "schema_version": 1, "map_id": "old-office", "resolution_m": 0.1,
            "frame_id": "map", "max_voxels": 100,
            "voxels": [[1, 2, 3, 1]], "known_free_xy": [[0, 0]],
        }
        payload["content_hash"] = _payload_hash(payload)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "legacy.json"
            path.write_text(json.dumps(payload))
            loaded = SparseVoxelMap.load(path)
        self.assertEqual("old-office", loaded.identity().map_id)
        self.assertTrue(loaded.identity().version.startswith("legacy-"))

    def test_free_space_carving_removes_stale_obstacle_after_confirmations(self) -> None:
        voxel_map = SparseVoxelMap(resolution_m=0.1)
        pose = RobotPose(frame_id="odom")
        voxel_map.integrate(_cloud(((0.5, 0.0, 0.2),)), pose)
        before = voxel_map.voxel_count
        for _ in range(3):
            voxel_map.integrate(
                _cloud(((1.0, 0.0, 0.4),)), pose,
                carve_free_space=True, carve_misses=3,
            )
        self.assertLess(voxel_map.voxel_count, before + 1)

    def test_map_enforces_voxel_memory_limit(self) -> None:
        voxel_map = SparseVoxelMap(resolution_m=0.1, max_voxels=5)
        voxel_map.integrate(
            _cloud(tuple((index * 0.2, 0.0, 0.2) for index in range(20))),
            RobotPose(frame_id="odom"),
        )
        self.assertEqual(5, voxel_map.voxel_count)

    def test_persisted_free_space_builds_map_aligned_exploration_grid(self) -> None:
        voxel_map = SparseVoxelMap(resolution_m=0.1)
        pose = RobotPose(frame_id="odom")
        voxel_map.integrate(
            _cloud(((1.0, 0.0, 0.3), (0.0, 1.0, 0.3))), pose,
            carve_free_space=True,
        )
        grid = voxel_map.occupancy_grid(
            center_x_m=0.0, center_y_m=0.0, size_m=3.0, frame_id="odom",
        )
        robot_cell = grid.world_to_cell(0.0, 0.0)
        self.assertEqual("odom", grid.frame_id)
        self.assertIn(robot_cell, grid.known_free)
        self.assertTrue(grid.occupied)

    def test_local_cylinder_excludes_far_voxels_and_enforces_output_budget(self) -> None:
        voxel_map = SparseVoxelMap(resolution_m=0.1)
        voxel_map.integrate(
            _cloud(tuple((index * 0.2, 0.0, 0.2) for index in range(20))),
            RobotPose(frame_id="odom"),
        )
        local = voxel_map.points_in_cylinder(
            center_x_m=0.0, center_y_m=0.0, radius_m=0.6,
            z_min_m=-1.0, z_max_m=1.0,
        )
        self.assertGreater(len(local), 0)
        self.assertTrue(all(abs(point[0]) <= 0.7 for point in local))
        with self.assertRaisesRegex(ValueError, "point budget"):
            voxel_map.points_in_cylinder(
                center_x_m=2.0, center_y_m=0.0, radius_m=5.0,
                z_min_m=-1.0, z_max_m=1.0, max_points=2,
            )

        viewer = voxel_map.viewer_points_in_cylinder(
            center_x_m=2.0, center_y_m=0.0, radius_m=5.0,
            z_min_m=-1.0, z_max_m=1.0, max_points=2,
        )
        self.assertEqual(2, len(viewer))
        overview = voxel_map.viewer_overview_points(
            center_x_m=0.0, center_y_m=0.0,
            z_min_m=-1.0, z_max_m=1.0, max_points=8,
        )
        self.assertLessEqual(len(overview), 8)
        self.assertTrue(any(point[0] > 1.0 for point in overview))
        self.assertTrue(any(point[3] > voxel_map.resolution_m for point in overview))
