from __future__ import annotations

import unittest

from robot_brain.actuation.unitree import FakeUnitreeTransport, UnitreeState
from robot_brain.core.world_state import Position
from robot_brain.navigation.sensors import UnitreeNavigationSensorProvider
from robot_brain.perception.pointcloud import PointCloudSnapshot


class _SensorTransport(FakeUnitreeTransport):
    def __init__(
        self,
        *,
        frame_id: str = "base_link",
        cloud_age: float = 0.1,
        origin_xyz=None,
    ) -> None:
        super().__init__(UnitreeState(
            connected=True,
            is_standing=True,
            position=Position(x=1.0, y=2.0),
            heading_degrees=30.0,
            pose_frame_id="odom",
            pose_source="unitree_robotodom",
        ))
        self.frame_id = frame_id
        self.cloud_age = cloud_age
        self.origin_xyz = origin_xyz

    def read_lidar_snapshot(self) -> PointCloudSnapshot:
        return PointCloudSnapshot(
            points_xyz=((0.5, 0.0, 0.2),),
            frame_id=self.frame_id,
            sensor_timestamp=10.0,
            received_monotonic=20.0,
            source="test",
            timestamp_valid=True,
            origin_xyz=self.origin_xyz,
        )

    def lidar_age_seconds(self) -> float:
        return self.cloud_age


class NavigationSensorProviderTests(unittest.IsolatedAsyncioTestCase):
    async def test_fresh_base_frame_pose_and_cloud_are_ready(self) -> None:
        transport = _SensorTransport()
        await transport.connect()
        snapshot = await UnitreeNavigationSensorProvider(transport).get_snapshot()

        self.assertTrue(snapshot.ready)
        self.assertEqual("odom", snapshot.pose.frame_id)  # type: ignore[union-attr]
        self.assertEqual(1.0, snapshot.pose.x_m)  # type: ignore[union-attr]
        self.assertEqual("base_link", snapshot.obstacle_frame)

    async def test_world_frame_cloud_is_not_assumed_robot_relative(self) -> None:
        transport = _SensorTransport(frame_id="world")
        await transport.connect()
        snapshot = await UnitreeNavigationSensorProvider(transport).get_snapshot()

        self.assertFalse(snapshot.obstacle_data_ready)
        self.assertEqual("untrusted_obstacle_frame", snapshot.reason)

    async def test_unitree_world_cloud_is_normalized_when_origin_matches_odom(self) -> None:
        transport = _SensorTransport(frame_id="world", origin_xyz=(1.0, 2.0, 0.0))
        await transport.connect()
        snapshot = await UnitreeNavigationSensorProvider(transport).get_snapshot()

        self.assertTrue(snapshot.obstacle_data_ready)
        self.assertEqual("base_link", snapshot.pointcloud.frame_id)  # type: ignore[union-attr]
        self.assertIn("world_to_base", snapshot.pointcloud.source)  # type: ignore[union-attr]

    async def test_world_cloud_origin_mismatch_fails_closed(self) -> None:
        transport = _SensorTransport(frame_id="world", origin_xyz=(10.0, 10.0, 0.0))
        await transport.connect()
        snapshot = await UnitreeNavigationSensorProvider(transport).get_snapshot()

        self.assertFalse(snapshot.ready)
        self.assertEqual("untrusted_obstacle_frame", snapshot.reason)

    async def test_stale_cloud_fails_closed(self) -> None:
        transport = _SensorTransport(cloud_age=2.0)
        await transport.connect()
        snapshot = await UnitreeNavigationSensorProvider(
            transport, max_pointcloud_age_s=0.5
        ).get_snapshot()

        self.assertFalse(snapshot.ready)
        self.assertEqual("stale_pointcloud", snapshot.reason)

    async def test_sport_pose_fallback_is_diagnostic_only_by_default(self) -> None:
        transport = _SensorTransport()
        transport._state.pose_source = "sport_state"
        await transport.connect()
        snapshot = await UnitreeNavigationSensorProvider(transport).get_snapshot()

        self.assertFalse(snapshot.ready)
        self.assertEqual("authoritative_robotodom_unavailable", snapshot.reason)

        fallback = await UnitreeNavigationSensorProvider(
            transport, require_authoritative_odom=False
        ).get_snapshot()
        self.assertTrue(fallback.ready)
