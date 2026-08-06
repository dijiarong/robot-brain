from __future__ import annotations

import unittest

from robot_brain.navigation import NavigationPose, SparseVoxelMap
from robot_brain.navigation.native_go2 import NativeGo2NavigationClient
from tests.test_native_navigation import _WorldCloudTransport, _client
from tests.test_native_relocalization import _asymmetric_room, _body_view, _cloud
from robot_brain.core.robot_self_state import RobotPose


class NativeProviderRelocalizationTests(unittest.IsolatedAsyncioTestCase):
    async def test_relocalization_enables_persistent_absolute_goals(self) -> None:
        world = _asymmetric_room()
        reference = SparseVoxelMap(resolution_m=0.1, map_id="office")
        reference.integrate(_cloud(world), RobotPose(frame_id="map"))
        map_robot_pose = RobotPose(x_m=1.1, y_m=0.8, yaw_deg=20.0, frame_id="map")
        local_points = _body_view(world, map_robot_pose)
        transport = _WorldCloudTransport()
        transport.obstacles = local_points
        client = await _client(
            transport, voxel_map=reference, persistent_map=True,
        )
        self.assertIsInstance(client, NativeGo2NavigationClient)
        self.assertTrue(client.supports_absolute_goals)
        self.assertFalse((await client.get_localization_state()).usable_for_persistent_memory)

        result = await client.relocalize(
            NavigationPose(x_m=1.2, y_m=0.7, yaw_degrees=15.0, frame_id="map")
        )
        localization = await client.get_localization_state()

        self.assertTrue(result.accepted, result)
        self.assertTrue(client.supports_absolute_goals)
        self.assertTrue(localization.usable_for_persistent_memory)
        assert localization.pose is not None
        self.assertAlmostEqual(map_robot_pose.x_m, localization.pose.x_m, delta=0.15)
        self.assertAlmostEqual(map_robot_pose.y_m, localization.pose.y_m, delta=0.15)

    async def test_failed_relocalization_keeps_absolute_goals_disabled(self) -> None:
        reference = SparseVoxelMap(resolution_m=0.1, map_id="office")
        reference.integrate(
            _cloud(_asymmetric_room()), RobotPose(frame_id="map")
        )
        transport = _WorldCloudTransport()
        transport.obstacles = [(index * 0.1, index * 0.1, 0.3) for index in range(4)]
        client = await _client(transport, voxel_map=reference, persistent_map=True)
        result = await client.relocalize(
            NavigationPose(x_m=1.0, y_m=1.0, frame_id="map")
        )
        self.assertFalse(result.accepted)
        self.assertTrue(client.supports_absolute_goals)
        self.assertFalse((await client.get_localization_state()).usable_for_persistent_memory)
