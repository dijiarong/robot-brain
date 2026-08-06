#!/usr/bin/env python3
"""End-to-end native navigation proof with forbidden external stacks blocked."""
from __future__ import annotations

import argparse
import asyncio
import builtins
import json
import math
import sys
import time

FORBIDDEN = ("dimos", "rclpy", "open3d", "reactivex")
_original_import = builtins.__import__


def _blocked_import(name, globals=None, locals=None, fromlist=(), level=0):
    if name.split(".", 1)[0] in FORBIDDEN:
        raise ImportError(f"forbidden dependency requested: {name}")
    return _original_import(name, globals, locals, fromlist, level)


builtins.__import__ = _blocked_import

from config.settings import Settings  # noqa: E402
from robot_brain.actuation.unitree import FakeUnitreeTransport, UnitreeRobot, UnitreeState  # noqa: E402
from robot_brain.navigation import NavigationStatus, RelativeNavigationGoal  # noqa: E402
from robot_brain.navigation.native_go2 import NativeGo2NavigationClient  # noqa: E402
from robot_brain.navigation.sensors import UnitreeNavigationSensorProvider  # noqa: E402
from robot_brain.perception.pointcloud import PointCloudSnapshot  # noqa: E402


class OfflineLidarTransport(FakeUnitreeTransport):
    def __init__(self) -> None:
        super().__init__(UnitreeState(
            connected=True, is_standing=True, pose_frame_id="odom",
            pose_source="unitree_robotodom",
        ))
        self.obstacles = [(0.35, 0.0, 0.25)]

    def read_lidar_snapshot(self) -> PointCloudSnapshot:
        yaw = math.radians(self._state.heading_degrees)
        points = []
        for wx, wy, wz in self.obstacles:
            dx, dy = wx - self._state.position.x, wy - self._state.position.y
            points.append((
                dx * math.cos(yaw) + dy * math.sin(yaw),
                -dx * math.sin(yaw) + dy * math.cos(yaw), wz,
            ))
        points.append((3.0, 3.0, 0.2))
        return PointCloudSnapshot(
            points_xyz=tuple(points), frame_id="base_link",
            sensor_timestamp=time.time(), received_monotonic=time.monotonic(),
            source="offline-verifier", timestamp_valid=True,
        )

    def lidar_age_seconds(self) -> float:
        return 0.0


async def verify() -> dict[str, object]:
    transport = OfflineLidarTransport()
    await transport.connect()
    settings = Settings(
        robot_backend="unitree", navigation_backend="native_go2",
        unitree_dry_run=False, unitree_enable_motion=True,
        unitree_max_speed=0.5, unitree_max_drive_duration=0.5,
        memory_db_path=":memory:",
    )
    robot = UnitreeRobot(transport, settings)
    client = NativeGo2NavigationClient(
        robot, UnitreeNavigationSensorProvider(transport),
        linear_speed_mps=0.5, segment_duration_s=0.2,
        robot_radius_m=0.15, emergency_stop_m=0.12,
        reach_tolerance_m=0.08, settle_s=0.0,
    )
    handle = await client.set_relative_goal(RelativeNavigationGoal(
        forward_m=0.7, max_duration_s=5.0,
    ))
    deadline = time.monotonic() + 6.0
    state = await client.get_state()
    while not state.status.terminal and time.monotonic() < deadline:
        await asyncio.sleep(0.01)
        state = await client.get_state()
    loaded_forbidden = sorted(
        name for name in sys.modules if name.split(".", 1)[0] in FORBIDDEN
    )
    return {
        "ok": handle.accepted and state.status == NavigationStatus.SUCCEEDED and not loaded_forbidden,
        "provider": state.provider,
        "status": state.status.value,
        "stop_reason": state.stop_reason,
        "replan_count": state.replan_count,
        "path_points": len(state.path),
        "final_pose": state.pose.model_dump(mode="json") if state.pose else None,
        "loaded_forbidden_modules": loaded_forbidden,
        "forbidden_dependencies": list(FORBIDDEN),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.parse_args()
    report = asyncio.run(verify())
    print(json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True))
    raise SystemExit(0 if report["ok"] else 1)


if __name__ == "__main__":
    main()
