#!/usr/bin/env python3
"""Verify built-in Go2 LiDAR/odom and optionally one bounded local goal."""
from __future__ import annotations

import argparse
import asyncio
import json
import time

from config.settings import Settings
from robot_brain.actuation.unitree import UnitreeRobot
from robot_brain.actuation.unitree_webrtc import create_webrtc_transport
from robot_brain.navigation import NavigationStatus, RelativeNavigationGoal
from robot_brain.navigation.direct_go2 import DirectGo2NavigationClient
from robot_brain.navigation.sensors import UnitreeNavigationSensorProvider


CONFIRMATION = "I_UNDERSTAND_DIRECT_GO2_NAV"


async def verify(args: argparse.Namespace) -> int:
    if args.live and args.confirm != CONFIRMATION:
        print(json.dumps({
            "ok": False,
            "error": "live confirmation missing",
            "required_confirmation": CONFIRMATION,
        }, ensure_ascii=False, indent=2))
        return 5
    settings = Settings(
        robot_backend="unitree",
        perception_backend="unitree",
        navigation_backend="direct_go2",
        unitree_transport="webrtc",
        unitree_dry_run=not args.live,
        unitree_enable_motion=args.live,
    )
    transport = create_webrtc_transport(settings)
    robot = UnitreeRobot(transport, settings)
    report: dict[str, object] = {
        "provider": "direct_go2",
        "mode": "live" if args.live else "read_only",
        "robot_ip": settings.unitree_robot_ip or None,
    }
    try:
        await transport.connect()
        sensors = UnitreeNavigationSensorProvider(
            transport,
            max_pose_age_s=settings.odom_max_age_seconds,
            max_pointcloud_age_s=settings.direct_nav_pointcloud_max_age_s,
            require_authoritative_odom=settings.direct_nav_require_robotodom,
        )
        client = DirectGo2NavigationClient(
            robot,
            sensors,
            segment_duration_s=settings.direct_nav_segment_duration_s,
            obstacle_stop_m=settings.direct_nav_obstacle_stop_m,
            obstacle_half_width_m=settings.direct_nav_obstacle_half_width_m,
            min_progress_m=settings.odom_progress_min_m,
            min_progress_yaw_deg=settings.odom_progress_min_yaw_deg,
            max_no_progress_segments=settings.direct_nav_no_progress_segments,
        )
        deadline = time.monotonic() + args.sensor_timeout_s
        snapshot = await sensors.get_snapshot()
        while not snapshot.ready and time.monotonic() < deadline:
            await asyncio.sleep(0.1)
            snapshot = await sensors.get_snapshot()
        report["sensor_snapshot"] = {
            "ready": snapshot.ready,
            "reason": snapshot.reason,
            "pose": snapshot.pose.model_dump(mode="json") if snapshot.pose else None,
            "pose_age_seconds": snapshot.pose_age_seconds,
            "pose_source": snapshot.pose_source,
            "pointcloud_age_seconds": snapshot.pointcloud_age_seconds,
            "point_count": snapshot.pointcloud.point_count if snapshot.pointcloud else 0,
            "obstacle_frame": snapshot.obstacle_frame,
            "pointcloud_source": snapshot.pointcloud.source if snapshot.pointcloud else None,
            "sensor_timestamp_valid": (
                snapshot.pointcloud.timestamp_valid if snapshot.pointcloud else False
            ),
        }
        report["transport_health"] = transport.health.__dict__
        localization = await client.get_localization_state()
        report["localization"] = localization.model_dump(mode="json")
        if not snapshot.ready:
            report["ok"] = False
            report["error"] = snapshot.reason or "navigation sensors unavailable"
            print(json.dumps(report, ensure_ascii=False, indent=2, default=str))
            return 2
        if not args.live:
            report["ok"] = True
            print(json.dumps(report, ensure_ascii=False, indent=2, default=str))
            return 0

        goal = RelativeNavigationGoal(
            forward_m=args.forward_m,
            left_m=args.left_m,
            yaw_degrees=args.yaw_degrees,
            max_duration_s=args.timeout_s,
        )
        handle = await client.set_relative_goal(goal)
        report["goal"] = goal.model_dump(mode="json")
        report["goal_handle"] = handle.model_dump(mode="json")
        samples: list[dict[str, object]] = []
        state = await client.get_state()
        while not state.status.terminal:
            samples.append(state.model_dump(mode="json"))
            await asyncio.sleep(args.poll_interval_s)
            state = await client.get_state()
        samples.append(state.model_dump(mode="json"))
        report["samples"] = samples
        report["final_state"] = state.model_dump(mode="json")
        report["ok"] = state.status == NavigationStatus.SUCCEEDED
        print(json.dumps(report, ensure_ascii=False, indent=2, default=str))
        return 0 if report["ok"] else 4
    except Exception as exc:
        report["ok"] = False
        report["error"] = str(exc)
        print(json.dumps(report, ensure_ascii=False, indent=2, default=str))
        return 3
    finally:
        try:
            await robot.stop("direct Go2 verification finished")
        finally:
            await transport.disconnect()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Read Go2 built-in navigation sensors; optionally execute one local goal."
    )
    parser.add_argument("--live", action="store_true")
    parser.add_argument("--confirm", default="")
    parser.add_argument("--forward-m", type=float, default=0.1)
    parser.add_argument("--left-m", type=float, default=0.0)
    parser.add_argument("--yaw-degrees", type=float, default=0.0)
    parser.add_argument("--timeout-s", type=float, default=8.0)
    parser.add_argument("--sensor-timeout-s", type=float, default=8.0)
    parser.add_argument("--poll-interval-s", type=float, default=0.1)
    return parser.parse_args()


def main() -> None:
    raise SystemExit(asyncio.run(verify(parse_args())))


if __name__ == "__main__":
    main()
