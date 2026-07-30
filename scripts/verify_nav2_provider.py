#!/usr/bin/env python3
"""Verify robot-brain against the topsun-bot/Navigation ROS2 graph.

Read-only by default: action readiness, odom pose, localization, map identity.
Optional ``--live`` relative goal, or ``--live-absolute`` map-frame goal (phase 2).
"""
from __future__ import annotations

import argparse
import asyncio
import json
import math
import time

from config.settings import Settings
from robot_brain.navigation import (
    AbsoluteNavigationGoal,
    NavigationPose,
    NavigationStatus,
    RelativeNavigationGoal,
)
from robot_brain.navigation.nav2 import create_nav2_navigation_client


async def verify(args: argparse.Namespace) -> int:
    settings = Settings(navigation_backend="nav2")
    client = create_nav2_navigation_client(settings)
    report: dict[str, object] = {
        "provider": "nav2",
        "mode": (
            "live_absolute"
            if args.live_absolute
            else ("live" if args.live else "read_only")
        ),
        "action": settings.nav2_action_name,
        "odom_topic": settings.nav2_odom_topic,
        "goal_frame": settings.nav2_goal_frame,
        "map_frame": settings.nav2_map_frame,
        "configured_map_id": settings.nav2_map_id or None,
        "configured_map_version": settings.nav2_map_version,
        "supports_absolute_goals": client.supports_absolute_goals,
    }
    try:
        initial = await client.get_state()
        report["initial_state"] = initial.model_dump(mode="json")
        localization = await client.get_localization_state()
        report["localization"] = localization.model_dump(mode="json")
        report["usable_for_persistent_memory"] = localization.usable_for_persistent_memory

        if not initial.ready or initial.pose is None:
            report["ok"] = False
            report["error"] = "Nav2 action or odometry pose is unavailable"
            print(json.dumps(report, ensure_ascii=False, indent=2))
            return 2

        if not args.live and not args.live_absolute:
            # Phase-2 read-only gate: localization + identity when MAP_ID is set.
            if settings.nav2_map_id and not localization.usable_for_persistent_memory:
                report["ok"] = False
                report["error"] = (
                    "map_id configured but localization is not usable for "
                    "persistent memory (need localized pose in map frame)"
                )
                print(json.dumps(report, ensure_ascii=False, indent=2))
                return 5
            report["ok"] = True
            print(json.dumps(report, ensure_ascii=False, indent=2))
            return 0

        if args.live_absolute:
            if not client.supports_absolute_goals:
                report["ok"] = False
                report["error"] = "set RDB_NAV2_MAP_ID before --live-absolute"
                print(json.dumps(report, ensure_ascii=False, indent=2))
                return 6
            if not localization.usable_for_persistent_memory or localization.pose is None:
                report["ok"] = False
                report["error"] = "localization not ready for absolute goal"
                print(json.dumps(report, ensure_ascii=False, indent=2))
                return 5
            # Body-frame offset from current pose (dx forward, dy left), then express in map.
            # World-frame +x would reverse when the dog's yaw is ~90–180° (seen in phase-2).
            yaw = math.radians(localization.pose.yaw_degrees)
            c, s = math.cos(yaw), math.sin(yaw)
            dx_b, dy_b = args.absolute_dx_m, args.absolute_dy_m
            goal = AbsoluteNavigationGoal(
                map_id=settings.nav2_map_id,
                map_version=settings.nav2_map_version,
                pose=NavigationPose(
                    x_m=localization.pose.x_m + c * dx_b - s * dy_b,
                    y_m=localization.pose.y_m + s * dx_b + c * dy_b,
                    yaw_degrees=localization.pose.yaw_degrees + args.absolute_dyaw_deg,
                    frame_id=settings.nav2_map_frame,
                ),
                max_duration_s=args.timeout_s,
            )
            handle = await client.set_absolute_goal(goal)
            report["goal"] = goal.model_dump(mode="json")
        else:
            goal = RelativeNavigationGoal(
                forward_m=args.forward_m,
                left_m=args.left_m,
                yaw_degrees=args.yaw_degrees,
                max_duration_s=args.timeout_s,
            )
            handle = await client.set_relative_goal(goal)
            report["goal"] = goal.model_dump(mode="json")

        report["goal_handle"] = handle.model_dump(mode="json")
        if not handle.accepted:
            report["ok"] = False
            print(json.dumps(report, ensure_ascii=False, indent=2))
            return 3

        deadline = time.monotonic() + args.timeout_s
        samples: list[dict[str, object]] = []
        state = await client.get_state()
        while not state.status.terminal and time.monotonic() < deadline:
            samples.append(state.model_dump(mode="json"))
            await asyncio.sleep(args.poll_interval_s)
            state = await client.get_state()
        if not state.status.terminal:
            await client.cancel(handle.goal_id)
            state = await client.get_state()
            report["timed_out"] = True
        samples.append(state.model_dump(mode="json"))
        report["samples"] = samples
        report["final_state"] = state.model_dump(mode="json")
        report["ok"] = state.status == NavigationStatus.SUCCEEDED
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 0 if report["ok"] else 4
    finally:
        await client.aclose()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Read Navigation/Nav2 state + localization; optionally one relative "
            "or absolute (map) goal."
        )
    )
    parser.add_argument(
        "--live",
        action="store_true",
        help="Submit one relative navigation goal (moves the robot).",
    )
    parser.add_argument(
        "--live-absolute",
        action="store_true",
        help="Submit one map-frame absolute goal from current pose + offset.",
    )
    parser.add_argument("--forward-m", type=float, default=0.2)
    parser.add_argument("--left-m", type=float, default=0.0)
    parser.add_argument("--yaw-degrees", type=float, default=0.0)
    parser.add_argument("--absolute-dx-m", type=float, default=0.2,
                        help="Body-forward offset (m) for --live-absolute")
    parser.add_argument("--absolute-dy-m", type=float, default=0.0,
                        help="Body-left offset (m) for --live-absolute")
    parser.add_argument("--absolute-dyaw-deg", type=float, default=0.0)
    parser.add_argument("--timeout-s", type=float, default=12.0)
    parser.add_argument("--poll-interval-s", type=float, default=0.25)
    return parser.parse_args()


def main() -> None:
    raise SystemExit(asyncio.run(verify(parse_args())))


if __name__ == "__main__":
    main()
