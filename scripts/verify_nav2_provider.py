#!/usr/bin/env python3
"""Verify robot-brain against the topsun-bot/Navigation ROS2 graph."""
from __future__ import annotations

import argparse
import asyncio
import json
import time

from config.settings import Settings
from robot_brain.navigation import NavigationStatus, RelativeNavigationGoal
from robot_brain.navigation.nav2 import create_nav2_navigation_client


async def verify(args: argparse.Namespace) -> int:
    settings = Settings(navigation_backend="nav2")
    client = create_nav2_navigation_client(settings)
    report: dict[str, object] = {
        "provider": "nav2",
        "mode": "live" if args.live else "read_only",
        "action": settings.nav2_action_name,
        "odom_topic": settings.nav2_odom_topic,
        "goal_frame": settings.nav2_goal_frame,
    }
    try:
        initial = await client.get_state()
        report["initial_state"] = initial.model_dump(mode="json")
        if not initial.ready or initial.pose is None:
            report["ok"] = False
            report["error"] = "Nav2 action or odometry pose is unavailable"
            print(json.dumps(report, ensure_ascii=False, indent=2))
            return 2
        if not args.live:
            report["ok"] = True
            print(json.dumps(report, ensure_ascii=False, indent=2))
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
        description="Read Navigation/Nav2 state; optionally execute one bounded relative goal."
    )
    parser.add_argument(
        "--live",
        action="store_true",
        help="Actually submit a navigation goal. Without this flag the check is read-only.",
    )
    parser.add_argument("--forward-m", type=float, default=0.2)
    parser.add_argument("--left-m", type=float, default=0.0)
    parser.add_argument("--yaw-degrees", type=float, default=0.0)
    parser.add_argument("--timeout-s", type=float, default=12.0)
    parser.add_argument("--poll-interval-s", type=float, default=0.25)
    return parser.parse_args()


def main() -> None:
    raise SystemExit(asyncio.run(verify(parse_args())))


if __name__ == "__main__":
    main()
