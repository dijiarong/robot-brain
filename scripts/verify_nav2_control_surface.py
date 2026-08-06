#!/usr/bin/env python3
"""Verify Nav2 control surface: status, cancel, and optional re-goal.

Keeps operator rights intact for Mid-360 Navigation integration:
- always know navigation state (get_state)
- cancel anytime
- after cancel / network recovery, can set a goal again

Read-only by default (no motion). Use ``--live-forward`` for a short relative goal.
"""
from __future__ import annotations

import argparse
import asyncio
import json
import sys

from config.settings import Settings
from robot_brain.navigation import NavigationStatus, RelativeNavigationGoal
from robot_brain.navigation.nav2 import create_nav2_navigation_client


async def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--read-only",
        action="store_true",
        default=True,
        help="Only probe get_state / localization (default)",
    )
    parser.add_argument(
        "--live-forward",
        type=float,
        default=0.0,
        help="If >0, send a relative forward goal (metres), cancel it, then re-goal",
    )
    args = parser.parse_args()
    live = args.live_forward > 0

    settings = Settings(navigation_backend="nav2")
    client = create_nav2_navigation_client(settings)
    report: dict[str, object] = {
        "provider": "nav2",
        "action": settings.nav2_action_name,
        "odom_topic": settings.nav2_odom_topic,
        "mode": "live_cancel_regoal" if live else "read_only",
    }

    state = await client.get_state()
    report["initial_state"] = state.model_dump(mode="json")
    localization = await client.get_localization_state()
    report["localization"] = localization.model_dump(mode="json")

    if not state.ready or state.pose is None:
        report["ok"] = False
        report["error"] = "Nav2/odom not ready — is Orin Navigation stack up?"
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 2

    # Cancel is always allowed even with no active goal (idempotent idle).
    canceled = await client.cancel(None)
    report["cancel_idle"] = canceled.model_dump(mode="json")

    if not live:
        report["ok"] = True
        report["control_surface"] = {
            "status": True,
            "cancel": True,
            "re_goal": "skipped_read_only",
        }
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 0

    goal = RelativeNavigationGoal(
        forward_m=args.live_forward,
        left_m=0.0,
        require_final_yaw=False,
        max_duration_s=30.0,
    )
    first = await client.set_relative_goal(goal)
    report["first_goal"] = first.model_dump(mode="json")
    if not first.accepted:
        report["ok"] = False
        report["error"] = first.message or "goal rejected"
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 3

    await asyncio.sleep(0.5)
    mid = await client.get_state()
    report["during_goal"] = mid.model_dump(mode="json")

    stopped = await client.cancel(first.goal_id)
    report["cancel_active"] = stopped.model_dump(mode="json")
    if stopped.status not in {NavigationStatus.CANCELED, NavigationStatus.IDLE}:
        report["ok"] = False
        report["error"] = f"cancel left status={stopped.status}"
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 4

    # Recoverability: after cancel, operator can start navigation again.
    second = await client.set_relative_goal(
        RelativeNavigationGoal(
            forward_m=min(0.15, args.live_forward),
            left_m=0.0,
            require_final_yaw=False,
            max_duration_s=20.0,
        )
    )
    report["second_goal"] = second.model_dump(mode="json")
    await client.cancel(second.goal_id)
    report["ok"] = bool(second.accepted)
    report["control_surface"] = {
        "status": True,
        "cancel": True,
        "re_goal": bool(second.accepted),
    }
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if second.accepted else 5


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
