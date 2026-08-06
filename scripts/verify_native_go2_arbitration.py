#!/usr/bin/env python3
"""Verify navigation-to-teleop preemption and emergency-stop lease invalidation."""
from __future__ import annotations

import argparse
import asyncio
import json
from pathlib import Path

from config.settings import Settings
from robot_brain.actuation.unitree import FakeUnitreeTransport, UnitreeRobot
from robot_brain.actuation.unitree_webrtc import create_webrtc_transport
from robot_brain.navigation import FakeNavigationClient, NavigationStatus, RelativeNavigationGoal
from robot_brain.navigation.native_go2 import NativeGo2NavigationClient
from robot_brain.navigation.sensors import UnitreeNavigationSensorProvider
from robot_brain.teleop.session import ControlEventType, TeleopSession


CONFIRMATION = "I_UNDERSTAND_GO2_CONTROL_ARBITRATION"


async def verify(*, live: bool, forward_m: float = .5,
                 sensor_timeout_s: float = 15) -> dict[str, object]:
    settings = Settings(
        robot_backend="unitree", perception_backend="unitree",
        navigation_backend="native_go2",
        unitree_transport="webrtc" if live else "fake",
        unitree_dry_run=not live, unitree_enable_motion=live,
        memory_db_path=":memory:",
    )
    transport = create_webrtc_transport(settings) if live else FakeUnitreeTransport()
    robot = UnitreeRobot(transport, settings)
    navigation = None
    sensors_report = None
    await transport.connect()
    try:
        if live:
            sensors = UnitreeNavigationSensorProvider(
                transport, max_pose_age_s=settings.odom_max_age_seconds,
                max_pointcloud_age_s=settings.direct_nav_pointcloud_max_age_s,
                require_authoritative_odom=settings.direct_nav_require_robotodom,
            )
            deadline = asyncio.get_running_loop().time()+sensor_timeout_s
            snapshot = await sensors.get_snapshot()
            while not snapshot.ready and asyncio.get_running_loop().time() < deadline:
                await asyncio.sleep(.1)
                snapshot = await sensors.get_snapshot()
            sensors_report = {"ready": snapshot.ready, "reason": snapshot.reason,
                              "pose_source": snapshot.pose_source,
                              "obstacle_frame": snapshot.obstacle_frame}
            if not snapshot.ready:
                return {"ok": False, "mode": "live", "sensors": sensors_report,
                        "reason": "sensors_not_ready"}
            navigation = NativeGo2NavigationClient(robot, sensors)
            for posture in ("stand_up", "balance_stand", "free_walk"):
                await robot.set_posture(posture)
                await asyncio.sleep(.5)
            await robot.enable_omni_teleop()
        else:
            navigation = FakeNavigationClient(outcomes=[NavigationStatus.ACTIVE])
        handle = await navigation.set_relative_goal(RelativeNavigationGoal(
            forward_m=forward_m, max_duration_s=10,
        ))
        session = TeleopSession(robot, settings, navigation)
        lease = await session.acquire_lease("acceptance-operator")
        navigation_state = await navigation.get_state()
        zero = await session.set_velocity(lease.lease_id, 0, 0, 0) if lease.granted else None
        await session.emergency_stop("acceptance emergency stop")
        stale = await session.set_velocity(lease.lease_id, .1, 0, 0) if lease.granted else None
        events = []
        while not session.events.empty():
            event = session.events.get_nowait()
            events.append({"type": event.type.value, "message": event.message})
        navigation_preempted = (
            handle.accepted and navigation_state.status == NavigationStatus.CANCELED
            and any(row["type"] == ControlEventType.PREEMPTED.value for row in events)
        )
        estop_stopped = any(row.get("action") == "stop" and "emergency stop" in
                            row.get("reason", "") for row in robot.action_history)
        report = {
            "mode": "live" if live else "dry_run",
            "provider": "native_go2" if live else "fake_navigation_contract",
            "sensors": sensors_report, "navigation_preempted": navigation_preempted,
            "teleop_lease_granted": lease.granted,
            "zero_setpoint_accepted": bool(zero and zero.accepted),
            "estop_stopped": estop_stopped,
            "lease_invalidated": bool(stale and not stale.accepted),
            "motion_rejected_during_estop": bool(stale and not stale.accepted),
            "events": events, "command_audit": robot.action_history,
        }
        report["ok"] = all(report[key] is True for key in (
            "navigation_preempted", "teleop_lease_granted", "zero_setpoint_accepted",
            "estop_stopped", "lease_invalidated", "motion_rejected_during_estop",
        ))
        return report
    finally:
        await robot.stop("arbitration verifier finished")
        close = getattr(navigation, "aclose", None)
        if callable(close):
            await close()
        await transport.disconnect()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--live", action="store_true")
    parser.add_argument("--confirm", default="")
    parser.add_argument("--forward-m", type=float, default=.5)
    parser.add_argument("--sensor-timeout-s", type=float, default=15)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.live and args.confirm != CONFIRMATION:
        parser.error(f"--live requires --confirm {CONFIRMATION}")
    report = asyncio.run(verify(live=args.live, forward_m=args.forward_m,
                                sensor_timeout_s=args.sensor_timeout_s))
    encoded = json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True,
                         default=str, allow_nan=False)+"\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open("x", encoding="utf-8") as stream:
            stream.write(encoded)
    print(encoded, end="")
    raise SystemExit(0 if report["ok"] else 3)


if __name__ == "__main__":
    main()
