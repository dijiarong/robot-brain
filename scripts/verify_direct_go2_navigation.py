#!/usr/bin/env python3
"""Verify built-in Go2 LiDAR/odom and optionally one bounded local goal."""
from __future__ import annotations

import argparse
import asyncio
import contextlib
import json
import time

from config.settings import Settings
from robot_brain.actuation.unitree import UnitreeRobot
from robot_brain.actuation.unitree_webrtc import create_webrtc_transport
from robot_brain.navigation import NavigationStatus, RelativeNavigationGoal
from robot_brain.navigation.direct_go2 import DirectGo2NavigationClient
from robot_brain.navigation.sensors import UnitreeNavigationSensorProvider


CONFIRMATION = "I_UNDERSTAND_DIRECT_GO2_NAV"

_SPORT_MODE_LABELS = {
    0: "idle/stand",
    1: "balanceStand",
    3: "locomotion",
    5: "lieDown",
    7: "damping",
}


async def _prep_locomotion(robot: UnitreeRobot) -> dict[str, object]:
    """Wake gait before Move/joystick drive — same recipe as teleop/smoke Level 2.

    On this Go2 Remote path ``sport_mode`` often stays 0 even while walking, so
    we do not retry FreeWalk forever. Keep the prep short.
    """
    steps: list[dict[str, object]] = []

    async def _snapshot(label: str) -> dict[str, object]:
        state = await robot.transport.read_state()
        last_api = getattr(robot.transport, "_last_sport_api", {}) or {}
        row = {
            "step": label,
            "is_standing": state.is_standing,
            "sport_mode": state.sport_mode,
            "sport_mode_label": _SPORT_MODE_LABELS.get(
                state.sport_mode if state.sport_mode is not None else -1, "?"
            ),
            "last_sport_api": last_api,
        }
        steps.append(row)
        print(
            f"[Prep] after {label}: standing={row['is_standing']} "
            f"sport_mode={row['sport_mode']} ({row['sport_mode_label']}) "
            f"api_status={last_api.get('status_code')}",
            flush=True,
        )
        return row

    # Settles trimmed: APIs already return status_code=0 quickly on this dog.
    for posture, settle_s in (
        ("stand_up", 2.0),
        ("balance_stand", 1.5),
        ("free_walk", 1.5),
    ):
        await robot.set_posture(posture)
        await asyncio.sleep(settle_s)
        await _snapshot(posture)

    await robot.enable_omni_teleop()
    await asyncio.sleep(0.5)
    final = await _snapshot("enable_omni_teleop")
    return {
        "steps": steps,
        "is_standing": final.get("is_standing"),
        "sport_mode": final.get("sport_mode"),
        "sport_mode_label": final.get("sport_mode_label"),
        "warning": None,
    }


async def _gait_probe(robot: UnitreeRobot, *, vx: float, duration_s: float) -> dict[str, object]:
    """One continuous forward push to check whether legs actually step."""
    before = await robot.transport.read_state()
    print(
        f"[GaitProbe] continuous forward vx={vx} m/s for {duration_s}s — "
        "watch the legs now",
        flush=True,
    )
    await robot.drive(vx=vx, duration=duration_s)
    await asyncio.sleep(0.5)
    after = await robot.transport.read_state()
    dx = after.position.x - before.position.x
    dy = after.position.y - before.position.y
    result = {
        "vx": vx,
        "duration_s": duration_s,
        "drive_via_move": robot._settings.unitree_webrtc_drive_via_move,  # noqa: SLF001
        "displacement_m": (dx * dx + dy * dy) ** 0.5,
        "pose_before": {
            "x": before.position.x,
            "y": before.position.y,
            "yaw": before.heading_degrees,
            "sport_mode": before.sport_mode,
        },
        "pose_after": {
            "x": after.position.x,
            "y": after.position.y,
            "yaw": after.heading_degrees,
            "sport_mode": after.sport_mode,
        },
        "visual_check": "Did the legs lift and step? (yes/no — answer in chat)",
    }
    print(
        f"[GaitProbe] done displacement≈{result['displacement_m']:.3f}m "
        f"mode {before.sport_mode}->{after.sport_mode}",
        flush=True,
    )
    return result


async def _cancel_probe(
    robot: UnitreeRobot,
    *,
    vx: float,
    drive_duration_s: float,
    stop_after_s: float,
) -> dict[str, object]:
    """Continuous forward walk, then mid-stream stop — clearer than Ctrl+C on short segments."""
    before = await robot.transport.read_state()
    print(
        f"[CancelProbe] continuous forward vx={vx} for {drive_duration_s}s; "
        f"auto-stop after {stop_after_s}s — watch whether gait cuts off",
        flush=True,
    )
    drive_task = asyncio.create_task(robot.drive(vx=vx, duration=drive_duration_s))
    mid_pose: dict[str, float] | None = None
    stop_latency_s: float | None = None
    interrupted = False
    try:
        await asyncio.sleep(stop_after_s)
        mid = await robot.transport.read_state()
        mid_pose = {
            "x": mid.position.x,
            "y": mid.position.y,
            "yaw": mid.heading_degrees,
        }
        print("[CancelProbe] STOP NOW (operator stop)", flush=True)
        t0 = time.monotonic()
        await robot.stop("cancel probe mid-drive")
        try:
            await asyncio.wait_for(asyncio.shield(drive_task), timeout=3.0)
        except TimeoutError:
            drive_task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await drive_task
        stop_latency_s = time.monotonic() - t0
    except asyncio.CancelledError:
        interrupted = True
        print("[CancelProbe] Ctrl+C — issuing stop", flush=True)
        t0 = time.monotonic()
        await robot.stop("cancel probe Ctrl+C")
        if not drive_task.done():
            drive_task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await drive_task
        stop_latency_s = time.monotonic() - t0
        raise
    await asyncio.sleep(0.8)
    after = await robot.transport.read_state()
    dx_end = after.position.x - before.position.x
    dy_end = after.position.y - before.position.y
    coast_m = None
    if mid_pose is not None:
        cdx = after.position.x - mid_pose["x"]
        cdy = after.position.y - mid_pose["y"]
        coast_m = (cdx * cdx + cdy * cdy) ** 0.5
    end_reason = getattr(robot.transport, "last_drive_end_reason", None)
    result = {
        "vx": vx,
        "drive_duration_s": drive_duration_s,
        "stop_after_s": stop_after_s,
        "stop_latency_s": stop_latency_s,
        "interrupted_by_ctrl_c": interrupted,
        "last_drive_end_reason": (
            None if end_reason is None else getattr(end_reason, "value", str(end_reason))
        ),
        "displacement_total_m": (dx_end * dx_end + dy_end * dy_end) ** 0.5,
        "coast_after_stop_m": coast_m,
        "pose_before": {
            "x": before.position.x,
            "y": before.position.y,
            "yaw": before.heading_degrees,
        },
        "pose_at_stop": mid_pose,
        "pose_after": {
            "x": after.position.x,
            "y": after.position.y,
            "yaw": after.heading_degrees,
        },
        "visual_check": (
            "Did forward walking cut off promptly after STOP NOW? "
            "(yes/no — answer in chat)"
        ),
    }
    print(
        f"[CancelProbe] done stop_latency≈{stop_latency_s}s "
        f"coast_after_stop≈{coast_m}m end_reason={result['last_drive_end_reason']}",
        flush=True,
    )
    return result


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
            linear_speed_mps=settings.unitree_max_speed,
            yaw_speed_rps=settings.unitree_max_yaw_speed,
            segment_duration_s=settings.direct_nav_segment_duration_s,
            obstacle_stop_m=settings.direct_nav_obstacle_stop_m,
            obstacle_half_width_m=settings.direct_nav_obstacle_half_width_m,
            min_progress_m=settings.odom_progress_min_m,
            min_progress_yaw_deg=settings.odom_progress_min_yaw_deg,
            max_no_progress_segments=settings.direct_nav_no_progress_segments,
            odom_settle_s=settings.direct_nav_odom_settle_s,
            reach_tolerance_m=settings.direct_nav_reach_tolerance_m,
            reach_tolerance_yaw_deg=settings.direct_nav_reach_tolerance_yaw_deg,
        )
        report["nav_config"] = {
            "dry_run": settings.unitree_dry_run,
            "enable_motion": settings.unitree_enable_motion,
            "linear_speed_mps": settings.unitree_max_speed,
            "yaw_speed_rps": settings.unitree_max_yaw_speed,
            "segment_duration_s": settings.direct_nav_segment_duration_s,
            "max_drive_duration_s": settings.unitree_max_drive_duration,
            "min_progress_m": settings.odom_progress_min_m,
            "max_no_progress_segments": settings.direct_nav_no_progress_segments,
            "odom_settle_s": settings.direct_nav_odom_settle_s,
            "reach_tolerance_m": settings.direct_nav_reach_tolerance_m,
            "reach_tolerance_yaw_deg": settings.direct_nav_reach_tolerance_yaw_deg,
            "drive_via_move": settings.unitree_webrtc_drive_via_move,
        }
        if (
            args.live
            and settings.unitree_max_drive_duration < 1.5
        ):
            print(
                "[WARN] RDB_UNITREE_MAX_DRIVE_DURATION="
                f"{settings.unitree_max_drive_duration}s is too short for Go2 gait; "
                "dog often leans without stepping. Use >= 2.0s for live nav.",
                flush=True,
            )
        deadline = time.monotonic() + args.sensor_timeout_s
        snapshot = await sensors.get_snapshot()
        while not snapshot.ready and time.monotonic() < deadline:
            await asyncio.sleep(0.1)
            snapshot = await sensors.get_snapshot()
        raw_cloud = transport.read_lidar_snapshot()
        report["raw_lidar"] = {
            "frame_id": raw_cloud.frame_id if raw_cloud else None,
            "origin_xyz": raw_cloud.origin_xyz if raw_cloud else None,
            "point_count": raw_cloud.point_count if raw_cloud else 0,
            "source": raw_cloud.source if raw_cloud else None,
        }
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
            "pointcloud_origin": (
                snapshot.pointcloud.origin_xyz if snapshot.pointcloud else None
            ),
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

        print(
            "[Prep] stand_up → balance_stand → free_walk → SwitchJoystick "
            "(dog should enter walk-ready stance)",
            flush=True,
        )
        report["locomotion_prep"] = await _prep_locomotion(robot)
        print(f"[Prep] done: {report['locomotion_prep']}", flush=True)

        if args.gait_probe or args.cancel_probe:
            # Force longer continuous Move push; ignore short nav segments.
            settings.unitree_max_drive_duration = max(
                settings.unitree_max_drive_duration,
                args.gait_probe_duration_s,
                args.cancel_probe_drive_s,
            )
            settings.unitree_webrtc_drive_via_move = True

        if args.gait_probe:
            report["gait_probe"] = await _gait_probe(
                robot,
                vx=args.gait_probe_vx,
                duration_s=args.gait_probe_duration_s,
            )
            report["motion_audit"] = [
                {
                    "action": item.get("action"),
                    "posture": item.get("posture"),
                    "vx": item.get("vx"),
                    "vy": item.get("vy"),
                    "vyaw": item.get("vyaw"),
                    "duration": item.get("duration"),
                    "end_reason": item.get("end_reason"),
                    "success": item.get("success"),
                }
                for item in robot.action_history
            ]
            # Probe does not claim navigation success; human visual check decides.
            report["ok"] = True
            report["note"] = (
                "gait probe finished — reply whether legs stepped (yes/no)"
            )
            print(json.dumps(report, ensure_ascii=False, indent=2, default=str))
            return 0

        if args.cancel_probe:
            report["cancel_probe"] = await _cancel_probe(
                robot,
                vx=args.cancel_probe_vx,
                drive_duration_s=args.cancel_probe_drive_s,
                stop_after_s=args.cancel_probe_stop_after_s,
            )
            report["motion_audit"] = [
                {
                    "action": item.get("action"),
                    "posture": item.get("posture"),
                    "vx": item.get("vx"),
                    "vy": item.get("vy"),
                    "vyaw": item.get("vyaw"),
                    "duration": item.get("duration"),
                    "end_reason": item.get("end_reason"),
                    "success": item.get("success"),
                }
                for item in robot.action_history
            ]
            report["ok"] = True
            report["note"] = (
                "cancel probe finished — reply whether walking cut off after STOP NOW"
            )
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
        last_key: tuple[object, ...] | None = None
        while not state.status.terminal:
            key = (
                state.status.value,
                round(float(state.progress or 0.0), 3),
                None if state.pose is None else round(state.pose.x_m, 4),
                None if state.pose is None else round(state.pose.y_m, 4),
                state.message,
            )
            if key != last_key:
                samples.append(state.model_dump(mode="json"))
                last_key = key
            await asyncio.sleep(args.poll_interval_s)
            state = await client.get_state()
        samples.append(state.model_dump(mode="json"))
        report["samples"] = samples
        report["final_state"] = state.model_dump(mode="json")
        start_pose = report["sensor_snapshot"]["pose"]  # type: ignore[index]
        end_pose = None if state.pose is None else state.pose.model_dump(mode="json")
        if isinstance(start_pose, dict) and isinstance(end_pose, dict):
            dx = float(end_pose["x_m"]) - float(start_pose["x_m"])
            dy = float(end_pose["y_m"]) - float(start_pose["y_m"])
            report["net_displacement_m"] = (dx * dx + dy * dy) ** 0.5
        report["motion_audit"] = [
            {
                "action": item.get("action"),
                "posture": item.get("posture"),
                "vx": item.get("vx"),
                "vy": item.get("vy"),
                "vyaw": item.get("vyaw"),
                "duration": item.get("duration"),
                "end_reason": item.get("end_reason"),
                "success": item.get("success"),
            }
            for item in robot.action_history
        ]
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
    parser.add_argument(
        "--gait-probe",
        action="store_true",
        help="Skip nav goal; run one continuous forward push to test real stepping",
    )
    parser.add_argument("--gait-probe-vx", type=float, default=0.35)
    parser.add_argument("--gait-probe-duration-s", type=float, default=3.0)
    parser.add_argument(
        "--cancel-probe",
        action="store_true",
        help="Continuous forward walk then mid-stream stop (estop visibility)",
    )
    parser.add_argument("--cancel-probe-vx", type=float, default=0.35)
    parser.add_argument("--cancel-probe-drive-s", type=float, default=8.0)
    parser.add_argument("--cancel-probe-stop-after-s", type=float, default=2.5)
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
