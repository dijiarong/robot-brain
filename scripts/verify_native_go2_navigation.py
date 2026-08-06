#!/usr/bin/env python3
"""Read-only by default; optionally run one gated native Go2 navigation goal."""
from __future__ import annotations

import argparse
import asyncio
import json
import math
from pathlib import Path
import tempfile
import time

from config.settings import Settings
from robot_brain.actuation.unitree import UnitreeRobot
from robot_brain.actuation.unitree_webrtc import create_webrtc_transport
from robot_brain.navigation import NavigationPose, NavigationStatus, RelativeNavigationGoal, SparseVoxelMap
from robot_brain.navigation.grid import costmap_from_pointcloud
from robot_brain.navigation.native_go2 import NativeGo2NavigationClient
from robot_brain.navigation.planner import astar_path
from robot_brain.navigation.sensors import UnitreeNavigationSensorProvider

CONFIRMATION = "I_UNDERSTAND_NATIVE_GO2_NAV"


def _preflight_report_path(report_path: str) -> None:
    """Fail before connecting or moving when the evidence path is unusable."""
    if not report_path:
        return
    target = Path(report_path)
    target.parent.mkdir(parents=True, exist_ok=True)
    if target.exists():
        raise FileExistsError(f"report path already exists: {target}")
    # Keep exclusive-create semantics for the final report, but prove now that
    # its directory is writable so a live run cannot finish without evidence.
    with tempfile.NamedTemporaryFile(
        mode="w", encoding="utf-8", prefix=".native-nav-write-check-",
        dir=target.parent, delete=True,
    ) as stream:
        stream.write("ok\n")
        stream.flush()


def _strict_json(value):
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {str(key): _strict_json(child) for key, child in value.items()}
    if isinstance(value, (list, tuple)):
        return [_strict_json(child) for child in value]
    return value


def _emit(report, report_path: str = "") -> None:
    encoded = json.dumps(
        _strict_json(report), ensure_ascii=False, indent=2,
        sort_keys=True, default=str, allow_nan=False,
    ) + "\n"
    if report_path:
        target = Path(report_path)
        target.parent.mkdir(parents=True, exist_ok=True)
        with target.open("x", encoding="utf-8") as stream:
            stream.write(encoded)
    print(encoded, end="")


def _maximum_path_deviation(trace, start_xy, goal_xy) -> float:
    dx, dy = goal_xy[0]-start_xy[0], goal_xy[1]-start_xy[1]
    length = max(1e-9, (dx*dx + dy*dy) ** 0.5)
    maximum = 0.0
    for event in trace:
        if event.get("event") != "plan_geometry":
            continue
        for point in event.get("path_xy", ()):
            if not isinstance(point, (list, tuple)) or len(point) < 2:
                continue
            cross = abs(dx*(float(point[1])-start_xy[1]) - dy*(float(point[0])-start_xy[0]))
            maximum = max(maximum, cross/length)
    return maximum


def _trajectory_metrics(trace, start_pose, requested_forward_m, requested_left_m):
    """Project observed odometry samples into the requested body-frame route."""
    if start_pose is None:
        return {"forward_progress_m": None, "maximum_lateral_deviation_m": None,
                "motion_samples": 0}
    yaw = math.radians(start_pose.yaw_deg)
    route_yaw = math.atan2(requested_left_m, requested_forward_m)
    axis = yaw + route_yaw
    forward, lateral, count = 0.0, 0.0, 0
    for row in trace:
        if row.get("event") != "motion_sample":
            continue
        try:
            dx = float(row["x_m"])-start_pose.x_m
            dy = float(row["y_m"])-start_pose.y_m
        except (KeyError, TypeError, ValueError):
            continue
        forward = max(forward, dx*math.cos(axis)+dy*math.sin(axis))
        lateral = max(lateral, abs(-dx*math.sin(axis)+dy*math.cos(axis)))
        count += 1
    return {"forward_progress_m": forward, "maximum_lateral_deviation_m": lateral,
            "motion_samples": count}


def _scenario_failures(
    scenario, status, trace, action_history, path_deviation_m, *,
    stop_reason=None, trajectory=None, requested_distance_m=None,
    cancel_latency_s=None,
):
    emergency = any(row.get("event") == "emergency_stop" for row in trace)
    obstacle_stop = any(row.get("action") == "stop" and "obstacle" in row.get("reason", "")
                        for row in action_history)
    cancel_stop = any(row.get("action") == "stop" and "cancel" in row.get("reason", "")
                      for row in action_history)
    no_progress_stop = any(
        row.get("action") == "stop" and "no progress" in row.get("reason", "")
        for row in action_history
    )
    failures: list[str] = []
    if scenario in {"straight", "obstacle"}:
        if status != NavigationStatus.SUCCEEDED or stop_reason != "goal_reached":
            failures.append("goal_not_reached")
        metrics = trajectory or {}
        progress = metrics.get("forward_progress_m")
        minimum = max(0.05, float(requested_distance_m or 0)-0.15)
        if progress is None or progress < minimum:
            failures.append("insufficient_observed_odometry_progress")
    if scenario == "obstacle":
        if path_deviation_m <= 0.05:
            failures.append("no_planned_geometric_detour")
        if (trajectory or {}).get("maximum_lateral_deviation_m") is None or (
            trajectory or {}
        ).get("maximum_lateral_deviation_m", 0) <= 0.05:
            failures.append("no_observed_geometric_detour")
    elif scenario == "cancel":
        if status != NavigationStatus.CANCELED or stop_reason != "canceled":
            failures.append("cancel_terminal_mismatch")
        if not cancel_stop:
            failures.append("cancel_stop_not_audited")
        if cancel_latency_s is None or cancel_latency_s > 1.25:
            failures.append("cancel_latency_exceeded")
    elif scenario == "sudden_block":
        if not emergency:
            failures.append("emergency_event_missing")
        if not obstacle_stop:
            failures.append("obstacle_stop_not_audited")
    elif scenario == "stuck":
        if status != NavigationStatus.NO_PROGRESS or stop_reason != "no_progress":
            failures.append("no_progress_terminal_mismatch")
        if not no_progress_stop:
            failures.append("no_progress_stop_not_audited")
    return failures


def _scenario_passed(scenario, status, trace, action_history, path_deviation_m, **evidence):
    return not _scenario_failures(
        scenario, status, trace, action_history, path_deviation_m, **evidence,
    )


def _read_only_planning_probe(snapshot, settings, forward_m: float, left_m: float):
    """Capture the exact local A* input without issuing a motion command."""
    if snapshot.pointcloud is None:
        return None
    grid = costmap_from_pointcloud(
        snapshot.pointcloud,
        size_m=settings.native_nav_map_size_m,
        resolution_m=settings.native_nav_resolution_m,
        robot_radius_m=settings.native_nav_robot_radius_m,
    )
    start = grid.world_to_cell(0.0, 0.0)
    goal = grid.world_to_cell(forward_m, left_m)
    path = astar_path(grid, (0.0, 0.0), (forward_m, left_m))
    return {
        "resolution_m": grid.resolution_m,
        "width": grid.width,
        "height": grid.height,
        "origin_x_m": grid.origin_x_m,
        "origin_y_m": grid.origin_y_m,
        "robot_radius_m": settings.native_nav_robot_radius_m,
        "start_cell": list(start) if start is not None else None,
        "goal_cell": list(goal) if goal is not None else None,
        "start_occupied": start in grid.occupied if start is not None else None,
        "goal_occupied": goal in grid.occupied if goal is not None else None,
        "occupied_count": len(grid.occupied),
        "known_free_count": len(grid.known_free),
        "occupied_cells": [list(cell) for cell in sorted(grid.occupied)],
        "path_xy": [list(point) for point in path] if path is not None else None,
    }


async def run(args: argparse.Namespace) -> int:
    if args.live and args.confirm != CONFIRMATION:
        _emit({"ok": False, "error": "live confirmation missing",
               "required": CONFIRMATION}, args.report_path)
        return 5
    settings = Settings(
        robot_backend="unitree", perception_backend="unitree",
        navigation_backend="native_go2", unitree_transport="webrtc",
        unitree_dry_run=not args.live, unitree_enable_motion=args.live,
    )
    transport = create_webrtc_transport(settings)
    robot = UnitreeRobot(transport, settings)
    report: dict[str, object] = {
        "provider": "native_go2", "mode": "live" if args.live else "read_only",
        "scenario": args.scenario,
        "started_at": time.time(), "goal": {
            "forward_m": args.forward_m, "left_m": args.left_m,
            "max_duration_s": args.timeout_s,
        },
    }
    try:
        await transport.connect()
        sensors = UnitreeNavigationSensorProvider(
            transport, max_pose_age_s=settings.odom_max_age_seconds,
            max_pointcloud_age_s=settings.direct_nav_pointcloud_max_age_s,
            require_authoritative_odom=settings.direct_nav_require_robotodom,
        )
        voxel_map = SparseVoxelMap.load(args.map_path) if args.map_path else None
        client = NativeGo2NavigationClient(
            robot, sensors, linear_speed_mps=settings.unitree_max_speed,
            segment_duration_s=settings.direct_nav_segment_duration_s,
            map_size_m=settings.native_nav_map_size_m,
            resolution_m=settings.native_nav_resolution_m,
            robot_radius_m=settings.native_nav_robot_radius_m,
            emergency_stop_m=settings.native_nav_emergency_stop_m,
            reach_tolerance_m=settings.direct_nav_reach_tolerance_m,
            reach_tolerance_yaw_deg=settings.direct_nav_reach_tolerance_yaw_deg,
            min_progress_m=settings.odom_progress_min_m,
            max_no_progress_segments=settings.direct_nav_no_progress_segments,
            max_no_path_replans=settings.native_nav_max_no_path_replans,
            settle_s=settings.direct_nav_odom_settle_s,
            voxel_map=voxel_map,
            persistent_map=voxel_map is not None,
        )
        deadline = time.monotonic() + args.sensor_timeout_s
        snapshot = await sensors.get_snapshot()
        while not snapshot.ready and time.monotonic() < deadline:
            await asyncio.sleep(0.1)
            snapshot = await sensors.get_snapshot()
        report["sensors"] = {
            "ready": snapshot.ready, "reason": snapshot.reason,
            "pose": snapshot.pose.model_dump(mode="json") if snapshot.pose else None,
            "pose_age_seconds": snapshot.pose_age_seconds,
            "pointcloud_age_seconds": snapshot.pointcloud_age_seconds,
            "point_count": snapshot.pointcloud.point_count if snapshot.pointcloud else 0,
            "obstacle_frame": snapshot.obstacle_frame, "pose_source": snapshot.pose_source,
        }
        if voxel_map is not None:
            initial = (
                NavigationPose(
                    x_m=args.initial_map_x, y_m=args.initial_map_y,
                    yaw_degrees=args.initial_map_yaw, frame_id="map",
                )
                if args.initial_map_x is not None and args.initial_map_y is not None
                else None
            )
            relocalization = await client.relocalize(
                initial, allow_global_fallback=args.global_relocalization,
            )
            report["relocalization"] = {
                **relocalization.__dict__,
                "pose": relocalization.pose.model_dump(mode="json")
                if relocalization.pose else None,
            }
            if not relocalization.accepted:
                report.update(ok=False, stop_reason="relocalization_failed")
                _emit(report, args.report_path)
                return 4
        if not snapshot.ready:
            report.update(ok=False, stop_reason=snapshot.reason)
            _emit(report, args.report_path)
            return 2
        if not args.live:
            report["planning_probe"] = _read_only_planning_probe(
                snapshot, settings, args.forward_m, args.left_m,
            )
            report.update(ok=True, stop_reason="read_only_complete")
            _emit(report, args.report_path)
            return 0

        for posture in ("stand_up", "balance_stand", "free_walk"):
            await robot.set_posture(posture)
            await asyncio.sleep(1.0)
        await robot.enable_omni_teleop()
        before = await sensors.get_snapshot()
        handle = await client.set_relative_goal(RelativeNavigationGoal(
            forward_m=args.forward_m, left_m=args.left_m,
            # The five translation/safety scenarios validate position and stop
            # behavior; they do not request a terminal body orientation.
            require_final_yaw=False, max_duration_s=args.timeout_s,
        ))
        motion_started = time.monotonic()
        cancel_requested_at = None
        cancel_completed_at = None
        state = await client.get_state()
        while not state.status.terminal:
            if (
                args.scenario == "cancel"
                and cancel_requested_at is None
                and time.monotonic() - motion_started >= args.cancel_after_s
            ):
                cancel_requested_at = time.monotonic()
                state = await client.cancel(handle.goal_id)
                cancel_completed_at = time.monotonic()
                break
            await asyncio.sleep(0.1)
            state = await client.get_state()
        after = await sensors.get_snapshot()
        trace = client.trace
        emergency_events = [row for row in trace if row.get("event") == "emergency_stop"]
        plan_events = [row for row in trace if row.get("event") == "plan"]
        if before.pose is not None:
            yaw = math.radians(before.pose.yaw_deg)
            goal_x = (before.pose.x_m + args.forward_m * math.cos(yaw)
                      - args.left_m * math.sin(yaw))
            goal_y = (before.pose.y_m + args.forward_m * math.sin(yaw)
                      + args.left_m * math.cos(yaw))
            path_deviation = _maximum_path_deviation(
                trace, (before.pose.x_m, before.pose.y_m), (goal_x, goal_y)
            )
        else:
            path_deviation = 0.0
        trajectory = _trajectory_metrics(
            trace, before.pose, args.forward_m, args.left_m,
        )
        cancel_latency = (
            cancel_completed_at - cancel_requested_at
            if cancel_requested_at is not None and cancel_completed_at is not None
            else None
        )
        evidence = {
            "stop_reason": state.stop_reason,
            "trajectory": trajectory,
            "requested_distance_m": math.hypot(args.forward_m, args.left_m),
            "cancel_latency_s": cancel_latency,
        }
        failures = _scenario_failures(
            args.scenario, state.status, trace, robot.action_history, path_deviation,
            **evidence,
        )
        expected = not failures
        report.update(
            ok=expected,
            goal_id=handle.goal_id,
            state=state.model_dump(mode="json"),
            pose_before=before.pose.model_dump(mode="json") if before.pose else None,
            pose_after=after.pose.model_dump(mode="json") if after.pose else None,
            stop_reason=state.stop_reason,
            trace=trace,
            command_audit=robot.action_history,
            acceptance={
                "expected_terminal": {
                    "straight": "succeeded", "obstacle": "succeeded",
                    "cancel": "canceled", "sudden_block": "emergency_stop_observed",
                    "stuck": "no_progress",
                }[args.scenario],
                "plan_events": len(plan_events),
                "emergency_stop_events": len(emergency_events),
                "maximum_path_deviation_m": path_deviation,
                "trajectory": trajectory,
                "cancel_latency_s": cancel_latency,
                "failures": failures,
                "operator_setup": {
                    "straight": "clear 0.3 m corridor",
                    "obstacle": "place a static obstacle in the 1-3 m route before start",
                    "cancel": "clear corridor; cancellation is issued automatically",
                    "sudden_block": "insert an obstacle only after motion begins; keep hands clear",
                    "stuck": "safely restrain or lift the robot after motion begins",
                }[args.scenario],
            },
        )
        _emit(report, args.report_path)
        return 0 if report["ok"] else 3
    finally:
        await robot.stop("native navigation verifier finished")
        await transport.disconnect()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--live", action="store_true")
    parser.add_argument(
        "--scenario", choices=("straight", "obstacle", "cancel", "sudden_block", "stuck"),
        default="straight",
    )
    parser.add_argument("--confirm", default="")
    parser.add_argument("--forward-m", type=float, default=0.3)
    parser.add_argument("--left-m", type=float, default=0.0)
    parser.add_argument("--timeout-s", type=float, default=20.0)
    parser.add_argument("--sensor-timeout-s", type=float, default=15.0)
    parser.add_argument("--cancel-after-s", type=float, default=0.5)
    parser.add_argument("--map-path", default="")
    parser.add_argument("--initial-map-x", type=float)
    parser.add_argument("--initial-map-y", type=float)
    parser.add_argument("--initial-map-yaw", type=float, default=0.0)
    parser.add_argument("--global-relocalization", action="store_true")
    parser.add_argument("--report-path", default="")
    args = parser.parse_args()
    if args.cancel_after_s < 0:
        parser.error("--cancel-after-s cannot be negative")
    try:
        _preflight_report_path(args.report_path)
    except OSError as exc:
        parser.error(f"--report-path is not writable: {exc}")
    raise SystemExit(asyncio.run(run(args)))


if __name__ == "__main__":
    main()
