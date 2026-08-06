#!/usr/bin/env python3
"""Offline end-to-end service wiring proof using the production native client type."""
from __future__ import annotations

import asyncio
import argparse
import json
from pathlib import Path

from config.settings import Settings
from robot_brain.actuation.unitree import FakeUnitreeTransport, UnitreeRobot
from robot_brain.navigation.native_go2 import NativeGo2NavigationClient
from robot_brain.navigation.sensors import NavigationSensorSnapshot
from robot_brain.perception.mock import MockPerception
from robot_brain.runtime.loop import AgentRuntime
from robot_brain.runtime.scheduler import AgentScheduler
from robot_brain.service.runner import AgentService
from robot_brain.skills.builtin.navigation import CancelNavigationParams


class _UnavailableSensors:
    async def get_snapshot(self):
        return NavigationSensorSnapshot(
            pose=None, pointcloud=None, pose_age_seconds=float("inf"),
            pointcloud_age_seconds=float("inf"), pose_ready=False,
            obstacle_data_ready=False, obstacle_frame=None, reason="offline_service_probe",
        )


async def verify() -> dict[str, object]:
    settings = Settings(
        robot_backend="unitree", perception_backend="mock",
        navigation_backend="native_go2", unitree_transport="fake",
        unitree_dry_run=True, unitree_enable_motion=False,
        memory_db_path=":memory:",
    )
    transport = FakeUnitreeTransport()
    await transport.connect()
    robot = UnitreeRobot(transport, settings)
    navigation = NativeGo2NavigationClient(robot, _UnavailableSensors())
    runtime = AgentRuntime.create(
        settings=settings, robot=robot, perception=MockPerception(robot),
        navigation=navigation,
    )
    service = AgentService(AgentScheduler(runtime), poll_interval=.01,
                           close_runtime_on_stop=False)
    try:
        await service.start()
        status = service.status()
        skill = runtime.context.skills.get("nav_cancel")
        result = await skill.execute(
            CancelNavigationParams(), robot, runtime.context.world,
        ) if skill is not None else None
        provider = status.get("navigation", {}).get("provider")
        service_health = service.running and status["service"]["running"]
        skill_invocation = bool(result is not None and result.success)
        motion_gate_enforced = bool(
            settings.unitree_dry_run and not settings.unitree_enable_motion
            and not any(row.get("action") == "drive" for row in robot.action_history)
        )
        return {
            "ok": provider == "NativeGo2NavigationClient" and service_health
                  and skill_invocation and motion_gate_enforced,
            "provider": "native_go2" if provider == "NativeGo2NavigationClient" else provider,
            "provider_type": provider, "service_health": service_health,
            "skill_invocation": skill_invocation,
            "motion_gate_enforced": motion_gate_enforced,
            "registered_skills": sorted(runtime.context.skills.names()),
            "registered_tools": sorted(tool.name for tool in runtime.context.tools.all()),
            "dry_run": settings.unitree_dry_run,
            "motion_enabled": settings.unitree_enable_motion,
        }
    finally:
        await service.stop()
        await runtime.aclose()
        await transport.disconnect()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    report = asyncio.run(verify())
    encoded = json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open("x", encoding="utf-8") as stream:
            stream.write(encoded)
    print(encoded, end="")
    raise SystemExit(0 if report["ok"] else 3)


if __name__ == "__main__":
    main()
