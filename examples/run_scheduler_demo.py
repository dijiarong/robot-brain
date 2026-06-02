"""Run priority scheduling, warning preemption, and automatic recharge with mocks."""
from __future__ import annotations

import asyncio
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from config.settings import Settings
from robot_brain.actuation.base import RobotState
from robot_brain.actuation.mock import MockRobot
from robot_brain.core.events import Event, EventType
from robot_brain.runtime.loop import AgentRuntime
from robot_brain.runtime.scheduler import AgentScheduler


async def main() -> None:
    robot = MockRobot(RobotState(battery_level=20.0))
    runtime = AgentRuntime.create(settings=Settings(enable_verbose_log=False), robot=robot)
    scheduler = AgentScheduler(runtime)

    scheduler.submit("patrol the lobby")
    await scheduler.handle_event(Event(type=EventType.WARNING, message="unattended box detected in lobby"))

    results = await scheduler.run_until_idle()

    print("scheduler:")
    for result in results:
        print(" -", result.status, result.message)
    print("tasks:")
    for task in scheduler.list_tasks():
        print(" -", task.status, task.priority, task.objective)
    print("actions:")
    for action in robot.action_history:
        print(" -", action)
    runtime.close()


if __name__ == "__main__":
    asyncio.run(main())
