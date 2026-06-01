"""Run a patrol, report an anomaly, and dock on low battery using only mocks."""
from __future__ import annotations

import asyncio
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from config.settings import Settings
from robodog_brain.actuation.mock import MockRobot
from robodog_brain.core.world_state import DetectedObject, Position
from robodog_brain.perception.base import Observation
from robodog_brain.perception.mock import MockPerception
from robodog_brain.runtime.loop import AgentRuntime


async def main() -> None:
    robot = MockRobot()
    perception = MockPerception(robot)
    runtime = AgentRuntime.create(settings=Settings(enable_verbose_log=False), robot=robot, perception=perception)

    patrol = await runtime.run_command("patrol the lobby")

    perception.push(
        Observation(
            detected_objects=[
                DetectedObject(object_id="box-7", kind="unattended_box", position=Position(x=4, y=3))
            ],
            alerts=["unattended box detected in lobby"],
        )
    )
    anomaly = await runtime.run_command("inspect the lobby anomaly")

    perception.push(Observation(battery_level=20.0))
    recharge = await runtime.run_command("continue patrol")

    print("patrol:", patrol.status, patrol.message)
    print("anomaly:", anomaly.status, anomaly.message)
    print("recharge:", recharge.status, recharge.message)
    print("actions:")
    for action in robot.action_history:
        print(" -", action)


if __name__ == "__main__":
    asyncio.run(main())
