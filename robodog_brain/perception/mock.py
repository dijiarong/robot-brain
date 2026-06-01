"""Scriptable perception adapter backed by the in-memory robot."""
from __future__ import annotations

from collections import deque
from collections.abc import Iterable

from robodog_brain.actuation.base import RobotInterface
from robodog_brain.perception.base import Observation, PerceptionAdapter


class MockPerception(PerceptionAdapter):
    def __init__(
        self,
        robot: RobotInterface,
        observations: Iterable[Observation] | None = None,
    ) -> None:
        self.robot = robot
        self._observations = deque(observations or [])

    def push(self, observation: Observation) -> None:
        self._observations.append(observation)

    async def observe(self) -> Observation:
        if self._observations:
            return self._observations.popleft().model_copy(deep=True)
        state = await self.robot.get_state()
        return Observation(**state.model_dump(exclude={"stopped", "docked"}))
