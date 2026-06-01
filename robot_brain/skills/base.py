"""Contract for capabilities exposed to the planner."""
from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any

from pydantic import BaseModel, Field

from robot_brain.actuation.base import RobotInterface
from robot_brain.core.world_state import WorldState


class EmptyParams(BaseModel):
    pass


class SkillResult(BaseModel):
    success: bool
    message: str
    data: dict[str, Any] = Field(default_factory=dict)


class Skill(ABC):
    name: str
    description: str
    params_model: type[BaseModel] = EmptyParams

    def params_schema(self) -> dict[str, Any]:
        return self.params_model.model_json_schema()

    def parse_params(self, params: dict[str, Any]) -> BaseModel:
        return self.params_model.model_validate(params)

    def preconditions(self, world: WorldState) -> bool:
        return not world.estop_active

    @abstractmethod
    async def execute(
        self,
        params: BaseModel,
        robot: RobotInterface,
        world: WorldState,
    ) -> SkillResult: ...

    def is_done(self, world: WorldState) -> bool:
        return True
