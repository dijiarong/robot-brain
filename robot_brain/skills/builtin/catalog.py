"""Small built-in skill set for the mock-capable skeleton."""
from __future__ import annotations

from typing import Literal

from pydantic import BaseModel, Field

from robot_brain.actuation.base import RobotInterface
from robot_brain.core.world_state import Position, WorldState
from robot_brain.skills.base import Skill, SkillResult


class NavigateParams(BaseModel):
    target: Position
    speed: float = Field(default=0.8, gt=0.0)


class NavigateSkill(Skill):
    name = "navigate"
    description = "Move to a map position at a requested linear speed."
    params_model = NavigateParams

    async def execute(self, params: NavigateParams, robot: RobotInterface, world: WorldState) -> SkillResult:
        await robot.move_to(params.target, params.speed)
        return SkillResult(success=True, message=f"navigated to ({params.target.x}, {params.target.y})")


class PatrolParams(BaseModel):
    waypoints: list[Position] = Field(min_length=1)
    speed: float = Field(default=0.6, gt=0.0)


class PatrolSkill(Skill):
    name = "patrol"
    description = "Visit a sequence of waypoints for inspection."
    params_model = PatrolParams

    async def execute(self, params: PatrolParams, robot: RobotInterface, world: WorldState) -> SkillResult:
        for waypoint in params.waypoints:
            await robot.move_to(waypoint, params.speed)
        return SkillResult(success=True, message=f"patrolled {len(params.waypoints)} waypoint(s)")


class RecognizeParams(BaseModel):
    object_id: str | None = None
    kind: str | None = None


class RecognizeSkill(Skill):
    name = "recognize"
    description = "Look up known perceived objects by id or kind."
    params_model = RecognizeParams

    async def execute(self, params: RecognizeParams, robot: RobotInterface, world: WorldState) -> SkillResult:
        matches = [
            item.model_dump(mode="json")
            for item in world.known_objects.values()
            if (params.object_id is None or item.object_id == params.object_id)
            and (params.kind is None or item.kind == params.kind)
        ]
        return SkillResult(success=bool(matches), message=f"recognized {len(matches)} object(s)", data={"objects": matches})


class FollowParams(BaseModel):
    target_id: str
    distance: float = Field(default=2.0, ge=1.0, le=10.0)


class FollowSkill(Skill):
    name = "follow"
    description = "Follow a perceived target while maintaining a safe distance."
    params_model = FollowParams

    async def execute(self, params: FollowParams, robot: RobotInterface, world: WorldState) -> SkillResult:
        await robot.follow(params.target_id, params.distance)
        return SkillResult(success=True, message=f"following {params.target_id}")


class DockParams(BaseModel):
    station: str = "home"


class DockSkill(Skill):
    name = "dock"
    description = "Return to a charging station and recharge."
    params_model = DockParams

    def preconditions(self, world: WorldState) -> bool:
        return True

    async def execute(self, params: DockParams, robot: RobotInterface, world: WorldState) -> SkillResult:
        await robot.dock(params.station)
        return SkillResult(success=True, message=f"docked at {params.station}")


class ReportParams(BaseModel):
    message: str
    severity: Literal["info", "warning", "critical"] = "info"


class ReportSkill(Skill):
    name = "report"
    description = "Send a structured status or anomaly report."
    params_model = ReportParams

    def preconditions(self, world: WorldState) -> bool:
        return True

    async def execute(self, params: ReportParams, robot: RobotInterface, world: WorldState) -> SkillResult:
        await robot.report(params.message, params.severity)
        return SkillResult(success=True, message=f"reported {params.severity}: {params.message}")


class StopParams(BaseModel):
    reason: str = "safety stop"


class StopSkill(Skill):
    name = "stop"
    description = "Immediately stop robot motion."
    params_model = StopParams

    def preconditions(self, world: WorldState) -> bool:
        return True

    async def execute(self, params: StopParams, robot: RobotInterface, world: WorldState) -> SkillResult:
        await robot.stop(params.reason)
        return SkillResult(success=True, message=f"stopped: {params.reason}")


def default_skills() -> list[Skill]:
    return [
        NavigateSkill(),
        PatrolSkill(),
        RecognizeSkill(),
        FollowSkill(),
        DockSkill(),
        ReportSkill(),
        StopSkill(),
    ]
