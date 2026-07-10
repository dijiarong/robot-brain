"""Go2-native skills that map to ``UnitreeRobot.drive()``.

Each skill goes through the full safety chain:
clamp → precondition → segment planning → drive → audit.
"""
from __future__ import annotations

import logging
import math
from typing import Any, Literal

from pydantic import BaseModel, Field

from config.settings import Settings
from robot_brain.actuation.base import RobotInterface
from robot_brain.actuation.unitree import UnitreeRobot
from robot_brain.core.world_state import WorldState
from robot_brain.skills.base import Skill, SkillResult
from robot_brain.tools.base import CapabilityMetadata, ToolContext
from robot_brain.tools.builtin.control import Go2DriveSegmentParams, Go2DriveSegmentTool

from . import go2_motion

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Param models
# ---------------------------------------------------------------------------

class NudgeParams(BaseModel):
    direction: Literal["forward", "back", "left", "right"]
    distance_cm: float = Field(
        default=20.0,
        description="Distance in cm (clamped to 10–50)",
    )


class ScanParams(BaseModel):
    yaw_degrees: float = Field(
        default=45.0,
        description="Rotation angle in degrees (clamped to ±90). Positive = CCW / left turn.",
    )


class RetreatParams(BaseModel):
    distance_cm: float = Field(
        default=30.0,
        description="Backward distance in cm (clamped to 10–100)",
    )


# ---------------------------------------------------------------------------
# Shared skill base
# ---------------------------------------------------------------------------

class _Go2Skill(Skill):
    """Base for Go2 skills: type-guard + precondition + segment-drive pattern."""

    def __init__(self, settings: Settings) -> None:
        self._settings = settings

    # ------------------------------------------------------------------
    async def execute(
        self,
        params: BaseModel,
        robot: RobotInterface,
        world: WorldState,
    ) -> SkillResult:
        if not isinstance(robot, UnitreeRobot):
            return SkillResult(
                success=False,
                message=f"{self.name} requires UnitreeRobot, got {type(robot).__name__}",
                data={"reason": "wrong robot type"},
            )

        # ----- precondition -------------------------------------------
        reason = go2_motion.check_robot_self_state(world, self._settings)
        if reason is not None:
            logger.warning("%s blocked: %s", self.name, reason)
            return SkillResult(
                success=False,
                message=reason,
                data={"reason": reason},
            )

        # ----- delegate to subclass -----------------------------------
        return await self._execute_go2(params, robot, world)

    async def _execute_go2(
        self,
        params: BaseModel,
        robot: UnitreeRobot,
        world: WorldState,
    ) -> SkillResult:
        raise NotImplementedError


# ---------------------------------------------------------------------------
# Nudge
# ---------------------------------------------------------------------------

class NudgeSkill(_Go2Skill):
    name = "nudge"
    description = (
        "Move a short distance forward, back, left, or right. "
        "Distance 10–50 cm. Use for small position adjustments, not navigation."
    )
    params_model = NudgeParams

    def __init__(self, settings: Settings, *, drive_tool: Go2DriveSegmentTool | None = None) -> None:
        super().__init__(settings)
        self._tool = drive_tool or Go2DriveSegmentTool()

    @property
    def capability_metadata(self) -> CapabilityMetadata:
        # Delegate safety semantics to the low-level motion tool: linear motion,
        # unitree-only, requires confirmation. SafetyPolicy enforces these.
        return self._tool.metadata

    async def _execute_go2(
        self,
        params: NudgeParams,  # type: ignore[override]
        robot: UnitreeRobot,
        world: WorldState,
    ) -> SkillResult:
        distance_cm = max(10.0, min(50.0, params.distance_cm))
        clamped: dict[str, Any] = {"distance_cm": distance_cm}
        requested: dict[str, Any] = {
            "direction": params.direction,
            "distance_cm": params.distance_cm,
        }

        distance_m = distance_cm / 100.0
        seg_dur = self._settings.unitree_max_drive_duration
        durations = go2_motion.plan_linear_segments(distance_m, seg_dur)

        if params.direction == "forward":
            vx, vy = go2_motion.LINEAR_SPEED, 0.0
        elif params.direction == "back":
            vx, vy = -go2_motion.LINEAR_SPEED, 0.0
        elif params.direction == "left":
            vx, vy = 0.0, go2_motion.LINEAR_SPEED
        else:  # right
            vx, vy = 0.0, -go2_motion.LINEAR_SPEED

        # NudgeSkill owns distance/direction/segment semantics; the low-level
        # tool owns one timed drive. Call it per segment and aggregate the
        # audit dict in the same shape run_go2_drive_segments produced.
        ctx = ToolContext(settings=self._settings, world=world, robot=robot)
        segments: list[dict[str, Any]] = []
        success = True
        for i, d in enumerate(durations):
            tool_params = Go2DriveSegmentParams(vx=vx, vy=vy, vyaw=0.0, duration=d)
            seg_result = await self._tool.execute(tool_params, ctx)
            segments.append({"index": i, **seg_result.data})
            if not seg_result.success:
                success = False
                break

        return SkillResult(
            success=success,
            message=f"nudge {params.direction} {distance_cm:.0f}cm"
                    f"{' ✓' if success else ' ✗'}",
            data={
                "skill": "nudge",
                "requested": requested,
                "clamped": clamped,
                "segments": segments,
                "segment_count": len(segments),
                "success": success,
            },
        )


# ---------------------------------------------------------------------------
# Scan
# ---------------------------------------------------------------------------

class ScanSkill(_Go2Skill):
    name = "scan"
    description = (
        "Rotate in place to observe surroundings. "
        "Angle ±90°. Positive = CCW/left turn. Use for looking around, not turning to a heading."
    )
    params_model = ScanParams

    async def _execute_go2(
        self,
        params: ScanParams,  # type: ignore[override]
        robot: UnitreeRobot,
        world: WorldState,
    ) -> SkillResult:
        yaw_deg = max(-90.0, min(90.0, params.yaw_degrees))
        clamped: dict[str, Any] = {"yaw_degrees": yaw_deg}
        requested: dict[str, Any] = {"yaw_degrees": params.yaw_degrees}

        # Zero / negligible rotation — no motion needed.
        if abs(yaw_deg) < 1.0:
            return SkillResult(
                success=True,
                message=f"scan {yaw_deg:.1f}° — no rotation needed",
                data={
                    "skill": "scan",
                    "requested": requested,
                    "clamped": clamped,
                    "segments": [],
                    "segment_count": 0,
                    "success": True,
                },
            )

        yaw_rad = math.radians(yaw_deg)
        vyaw = go2_motion.YAW_SPEED if yaw_rad > 0 else -go2_motion.YAW_SPEED
        seg_dur = self._settings.unitree_max_drive_duration
        durations = go2_motion.plan_yaw_segments(yaw_rad, seg_dur)

        result = await go2_motion.run_go2_drive_segments(
            robot, vyaw=vyaw, durations=durations,
        )

        return SkillResult(
            success=result["success"],
            message=f"scan {yaw_deg:.0f}° {'✓' if result['success'] else '✗'}",
            data={
                "skill": "scan",
                "requested": requested,
                "clamped": clamped,
                **result,
            },
        )


# ---------------------------------------------------------------------------
# Retreat
# ---------------------------------------------------------------------------

class RetreatSkill(_Go2Skill):
    name = "retreat"
    description = (
        "Move straight back a safe distance (10–100 cm). "
        "Always backward — no forward motion. Use to increase distance from an obstacle."
    )
    params_model = RetreatParams

    async def _execute_go2(
        self,
        params: RetreatParams,  # type: ignore[override]
        robot: UnitreeRobot,
        world: WorldState,
    ) -> SkillResult:
        distance_cm = max(10.0, min(100.0, params.distance_cm))
        clamped: dict[str, Any] = {"distance_cm": distance_cm}
        requested: dict[str, Any] = {"distance_cm": params.distance_cm}

        distance_m = distance_cm / 100.0
        seg_dur = self._settings.unitree_max_drive_duration
        durations = go2_motion.plan_linear_segments(distance_m, seg_dur)

        result = await go2_motion.run_go2_drive_segments(
            robot, vx=-go2_motion.LINEAR_SPEED, durations=durations,
        )

        return SkillResult(
            success=result["success"],
            message=f"retreat {distance_cm:.0f}cm {'✓' if result['success'] else '✗'}",
            data={
                "skill": "retreat",
                "requested": requested,
                "clamped": clamped,
                **result,
            },
        )


# ---------------------------------------------------------------------------
# Factory
# ---------------------------------------------------------------------------

def go2_skills(
    settings: Settings,
    *,
    perception: Any | None = None,
    drive_tool: Go2DriveSegmentTool | None = None,
) -> list[Skill]:
    from robot_brain.skills.builtin.explore import ExploreSkill

    return [
        NudgeSkill(settings, drive_tool=drive_tool),
        ScanSkill(settings),
        RetreatSkill(settings),
        ExploreSkill(settings, perception=perception),
    ]
