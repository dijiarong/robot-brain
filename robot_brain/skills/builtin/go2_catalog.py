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


class Go2LocalNavParams(BaseModel):
    forward_m: float = Field(default=0.0, ge=-1.0, le=1.0)
    left_m: float = Field(default=0.0, ge=-0.5, le=0.5)
    yaw_degrees: float = Field(default=0.0, ge=-90.0, le=90.0)
    max_duration_s: float = Field(default=12.0, ge=0.5, le=20.0)


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
# Local Nav
# ---------------------------------------------------------------------------

class Go2LocalNavSkill(_Go2Skill):
    name = "go2_local_nav"
    description = (
        "Move the Go2 toward a short relative local target: forward/back up to 1m, "
        "left/right up to 0.5m, optional yaw ±90°. Uses bounded drive segments; "
        "not a map/global navigate skill."
    )
    params_model = Go2LocalNavParams

    def __init__(self, settings: Settings, *, perception: Any | None = None) -> None:
        super().__init__(settings)
        self._perception = perception

    async def _execute_go2(
        self,
        params: Go2LocalNavParams,  # type: ignore[override]
        robot: UnitreeRobot,
        world: WorldState,
    ) -> SkillResult:
        if params.forward_m > 0 and self._front_blocked(world):
            return SkillResult(
                success=False,
                message="go2_local_nav blocked: front obstacle",
                data={"skill": self.name, "reason": "front_obstacle"},
            )

        await self._poll_perception(world)
        pose_before = self._pose_snapshot(world)

        seg_dur = self._settings.unitree_max_drive_duration
        elapsed_budget = params.max_duration_s
        move_segments: list[dict[str, Any]] = []
        yaw_segments: list[dict[str, Any]] = []

        distance_m = math.hypot(params.forward_m, params.left_m)
        if distance_m > 0.001 and elapsed_budget > 0:
            durations = self._cap_durations(
                go2_motion.plan_linear_segments(distance_m, seg_dur),
                elapsed_budget,
            )
            scale = go2_motion.LINEAR_SPEED / distance_m
            result = await go2_motion.run_go2_drive_segments(
                robot,
                vx=params.forward_m * scale,
                vy=params.left_m * scale,
                durations=durations,
            )
            move_segments = list(result.get("segments", []))
            elapsed_budget -= sum(float(s.get("duration", 0.0)) for s in move_segments)
            if not result["success"]:
                return self._local_nav_result(
                    False, "drive_error", params, pose_before, world, move_segments, yaw_segments
                )

        if abs(params.yaw_degrees) >= 1.0 and elapsed_budget > 0:
            yaw_rad = math.radians(params.yaw_degrees)
            durations = self._cap_durations(
                go2_motion.plan_yaw_segments(yaw_rad, seg_dur),
                elapsed_budget,
            )
            vyaw = go2_motion.YAW_SPEED if yaw_rad > 0 else -go2_motion.YAW_SPEED
            result = await go2_motion.run_go2_drive_segments(robot, vyaw=vyaw, durations=durations)
            yaw_segments = list(result.get("segments", []))
            if not result["success"]:
                return self._local_nav_result(
                    False, "drive_error", params, pose_before, world, move_segments, yaw_segments
                )

        await self._poll_perception(world)
        return self._local_nav_result(
            True, "completed", params, pose_before, world, move_segments, yaw_segments
        )

    def _local_nav_result(
        self,
        success: bool,
        stop_reason: str,
        params: Go2LocalNavParams,
        pose_before: dict[str, Any] | None,
        world: WorldState,
        move_segments: list[dict[str, Any]],
        yaw_segments: list[dict[str, Any]],
    ) -> SkillResult:
        pose_after = self._pose_snapshot(world)
        delta = self._motion_delta(pose_before, pose_after)
        total = len(move_segments) + len(yaw_segments)
        return SkillResult(
            success=success,
            message=f"go2_local_nav {stop_reason}: {total} segments",
            data={
                "skill": self.name,
                "requested": params.model_dump(mode="json"),
                "pose_before": pose_before,
                "pose_after": pose_after,
                "motion_delta": delta,
                "move_segments": move_segments,
                "yaw_segments": yaw_segments,
                "segment_count": total,
                "stop_reason": stop_reason,
            },
        )

    async def _poll_perception(self, world: WorldState) -> None:
        if self._perception is None:
            return
        observation = await self._perception.observe()
        world.apply_observation(
            observation,
            object_ttl_seconds=self._settings.object_ttl_seconds,
        )

    @staticmethod
    def _pose_snapshot(world: WorldState) -> dict[str, Any] | None:
        ss = world.robot_self_state
        if ss is None or ss.odometry is None or ss.odometry.pose is None:
            return None
        return ss.odometry.pose.model_dump(mode="json")

    @staticmethod
    def _motion_delta(
        before: dict[str, Any] | None,
        after: dict[str, Any] | None,
    ) -> dict[str, Any] | None:
        if before is None or after is None:
            return None
        dx = float(after.get("x_m", 0.0)) - float(before.get("x_m", 0.0))
        dy = float(after.get("y_m", 0.0)) - float(before.get("y_m", 0.0))
        yaw_before = float(before.get("yaw_deg", 0.0))
        yaw_after = float(after.get("yaw_deg", 0.0))
        delta_yaw = (yaw_after - yaw_before + 180.0) % 360.0 - 180.0
        return {
            "delta_m": (dx * dx + dy * dy) ** 0.5,
            "delta_yaw_deg": delta_yaw,
            "valid": True,
        }

    @staticmethod
    def _cap_durations(durations: list[float], budget: float) -> list[float]:
        capped: list[float] = []
        remaining = budget
        for duration in durations:
            if remaining <= 0:
                break
            d = min(duration, remaining)
            if d > 0:
                capped.append(d)
            remaining -= d
        return capped

    def _front_blocked(self, world: WorldState) -> bool:
        ss = world.robot_self_state
        if ss is None or ss.ultrasonic is None or ss.ultrasonic.front_m is None:
            return False
        return ss.ultrasonic.front_m < self._settings.obstacle_proximity_threshold


# ---------------------------------------------------------------------------
# Factory
# ---------------------------------------------------------------------------

def go2_skills(
    settings: Settings,
    *,
    perception: Any | None = None,
    drive_tool: Go2DriveSegmentTool | None = None,
    passability: Any | None = None,
) -> list[Skill]:
    from robot_brain.skills.builtin.explore import ExploreSkill

    return [
        NudgeSkill(settings, drive_tool=drive_tool),
        ScanSkill(settings),
        RetreatSkill(settings),
        Go2LocalNavSkill(settings, perception=perception),
        ExploreSkill(settings, perception=perception, passability=passability),
    ]
