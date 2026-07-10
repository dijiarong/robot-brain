"""Built-in atomic tools.

``StopMotionTool`` is the first migrated low-level capability: ``StopSkill``
delegates to it so that safety semantics (motion_kind=stop) live on the tool's
metadata rather than on hardcoded skill-name sets.

``Go2DriveSegmentTool`` (phase E) is the low-level motion primitive the Go2
skills build on: it executes a single timed ``drive()`` segment. ``NudgeSkill``
keeps the distance/direction/segment-planning semantics and calls this tool
per segment, so motion safety (motion_kind=linear, backend=unitree,
confirmation) is expressed via metadata and enforced by ``SafetyPolicy``.

Future tools (see docs/plans/2026-07-10-170002-...md §后续方向):
- motion tools: scan/retreat building blocks on top of the drive segment
- read-only perception tools: recognize, observe
"""
from __future__ import annotations

from pydantic import BaseModel, Field

from robot_brain.tools.base import (
    CapabilityMetadata,
    MotionKind,
    RiskLevel,
    Tool,
    ToolResult,
)


class StopMotionParams(BaseModel):
    reason: str = "safety stop"


class StopMotionTool(Tool):
    """Atomically stop robot motion.

    Allowed during emergency stop and critical battery because
    ``metadata.motion_kind == stop`` (see ``SafetyPolicy``).
    """

    name = "stop_motion"
    description = "Atomically stop robot motion. Always allowed, including during emergency stop."
    params_model = StopMotionParams
    metadata = CapabilityMetadata(
        risk_level=RiskLevel.LOW,
        motion_kind=MotionKind.STOP,
        requires_confirmation=False,
        backend_allowlist=None,
        planner_visible=False,
        tags=frozenset({"control", "motion"}),
    )

    async def execute(self, params: StopMotionParams, context) -> ToolResult:  # type: ignore[override]
        await context.robot.stop(params.reason)
        return ToolResult(
            success=True,
            message=f"stopped: {params.reason}",
            data={"tool": "stop_motion", "reason": params.reason},
        )


class Go2DriveSegmentParams(BaseModel):
    vx: float = 0.0
    vy: float = 0.0
    vyaw: float = 0.0
    duration: float = Field(gt=0.0, description="Segment duration in seconds.")


class Go2DriveSegmentTool(Tool):
    """Single timed Go2 ``drive()`` segment.

    Low-level motion primitive. ``motion_kind=linear`` and
    ``backend_allowlist=("unitree",)`` mean ``SafetyPolicy`` rejects it during
    estop / critical battery, off-unitree backends, and without operator
    confirmation. Skills (nudge/scan/retreat) plan segments and call this tool
    per segment; the tool only owns one timed drive.
    """

    name = "go2_drive_segment"
    description = "Execute a single timed Go2 drive segment (vx, vy, vyaw, duration)."
    params_model = Go2DriveSegmentParams
    metadata = CapabilityMetadata(
        risk_level=RiskLevel.MEDIUM,
        motion_kind=MotionKind.LINEAR,
        requires_confirmation=True,
        backend_allowlist=("unitree",),
        planner_visible=False,
        tags=frozenset({"go2", "motion"}),
    )

    async def execute(  # type: ignore[override]
        self, params: Go2DriveSegmentParams, context
    ) -> ToolResult:
        from robot_brain.actuation.unitree import UnitreeRobot

        robot = context.robot
        segment = {
            "vx": params.vx,
            "vy": params.vy,
            "vyaw": params.vyaw,
            "duration": round(params.duration, 3),
        }
        if not isinstance(robot, UnitreeRobot):
            return ToolResult(
                success=False,
                message=(
                    f"go2_drive_segment requires UnitreeRobot, "
                    f"got {type(robot).__name__}"
                ),
                data={**segment, "end_reason": "error", "error": "wrong robot type"},
            )
        try:
            await robot.drive(
                vx=params.vx, vy=params.vy, vyaw=params.vyaw, duration=params.duration
            )
            raw_reason = getattr(robot.transport, "last_drive_end_reason", None)
            end_reason = str(raw_reason) if raw_reason is not None else "completed"
            return ToolResult(
                success=True,
                message=f"drove segment {params.duration:.3f}s ({end_reason})",
                data={**segment, "end_reason": end_reason},
            )
        except Exception as exc:  # noqa: BLE001 - mirror run_go2_drive_segments audit
            return ToolResult(
                success=False,
                message=f"segment error: {exc}",
                data={**segment, "end_reason": "error", "error": str(exc)},
            )

