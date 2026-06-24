"""Bounded Explore — composite skill combining scan/nudge/retreat in a loop.

Implements a rule-driven exploration cycle with hard stop conditions.
Works on both mock and Go2 (unitree) backends. Does NOT call nested Skills;
instead directly uses robot-level motion (MockRobot.move_to/turn or
go2_motion.run_go2_drive_segments) — matching the architecture decision in
docs/plans/2026-06-24-160000-bounded-explore-mode.md §1.
"""
from __future__ import annotations

import logging
import math
import time
from typing import Any

from pydantic import BaseModel, Field

from config.settings import Settings
from robot_brain.actuation.base import RobotInterface
from robot_brain.core.world_state import WorldState
from robot_brain.skills.base import Skill, SkillResult

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Params
# ---------------------------------------------------------------------------

class ExploreParams(BaseModel):
    max_steps: int = Field(default=5, ge=1, le=20, description="Max explore cycles; hard cap in Validator")
    step_distance_cm: float = Field(default=20.0, ge=10.0, le=50.0)
    scan_degrees: float = Field(default=45.0, ge=10.0, le=90.0)
    report_every: int = Field(default=2, ge=1, le=10, description="Report every N steps")


# ---------------------------------------------------------------------------
# ExploreSkill
# ---------------------------------------------------------------------------

class ExploreSkill(Skill):
    """Bounded exploration: scan → decide → move, repeated up to max_steps."""

    name = "explore"
    description = (
        "Explore the surroundings by scanning and moving in short increments. "
        "Bounded by max_steps (1–20) and time. Stops on obstacles, low battery, or errors. "
        "Use when no specific navigation target is given."
    )
    params_model = ExploreParams

    def __init__(self, settings: Settings, *, perception: Any | None = None) -> None:
        self._settings = settings
        self._perception = perception  # Optional PerceptionAdapter for live polling

    async def execute(
        self,
        params: BaseModel,
        robot: RobotInterface,
        world: WorldState,
    ) -> SkillResult:
        assert isinstance(params, ExploreParams)
        p = params

        # Determine backend
        from robot_brain.actuation.unitree import UnitreeRobot
        is_go2 = isinstance(robot, UnitreeRobot)

        if is_go2:
            return await self._run_go2_loop(p, robot, world)
        else:
            return await self._run_mock_loop(p, robot, world)

    # ------------------------------------------------------------------
    # Mock backend
    # ------------------------------------------------------------------

    async def _run_mock_loop(
        self,
        params: ExploreParams,
        robot: RobotInterface,
        world: WorldState,
    ) -> SkillResult:
        actions: list[str] = []
        steps_done = 0
        start_time = time.monotonic()
        stop_reason = "max_steps"

        for step in range(params.max_steps):
            # --- Poll perception first (get fresh state for this step) ---
            await self._poll_perception(world)

            # --- Stop conditions ---
            sr = self._check_stop_conditions(world, start_time)
            if sr is not None:
                stop_reason = sr
                break

            # --- Scan ---
            new_heading = world.heading_degrees + params.scan_degrees
            await robot.turn(new_heading)
            world.heading_degrees = new_heading
            actions.append("scan")

            # --- Poll perception again (orientation changed) ---
            await self._poll_perception(world)

            # --- Decide based on obstacles ---
            obstacle_front = self._is_obstacle_front(world)

            if obstacle_front:
                # Check all directions
                if self._is_all_blocked(world):
                    stop_reason = "blocked"
                    actions.append("blocked")
                    break
                # Try turning ±90° to find a clear path
                turn_heading = world.heading_degrees + 90.0
                await robot.turn(turn_heading)
                world.heading_degrees = turn_heading
                actions.append("scan_alt")
                await self._poll_perception(world)

                # If still blocked after turning, retreat
                if self._is_obstacle_front(world):
                    step_m = params.step_distance_cm / 100.0
                    pos = world.position
                    rad = math.radians(world.heading_degrees + 180)
                    from robot_brain.core.world_state import Position
                    new_pos = Position(
                        x=pos.x + step_m * math.cos(rad),
                        y=pos.y + step_m * math.sin(rad),
                    )
                    await robot.move_to(new_pos, speed=0.15)
                    world.position = new_pos
                    actions.append("retreat")
                else:
                    # Clear after turning — nudge in new direction
                    step_m = params.step_distance_cm / 100.0
                    pos = world.position
                    rad = math.radians(world.heading_degrees)
                    from robot_brain.core.world_state import Position
                    new_pos = Position(
                        x=pos.x + step_m * math.cos(rad),
                        y=pos.y + step_m * math.sin(rad),
                    )
                    await robot.move_to(new_pos, speed=0.15)
                    world.position = new_pos
                    actions.append("nudge")
            else:
                # Nudge forward
                step_m = params.step_distance_cm / 100.0
                pos = world.position
                rad = math.radians(world.heading_degrees)
                from robot_brain.core.world_state import Position
                new_pos = Position(
                    x=pos.x + step_m * math.cos(rad),
                    y=pos.y + step_m * math.sin(rad),
                )
                await robot.move_to(new_pos, speed=0.15)
                world.position = new_pos
                actions.append("nudge")

            steps_done += 1

            # --- Periodic report ---
            if steps_done % params.report_every == 0:
                actions.append("report")

        success = stop_reason in ("max_steps", "blocked", "max_duration")
        return SkillResult(
            success=success,
            message=f"explore {'completed' if success else 'aborted'}: {steps_done} steps, stop_reason={stop_reason}",
            data={
                "skill": "explore",
                "steps_completed": steps_done,
                "actions": actions,
                "stop_reason": stop_reason,
            },
        )

    # ------------------------------------------------------------------
    # Go2 (unitree) backend
    # ------------------------------------------------------------------

    async def _run_go2_loop(
        self,
        params: ExploreParams,
        robot: Any,  # UnitreeRobot
        world: WorldState,
    ) -> SkillResult:
        from robot_brain.skills.builtin import go2_motion

        actions: list[str] = []
        segments_total = 0
        steps_done = 0
        start_time = time.monotonic()
        stop_reason = "max_steps"

        seg_dur = self._settings.unitree_max_drive_duration

        for step in range(params.max_steps):
            # --- Poll perception first (get fresh state for this step) ---
            await self._poll_perception(world)

            # --- Stop conditions ---
            sr = self._check_stop_conditions(world, start_time)
            if sr is not None:
                stop_reason = sr
                break

            # --- Precondition check ---
            reason = go2_motion.check_robot_self_state(world, self._settings)
            if reason is not None:
                stop_reason = f"precondition:{reason}"
                break

            # --- Scan (rotate) ---
            yaw_rad = math.radians(params.scan_degrees)
            vyaw = go2_motion.YAW_SPEED
            durations = go2_motion.plan_yaw_segments(yaw_rad, seg_dur)
            result = await go2_motion.run_go2_drive_segments(
                robot, vyaw=vyaw, durations=durations,
            )
            segments_total += result["segment_count"]
            actions.append("scan")

            if not result["success"]:
                stop_reason = "drive_error"
                break

            # --- Poll perception again (orientation changed) ---
            await self._poll_perception(world)

            # --- Decide based on obstacles ---
            obstacle_front = self._is_obstacle_front(world)

            if obstacle_front:
                if self._is_all_blocked(world):
                    stop_reason = "blocked"
                    actions.append("blocked")
                    break
                # Try turning 90° to find a clear path
                alt_yaw_rad = math.radians(90.0)
                alt_durations = go2_motion.plan_yaw_segments(alt_yaw_rad, seg_dur)
                alt_result = await go2_motion.run_go2_drive_segments(
                    robot, vyaw=go2_motion.YAW_SPEED, durations=alt_durations,
                )
                segments_total += alt_result["segment_count"]
                actions.append("scan_alt")

                if not alt_result["success"]:
                    stop_reason = "drive_error"
                    break

                await self._poll_perception(world)

                # If still blocked after turning, retreat
                if self._is_obstacle_front(world):
                    distance_m = params.step_distance_cm / 100.0
                    durations = go2_motion.plan_linear_segments(distance_m, seg_dur)
                    result = await go2_motion.run_go2_drive_segments(
                        robot, vx=-go2_motion.LINEAR_SPEED, durations=durations,
                    )
                    segments_total += result["segment_count"]
                    actions.append("retreat")
                else:
                    # Clear after turning — nudge in new direction
                    distance_m = params.step_distance_cm / 100.0
                    durations = go2_motion.plan_linear_segments(distance_m, seg_dur)
                    result = await go2_motion.run_go2_drive_segments(
                        robot, vx=go2_motion.LINEAR_SPEED, durations=durations,
                    )
                    segments_total += result["segment_count"]
                    actions.append("nudge")
            else:
                # Nudge forward
                distance_m = params.step_distance_cm / 100.0
                durations = go2_motion.plan_linear_segments(distance_m, seg_dur)
                result = await go2_motion.run_go2_drive_segments(
                    robot, vx=go2_motion.LINEAR_SPEED, durations=durations,
                )
                segments_total += result["segment_count"]
                actions.append("nudge")

            if not result["success"]:
                stop_reason = "drive_error"
                break

            steps_done += 1

            # --- Periodic report ---
            if steps_done % params.report_every == 0:
                actions.append("report")

        success = stop_reason in ("max_steps", "blocked", "max_duration")
        return SkillResult(
            success=success,
            message=f"explore {'completed' if success else 'aborted'}: {steps_done} steps, stop_reason={stop_reason}",
            data={
                "skill": "explore",
                "steps_completed": steps_done,
                "actions": actions,
                "stop_reason": stop_reason,
                "segments_total": segments_total,
            },
        )

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _check_stop_conditions(self, world: WorldState, start_time: float) -> str | None:
        """Return a stop_reason string if any hard stop condition is met."""
        # Time limit
        elapsed = time.monotonic() - start_time
        if elapsed >= self._settings.explore_max_duration:
            return "max_duration"

        # Battery
        if world.battery_level <= self._settings.low_battery_threshold:
            return "low_battery"

        # E-stop
        if world.estop_active:
            return "estop"

        # Robot error / stale
        ss = world.robot_self_state
        if ss is not None:
            if ss.error_code is not None and ss.error_code != 0:
                return "robot_error"
            if (
                ss.state_age_seconds is not None
                and ss.state_age_seconds > self._settings.unitree_state_max_age_seconds
            ):
                return "stale_state"

        return None

    def _has_proximity_data(self, world: WorldState) -> bool:
        """Return True if ultrasonic data is available."""
        ss = world.robot_self_state
        return ss is not None and ss.ultrasonic is not None and ss.ultrasonic.front_m is not None

    def _is_obstacle_front(self, world: WorldState) -> bool:
        """Check if there's an obstacle in front within proximity threshold.

        Returns True (blocked) when no proximity data is available — the
        conservative policy is to NOT move forward without sensor confirmation.
        """
        ss = world.robot_self_state
        if ss is None or ss.ultrasonic is None or ss.ultrasonic.front_m is None:
            return True  # No data → conservative: do NOT forward nudge
        threshold = self._settings.obstacle_proximity_threshold
        return ss.ultrasonic.front_m < threshold

    def _is_all_blocked(self, world: WorldState) -> bool:
        """Check if obstacles are detected on all sides.

        Returns False when data is unavailable — we only declare "all blocked"
        when ALL four sensors actively report a close reading.
        """
        ss = world.robot_self_state
        if ss is None or ss.ultrasonic is None:
            return False
        u = ss.ultrasonic
        threshold = self._settings.obstacle_proximity_threshold
        front_blocked = u.front_m is not None and u.front_m < threshold
        rear_blocked = u.rear_m is not None and u.rear_m < threshold
        left_blocked = u.left_m is not None and u.left_m < threshold
        right_blocked = u.right_m is not None and u.right_m < threshold
        return front_blocked and rear_blocked and left_blocked and right_blocked

    async def _poll_perception(self, world: WorldState) -> None:
        """If a perception adapter is injected, poll for fresh observations."""
        if self._perception is None:
            return
        try:
            observation = await self._perception.observe()
            if observation is not None:
                world.apply_observation(
                    observation,
                    object_ttl_seconds=self._settings.object_ttl_seconds,
                )
        except Exception as exc:
            logger.warning("explore: perception poll failed: %s", exc)
