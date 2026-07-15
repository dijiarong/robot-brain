"""Bounded Explore - composite skill combining scan/nudge/retreat in a loop.

Implements a rule-driven exploration cycle with hard stop conditions.
Works on both mock and Go2 (unitree) backends. Does NOT call nested Skills;
instead directly uses robot-level motion (MockRobot.move_to/turn or
go2_motion.run_go2_drive_segments) - matching the architecture decision in
docs/plans/2026-06-24-160000-bounded-explore-mode.md §1.

Iteration 17 added an optional VLM **passability hint** as a soft direction
suggestion. Ultrasonic proximity remains the hard safety gate; the VLM only
chooses the alt-scan direction (left vs right) and can withhold a forward
nudge ("stop"). When no analyzer is injected, behavior is identical to iter 16.

Iteration 18 adds a structured **step trace** (per-step sensor/hint/decision
audit) and **stop protection** (no_progress / semantic_hold / ping_pong) so
explore is explainable, stoppable, and field-verifiable. See
docs/plans/2026-07-11-204545-explore-field-verification-loop.md.
"""
from __future__ import annotations

import logging
import math
import time
from typing import TYPE_CHECKING, Any

from pydantic import BaseModel, Field

from config.settings import Settings
from robot_brain.actuation.base import RobotInterface
from robot_brain.core.world_state import WorldState
from robot_brain.skills.base import Skill, SkillResult

if TYPE_CHECKING:
    from robot_brain.core.passability import PassabilityHint
    from robot_brain.vlm.passability import PassabilityAnalyzer

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
# Step trace (iteration 18) - per-step audit for field verification
# ---------------------------------------------------------------------------

class ExploreStepTrace(BaseModel):
    step_index: int
    heading_before: float | None = None
    heading_after: float | None = None
    ultrasonic: dict[str, float | None] = Field(default_factory=dict)
    passability_hint: dict[str, Any] | None = None
    chosen_action: str
    fallback_reason: str = ""
    stop_reason: str | None = None
    pose_before: dict[str, Any] | None = None
    pose_after: dict[str, Any] | None = None
    motion_delta: dict[str, Any] | None = None
    progress_source: str = "behavior"
    #: Go2 drive-segment audit, split by phase (empty on mock / when unused).
    scan_segments: list[dict[str, Any]] = Field(default_factory=list)
    alt_scan_segments: list[dict[str, Any]] = Field(default_factory=list)
    move_segments: list[dict[str, Any]] = Field(default_factory=list)
    duration_ms: float | None = None


class _ExploreGuard:
    """Behavior-trace stop protection (no odometry/SLAM).

    Tracks consecutive non-progress steps, VLM holds, and left/right
    alternation. Returns a stop_reason when a threshold is hit.
    """

    def __init__(self, settings: Settings) -> None:
        self._settings = settings
        self._no_progress = 0
        self._holds = 0
        self._alt_dirs: list[str] = []

    def update(
        self,
        primary_action: str,
        alt_dir: str | None,
        *,
        progress_made: bool | None = None,
    ) -> str | None:
        """Record a completed step; return a stop_reason if a guard triggers."""
        if progress_made is None:
            progress_made = primary_action == "nudge"

        if progress_made:
            self._no_progress = 0
            self._holds = 0
        else:
            self._no_progress += 1
            self._holds = self._holds + 1 if primary_action == "vlm_hold" else 0

        if alt_dir is not None:
            self._alt_dirs.append(alt_dir)

        if self._holds >= self._settings.explore_max_holds:
            return "semantic_hold"
        if self._no_progress >= self._settings.explore_no_progress_steps:
            return "no_progress"
        if self._is_ping_pong():
            return "ping_pong"
        return None

    def _is_ping_pong(self) -> bool:
        n = self._settings.explore_ping_pong_steps
        dirs = self._alt_dirs
        if len(dirs) < n:
            return False
        recent = dirs[-n:]
        if any(d not in ("left", "right") for d in recent):
            return False
        return all(recent[i] != recent[i - 1] for i in range(1, len(recent)))


# ---------------------------------------------------------------------------
# ExploreSkill
# ---------------------------------------------------------------------------

class ExploreSkill(Skill):
    """Bounded exploration: scan -> decide -> move, repeated up to max_steps."""

    name = "explore"
    description = (
        "Explore the surroundings by scanning and moving in short increments. "
        "Bounded by max_steps (1–20) and time. Stops on obstacles, low battery, or errors. "
        "Use when no specific navigation target is given."
    )
    params_model = ExploreParams

    def __init__(
        self,
        settings: Settings,
        *,
        perception: Any | None = None,
        passability: "PassabilityAnalyzer | None" = None,
    ) -> None:
        self._settings = settings
        self._perception = perception  # Optional PerceptionAdapter for live polling
        self._passability = passability  # Optional VLM passability analyzer
        # Diagnostics (populated by execute, exposed via diagnostics()).
        self._last_stop_reason: str | None = None
        self._last_steps_completed: int = 0
        self._last_trace: list[ExploreStepTrace] = []

    def diagnostics(self) -> dict[str, Any]:
        """Snapshot for the service status API."""
        return {
            "last_stop_reason": self._last_stop_reason,
            "last_steps_completed": self._last_steps_completed,
            "last_trace_count": len(self._last_trace),
        }

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
        trace: list[ExploreStepTrace] = []
        guard = _ExploreGuard(self._settings)
        steps_done = 0
        start_time = time.monotonic()
        stop_reason = "max_steps"

        for step in range(params.max_steps):
            heading_before = world.heading_degrees
            step_start = time.monotonic()

            # --- Poll perception first (get fresh state for this step) ---
            await self._poll_perception(world)
            pose_before = self._pose_snapshot(world)

            # --- Hard stop conditions (pre-step; no trace entry) ---
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

            # --- VLM soft hint (one call per step, at the decision point) ---
            hint = await self._maybe_analyze_passability(world)

            chosen_action, alt_dir, fallback_reason = await self._decide_and_move_mock(
                params, robot, world, hint, actions
            )

            heading_after = world.heading_degrees
            if chosen_action == "blocked":
                trace.append(self._make_trace(
                    step, heading_before, heading_after, world, hint,
                    chosen_action, fallback_reason, "blocked", [], [], [], step_start,
                    pose_before,
                ))
                stop_reason = "blocked"
                break
            pose_after = self._pose_snapshot(world)
            progress_made, progress_source = self._progress_from_delta(chosen_action, pose_before, pose_after)
            guard_stop = guard.update(chosen_action, alt_dir, progress_made=progress_made)
            trace.append(self._make_trace(
                step, heading_before, heading_after, world, hint,
                chosen_action, fallback_reason, guard_stop, [], [], [], step_start,
                pose_before, progress_source,
            ))
            steps_done += 1
            if guard_stop is not None:
                stop_reason = guard_stop
                break

            if steps_done % params.report_every == 0:
                actions.append("report")

        return self._finalize(steps_done, actions, trace, stop_reason)

    async def _decide_and_move_mock(
        self,
        params: ExploreParams,
        robot: RobotInterface,
        world: WorldState,
        hint: "PassabilityHint | None",
        actions: list[str],
    ) -> tuple[str, str | None, str]:
        """Execute the mock motion decision; return (chosen_action, alt_dir, fallback_reason)."""
        obstacle_front = self._is_obstacle_front(world)
        if not obstacle_front:
            if self._should_nudge_forward(hint):
                await self._mock_nudge(robot, world, params, forward=True)
                actions.append("nudge")
                return "nudge", None, ""
            actions.append("vlm_hold")
            return "vlm_hold", None, "vlm_stop"

        if self._is_all_blocked(world):
            actions.append("blocked")
            return "blocked", None, "all_blocked"

        alt_angle, alt_tag, alt_reason = self._choose_alt_turn(world, hint)
        alt_dir = self._alt_dir(alt_tag)
        turn_heading = world.heading_degrees + alt_angle
        await robot.turn(turn_heading)
        world.heading_degrees = turn_heading
        actions.append(alt_tag)
        await self._poll_perception(world)

        if self._is_obstacle_front(world):
            await self._mock_nudge(robot, world, params, forward=False)
            actions.append("retreat")
            return "retreat", alt_dir, alt_reason
        if self._should_nudge_forward(hint):
            await self._mock_nudge(robot, world, params, forward=True)
            actions.append("nudge")
            return "nudge", alt_dir, alt_reason
        actions.append("vlm_hold")
        return "vlm_hold", alt_dir, "vlm_stop"

    @staticmethod
    async def _mock_nudge(
        robot: RobotInterface, world: WorldState, params: ExploreParams, *, forward: bool
    ) -> None:
        step_m = params.step_distance_cm / 100.0
        pos = world.position
        heading = world.heading_degrees if forward else world.heading_degrees + 180
        rad = math.radians(heading)
        from robot_brain.core.world_state import Position

        new_pos = Position(x=pos.x + step_m * math.cos(rad), y=pos.y + step_m * math.sin(rad))
        await robot.move_to(new_pos, speed=0.15)
        world.position = new_pos

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
        trace: list[ExploreStepTrace] = []
        guard = _ExploreGuard(self._settings)
        segments_total = 0
        steps_done = 0
        start_time = time.monotonic()
        stop_reason = "max_steps"

        seg_dur = self._settings.unitree_max_drive_duration

        for step in range(params.max_steps):
            heading_before = world.heading_degrees
            step_start = time.monotonic()

            # --- Poll perception first ---
            await self._poll_perception(world)
            pose_before = self._pose_snapshot(world)

            # --- Hard stop conditions (pre-step) ---
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
            durations = go2_motion.plan_yaw_segments(yaw_rad, seg_dur)
            scan_result = await go2_motion.run_go2_drive_segments(
                robot, vyaw=go2_motion.YAW_SPEED, durations=durations,
            )
            scan_segments = list(scan_result.get("segments", []))
            actions.append("scan")
            if not scan_result["success"]:
                trace.append(self._make_trace(
                    step, heading_before, world.heading_degrees, world, None,
                    "drive_error", "", "drive_error", scan_segments, [], [], step_start,
                    pose_before,
                ))
                stop_reason = "drive_error"
                break

            # --- Poll perception again (orientation changed) ---
            await self._poll_perception(world)

            # --- VLM soft hint ---
            hint = await self._maybe_analyze_passability(world)

            chosen_action, alt_dir, fallback_reason, alt_segs, move_segs = await self._decide_and_move_go2(
                params, robot, world, hint, actions, seg_dur,
            )
            if chosen_action not in ("drive_error", "blocked"):
                await self._poll_perception(world)
            heading_after = world.heading_degrees
            if chosen_action == "drive_error":
                trace.append(self._make_trace(
                    step, heading_before, heading_after, world, hint,
                    chosen_action, fallback_reason, "drive_error",
                    scan_segments, alt_segs, [], step_start,
                    pose_before,
                ))
                stop_reason = "drive_error"
                break
            if chosen_action == "blocked":
                trace.append(self._make_trace(
                    step, heading_before, heading_after, world, hint,
                    chosen_action, fallback_reason, "blocked",
                    scan_segments, [], [], step_start,
                    pose_before,
                ))
                stop_reason = "blocked"
                break
            segments_total += len(scan_segments) + len(alt_segs) + len(move_segs)
            pose_after = self._pose_snapshot(world)
            progress_made, progress_source = self._progress_from_delta(chosen_action, pose_before, pose_after)
            guard_stop = guard.update(chosen_action, alt_dir, progress_made=progress_made)
            trace.append(self._make_trace(
                step, heading_before, heading_after, world, hint,
                chosen_action, fallback_reason, guard_stop,
                scan_segments, alt_segs, move_segs, step_start,
                pose_before, progress_source,
            ))
            steps_done += 1
            if guard_stop is not None:
                stop_reason = guard_stop
                break

            if steps_done % params.report_every == 0:
                actions.append("report")

        return self._finalize(steps_done, actions, trace, stop_reason, segments_total=segments_total)

    async def _decide_and_move_go2(
        self,
        params: ExploreParams,
        robot: Any,
        world: WorldState,
        hint: "PassabilityHint | None",
        actions: list[str],
        seg_dur: float,
    ) -> tuple[str, str | None, str, list[dict[str, Any]], list[dict[str, Any]]]:
        """Execute the Go2 motion decision.

        Returns ``(chosen_action, alt_dir, reason, alt_scan_segments,
        move_segments)``. ``chosen_action == "drive_error"`` signals a failed
        alt/move drive (caller records a trace and stops).
        """
        from robot_brain.skills.builtin import go2_motion

        obstacle_front = self._is_obstacle_front(world)
        if not obstacle_front:
            if self._should_nudge_forward(hint):
                segs = await self._go2_drive(robot, params, go2_motion, seg_dur, forward=True)
                actions.append("nudge")
                if segs is None:
                    return "drive_error", None, "", [], []
                return "nudge", None, "", [], segs
            actions.append("vlm_hold")
            return "vlm_hold", None, "vlm_stop", [], []

        if self._is_all_blocked(world):
            actions.append("blocked")
            return "blocked", None, "all_blocked", [], []

        alt_angle, alt_tag, alt_reason = self._choose_alt_turn(world, hint)
        alt_dir = self._alt_dir(alt_tag)
        alt_yaw_rad = math.radians(abs(alt_angle))
        alt_vyaw = go2_motion.YAW_SPEED if alt_angle >= 0 else -go2_motion.YAW_SPEED
        alt_durations = go2_motion.plan_yaw_segments(alt_yaw_rad, seg_dur)
        alt_result = await go2_motion.run_go2_drive_segments(
            robot, vyaw=alt_vyaw, durations=alt_durations,
        )
        alt_segs = list(alt_result.get("segments", []))
        actions.append(alt_tag)
        if not alt_result["success"]:
            return "drive_error", alt_dir, alt_reason, alt_segs, []

        await self._poll_perception(world)

        if self._is_obstacle_front(world):
            segs = await self._go2_drive(robot, params, go2_motion, seg_dur, forward=False)
            actions.append("retreat")
            if segs is None:
                return "drive_error", alt_dir, alt_reason, alt_segs, []
            return "retreat", alt_dir, alt_reason, alt_segs, segs
        if self._should_nudge_forward(hint):
            segs = await self._go2_drive(robot, params, go2_motion, seg_dur, forward=True)
            actions.append("nudge")
            if segs is None:
                return "drive_error", alt_dir, alt_reason, alt_segs, []
            return "nudge", alt_dir, alt_reason, alt_segs, segs
        actions.append("vlm_hold")
        return "vlm_hold", alt_dir, "vlm_stop", alt_segs, []

    @staticmethod
    async def _go2_drive(
        robot: Any, params: ExploreParams, go2_motion: Any, seg_dur: float, *, forward: bool
    ) -> list[dict[str, Any]] | None:
        distance_m = params.step_distance_cm / 100.0
        durations = go2_motion.plan_linear_segments(distance_m, seg_dur)
        vx = go2_motion.LINEAR_SPEED if forward else -go2_motion.LINEAR_SPEED
        result = await go2_motion.run_go2_drive_segments(robot, vx=vx, durations=durations)
        if not result["success"]:
            return None
        return list(result.get("segments", []))

    # ------------------------------------------------------------------
    # Shared helpers
    # ------------------------------------------------------------------

    def _make_trace(
        self,
        step_index: int,
        heading_before: float,
        heading_after: float,
        world: WorldState,
        hint: "PassabilityHint | None",
        chosen_action: str,
        fallback_reason: str,
        stop_reason: str | None,
        scan_segments: list[dict[str, Any]],
        alt_scan_segments: list[dict[str, Any]],
        move_segments: list[dict[str, Any]],
        step_start: float,
        pose_before: dict[str, Any] | None = None,
        progress_source: str = "behavior",
    ) -> ExploreStepTrace:
        pose_after = self._pose_snapshot(world)
        motion_delta = self._motion_delta(pose_before, pose_after)
        return ExploreStepTrace(
            step_index=step_index,
            heading_before=heading_before,
            heading_after=heading_after,
            ultrasonic=self._ultrasonic_snapshot(world),
            passability_hint=self._hint_summary(hint),
            chosen_action=chosen_action,
            fallback_reason=fallback_reason,
            stop_reason=stop_reason,
            pose_before=pose_before,
            pose_after=pose_after,
            motion_delta=motion_delta,
            progress_source=progress_source,
            scan_segments=scan_segments,
            alt_scan_segments=alt_scan_segments,
            move_segments=move_segments,
            duration_ms=round((time.monotonic() - step_start) * 1000.0, 1),
        )

    @staticmethod
    def _pose_snapshot(world: WorldState) -> dict[str, Any] | None:
        ss = world.robot_self_state
        if ss is None or ss.odometry is None or ss.odometry.pose is None:
            return None
        p = ss.odometry.pose
        return p.model_dump(mode="json")

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

    def _progress_from_delta(
        self,
        chosen_action: str,
        pose_before: dict[str, Any] | None,
        pose_after: dict[str, Any] | None,
    ) -> tuple[bool | None, str]:
        delta = self._motion_delta(pose_before, pose_after)
        if delta is None:
            return None, "behavior"
        if chosen_action != "nudge":
            return False, "odom"
        return delta["delta_m"] >= self._settings.odom_progress_min_m, "odom"

    @staticmethod
    def _ultrasonic_snapshot(world: WorldState) -> dict[str, float | None]:
        ss = world.robot_self_state
        if ss is None or ss.ultrasonic is None:
            return {}
        u = ss.ultrasonic
        return {"front_m": u.front_m, "rear_m": u.rear_m, "left_m": u.left_m, "right_m": u.right_m}

    @staticmethod
    def _hint_summary(hint: "PassabilityHint | None") -> dict[str, Any] | None:
        if hint is None:
            return None
        return {
            "recommended_direction": hint.recommended_direction,
            "confidence": hint.confidence,
            "reason": hint.reason,
        }

    @staticmethod
    def _alt_dir(alt_tag: str) -> str | None:
        if alt_tag == "scan_alt_left":
            return "left"
        if alt_tag == "scan_alt_right":
            return "right"
        return None

    def _finalize(
        self,
        steps_done: int,
        actions: list[str],
        trace: list[ExploreStepTrace],
        stop_reason: str,
        *,
        segments_total: int = 0,
    ) -> SkillResult:
        success = stop_reason in ("max_steps", "blocked", "max_duration")
        # Update diagnostics.
        self._last_stop_reason = stop_reason
        self._last_steps_completed = steps_done
        self._last_trace = trace
        data: dict[str, Any] = {
            "skill": "explore",
            "steps_completed": steps_done,
            "actions": actions,
            "trace": [t.model_dump(mode="json") for t in trace],
            "stop_reason": stop_reason,
        }
        if segments_total:
            data["segments_total"] = segments_total
        return SkillResult(
            success=success,
            message=f"explore {'completed' if success else 'aborted'}: {steps_done} steps, stop_reason={stop_reason}",
            data=data,
        )

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

        Returns True (blocked) when no proximity data is available - the
        conservative policy is to NOT move forward without sensor confirmation.
        """
        ss = world.robot_self_state
        if ss is None or ss.ultrasonic is None or ss.ultrasonic.front_m is None:
            return True  # No data -> conservative: do NOT forward nudge
        threshold = self._settings.obstacle_proximity_threshold
        return ss.ultrasonic.front_m < threshold

    def _is_all_blocked(self, world: WorldState) -> bool:
        """Check if obstacles are detected on all sides.

        Returns False when data is unavailable - we only declare "all blocked"
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

    async def _maybe_analyze_passability(self, world: WorldState) -> "PassabilityHint | None":
        """Ask the VLM for a passability hint if an analyzer is wired.

        Returns None when no analyzer is configured or the call fails/skipped;
        the caller then falls back to rule-based direction choice.
        """
        if self._passability is None:
            return None
        try:
            return await self._passability.analyze_if_due(world)
        except Exception as exc:
            logger.warning("explore: passability analyze failed: %s", exc)
            return None

    def _hint_usable(self, hint: "PassabilityHint | None") -> bool:
        """A hint counts only if confident enough; otherwise fall back to rules."""
        return hint is not None and hint.confidence >= self._settings.vlm_confidence_min

    def _choose_alt_turn(
        self, world: WorldState, hint: "PassabilityHint | None"
    ) -> tuple[float, str, str]:
        """Pick the alt-scan direction: VLM soft hint -> ultrasonic -> +90 default.

        Returns ``(turn_angle_deg, action_tag, reason)``. Positive angle = CCW =
        left turn (matches ScanSkill's sign convention); negative = CW = right.
        ``reason`` is one of ``"vlm"`` / ``"ultrasonic"`` / ``"default"``.
        """
        if self._hint_usable(hint):
            direction = hint.recommended_direction  # type: ignore[union-attr]
            if direction == "left":
                return 90.0, "scan_alt_left", "vlm"
            if direction == "right":
                return -90.0, "scan_alt_right", "vlm"
            # "forward"/"stop" carry no alt-direction preference -> fall through
        ss = world.robot_self_state
        if ss is not None and ss.ultrasonic is not None:
            u = ss.ultrasonic
            if u.left_m is not None and u.right_m is not None:
                if u.left_m > u.right_m:
                    return 90.0, "scan_alt_left", "ultrasonic"
                if u.right_m > u.left_m:
                    return -90.0, "scan_alt_right", "ultrasonic"
        return 90.0, "scan_alt", "default"

    def _should_nudge_forward(self, hint: "PassabilityHint | None") -> bool:
        """Front is clear; a usable VLM 'stop' withholds the forward nudge."""
        if self._hint_usable(hint) and hint.recommended_direction == "stop":  # type: ignore[union-attr]
            return False
        return True

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
