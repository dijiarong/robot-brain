"""Bounded continuous visual-servo orchestration through NavigationClient."""
from __future__ import annotations

import asyncio
from dataclasses import dataclass
import inspect
import math
import time
from typing import Awaitable, Callable

from robot_brain.navigation.base import NavigationClient, NavigationStatus, RelativeNavigationGoal
from robot_brain.navigation.visual_navigation import (
    CameraIntrinsics,
    compute_visual_servo,
    compute_visual_servo_3d,
    robust_target_from_points,
)


@dataclass(frozen=True)
class VisualTargetObservation:
    observed_monotonic: float
    bbox: tuple[float, float, float, float] | None = None
    points_body_xyz: tuple[tuple[float, float, float], ...] = ()
    confidence: float = 1.0


@dataclass(frozen=True)
class VisualServoResult:
    completed: bool
    canceled: bool
    stop_reason: str
    iterations: int
    commands: int
    lost_frames: int
    final_distance_m: float | None


ObservationProvider = Callable[
    [], VisualTargetObservation | None | Awaitable[VisualTargetObservation | None]
]
VisualEventSink = Callable[[str, dict[str, object]], None]


class ContinuousVisualServoController:
    def __init__(
        self, navigation: NavigationClient, observations: ObservationProvider,
        camera: CameraIntrinsics, *, max_observation_age_s: float = 0.5,
        observation_timeout_s: float = 10.0,
        minimum_confidence: float = 0.5, command_horizon_s: float = 0.25,
        target_distance_m: float = 1.5, distance_tolerance_m: float = 0.15,
        stable_frames: int = 3, max_lost_frames: int = 2,
        max_iterations: int = 100, poll_interval_s: float = 0.02,
        event_sink: VisualEventSink | None = None,
    ) -> None:
        if any(value <= 0 or not math.isfinite(value) for value in (
            max_observation_age_s, observation_timeout_s, command_horizon_s, target_distance_m,
            distance_tolerance_m, poll_interval_s,
        )) or not 0 <= minimum_confidence <= 1:
            raise ValueError("invalid visual servo safety limits")
        if stable_frames <= 0 or max_lost_frames < 0 or max_iterations <= 0:
            raise ValueError("invalid visual servo iteration limits")
        self._navigation = navigation
        self._observations = observations
        self._camera = camera
        self._max_age = max_observation_age_s
        self._observation_timeout = observation_timeout_s
        self._min_confidence = minimum_confidence
        self._horizon = command_horizon_s
        self._target_distance = target_distance_m
        self._distance_tolerance = distance_tolerance_m
        self._stable_required = stable_frames
        self._max_lost = max_lost_frames
        self._max_iterations = max_iterations
        self._poll_interval = poll_interval_s
        self._event_sink = event_sink
        self._cancel = asyncio.Event()
        self._active_goal: str | None = None

    async def cancel(self) -> None:
        self._cancel.set()
        if self._active_goal is not None:
            await self._navigation.cancel(self._active_goal)

    async def run(self) -> VisualServoResult:
        self._cancel = asyncio.Event()
        commands = lost = stable = 0
        final_distance = None
        for iteration in range(1, self._max_iterations + 1):
            if self._cancel.is_set():
                return self._finish(False, True, "canceled", iteration-1,
                                    commands, lost, final_distance)
            try:
                observation = await self._next_observation()
                reason = self._observation_error(observation)
            except TimeoutError:
                observation, reason = None, "detection_timeout"
            except Exception:
                observation, reason = None, "detection_provider_error"
            if reason is not None:
                self._event("observation_rejected", reason=reason, lost_frames=lost+1)
                lost += 1
                stable = 0
                if lost > self._max_lost:
                    await self._stop_active_goal()
                    return self._finish(False, False, reason, iteration,
                                        commands, lost, final_distance)
                await asyncio.sleep(self._poll_interval)
                continue
            assert observation is not None
            lost = 0
            bbox_center_error = (
                (((observation.bbox[0]+observation.bbox[2])/2)-self._camera.cx)
                / self._camera.fx if observation.bbox is not None else None
            )
            if observation.points_body_xyz:
                target = robust_target_from_points(observation.points_body_xyz)
                command = (
                    compute_visual_servo_3d(target, target_distance_m=self._target_distance)
                    if target is not None else None
                )
            else:
                command = (
                    compute_visual_servo(
                        observation.bbox, self._camera,
                        target_distance_m=self._target_distance,
                    ) if observation.bbox is not None else None
                )
            if command is None or not command.valid or command.estimated_distance_m is None:
                await self._stop_active_goal()
                return self._finish(False, False, "invalid_visual_projection",
                                    iteration, commands, lost, final_distance)
            final_distance = command.estimated_distance_m
            centered = abs(command.yaw_rps) <= 0.05
            at_distance = abs(final_distance-self._target_distance) <= self._distance_tolerance
            next_stable = stable+1 if centered and at_distance else 0
            self._event(
                "observation", confidence=observation.confidence,
                age_s=time.monotonic()-observation.observed_monotonic,
                normalized_center_error=bbox_center_error,
                estimated_distance_m=final_distance, centered=centered,
                at_distance=at_distance, stable_frames=next_stable,
                source="points_3d" if observation.points_body_xyz else "bbox_2d",
            )
            if centered and at_distance:
                stable += 1
                if stable >= self._stable_required:
                    return self._finish(True, False, "target_reached", iteration,
                                        commands, lost, final_distance)
                await asyncio.sleep(self._poll_interval)
                continue
            stable = 0
            goal = RelativeNavigationGoal(
                forward_m=max(-3.0, min(3.0, command.forward_mps*self._horizon)),
                yaw_degrees=max(-90.0, min(90.0, math.degrees(command.yaw_rps*self._horizon))),
                max_duration_s=max(0.5, min(5.0, self._horizon*4.0)),
            )
            handle = await self._navigation.set_relative_goal(goal)
            if not handle.accepted:
                return self._finish(False, False, "navigation_rejected", iteration,
                                    commands, lost, final_distance)
            commands += 1
            self._event(
                "command", command_index=commands, forward_m=goal.forward_m,
                yaw_degrees=goal.yaw_degrees, estimated_distance_m=final_distance,
            )
            self._active_goal = handle.goal_id
            status = await self._wait_terminal()
            self._active_goal = None
            self._event("navigation_terminal", command_index=commands, status=status.value)
            if self._cancel.is_set() or status == NavigationStatus.CANCELED:
                return self._finish(False, True, "canceled", iteration,
                                    commands, lost, final_distance)
            if status != NavigationStatus.SUCCEEDED:
                return self._finish(False, False, f"navigation_{status.value}",
                                    iteration, commands, lost, final_distance)
        await self._stop_active_goal()
        return self._finish(False, False, "max_iterations", self._max_iterations,
                            commands, lost, final_distance)

    async def _next_observation(self) -> VisualTargetObservation | None:
        value = self._observations()
        return (
            await asyncio.wait_for(value, timeout=self._observation_timeout)
            if inspect.isawaitable(value) else value
        )

    def _observation_error(self, value: VisualTargetObservation | None) -> str | None:
        if value is None:
            return "target_lost"
        if not math.isfinite(value.observed_monotonic):
            return "invalid_detection_timestamp"
        age = time.monotonic()-value.observed_monotonic
        if age < 0 or age > self._max_age:
            return "stale_detection"
        if not math.isfinite(value.confidence) or value.confidence < self._min_confidence:
            return "low_confidence_detection"
        if value.bbox is None and not value.points_body_xyz:
            return "target_lost"
        return None

    async def _wait_terminal(self) -> NavigationStatus:
        while True:
            if self._cancel.is_set():
                await self._navigation.cancel(self._active_goal)
            state = await self._navigation.get_state()
            if state.status.terminal:
                return state.status
            await asyncio.sleep(self._poll_interval)

    async def _stop_active_goal(self) -> None:
        if self._active_goal is not None:
            await self._navigation.cancel(self._active_goal)

    def _finish(self, completed, canceled, reason, iterations, commands, lost, distance):
        result = VisualServoResult(completed, canceled, reason, iterations,
                                   commands, lost, distance)
        self._event("finished", **result.__dict__,
                    stable_frames_required=self._stable_required)
        return result

    def _event(self, kind: str, **fields: object) -> None:
        if self._event_sink is None:
            return
        try:
            self._event_sink(f"visual_servo_{kind}", fields)
        except Exception:
            # Diagnostics must not prevent the controller from issuing a stop.
            pass


def evaluate_visual_servo_trace(events) -> dict[str, object]:
    """Require a complete, converged visual observation/control evidence chain."""
    observations = [row for row in events if row.get("event") == "visual_servo_observation"]
    commands = [row for row in events if row.get("event") == "visual_servo_command"]
    terminals = [row for row in events if row.get("event") == "visual_servo_navigation_terminal"]
    finished = [row for row in events if row.get("event") == "visual_servo_finished"]
    failures: list[str] = []
    if not observations:
        failures.append("no_valid_visual_observations")
    if len(terminals) != len(commands):
        failures.append("incomplete_navigation_command_chain")
    if any(row.get("status") != NavigationStatus.SUCCEEDED.value for row in terminals):
        failures.append("visual_navigation_segment_failed")
    final = finished[-1] if finished else None
    if final is None or not final.get("completed") or final.get("stop_reason") != "target_reached":
        failures.append("target_not_stably_reached")
    elif not observations or observations[-1].get("stable_frames") != final.get(
        "stable_frames_required"
    ):
        failures.append("stable_frame_evidence_incomplete")
    distances = [float(row["estimated_distance_m"]) for row in observations
                 if isinstance(row.get("estimated_distance_m"), (int, float))]
    center_errors = [abs(float(row["normalized_center_error"])) for row in observations
                     if isinstance(row.get("normalized_center_error"), (int, float))]
    if commands and len(observations) < 2:
        failures.append("target_not_reacquired_after_motion")
    return {
        "ok": not failures, "failures": failures,
        "observations": len(observations), "commands": len(commands),
        "navigation_terminals": len(terminals),
        "initial_distance_m": distances[0] if distances else None,
        "final_distance_m": distances[-1] if distances else None,
        "initial_center_error": center_errors[0] if center_errors else None,
        "final_center_error": center_errors[-1] if center_errors else None,
        "stop_reason": final.get("stop_reason") if final else None,
    }
