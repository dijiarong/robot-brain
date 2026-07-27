"""Unitree robot adapter implementing RobotInterface with safety clamps."""
from __future__ import annotations

import asyncio
import logging
import math
import time
from abc import ABC, abstractmethod
from collections.abc import Callable
from typing import Any

from pydantic import BaseModel, Field

from config.settings import Settings
from robot_brain.actuation.base import RobotInterface, RobotState
from robot_brain.core.world_state import Position

logger = logging.getLogger(__name__)


class UnitreeState(BaseModel):
    """Raw state read from the Unitree SDK transport."""

    connected: bool = False
    battery_level: float = 100.0
    position: Position = Field(default_factory=Position)
    heading_degrees: float = 0.0
    is_standing: bool = False
    is_moving: bool = False
    error_code: int = 0
    # Go2 SportModeState.mode when available (WebRTC / SDK).
    sport_mode: int | None = None
    # Body-frame velocity (vx, vy, vz) in m/s — raw Go2 format.
    velocity: tuple[float, float, float] = (0.0, 0.0, 0.0)
    # IMU orientation (roll, pitch, yaw) in radians — raw Go2 format.
    imu_rpy: tuple[float, float, float] = (0.0, 0.0, 0.0)
    # Ultrasonic distances in metres (front, left, right, rear) — Go2 LowState.
    ultrasonic: tuple[float, float, float, float] | None = None
    pose_frame_id: str = "world"
    pose_timestamp: float | None = None
    pose_source: str = "sport_state"


class UnitreeCommand(BaseModel):
    """Command DTO sent to the transport layer."""

    action: str
    parameters: dict[str, Any] = Field(default_factory=dict)
    timestamp: float = Field(default_factory=time.time)


class UnitreeTransport(ABC):
    """Abstract transport layer — real SDK or fake for testing."""

    @abstractmethod
    async def connect(self) -> None: ...

    @abstractmethod
    async def disconnect(self) -> None: ...

    @abstractmethod
    async def read_state(self) -> UnitreeState: ...

    @abstractmethod
    async def send_command(self, command: UnitreeCommand) -> bool: ...

    @property
    @abstractmethod
    def is_connected(self) -> bool: ...

    def state_age_seconds(self) -> float:
        """Seconds since the last sport-state callback (inf if never received)."""
        return float("inf")

    def odometry_age_seconds(self) -> float:
        """Age of the navigation pose; defaults to general state freshness."""
        return self.state_age_seconds()


class FakeUnitreeTransport(UnitreeTransport):
    """In-memory fake transport for CI and unit tests."""

    def __init__(self, initial_state: UnitreeState | None = None) -> None:
        self._state = initial_state or UnitreeState(connected=True, is_standing=True)
        self._connected = False
        self.command_log: list[UnitreeCommand] = []
        self.fail_next: bool = False

    @property
    def is_connected(self) -> bool:
        return self._connected

    async def connect(self) -> None:
        self._connected = True
        self._state.connected = True

    async def disconnect(self) -> None:
        self._connected = False
        self._state.connected = False

    async def read_state(self) -> UnitreeState:
        if not self._connected:
            raise ConnectionError("Transport not connected")
        return self._state.model_copy(deep=True)

    def state_age_seconds(self) -> float:
        """Fake transport always reports fresh state."""
        return 0.0

    async def send_command(self, command: UnitreeCommand) -> bool:
        if not self._connected:
            raise ConnectionError("Transport not connected")
        if self.fail_next:
            self.fail_next = False
            raise RuntimeError(f"Simulated transport failure for {command.action}")
        self.command_log.append(command)
        self._apply_command(command)
        return True

    def _apply_command(self, command: UnitreeCommand) -> None:
        if command.action == "stop":
            self._state.is_moving = False
            self._state.velocity = (0.0, 0.0, 0.0)
        elif command.action == "move":
            target = command.parameters.get("target", {})
            self._state.position = Position(x=target.get("x", 0), y=target.get("y", 0))
            self._state.is_moving = False
        elif command.action == "turn":
            self._state.heading_degrees = command.parameters.get("heading_degrees", 0)
        elif command.action == "drive":
            vx = float(command.parameters.get("vx", 0.0))
            vy = float(command.parameters.get("vy", 0.0))
            vyaw = float(command.parameters.get("vyaw", 0.0))
            duration = float(command.parameters.get("duration", 0.0))
            heading_rad = math.radians(self._state.heading_degrees)
            dx_body = vx * duration
            dy_body = vy * duration
            dx_world = dx_body * math.cos(heading_rad) - dy_body * math.sin(heading_rad)
            dy_world = dx_body * math.sin(heading_rad) + dy_body * math.cos(heading_rad)
            self._state.position = Position(
                x=self._state.position.x + dx_world,
                y=self._state.position.y + dy_world,
            )
            self._state.heading_degrees += math.degrees(vyaw * duration)
            self._state.is_moving = False
            self._state.velocity = (0.0, 0.0, 0.0)


class UnitreeRobot(RobotInterface):
    """Unitree adapter with internal safety clamps and dry-run support."""

    # Non-translating posture commands supported by live transports.
    ALLOWED_POSTURES = frozenset(
        {
            "balance_stand",
            "stand_up",
            "stand_down",
            "recovery_stand",
            "damp",
            "sit",
            "free_walk",
            "hello",
        }
    )

    def __init__(self, transport: UnitreeTransport, settings: Settings) -> None:
        self._transport = transport
        self._settings = settings
        self._action_history: list[dict[str, Any]] = []
        self._stopped = False

    @property
    def action_history(self) -> list[dict[str, Any]]:
        return self._action_history

    @property
    def transport(self) -> UnitreeTransport:
        """Expose the underlying transport for perception adapters."""
        return self._transport

    @property
    def dry_run(self) -> bool:
        return self._settings.unitree_dry_run

    async def get_state(self) -> RobotState:
        try:
            raw = await self._transport.read_state()
        except Exception as exc:
            logger.error("Failed to read Unitree state: %s", exc)
            self._record("get_state", success=False, error=str(exc))
            return RobotState(stopped=True)
        self._record("get_state", success=True, raw=raw.model_dump(mode="json"))
        return RobotState(
            position=raw.position.model_copy(deep=True),
            heading_degrees=raw.heading_degrees,
            battery_level=raw.battery_level,
            stopped=not raw.is_moving,
        )

    async def stop(self, reason: str = "") -> None:
        self._record("stop", reason=reason)
        self._stopped = True
        if self.dry_run:
            logger.info("[DRY-RUN] stop: %s", reason)
            return
        try:
            await self._transport.send_command(
                UnitreeCommand(action="stop", parameters={"reason": reason})
            )
        except Exception as exc:
            logger.error("Stop command failed (best-effort): %s", exc)

    async def release_drive(self, reason: str = "") -> None:
        """Zero joystick output without StopMove (DimOS-style teleop release)."""
        self._record("release_drive", reason=reason)
        if self.dry_run:
            logger.info("[DRY-RUN] release_drive: %s", reason)
            return
        try:
            await self._transport.send_command(
                UnitreeCommand(action="release", parameters={"reason": reason})
            )
        except Exception as exc:
            logger.warning("release_drive failed (best-effort): %s", exc)

    async def move_to(self, target: Position, speed: float) -> None:
        clamped_speed = min(speed, self._settings.unitree_max_speed, self._settings.max_linear_speed)
        try:
            current = await self._transport.read_state()
        except Exception as exc:
            logger.error("Cannot read state before move: %s", exc)
            self._record("move_to", success=False, error=str(exc))
            raise RuntimeError(f"Unitree move_to aborted: cannot read state: {exc}") from exc

        distance = current.position.distance_to(target)
        max_step = min(self._settings.unitree_max_step, self._settings.max_step_distance)
        if distance > max_step:
            self._record(
                "move_to", success=False, reason="distance_clamped",
                requested=distance, max_allowed=max_step,
            )
            raise ValueError(
                f"move_to rejected: distance {distance:.2f} exceeds max step {max_step:.2f}"
            )

        self._record(
            "move_to", target=target.model_dump(), speed=clamped_speed,
            original_speed=speed, distance=distance,
        )
        self._stopped = False

        if self.dry_run:
            logger.info("[DRY-RUN] move_to (%s) speed=%.2f", target, clamped_speed)
            return

        try:
            await self._transport.send_command(
                UnitreeCommand(
                    action="move",
                    parameters={"target": target.model_dump(), "speed": clamped_speed},
                )
            )
        except Exception as exc:
            logger.error("move_to failed, issuing best-effort stop: %s", exc)
            await self.stop(f"move_to exception: {exc}")
            raise RuntimeError(f"Unitree move_to failed: {exc}") from exc

    async def turn(self, heading_degrees: float) -> None:
        clamped = max(-45.0, min(45.0, heading_degrees))
        self._record("turn", heading_degrees=clamped, original=heading_degrees)

        if self.dry_run:
            logger.info("[DRY-RUN] turn %.1f degrees", clamped)
            return

        try:
            await self._transport.send_command(
                UnitreeCommand(action="turn", parameters={"heading_degrees": clamped})
            )
        except Exception as exc:
            logger.error("turn failed, issuing best-effort stop: %s", exc)
            await self.stop(f"turn exception: {exc}")
            raise RuntimeError(f"Unitree turn failed: {exc}") from exc

    async def set_posture(self, posture: str) -> None:
        """Issue a non-translating posture/stop command (Unitree-specific).

        Honors dry-run; on failure issues a best-effort stop and re-raises.
        """
        if posture not in self.ALLOWED_POSTURES:
            self._record("set_posture", posture=posture, success=False, reason="unsupported")
            raise ValueError(
                f"Unsupported posture '{posture}'. Allowed: {sorted(self.ALLOWED_POSTURES)}"
            )

        self._record("set_posture", posture=posture)

        if self.dry_run:
            logger.info("[DRY-RUN] set_posture: %s", posture)
            return

        try:
            await self._transport.send_command(UnitreeCommand(action=posture))
        except Exception as exc:
            logger.error("set_posture(%s) failed, issuing best-effort stop: %s", posture, exc)
            await self.stop(f"set_posture exception: {exc}")
            raise RuntimeError(f"Unitree set_posture failed: {exc}") from exc

    async def wave(self) -> None:
        """Run Go2's built-in Hello gesture (a front-leg wave)."""
        self._record("wave", gesture="hello")
        await self.set_posture("hello")

    async def enable_omni_teleop(self) -> None:
        """Enable SwitchJoystick + low SpeedLevel after FreeWalk (WebRTC MCF)."""
        self._record("enable_omni_teleop")
        if self.dry_run:
            logger.info("[DRY-RUN] enable_omni_teleop")
            return
        if hasattr(self._transport, "enable_omni_teleop"):
            await self._transport.enable_omni_teleop()

    async def stream_hold(
        self,
        get_velocity: Callable[[], tuple[float, float, float]],
        *,
        session_deadline: float,
    ) -> None:
        """Continuous hold teleop — one 50Hz stream, no per-tick zero frames."""
        max_lin = min(self._settings.unitree_max_speed, self._settings.max_linear_speed)
        max_yaw = self._settings.unitree_max_yaw_speed

        def clamped() -> tuple[float, float, float]:
            vx, vy, vyaw = get_velocity()
            return (
                max(-max_lin, min(max_lin, vx)),
                max(-max_lin, min(max_lin, vy)),
                max(-max_yaw, min(max_yaw, vyaw)),
            )

        self._record("stream_hold", session_deadline=session_deadline)
        self._stopped = False

        if self.dry_run:
            loop = asyncio.get_event_loop()
            while loop.time() < session_deadline:
                vx, vy, vyaw = clamped()
                if not (vx or vy or vyaw):
                    await asyncio.sleep(0.05)
                    continue
                logger.info(
                    "[DRY-RUN] stream_hold vx=%.2f vy=%.2f vyaw=%.2f",
                    vx, vy, vyaw,
                )
                await asyncio.sleep(0.02)
            return

        if not hasattr(self._transport, "stream_hold"):
            raise NotImplementedError(
                f"{type(self._transport).__name__} has no stream_hold"
            )
        if hasattr(self._transport, "assert_drive_preconditions"):
            await self._transport.assert_drive_preconditions(self._settings)
        await self._transport.stream_hold(
            clamped,
            session_deadline=session_deadline,
            zero_on_exit=False,
        )

    async def drive(
        self,
        vx: float = 0.0,
        vy: float = 0.0,
        vyaw: float = 0.0,
        duration: float = 0.5,
        *,
        stream: bool = False,
    ) -> None:
        """Velocity teleop over the joystick channel (Unitree-specific).

        Body-frame velocities: ``vx`` forward (m/s), ``vy`` left (m/s),
        ``vyaw`` counter-clockwise (rad/s). The robot holds the velocity for
        ``duration`` seconds, then auto-stops. All inputs are clamped to the
        configured safety limits. Honors dry-run; on failure issues a
        best-effort stop and re-raises.

        When ``stream=True`` (continuous web teleop chunks), skip the
        post-drive stopped verification so the UI loop is not blocked.
        """
        max_lin = min(self._settings.unitree_max_speed, self._settings.max_linear_speed)
        max_yaw = self._settings.unitree_max_yaw_speed
        max_dur = self._settings.unitree_max_drive_duration

        cvx = max(-max_lin, min(max_lin, vx))
        cvy = max(-max_lin, min(max_lin, vy))
        cvyaw = max(-max_yaw, min(max_yaw, vyaw))
        cdur = max(0.0, min(max_dur, duration))

        pre_state: dict[str, Any] | None = None
        if not self.dry_run and hasattr(self._transport, "assert_drive_preconditions"):
            raw_pre = await self._transport.assert_drive_preconditions(self._settings)
            pre_state = raw_pre.model_dump(mode="json")

        started = time.time()
        self._record(
            "drive",
            vx=cvx,
            vy=cvy,
            vyaw=cvyaw,
            duration=cdur,
            original={"vx": vx, "vy": vy, "vyaw": vyaw, "duration": duration},
            pre_state=pre_state,
        )
        self._stopped = False

        if self.dry_run:
            logger.info(
                "[DRY-RUN] drive vx=%.2f vy=%.2f vyaw=%.2f dur=%.2fs",
                cvx, cvy, cvyaw, cdur,
            )
            self._patch_last_history(
                end_reason="dry_run",
                elapsed=time.time() - started,
                success=True,
            )
            return

        end_reason: str | None = None
        try:
            await self._transport.send_command(
                UnitreeCommand(
                    action="drive",
                    parameters={"vx": cvx, "vy": cvy, "vyaw": cvyaw, "duration": cdur},
                )
            )
            last = getattr(self._transport, "last_drive_end_reason", None)
            end_reason = str(last) if last is not None else "completed"
        except Exception as exc:
            logger.error("drive failed, issuing best-effort stop: %s", exc)
            end_reason = "transport_error"
            await self.stop(f"drive exception: {exc}")
            self._patch_last_history(
                end_reason=end_reason,
                elapsed=time.time() - started,
                success=False,
            )
            raise RuntimeError(f"Unitree drive failed: {exc}") from exc

        post_state: dict[str, Any] | None = None
        stopped_ok = True
        if (
            not stream
            and hasattr(self._transport, "verify_stopped_after_drive")
        ):
            stopped_ok = await self._transport.verify_stopped_after_drive(self._settings)
            try:
                raw_post = await self._transport.read_state()
                post_state = raw_post.model_dump(mode="json")
            except Exception as exc:
                logger.warning("post-drive state read failed: %s", exc)

        if not stopped_ok:
            end_reason = "post_drive_still_moving"
            self._patch_last_history(
                end_reason=end_reason,
                elapsed=time.time() - started,
                post_state=post_state,
                success=False,
            )
            await self.stop("post-drive still moving")
            raise RuntimeError(
                "Unitree drive finished but robot still reports motion; issued stop"
            )

        self._patch_last_history(
            end_reason=end_reason,
            elapsed=time.time() - started,
            post_state=post_state,
            success=True,
        )

    def _patch_last_history(self, **fields: Any) -> None:
        if not self._action_history:
            return
        if self._action_history[-1].get("action") != "drive":
            return
        self._action_history[-1].update(fields)

    async def dock(self, station: str) -> None:
        self._record("dock", station=station, supported=False)
        raise NotImplementedError("Unitree dock not supported in this version")

    async def follow(self, target_id: str, distance: float) -> None:
        self._record("follow", target_id=target_id, distance=distance, supported=False)
        raise NotImplementedError("Unitree follow not supported in this version")

    async def report(self, message: str, severity: str) -> None:
        self._record("report", message=message, severity=severity)
        logger.info("[Unitree report] %s: %s", severity, message)

    def _record(self, action: str, **params: Any) -> None:
        entry = {"action": action, "timestamp": time.time(), **params}
        self._action_history.append(entry)
