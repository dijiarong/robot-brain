"""Unitree robot adapter implementing RobotInterface with safety clamps."""
from __future__ import annotations

import logging
import time
from abc import ABC, abstractmethod
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
        elif command.action == "move":
            target = command.parameters.get("target", {})
            self._state.position = Position(x=target.get("x", 0), y=target.get("y", 0))
            self._state.is_moving = False
        elif command.action == "turn":
            self._state.heading_degrees = command.parameters.get("heading_degrees", 0)


class UnitreeRobot(RobotInterface):
    """Unitree adapter with internal safety clamps and dry-run support."""

    def __init__(self, transport: UnitreeTransport, settings: Settings) -> None:
        self._transport = transport
        self._settings = settings
        self._action_history: list[dict[str, Any]] = []
        self._stopped = False

    @property
    def action_history(self) -> list[dict[str, Any]]:
        return self._action_history

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
