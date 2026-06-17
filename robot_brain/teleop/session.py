"""Transport-agnostic teleop session: single-holder lease + deadman watchdog.

This is the Stage-1 control ingress core. It wraps an already-safe
``UnitreeRobot`` (clamps, dry-run, best-effort stop) and adds the two
semantics the velocity-teleop path needs but the adapter does not own:

* **motion lease** — exactly one operator may drive at a time; others are
  rejected until the lease is released or expires.
* **deadman watchdog** — if no fresh setpoint arrives within
  ``deadman_ms`` the drive is zeroed (the robot stops), so a dropped network
  link or a stalled operator never leaves the dog moving.

A later gRPC shell only needs to translate wire messages into
``acquire_lease`` / ``set_velocity`` / ``release_lease`` / ``emergency_stop``
calls and drain :attr:`TeleopSession.events`; no robot logic lives in the
transport.
"""
from __future__ import annotations

import asyncio
import time
from dataclasses import dataclass, field
from enum import Enum
from uuid import uuid4

from config.settings import Settings
from robot_brain.actuation.unitree import UnitreeRobot


class ControlEventType(str, Enum):
    """Async events emitted by the session (mirrors the gRPC ControlEvent)."""

    ACCEPTED = "accepted"
    REJECTED = "rejected"
    STOPPED = "stopped"
    PREEMPTED = "preempted"
    WATCHDOG = "watchdog"
    OBSTACLE = "obstacle"


@dataclass
class ControlEvent:
    type: ControlEventType
    message: str = ""


@dataclass
class LeaseResult:
    granted: bool
    lease_id: str = ""
    expires_at: float = 0.0
    reason: str = ""


@dataclass
class SetpointResult:
    accepted: bool
    reason: str = ""


@dataclass
class _Setpoint:
    vx: float = 0.0
    vy: float = 0.0
    vyaw: float = 0.0
    at: float = field(default_factory=time.monotonic)

    def is_zero(self) -> bool:
        return not (self.vx or self.vy or self.vyaw)


# Obstacle safety: front ultrasonic below this threshold (metres) triggers an
# OBSTACLE event and clamps forward velocity to zero.
_OBSTACLE_FRONT_THRESHOLD_M: float = 0.30  # 30 cm


def _obstacle_msg(distance_m: float) -> str:
    """Human-readable obstacle event message."""
    if distance_m < 1.0:
        return f"obstacle:{int(distance_m * 100)}cm:front"
    return f"obstacle:{distance_m:.2f}m:front"


class TeleopSession:
    """Single-holder velocity teleop with a deadman watchdog.

    Time is tracked with :func:`time.monotonic` so wall-clock changes never
    affect lease/deadman decisions.
    """

    def __init__(self, robot: UnitreeRobot, settings: Settings) -> None:
        self._robot = robot
        self._deadman_s = max(0.0, settings.teleop_deadman_ms / 1000.0)
        self._lease_ttl_s = max(0.0, settings.teleop_lease_ttl_ms / 1000.0)
        self._chunk_s = max(0.05, settings.teleop_chunk_seconds)

        self._lock = asyncio.Lock()
        self._lease_id: str = ""
        self._operator: str = ""
        self._lease_expires: float = 0.0

        self._setpoint = _Setpoint(at=0.0)
        self._drive_task: asyncio.Task[None] | None = None

        self.events: asyncio.Queue[ControlEvent] = asyncio.Queue()

    # ------------------------------------------------------------------ leases
    async def acquire_lease(self, operator_id: str, ttl_ms: int | None = None) -> LeaseResult:
        """Grant control to *operator_id* unless another holder is still live."""
        async with self._lock:
            now = time.monotonic()
            ttl_s = self._lease_ttl_s if ttl_ms is None else max(0.0, ttl_ms / 1000.0)
            if self._lease_id and now < self._lease_expires and operator_id != self._operator:
                return LeaseResult(
                    granted=False,
                    reason=f"control held by '{self._operator}'",
                )
            self._lease_id = uuid4().hex
            self._operator = operator_id
            self._lease_expires = now + ttl_s
            return LeaseResult(
                granted=True,
                lease_id=self._lease_id,
                expires_at=self._lease_expires,
            )

    async def release_lease(self, lease_id: str) -> bool:
        """Release the lease and stop driving. No-op if *lease_id* is stale."""
        async with self._lock:
            if lease_id != self._lease_id:
                return False
            await self._end_drive_locked(release=True)
            self._clear_lease_locked()
        await self.events.put(ControlEvent(ControlEventType.STOPPED, "lease released"))
        return True

    # ---------------------------------------------------------------- setpoints
    async def set_velocity(
        self, lease_id: str, vx: float, vy: float, vyaw: float
    ) -> SetpointResult:
        """Update the latest body-frame velocity setpoint and renew the lease.

        If the operator is driving forward and the front ultrasonic sensor
        reports an obstacle closer than the safety threshold, the forward
        component is clamped to zero and an ``OBSTACLE`` event is emitted.
        """
        async with self._lock:
            now = time.monotonic()
            if not self._lease_id or now >= self._lease_expires:
                return SetpointResult(False, "no active lease")
            if lease_id != self._lease_id:
                return SetpointResult(False, "lease_id does not hold control")

            # ── obstacle check on forward drive ──────────────────────
            if vx > 0:
                obs = await self._read_front_ultrasonic()
                if obs is not None and obs < _OBSTACLE_FRONT_THRESHOLD_M:
                    await self.events.put(
                        ControlEvent(
                            ControlEventType.OBSTACLE,
                            _obstacle_msg(obs),
                        )
                    )
                    vx = 0.0  # prevent driving into obstacle

            self._setpoint = _Setpoint(vx=vx, vy=vy, vyaw=vyaw, at=now)
            self._lease_expires = now + self._lease_ttl_s

            if self._setpoint.is_zero():
                await self._end_drive_locked(release=True)
            elif self._drive_task is None or self._drive_task.done():
                self._drive_task = asyncio.create_task(self._drive_loop(self._lease_id))
        await self.events.put(ControlEvent(ControlEventType.ACCEPTED))
        return SetpointResult(True)

    # ------------------------------------------------------------------- estop
    async def emergency_stop(self, reason: str = "") -> None:
        """Hard stop independent of any lease, then invalidate the lease."""
        async with self._lock:
            await self._end_drive_locked(release=False)
            await self._robot.stop(reason or "teleop emergency stop")
            self._clear_lease_locked()
        await self.events.put(ControlEvent(ControlEventType.STOPPED, reason or "estop"))

    # ----------------------------------------------------------------- internal
    async def _read_front_ultrasonic(self) -> float | None:
        """Return the front ultrasonic distance in metres, or None on failure."""
        try:
            raw = await self._robot.transport.read_state()
        except Exception:
            return None
        if raw.ultrasonic is None or raw.ultrasonic[0] <= 0:
            return None
        return raw.ultrasonic[0]

    def _clear_lease_locked(self) -> None:
        self._lease_id = ""
        self._operator = ""
        self._lease_expires = 0.0
        self._setpoint = _Setpoint(at=0.0)

    async def _end_drive_locked(self, *, release: bool) -> None:
        task = self._drive_task
        self._drive_task = None
        if task is not None and not task.done():
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass
        if release:
            await self._robot.release_drive("teleop stop")

    def _stale(self, now: float) -> bool:
        return (now - self._setpoint.at) > self._deadman_s

    async def _drive_loop(self, lease_id: str) -> None:
        """Continuously drive the latest setpoint until lease/deadman ends it."""
        loop = asyncio.get_event_loop()

        def live_velocity() -> tuple[float, float, float]:
            # Deadman enforced at the 50Hz stream tick: stale -> zero -> stop.
            if self._stale(time.monotonic()):
                return (0.0, 0.0, 0.0)
            sp = self._setpoint
            return (sp.vx, sp.vy, sp.vyaw)

        watchdog_fired = False
        try:
            while True:
                now = time.monotonic()
                if lease_id != self._lease_id or now >= self._lease_expires:
                    break
                if self._stale(now):
                    watchdog_fired = True
                    break
                await self._robot.stream_hold(
                    live_velocity,
                    session_deadline=loop.time() + self._chunk_s,
                )
        except asyncio.CancelledError:
            raise
        finally:
            await self._robot.release_drive("teleop drive loop end")
            if watchdog_fired:
                await self.events.put(
                    ControlEvent(ControlEventType.WATCHDOG, "deadman: no setpoint")
                )
