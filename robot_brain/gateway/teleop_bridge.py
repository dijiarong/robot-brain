"""Map browser DataChannel move commands → TeleopSession (lease + deadman).

Optimizations:
- Setpoint coalescing: redundant values (same vx/vy/vyaw) are skipped, reducing
  lease-renewal and transport churn.
- Fast zero: zero-velocity setpoints are sent immediately (no rate-limiting)
  so the robot stops as fast as possible.
- Bounded resend loop: if the browser holds the same non-zero joystick value,
  the loop still renews the lease at a cooldown interval to keep the deadman
  alive, but skips redundant transport publishes.
"""
from __future__ import annotations

import asyncio
import logging
import time
from typing import Any

from robot_brain.teleop.session import TeleopSession

logger = logging.getLogger(__name__)

_MAX_LINEAR = 0.5
_MAX_ANGULAR = 1.0
_SEND_HZ = 20.0
# Minimum interval between sending the same non-zero setpoint to the session.
# Shorter than the deadman window so the lease stays alive.
_LEASE_RENEW_COOLDOWN = 0.1  # seconds
# Threshold for treating velocities as "changed" (avoids float-jitter spam).
_VELOCITY_EPSILON = 0.005


class TeleopBridge:
    """Bridges browser joystick DataChannel messages into TeleopSession.

    Coalesces redundant setpoints — only calls set_velocity when the value
    actually changes, except for periodic lease renewal to keep the deadman
    watchdog alive.
    """

    def __init__(self, session: TeleopSession, operator_id: str) -> None:
        self._session = session
        self._operator_id = operator_id
        self._target_vx = 0.0
        self._target_vy = 0.0
        self._target_vyaw = 0.0
        self._last_sent_vx: float | None = None
        self._last_sent_vy: float | None = None
        self._last_sent_vyaw: float | None = None
        self._last_setpoint_time: float = 0.0
        self._lease_id = ""
        self._task: asyncio.Task[None] | None = None

    def set_joystick(self, linear_x: float, linear_y: float, angular_z: float) -> None:
        """Update target velocity from browser joystick input.

        Scales normalized [-1, 1] inputs to m/s and rad/s. Starts or restarts
        the sender loop if needed.
        """
        self._target_vx = float(linear_x) * _MAX_LINEAR
        self._target_vy = float(linear_y) * _MAX_LINEAR
        self._target_vyaw = float(angular_z) * _MAX_ANGULAR
        if self._task is None or self._task.done():
            self._task = asyncio.create_task(self._sender_loop())

    async def stop(self) -> None:
        """Signal the sender loop to exit, release the lease, and wait for completion."""
        self._target_vx = self._target_vy = self._target_vyaw = 0.0
        if self._task and not self._task.done():
            await asyncio.sleep(0.3)
            self._task.cancel()
            try:
                await self._task
            except asyncio.CancelledError:
                pass
        self._task = None
        if self._lease_id:
            try:
                await asyncio.wait_for(
                    self._session.release_lease(self._lease_id),
                    timeout=2.0,
                )
            except asyncio.TimeoutError:
                logger.warning("[Gateway] release_lease timed out for %s", self._operator_id)
            self._lease_id = ""

    async def _sender_loop(self) -> None:
        """20Hz loop: acquires lease, sends velocity setpoints, renews lease.

        Coalescing logic:
        - Zero setpoint after non-zero: sent immediately (fast stop).
        - Non-zero setpoint: sent if value differs from last sent by > epsilon.
        - If value unchanged for > LEASE_RENEW_COOLDOWN, send to renew lease.
        """
        interval = 1.0 / _SEND_HZ
        idle_ticks = 0
        try:
            while True:
                vx, vy, vyaw = self._target_vx, self._target_vy, self._target_vyaw
                moving = bool(vx or vy or vyaw)
                now = time.monotonic()

                if moving:
                    idle_ticks = 0
                else:
                    idle_ticks += 1
                    if idle_ticks > 5:
                        break

                # --- Coalescing decision ---
                if self._last_sent_vx is None:
                    # First send: always publish.
                    should_send = True
                elif not moving and (self._last_sent_vx or self._last_sent_vy or self._last_sent_vyaw):
                    # Transition: non-zero → zero — send immediately (fast stop).
                    should_send = True
                elif moving and not self._is_significantly_different(vx, vy, vyaw):
                    # Same non-zero value — only send periodically to renew lease.
                    since_last = now - self._last_setpoint_time
                    should_send = since_last >= _LEASE_RENEW_COOLDOWN
                else:
                    should_send = True

                if should_send:
                    # --- Acquire or reuse lease ---
                    if not self._lease_id:
                        lease = await self._session.acquire_lease(self._operator_id)
                        if not lease.granted:
                            logger.warning("[Gateway] control denied: %s", lease.reason)
                            await asyncio.sleep(0.5)
                            continue
                        self._lease_id = lease.lease_id

                    result = await self._session.set_velocity(self._lease_id, vx, vy, vyaw)
                    if not result.accepted:
                        logger.warning("[Gateway] setpoint rejected: %s", result.reason)
                        self._lease_id = ""
                    else:
                        self._last_sent_vx = vx
                        self._last_sent_vy = vy
                        self._last_sent_vyaw = vyaw
                        self._last_setpoint_time = now

                await asyncio.sleep(interval)
        finally:
            if self._lease_id:
                try:
                    await asyncio.wait_for(
                        self._session.set_velocity(self._lease_id, 0.0, 0.0, 0.0),
                        timeout=2.0,
                    )
                except asyncio.TimeoutError:
                    pass
                try:
                    await asyncio.wait_for(
                        self._session.release_lease(self._lease_id),
                        timeout=2.0,
                    )
                except asyncio.TimeoutError:
                    pass
                self._lease_id = ""

    def _is_significantly_different(self, vx: float, vy: float, vyaw: float) -> bool:
        """True if any component differs from the last-sent value by > epsilon."""
        if self._last_sent_vx is None:
            return True
        return (
            abs(vx - self._last_sent_vx) > _VELOCITY_EPSILON
            or abs(vy - self._last_sent_vy) > _VELOCITY_EPSILON
            or abs(vyaw - self._last_sent_vyaw) > _VELOCITY_EPSILON
        )

    @staticmethod
    def parse_move(msg: dict[str, Any]) -> tuple[float, float, float] | None:
        """Parse a browser DataChannel 'move' message into (linear_x, linear_y, angular_z).

        Returns None if the message is not a valid move command.
        """
        if msg.get("type") != "move":
            return None
        data = msg.get("data") or {}
        try:
            return (
                float(data.get("linear_x", 0)),
                float(data.get("linear_y", 0)),
                float(data.get("angular_z", 0)),
            )
        except (TypeError, ValueError):
            return None
