"""Real Unitree SDK transport for Go2 via unitree_sdk2_python.

This module uses dynamic imports — if the SDK is not installed, importing this
module will not fail, but calling create_sdk_transport() will raise a clear error.

The Go2 SDK communicates via CycloneDDS. State is read by subscribing to the
"rt/sportmodestate" DDS topic. Commands are sent via the SportClient service.

This iteration is READ-ONLY: send_command() rejects all operations.
"""
from __future__ import annotations

import asyncio
import logging
import math
import threading
import time
from typing import Any

from config.settings import Settings
from robot_brain.actuation.unitree import UnitreeCommand, UnitreeState, UnitreeTransport
from robot_brain.core.world_state import Position

logger = logging.getLogger(__name__)

# Default topic names for Go2
SPORT_STATE_TOPIC = "rt/sportmodestate"
LOW_STATE_TOPIC = "rt/lowstate"


def _import_sdk() -> Any:
    """Dynamically import unitree_sdk2py, raising a clear error if absent."""
    try:
        import unitree_sdk2py  # noqa: F401
        return unitree_sdk2py
    except ImportError as exc:
        raise RuntimeError(
            "unitree_sdk2_python is not installed. "
            "Install from: https://github.com/unitreerobotics/unitree_sdk2_python\n"
            "Requires CycloneDDS and Linux/macOS with proper network config.\n"
            "To run without real hardware, use RDB_UNITREE_TRANSPORT=fake"
        ) from exc


class UnitreeSDKTransport(UnitreeTransport):
    """Real transport using unitree_sdk2_python for Go2.

    Subscribes to DDS topics for state. This iteration only supports
    read operations — send_command() will reject motion commands.
    """

    def __init__(self, settings: Settings, sdk_client: Any = None) -> None:
        self._settings = settings
        self._sdk_client = sdk_client  # Injected for testing; None triggers real init
        self._connected = False
        self._last_sport_state: Any = None
        self._last_low_state: Any = None
        self._state_lock = threading.Lock()
        self._subscriber: Any = None
        self._low_subscriber: Any = None

    @property
    def is_connected(self) -> bool:
        return self._connected

    async def connect(self) -> None:
        if self._connected:
            return

        if self._sdk_client is not None:
            # Injected client (for testing)
            self._connected = True
            logger.info("UnitreeSDKTransport connected (injected client)")
            return

        _import_sdk()

        try:
            from unitree_sdk2py.core.channel import (
                ChannelFactoryInitialize,
                ChannelSubscriber,
            )
            from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_

            # Initialize DDS channel
            net_iface = self._settings.unitree_net_iface or ""
            if net_iface:
                ChannelFactoryInitialize(0, net_iface)
            else:
                ChannelFactoryInitialize(0)

            # Subscribe to sport mode state
            self._subscriber = ChannelSubscriber(SPORT_STATE_TOPIC, SportModeState_)
            self._subscriber.Init(self._on_sport_state, 10)

            # Optionally subscribe to low state for battery
            try:
                from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_

                self._low_subscriber = ChannelSubscriber(LOW_STATE_TOPIC, LowState_)
                self._low_subscriber.Init(self._on_low_state, 10)
            except Exception as exc:
                logger.warning("Could not subscribe to low state (battery): %s", exc)

            self._connected = True
            logger.info(
                "UnitreeSDKTransport connected (model=%s, iface=%s, topic=%s)",
                self._settings.unitree_model,
                net_iface or "default",
                SPORT_STATE_TOPIC,
            )
        except Exception as exc:
            self._connected = False
            raise ConnectionError(
                f"Failed to connect to Unitree Go2: {exc}. "
                f"Check Wi-Fi connection to robot and network interface config."
            ) from exc

    async def disconnect(self) -> None:
        self._connected = False
        self._subscriber = None
        self._low_subscriber = None
        self._last_sport_state = None
        self._last_low_state = None
        logger.info("UnitreeSDKTransport disconnected")

    async def read_state(self) -> UnitreeState:
        if not self._connected:
            raise ConnectionError("UnitreeSDKTransport not connected")

        if self._sdk_client is not None:
            return await self._read_from_injected_client()

        return await self._read_from_dds()

    async def send_command(self, command: UnitreeCommand) -> bool:
        """Read-only in this iteration — all commands are rejected."""
        if not self._connected:
            raise ConnectionError("UnitreeSDKTransport not connected")

        raise NotImplementedError(
            f"UnitreeSDKTransport is read-only in this iteration. "
            f"Command rejected: {command.action}. "
            f"Use FakeUnitreeTransport or wait for the next iteration to enable real commands."
        )

    def _on_sport_state(self, msg: Any) -> None:
        """DDS callback for SportModeState_ — runs in SDK thread."""
        with self._state_lock:
            self._last_sport_state = msg

    def _on_low_state(self, msg: Any) -> None:
        """DDS callback for LowState_ — runs in SDK thread."""
        with self._state_lock:
            self._last_low_state = msg

    async def _read_from_injected_client(self) -> UnitreeState:
        """Read state from an injected test client."""
        try:
            raw = await self._sdk_client.get_state()
            return self._map_state(raw)
        except Exception as exc:
            logger.error("Injected client read_state failed: %s", exc)
            raise ConnectionError(f"State read failed: {exc}") from exc

    async def _read_from_dds(self) -> UnitreeState:
        """Read latest state from the DDS subscription."""
        with self._state_lock:
            sport_state = self._last_sport_state
            low_state = self._last_low_state

        if sport_state is None:
            # No state received yet — might need to wait
            logger.warning("No sport state received yet, waiting 2s...")
            await asyncio.sleep(2.0)
            with self._state_lock:
                sport_state = self._last_sport_state
            if sport_state is None:
                raise ConnectionError(
                    "No state received from robot. "
                    "Check: robot powered on, Wi-Fi connected, correct topic."
                )

        return self._map_sport_state(sport_state, low_state)

    def _map_sport_state(self, sport: Any, low: Any = None) -> UnitreeState:
        """Map SDK SportModeState_ to our UnitreeState."""
        try:
            # Position from sport state
            pos = getattr(sport, "position", [0, 0, 0])
            position = Position(x=float(pos[0]), y=float(pos[1]))

            # Heading from IMU yaw
            imu = getattr(sport, "imu_state", None)
            if imu and hasattr(imu, "rpy"):
                heading = math.degrees(float(imu.rpy[2]))
            else:
                heading = 0.0

            # Mode: 0=passive, 1=stand_down, 2=stand_up, 3=default, 4=running, ...
            mode = int(getattr(sport, "mode", 0))
            is_standing = mode >= 2  # stand_up or higher

            # Velocity for is_moving
            vel = getattr(sport, "velocity", [0, 0, 0])
            speed = math.sqrt(float(vel[0]) ** 2 + float(vel[1]) ** 2)
            is_moving = speed > 0.01

            # Error code
            error_code = int(getattr(sport, "error_code", 0))

            # Battery from low state
            battery = 100.0
            if low is not None:
                # LowState_ has power_v (voltage) — map roughly to percentage
                # Go2 typical range: ~24V (empty) to ~33.6V (full, 8S LiPo)
                voltage = float(getattr(low, "power_v", 0))
                if voltage > 0:
                    battery = max(0.0, min(100.0, (voltage - 24.0) / (33.6 - 24.0) * 100.0))

            return UnitreeState(
                connected=True,
                battery_level=battery,
                position=position,
                heading_degrees=heading,
                is_standing=is_standing,
                is_moving=is_moving,
                error_code=error_code,
            )
        except Exception as exc:
            logger.warning("State mapping error: %s", exc)
            return UnitreeState(connected=True, error_code=-1)

    def _map_state(self, raw: dict[str, Any]) -> UnitreeState:
        """Map a raw dict (from injected client) to UnitreeState."""
        pos = raw.get("position") or {}
        return UnitreeState(
            connected=raw.get("connected") or True,
            battery_level=raw.get("battery_level") or 100.0,
            position=Position(
                x=pos.get("x", 0.0) if isinstance(pos, dict) else 0.0,
                y=pos.get("y", 0.0) if isinstance(pos, dict) else 0.0,
            ),
            heading_degrees=raw.get("heading_degrees") or 0.0,
            is_standing=raw.get("is_standing") or False,
            is_moving=raw.get("is_moving") or False,
            error_code=raw.get("error_code") or 0,
        )


def create_sdk_transport(settings: Settings) -> UnitreeSDKTransport:
    """Factory function for the real SDK transport."""
    return UnitreeSDKTransport(settings)
