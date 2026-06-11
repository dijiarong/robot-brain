"""Unitree Go2 transport via WebRTC data channel (LocalSTA mode).

Uses unitree-webrtc-connect to communicate with Go2 over the local network
(router/STA mode). This works when Go2 and the dev machine are on the same
LAN — no direct Wi-Fi hotspot connection required.

This iteration supports posture/stop commands (StandUp, StandDown, Sit,
BalanceStand, RecoveryStand, Damp, StopMove) plus velocity teleop ("drive")
over the WIRELESS_CONTROLLER joystick channel — the same path the Unitree app
and dimos's keyboard control use, which drives the robot reliably even when the
high-level sport (SPORT_MOD) controller ignores posture commands. All motion
requires RDB_UNITREE_ENABLE_MOTION=true (stop is always allowed when connected).
"""
from __future__ import annotations

import asyncio
import json
import logging
import math
import threading
import time
from typing import Any

from config.settings import Settings
from robot_brain.actuation.unitree import UnitreeCommand, UnitreeState, UnitreeTransport
from robot_brain.core.world_state import Position

logger = logging.getLogger(__name__)

# Sport request topic and command api_ids (from unitree_webrtc_connect constants).
# Hardcoded here so the command path does not need to import the library — keeps
# the injected-conn test path free of real SDK dependencies.
_SPORT_REQUEST_TOPIC = "rt/api/sport/request"
_SPORT_API_ID: dict[str, int] = {
    "stop": 1003,            # StopMove
    "balance_stand": 1002,   # BalanceStand
    "stand_up": 1004,        # StandUp
    "stand_down": 1005,      # StandDown (lie down)
    "recovery_stand": 1006,  # RecoveryStand
    "damp": 1001,            # Damp (motors relax)
    "sit": 1009,             # Sit
}
# Joystick (wireless controller) channel — emulates the remote/app sticks. This
# is what actually drives the Go2 on firmware where SPORT_MOD posture commands
# are ACKed but not executed (e.g. the "mcf" controller).
_WIRELESS_CONTROLLER_TOPIC = "rt/wirelesscontroller"
# Velocity at which a joystick axis reaches full deflection (magnitude 1.0).
# Used to convert m/s and rad/s into normalized [-1, 1] stick values.
_GO2_JOY_FULL_LINEAR = 1.5  # m/s
_GO2_JOY_FULL_YAW = 2.0     # rad/s
# How often joystick samples are streamed while a drive command is active.
_JOY_STREAM_HZ = 50.0
# Waypoint move/turn (closed-loop) is still deferred; teleport-style "drive"
# (open-loop velocity for a duration) is the supported translation primitive.
_TRANSLATION_ACTIONS = frozenset({"move", "turn"})


def _clamp_unit(value: float) -> float:
    """Clamp a joystick axis value into the normalized [-1.0, 1.0] range."""
    return max(-1.0, min(1.0, value))


def _mode_name_from_response(response: Any) -> str | None:
    """Extract the active motion-mode name from a MOTION_SWITCHER CheckMode reply.

    The mode lives at ``data.data`` as a JSON string like ``{"form":"0","name":"mcf"}``.
    """
    if not isinstance(response, dict):
        return None
    data = response.get("data")
    if isinstance(data, dict):
        inner = data.get("data")
        if isinstance(inner, str):
            try:
                inner = json.loads(inner)
            except (ValueError, TypeError):
                return None
        if isinstance(inner, dict):
            name = inner.get("name")
            return str(name) if name is not None else None
    return None


def _extract_status_code(response: Any) -> int | None:
    """Best-effort extraction of a Go2 response status code.

    The response is the parsed data-channel message; the status code lives at
    ``data.header.status.code`` (data may itself be a JSON string). Returns
    None when no code can be located, so callers treat it as "unknown".
    """
    if not isinstance(response, dict):
        return None
    candidates = [response, response.get("data")]
    for container in candidates:
        if isinstance(container, str):
            try:
                container = json.loads(container)
            except (ValueError, TypeError):
                continue
        if isinstance(container, dict):
            header = container.get("header")
            if isinstance(header, dict):
                status = header.get("status")
                if isinstance(status, dict) and "code" in status:
                    try:
                        return int(status["code"])
                    except (ValueError, TypeError):
                        return None
    return None


class _BenignWebRTCNoiseFilter(logging.Filter):
    """Drop benign log lines emitted by unitree_webrtc_connect.

    The library logs the legacy HTTP /offer signaling fallback as ERROR on the
    root logger even though it successfully falls back to the encrypted
    data-channel method. These lines are misleading noise, not real failures.

    Substrings are kept narrow on purpose: a bare "Failed to receive SDP Answer"
    can also be a *real* failure of the encrypted method, so we only drop the
    legacy /offer HTTP attempt and the explicitly-labelled "old method" line.
    """

    _BENIGN_SUBSTRINGS = (
        "An error occurred with the old method",
        "/offer",
    )

    def filter(self, record: logging.LogRecord) -> bool:
        msg = record.getMessage()
        return not any(s in msg for s in self._BENIGN_SUBSTRINGS)


def _install_benign_noise_filter() -> None:
    """Attach the benign-noise filter to the root logger once.

    The library logs via the root logger; attaching the filter to the logger
    itself (rather than its handlers) ensures records are dropped in
    Logger.handle() before reaching any handler, including the built-in
    lastResort handler used when no handlers are configured.
    """
    root = logging.getLogger()
    if not any(isinstance(f, _BenignWebRTCNoiseFilter) for f in root.filters):
        root.addFilter(_BenignWebRTCNoiseFilter())


def _import_webrtc() -> tuple[Any, Any, Any]:
    """Dynamically import unitree_webrtc_connect."""
    try:
        from unitree_webrtc_connect.webrtc_driver import (
            UnitreeWebRTCConnection,
            WebRTCConnectionMethod,
        )
        from unitree_webrtc_connect.constants import RTC_TOPIC
        return UnitreeWebRTCConnection, WebRTCConnectionMethod, RTC_TOPIC
    except ImportError as exc:
        raise RuntimeError(
            "unitree-webrtc-connect is not installed. "
            "Install with: pip install unitree-webrtc-connect\n"
            "To run without real hardware, use RDB_UNITREE_TRANSPORT=fake"
        ) from exc


class UnitreeWebRTCTransport(UnitreeTransport):
    """Real transport using WebRTC data channel for Go2 in STA/LAN mode.

    Subscribes to sport mode state and low state via WebRTC pub/sub, and sends
    posture/stop commands over SPORT_MOD plus velocity teleop ("drive") over the
    WIRELESS_CONTROLLER joystick channel.
    """

    def __init__(self, settings: Settings, webrtc_conn: Any = None) -> None:
        self._settings = settings
        self._injected_conn = webrtc_conn  # Injected for testing
        self._conn: Any = None  # The live UnitreeWebRTCConnection
        self._connected = False
        self._enable_motion = settings.unitree_enable_motion
        self._motion_mode = settings.unitree_motion_mode or "normal"
        self._last_sport_state: dict[str, Any] | None = None
        self._last_low_state: dict[str, Any] | None = None
        self._state_lock = threading.Lock()
        self._bg_loop: asyncio.AbstractEventLoop | None = None
        self._bg_thread: threading.Thread | None = None

    @property
    def is_connected(self) -> bool:
        return self._connected

    async def connect(self) -> None:
        if self._connected:
            return

        if self._injected_conn is not None:
            self._conn = self._injected_conn
            self._connected = True
            logger.info("UnitreeWebRTCTransport connected (injected)")
            return

        UnitreeWebRTCConnection, WebRTCConnectionMethod, RTC_TOPIC = _import_webrtc()

        ip = self._settings.unitree_robot_ip or None
        serial = self._settings.unitree_serial or None
        if not ip and not serial:
            # AP-mode default; for router/STA mode set RDB_UNITREE_ROBOT_IP to the LAN IP.
            ip = "192.168.123.161"

        # AES key for encrypted WebRTC signaling (Go2 firmware >= 1.1.15).
        import os
        aes_key = os.environ.get("UNITREE_AES_128_KEY") or os.environ.get("UNITREE_AES_KEY") or None

        target = f"serial={serial}" if serial and not ip else f"ip={ip}"
        logger.info(
            "Connecting to Go2 via WebRTC (%s, aes_key=%s)...",
            target,
            "set" if aes_key else "none",
        )

        _install_benign_noise_filter()

        # Run the async WebRTC connection in a background thread with its own event loop
        # Pattern follows dimos: create_task + run_forever (loop stays alive for callbacks)
        connection_error: list[BaseException] = []
        ready_event = threading.Event()

        def run_bg_loop() -> None:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            self._bg_loop = loop

            def _ignore_media_stream_errors(
                loop_: asyncio.AbstractEventLoop, context: dict[str, Any]
            ) -> None:
                # The video track recv loop raises MediaStreamError when the
                # connection tears down. We never consume video, so this is
                # benign teardown noise — swallow it, defer everything else.
                exc = context.get("exception")
                if exc is not None and type(exc).__name__ == "MediaStreamError":
                    return
                loop_.default_exception_handler(context)

            loop.set_exception_handler(_ignore_media_stream_errors)

            async def async_connect() -> None:
                try:
                    import inspect
                    init_params = inspect.signature(UnitreeWebRTCConnection.__init__).parameters
                    kwargs: dict[str, Any] = {
                        "connectionMethod": WebRTCConnectionMethod.LocalSTA,
                    }
                    if serial and "serialNumber" in init_params:
                        kwargs["serialNumber"] = serial
                    if ip:
                        kwargs["ip"] = ip
                    if aes_key and "aes_128_key" in init_params:
                        kwargs["aes_128_key"] = aes_key

                    conn = UnitreeWebRTCConnection(**kwargs)
                    await conn.connect()

                    # Remove the video track listener from RTCPeerConnection to prevent
                    # MediaStreamError from tearing down the connection.
                    # The "track" event handler is registered by init_webrtc() and tries
                    # to recv() video frames — which fails and kills the connection.
                    if hasattr(conn, "pc"):
                        conn.pc.remove_all_listeners("track")

                    # Disable traffic saving for continuous state updates
                    await conn.datachannel.disableTrafficSaving(True)

                    # Set native decoder (avoids voxel/lidar processing overhead)
                    conn.datachannel.set_decoder(decoder_type="native")

                    # Ensure the right motion controller is active. Basic sport
                    # posture commands (rt/api/sport/request) are served by the
                    # "normal" sport_mode service; other controllers (e.g. "mcf",
                    # the interactive menu on recent firmware) ACK the commands
                    # with code 0 but do not drive the motors.
                    pub_sub = conn.datachannel.pub_sub
                    try:
                        check = await pub_sub.publish_request_new(
                            RTC_TOPIC["MOTION_SWITCHER"], {"api_id": 1001}
                        )
                        current_mode = _mode_name_from_response(check)
                        logger.info("MotionSwitcher current mode=%s (raw=%s)", current_mode, check)
                    except Exception as exc:
                        current_mode = None
                        logger.warning("MotionSwitcher check (api_id=1001) failed: %s", exc)

                    # Best-effort select of the configured motion mode (matches
                    # dimos, which selects then ignores the result). Some firmware
                    # locks the robot to its current controller (e.g. "mcf") and
                    # rejects the switch, but sport posture commands still work, so
                    # we only log and proceed — never ReleaseMode here, since that
                    # can drop a standing robot.
                    if self._enable_motion and current_mode != self._motion_mode:
                        select = await pub_sub.publish_request_new(
                            RTC_TOPIC["MOTION_SWITCHER"],
                            {"api_id": 1002, "parameter": {"name": self._motion_mode}},
                        )
                        logger.info(
                            "MotionSwitcher select '%s' (api_id=1002) -> code=%s (current=%s)",
                            self._motion_mode, _extract_status_code(select), current_mode,
                        )

                    self._conn = conn

                    # Subscribe to state topics
                    conn.datachannel.pub_sub.subscribe(
                        RTC_TOPIC["LF_SPORT_MOD_STATE"], self._on_sport_state
                    )
                    conn.datachannel.pub_sub.subscribe(
                        RTC_TOPIC["LOW_STATE"], self._on_low_state
                    )

                    self._connected = True
                    ready_event.set()

                    # Keep loop alive — callbacks are dispatched here
                    while self._connected:
                        await asyncio.sleep(1)
                except BaseException as exc:
                    connection_error.append(exc)
                    ready_event.set()

            loop.create_task(async_connect())
            loop.run_forever()

        self._bg_thread = threading.Thread(target=run_bg_loop, daemon=True)
        self._bg_thread.start()

        # Wait for connection with timeout
        if not ready_event.wait(timeout=30.0):
            self._connected = False
            raise ConnectionError(
                f"WebRTC connection timed out after 30s ({target}). "
                f"Check: robot powered on, same network, correct IP/serial, "
                f"and UNITREE_AES_128_KEY for firmware >= 1.1.15."
            )

        if connection_error:
            self._connected = False
            err = connection_error[0]
            hint = ""
            if not aes_key:
                hint = (
                    " If firmware >= 1.1.15, set UNITREE_AES_128_KEY "
                    "(unitree-fetch-aes-key --email ... --device-type Go2)."
                )
            raise ConnectionError(
                f"Failed to connect via WebRTC to Go2 ({target}): {err}.{hint}"
            ) from err

        logger.info("UnitreeWebRTCTransport connected via WebRTC (%s)", target)

    def _on_sport_state(self, msg: Any) -> None:
        """Callback from WebRTC pub/sub for sport mode state."""
        with self._state_lock:
            if isinstance(msg, dict):
                self._last_sport_state = msg
            elif isinstance(msg, str):
                try:
                    self._last_sport_state = json.loads(msg)
                except json.JSONDecodeError:
                    pass

    def _on_low_state(self, msg: Any) -> None:
        """Callback from WebRTC pub/sub for low state."""
        with self._state_lock:
            if isinstance(msg, dict):
                self._last_low_state = msg
            elif isinstance(msg, str):
                try:
                    self._last_low_state = json.loads(msg)
                except json.JSONDecodeError:
                    pass

    async def disconnect(self) -> None:
        self._connected = False
        if self._conn is not None and hasattr(self._conn, "disconnect"):
            try:
                if self._bg_loop is not None and self._bg_loop.is_running():
                    future = asyncio.run_coroutine_threadsafe(
                        self._conn.disconnect(), self._bg_loop
                    )
                    future.result(timeout=5.0)
            except Exception as exc:
                logger.warning("WebRTC disconnect error: %s", exc)
        self._conn = None
        self._last_sport_state = None
        self._last_low_state = None
        logger.info("UnitreeWebRTCTransport disconnected")

    async def read_state(self) -> UnitreeState:
        if not self._connected:
            raise ConnectionError("UnitreeWebRTCTransport not connected")

        with self._state_lock:
            sport = self._last_sport_state
            low = self._last_low_state

        if sport is None:
            # State may take a moment to arrive after subscription
            for attempt in range(3):
                wait_sec = 2.0 * (attempt + 1)
                logger.info("Waiting for sport state (attempt %d, %.0fs)...", attempt + 1, wait_sec)
                await asyncio.sleep(wait_sec)
                with self._state_lock:
                    sport = self._last_sport_state
                    low = self._last_low_state
                if sport is not None:
                    break
            if sport is None:
                raise ConnectionError(
                    "No state received from robot via WebRTC after 12s. "
                    "Check: robot powered on, same network, correct IP."
                )

        return self._map_state(sport, low)

    async def send_command(self, command: UnitreeCommand) -> bool:
        """Send a posture/stop sport command to the Go2 over WebRTC.

        This iteration supports only non-translating commands (stop + posture).
        Translation commands (move/turn) are rejected. All commands except
        ``stop`` additionally require ``enable_motion`` to be set — ``stop`` is
        a safety command and is always permitted when connected.
        """
        if not self._connected:
            raise ConnectionError("UnitreeWebRTCTransport not connected")

        action = command.action

        if action == "drive":
            if not self._enable_motion:
                raise PermissionError(
                    "Motion disabled: refusing to send 'drive'. "
                    "Set RDB_UNITREE_ENABLE_MOTION=true to allow velocity teleop."
                )
            p = command.parameters
            vx = float(p.get("vx", 0.0))
            vy = float(p.get("vy", 0.0))
            vyaw = float(p.get("vyaw", 0.0))
            duration = float(p.get("duration", 0.0))
            try:
                await self._publish_drive(vx, vy, vyaw, duration)
            except Exception as exc:
                raise RuntimeError(f"WebRTC drive command failed: {exc}") from exc
            logger.info(
                "WebRTC drive sent: vx=%.2f vy=%.2f vyaw=%.2f duration=%.2fs",
                vx, vy, vyaw, duration,
            )
            return True

        if action in _TRANSLATION_ACTIONS:
            raise NotImplementedError(
                f"UnitreeWebRTCTransport does not support translation command "
                f"'{action}' in this iteration (posture/stop only)."
            )

        api_id = _SPORT_API_ID.get(action)
        if api_id is None:
            raise NotImplementedError(
                f"Unsupported Unitree action over WebRTC: '{action}'. "
                f"Supported: {sorted(_SPORT_API_ID)}."
            )

        if action != "stop" and not self._enable_motion:
            raise PermissionError(
                f"Motion disabled: refusing to send '{action}'. "
                f"Set RDB_UNITREE_ENABLE_MOTION=true to allow posture commands."
            )

        try:
            await self._publish_sport(api_id)
        except Exception as exc:
            raise RuntimeError(
                f"WebRTC sport command '{action}' (api_id={api_id}) failed: {exc}"
            ) from exc

        logger.info("WebRTC sport command sent: action=%s api_id=%s", action, api_id)
        return True

    async def _publish_sport(self, api_id: int, parameter: Any = None) -> Any:
        """Publish a sport request on the data channel.

        When the connection lives in the background event loop, the coroutine is
        scheduled there via run_coroutine_threadsafe; for an injected test
        connection it is awaited directly.
        """
        if self._conn is None:
            raise ConnectionError("No active WebRTC connection")

        options: dict[str, Any] = {"api_id": api_id}
        if parameter is not None:
            options["parameter"] = parameter

        pub_sub = self._conn.datachannel.pub_sub

        if self._bg_loop is not None and self._bg_loop.is_running():
            fut = asyncio.run_coroutine_threadsafe(
                pub_sub.publish_request_new(_SPORT_REQUEST_TOPIC, options),
                self._bg_loop,
            )
            response = await asyncio.to_thread(fut.result, 10.0)
        else:
            response = await pub_sub.publish_request_new(_SPORT_REQUEST_TOPIC, options)

        # Surface the robot's verdict: the response carries a status code. A
        # non-zero code means the robot received but rejected the command
        # (e.g. wrong control mode), which otherwise looks like success.
        code = _extract_status_code(response)
        logger.info(
            "WebRTC sport response: api_id=%s status_code=%s raw=%s",
            api_id, code, response,
        )
        if code is not None and code != 0:
            raise RuntimeError(
                f"robot rejected sport api_id={api_id} with status code {code}"
            )
        return response

    def _joystick_from_velocity(
        self, vx: float, vy: float, vyaw: float
    ) -> dict[str, float]:
        """Map body-frame velocities to normalized Go2 joystick axes.

        Mirrors the mapping used by the Unitree app / dimos: forward velocity
        drives the left stick Y, lateral velocity the (inverted) left stick X,
        and yaw the (inverted) right stick X. Values are clamped to [-1, 1].
        """
        return {
            "lx": _clamp_unit(-vy / _GO2_JOY_FULL_LINEAR),
            "ly": _clamp_unit(vx / _GO2_JOY_FULL_LINEAR),
            "rx": _clamp_unit(-vyaw / _GO2_JOY_FULL_YAW),
            "ry": 0.0,
        }

    async def _publish_drive(
        self, vx: float, vy: float, vyaw: float, duration: float
    ) -> None:
        """Stream joystick velocity for ``duration`` seconds, then auto-stop.

        The wireless-controller channel is fire-and-forget (no ACK), so we
        re-send the stick values at a fixed rate to hold the velocity, then
        always send a zero sample so the robot stops when the command ends.
        """
        if self._conn is None:
            raise ConnectionError("No active WebRTC connection")

        stick = self._joystick_from_velocity(vx, vy, vyaw)
        zero = {"lx": 0.0, "ly": 0.0, "rx": 0.0, "ry": 0.0}
        pub_sub = self._conn.datachannel.pub_sub
        period = 1.0 / _JOY_STREAM_HZ
        hold = max(0.0, duration)

        async def _stream() -> None:
            loop = asyncio.get_event_loop()
            deadline = loop.time() + hold
            # Always emit at least one sample, even for a zero-duration nudge.
            while True:
                pub_sub.publish_without_callback(_WIRELESS_CONTROLLER_TOPIC, data=stick)
                if loop.time() >= deadline:
                    break
                await asyncio.sleep(period)
            # Stop: send a couple of zero samples to make sure the robot halts.
            for _ in range(2):
                pub_sub.publish_without_callback(_WIRELESS_CONTROLLER_TOPIC, data=zero)
                await asyncio.sleep(period)

        if self._bg_loop is not None and self._bg_loop.is_running():
            fut = asyncio.run_coroutine_threadsafe(_stream(), self._bg_loop)
            await asyncio.to_thread(fut.result, hold + 5.0)
        else:
            await _stream()

    def _map_state(self, sport: dict[str, Any], low: dict[str, Any] | None) -> UnitreeState:
        """Map WebRTC sport mode state dict to UnitreeState."""
        try:
            # Position
            pos = sport.get("position", [0, 0, 0])
            if isinstance(pos, list) and len(pos) >= 2:
                position = Position(x=float(pos[0]), y=float(pos[1]))
            else:
                position = Position()

            # Heading from IMU
            imu = sport.get("imu_state", {})
            rpy = imu.get("rpy", [0, 0, 0]) if isinstance(imu, dict) else [0, 0, 0]
            heading = math.degrees(float(rpy[2])) if len(rpy) >= 3 else 0.0

            # Mode
            mode = int(sport.get("mode", 0))
            is_standing = mode >= 2

            # Velocity
            vel = sport.get("velocity", [0, 0, 0])
            if isinstance(vel, list) and len(vel) >= 2:
                speed = math.sqrt(float(vel[0]) ** 2 + float(vel[1]) ** 2)
            else:
                speed = 0.0
            is_moving = speed > 0.01

            # Error
            error_code = int(sport.get("error_code", 0))

            # Battery from low state
            battery = 100.0
            if low is not None:
                # power_v voltage → percentage (8S LiPo: 24V empty, 33.6V full)
                voltage = float(low.get("power_v", 0) or 0)
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
            logger.warning("WebRTC state mapping error: %s", exc)
            return UnitreeState(connected=True, error_code=-1)


def create_webrtc_transport(settings: Settings) -> UnitreeWebRTCTransport:
    """Factory function for the WebRTC transport."""
    return UnitreeWebRTCTransport(settings)
