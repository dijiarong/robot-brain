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
import socket
import threading
import time
from collections.abc import Callable
from typing import Any

from config.settings import Settings
from robot_brain.actuation.unitree import UnitreeCommand, UnitreeState, UnitreeTransport
from robot_brain.actuation.unitree_motion import MotionEndReason
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
    "free_walk": 2045,       # FreeWalk — omni locomotion on MCF (strafe/yaw need this)
    "sport_move": 1008,        # Move(vx, vy, vyaw) — omni velocity on sport API
    "switch_joystick": 1027, # SwitchJoystick — enable stick/Move control path
    "speed_level": 1015,     # SpeedLevel — gait speed (1=slow for teleop)
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
_ZERO_STICK = {"lx": 0.0, "ly": 0.0, "rx": 0.0, "ry": 0.0}
_GO2_WEBRTC_PORT = 9991
_AP_MODE_DEFAULT_IP = "192.168.123.161"

# unitree_go::msg::SportModeState.mode (unitree_ros2 README).
# 0=idle/default stand, 1=balanceStand, 3=locomotion, 5=lieDown, 7=damping, ...
_SPORT_MODE_DRIVE_BLOCKED = frozenset({5, 6, 7})  # lieDown, jointLock, damping
# Sport API ids (1001–1100) sometimes appear in the error_code field on MCF firmware — not faults.
_SPORT_API_ID_ECHO_MIN = 1001
_SPORT_API_ID_ECHO_MAX = 1100
# SportModeState.error_code=100 is often DDS/telemetry timeout on MCF — not a drive blocker.
_BENIGN_SPORT_ERROR_CODES = frozenset({100})


def _unwrap_topic_payload(msg: dict[str, Any]) -> dict[str, Any]:
    """Unwrap WebRTC pub/sub envelope so fields like ``mode`` are at top level."""
    inner = msg.get("data")
    if isinstance(inner, str):
        try:
            inner = json.loads(inner)
        except (ValueError, TypeError):
            return msg
    if isinstance(inner, dict):
        return inner
    return msg


def _looks_like_sport_state(payload: dict[str, Any]) -> bool:
    """True when payload resembles SportModeState, not a sport API response envelope."""
    header = payload.get("header")
    if isinstance(header, dict) and "identity" in header:
        return False
    return any(
        k in payload
        for k in ("mode", "position", "velocity", "imu_state", "imuState", "gait_type", "gaitType")
    )


def _parse_sport_error_code(payload: dict[str, Any]) -> int:
    """Parse SportModeState.error_code, ignoring MCF firmware API-id echo values."""
    raw = payload.get("error_code", payload.get("errorCode"))
    if raw is None:
        return 0
    try:
        code = int(raw)
    except (TypeError, ValueError):
        return 0
    if _SPORT_API_ID_ECHO_MIN <= code <= _SPORT_API_ID_ECHO_MAX:
        return 0
    return code


def _resolve_connect_target(settings: Settings) -> tuple[str | None, str | None, bool]:
    """Return (ip, serial, ip_is_default_fallback)."""
    ip = settings.unitree_robot_ip or None
    serial = settings.unitree_serial or None
    if not ip and not serial:
        return _AP_MODE_DEFAULT_IP, None, True
    return ip, serial, False


def _robot_port_reachable(ip: str, port: int = _GO2_WEBRTC_PORT, timeout: float = 2.0) -> bool:
    try:
        with socket.create_connection((ip, port), timeout=timeout):
            return True
    except OSError:
        return False


def _connect_hint(ip: str, *, ip_is_default: bool, aes_key: str | None) -> str:
    lines = [
        f"Cannot reach Go2 WebRTC at {ip}:{_GO2_WEBRTC_PORT}.",
    ]
    if ip_is_default:
        lines.append(
            "Using AP-mode default 192.168.123.161 — connect Mac to Go2 Wi-Fi hotspot, "
            "or set the robot LAN IP:"
        )
    else:
        lines.append("Check: robot powered on, same Wi-Fi/router, correct IP.")
    lines.append(
        "  export RDB_UNITREE_ROBOT_IP=<ip>   # or DIMOS_ROBOT_IP / ROBOT_IP"
    )
    lines.append("  dimos go2tool discover               # if DimOS installed")
    lines.append("  Unitree App → device info → IP")
    if not aes_key:
        lines.append(
            "Firmware >= 1.1.15 also needs: export UNITREE_AES_128_KEY=<32-hex>"
        )
    return " ".join(lines)


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
        self._last_sport_state_mono: float = 0.0
        # Single motion lease: one active drive stream at a time.
        self._motion_gen = 0
        self._motion_lock = threading.Lock()
        self._drive_idle = threading.Event()
        self._drive_idle.set()
        self._last_drive_end_reason: MotionEndReason | None = None

    @property
    def last_drive_end_reason(self) -> MotionEndReason | None:
        return self._last_drive_end_reason

    def state_age_seconds(self) -> float:
        """Seconds since the last sport-state callback, or inf if never received."""
        if self._last_sport_state_mono <= 0.0:
            return float("inf")
        return time.monotonic() - self._last_sport_state_mono

    @property
    def is_connected(self) -> bool:
        return self._connected

    async def connect(self) -> None:
        if self._connected:
            return

        if self._injected_conn is not None:
            self._conn = self._injected_conn
            self._connected = True
            # Seed standing state so preconditions and read_state work in tests.
            self._on_sport_state(
                {
                    "mode": 1,  # balanceStand
                    "error_code": 0,
                    "velocity": [0.0, 0.0, 0.0],
                    "position": [0.0, 0.0, 0.0],
                    "imu_state": {"rpy": [0.0, 0.0, 0.0]},
                }
            )
            logger.info("UnitreeWebRTCTransport connected (injected)")
            return

        UnitreeWebRTCConnection, WebRTCConnectionMethod, RTC_TOPIC = _import_webrtc()

        ip, serial, ip_is_default = _resolve_connect_target(self._settings)

        # AES key for encrypted WebRTC signaling (Go2 firmware >= 1.1.15).
        import os
        aes_key = os.environ.get("UNITREE_AES_128_KEY") or os.environ.get("UNITREE_AES_KEY") or None

        target = f"serial={serial}" if serial and not ip else f"ip={ip}"
        logger.info(
            "Connecting to Go2 via WebRTC (%s, aes_key=%s)...",
            target,
            "set" if aes_key else "none",
        )

        if ip and not _robot_port_reachable(ip):
            raise ConnectionError(_connect_hint(ip, ip_is_default=ip_is_default, aes_key=aes_key))

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
        if not ready_event.wait(timeout=self._settings.unitree_webrtc_connect_timeout):
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
                payload = _unwrap_topic_payload(msg)
                if not _looks_like_sport_state(payload):
                    return
                self._last_sport_state_mono = time.monotonic()
                self._last_sport_state = msg
            elif isinstance(msg, str):
                try:
                    parsed = json.loads(msg)
                    if isinstance(parsed, dict):
                        payload = _unwrap_topic_payload(parsed)
                        if not _looks_like_sport_state(payload):
                            return
                        self._last_sport_state_mono = time.monotonic()
                        self._last_sport_state = parsed
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
        if self._connected:
            try:
                await self._halt_motion(MotionEndReason.DISCONNECT, send_stopmove=True)
            except Exception as exc:
                logger.warning("Pre-disconnect halt failed: %s", exc)
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

    async def assert_drive_preconditions(self, settings: Settings) -> UnitreeState:
        """Verify connection, state freshness, posture and error code before drive."""
        if not self._connected:
            raise ConnectionError("UnitreeWebRTCTransport not connected")
        state = await self.read_state()
        age = self.state_age_seconds()
        if age > settings.unitree_state_max_age_seconds:
            raise RuntimeError(
                f"stale robot state ({age:.2f}s old, max {settings.unitree_state_max_age_seconds}s)"
            )
        if state.error_code != 0 and state.error_code not in _BENIGN_SPORT_ERROR_CODES:
            logger.warning(
                "Go2 sport error_code=%s before drive (non-fatal unless robot misbehaves)",
                state.error_code,
            )
        if not state.is_standing:
            mode_hint = f"sport_mode={state.sport_mode}" if state.sport_mode is not None else "sport_mode=unknown"
            raise RuntimeError(
                f"robot not ready for drive ({mode_hint}); "
                "lie down / damped — use teleop keys u (stand_up) then b (balance_stand)"
            )
        return state

    async def verify_stopped_after_drive(self, settings: Settings) -> bool:
        """Poll sport state until the robot reports not moving, within timeout."""
        deadline = time.monotonic() + settings.unitree_post_drive_stop_timeout
        while time.monotonic() < deadline:
            state = await self.read_state()
            if not state.is_moving:
                return True
            await asyncio.sleep(0.1)
        return False

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

        if action == "stop":
            await self._halt_motion(MotionEndReason.OPERATOR_STOP, send_stopmove=True)
            logger.info("WebRTC stop (halt motion + StopMove) completed")
            return True

        if action == "release":
            await self._halt_motion(MotionEndReason.RELEASE, send_stopmove=False)
            logger.debug("WebRTC release (zero joystick, no StopMove)")
            return True

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
            channel = "move(1008)" if self._settings.unitree_webrtc_drive_via_move else "joystick"
            logger.info(
                "WebRTC drive sent [%s]: vx=%.2f vy=%.2f vyaw=%.2f duration=%.2fs",
                channel,
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
            # StopMove while lie-down / idle often returns -1; zero frames already sent.
            if api_id == _SPORT_API_ID["stop"] and code == -1:
                logger.warning(
                    "StopMove rejected with code -1 (robot likely lie-down/idle); ignoring"
                )
                return response
            raise RuntimeError(
                f"robot rejected sport api_id={api_id} with status code {code}"
            )
        return response

    def _sport_move_payload(self, vx: float, vy: float, vyaw: float) -> dict[str, Any]:
        """Build fire-and-forget Move(1008) body (go2_ros2_sdk / WebRTC ``msg`` envelope)."""
        generated_id = int(time.time() * 1000) % 2147483648
        return {
            "header": {
                "identity": {
                    "id": generated_id,
                    "api_id": _SPORT_API_ID["sport_move"],
                }
            },
            "parameter": json.dumps({"x": vx, "y": vy, "z": vyaw}),
        }

    def _publish_move_velocity(
        self, pub_sub: Any, vx: float, vy: float, vyaw: float
    ) -> None:
        """Fire-and-forget sport Move(1008) — MCF omni path (matches DDS SportClient.Move)."""
        pub_sub.publish_without_callback(
            _SPORT_REQUEST_TOPIC,
            self._sport_move_payload(vx, vy, vyaw),
        )

    def _publish_joystick_velocity(
        self, pub_sub: Any, vx: float, vy: float, vyaw: float
    ) -> None:
        """DimOS-style wirelesscontroller stream (supports vx+vyaw arc while moving)."""
        pub_sub.publish_without_callback(
            _WIRELESS_CONTROLLER_TOPIC,
            data=self._joystick_from_velocity(vx, vy, vyaw),
        )

    def _use_joystick_for_velocity(self, vx: float, vy: float, vyaw: float) -> bool:
        """Pick drive channel: Move for strafe-only; joystick when yaw is involved."""
        if not self._settings.unitree_webrtc_drive_via_move:
            return True
        # MCF Move(1008) handles vy (strafe) but combined forward+turn arcs work on joystick.
        if vyaw != 0.0:
            return True
        if vy != 0.0:
            return False
        return False

    def _publish_drive_velocity(
        self, pub_sub: Any, vx: float, vy: float, vyaw: float
    ) -> str:
        """Publish one velocity sample; returns channel label for debugging."""
        if self._use_joystick_for_velocity(vx, vy, vyaw):
            self._publish_joystick_velocity(pub_sub, vx, vy, vyaw)
            return "joystick"
        self._publish_move_velocity(pub_sub, vx, vy, vyaw)
        return "move(1008)"

    async def enable_omni_teleop(self) -> None:
        """After FreeWalk: enable joystick/Move control and set a low speed level."""
        await self._publish_sport(
            _SPORT_API_ID["switch_joystick"], parameter={"data": True}
        )
        await self._publish_sport(_SPORT_API_ID["speed_level"], parameter={"data": 1})
        logger.info("WebRTC omni teleop enabled (SwitchJoystick + SpeedLevel=1)")

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

    async def _halt_motion(
        self, reason: MotionEndReason, *, send_stopmove: bool
    ) -> None:
        """Invalidate the active lease, wait for zero frames, optionally StopMove."""
        with self._motion_lock:
            self._motion_gen += 1
        await self._wait_drive_idle()
        if self._conn is not None:
            await self._send_drive_zeros()
        self._last_drive_end_reason = reason
        if send_stopmove:
            await self._publish_sport(_SPORT_API_ID["stop"])

    async def _wait_drive_idle(self, timeout: float = 10.0) -> None:
        """Block until the in-flight drive stream finishes (including zero frames)."""
        if self._drive_idle.is_set():
            return
        await asyncio.to_thread(self._drive_idle.wait, timeout)

    def _bump_motion_generation(self) -> int:
        with self._motion_lock:
            self._motion_gen += 1
            return self._motion_gen

    async def _publish_drive(
        self, vx: float, vy: float, vyaw: float, duration: float
    ) -> None:
        """Acquire motion lease and stream joystick velocity, then auto-stop."""
        if self._conn is None:
            raise ConnectionError("No active WebRTC connection")

        gen = self._bump_motion_generation()
        await self._wait_drive_idle()

        self._drive_idle.clear()
        try:
            await self._run_on_conn_loop(
                self._drive_stream(gen, vx, vy, vyaw, duration),
                timeout=max(duration, 0.0) + 10.0,
            )
            if gen == self._motion_gen and self._last_drive_end_reason not in (
                MotionEndReason.PREEMPTED,
                MotionEndReason.OPERATOR_STOP,
                MotionEndReason.DISCONNECT,
                MotionEndReason.CANCELLED,
                MotionEndReason.WATCHDOG,
                MotionEndReason.TRANSPORT_ERROR,
            ):
                self._last_drive_end_reason = MotionEndReason.COMPLETED
        finally:
            self._drive_idle.set()

    async def stream_hold(
        self,
        get_velocity: Callable[[], tuple[float, float, float]],
        *,
        session_deadline: float,
        zero_on_exit: bool = True,
    ) -> None:
        """Continuous teleop stream — velocity updates without per-tick zero frames."""
        if self._conn is None:
            raise ConnectionError("No active WebRTC connection")

        gen = self._bump_motion_generation()
        await self._wait_drive_idle()
        self._drive_idle.clear()
        timeout = max(0.0, session_deadline - time.time()) + 15.0
        try:
            await self._run_on_conn_loop(
                self._hold_stream(gen, get_velocity, session_deadline),
                timeout=timeout,
            )
            if gen == self._motion_gen:
                self._last_drive_end_reason = MotionEndReason.COMPLETED
        finally:
            self._drive_idle.set()
            if zero_on_exit and self._conn is not None:
                await self._send_drive_zeros()

    async def _hold_stream(
        self,
        gen: int,
        get_velocity: Callable[[], tuple[float, float, float]],
        session_deadline: float,
    ) -> None:
        """Publish velocity at 50Hz until preempted, disconnected, or session ends."""
        pub_sub = self._conn.datachannel.pub_sub
        use_move = self._settings.unitree_webrtc_drive_via_move
        period = 1.0 / _JOY_STREAM_HZ
        watchdog = self._settings.unitree_control_watchdog_seconds
        loop = asyncio.get_event_loop()
        last_send_mono = 0.0
        end_reason = MotionEndReason.COMPLETED

        try:
            while loop.time() < session_deadline:
                if gen != self._motion_gen:
                    end_reason = MotionEndReason.PREEMPTED
                    break
                if not self._connected:
                    end_reason = MotionEndReason.DISCONNECT
                    break
                vx, vy, vyaw = get_velocity()
                if not (vx or vy or vyaw):
                    await asyncio.sleep(period)
                    continue
                now_mono = time.monotonic()
                if last_send_mono > 0 and (now_mono - last_send_mono) > watchdog:
                    end_reason = MotionEndReason.WATCHDOG
                    break
                if use_move:
                    self._publish_drive_velocity(pub_sub, vx, vy, vyaw)
                else:
                    self._publish_joystick_velocity(pub_sub, vx, vy, vyaw)
                last_send_mono = time.monotonic()
                await asyncio.sleep(period)
        except asyncio.CancelledError:
            end_reason = MotionEndReason.CANCELLED
            raise
        except Exception:
            end_reason = MotionEndReason.TRANSPORT_ERROR
            raise
        finally:
            self._last_drive_end_reason = end_reason

    async def _run_on_conn_loop(
        self, coro: Any, *, timeout: float
    ) -> Any:
        if self._bg_loop is not None and self._bg_loop.is_running():
            fut = asyncio.run_coroutine_threadsafe(coro, self._bg_loop)
            return await asyncio.to_thread(fut.result, timeout)
        return await coro

    async def _drive_stream(
        self,
        gen: int,
        vx: float,
        vy: float,
        vyaw: float,
        duration: float,
    ) -> None:
        """Send non-zero joystick frames for ``duration``, then zero frames in ``finally``."""
        pub_sub = self._conn.datachannel.pub_sub
        use_move = self._settings.unitree_webrtc_drive_via_move
        period = 1.0 / _JOY_STREAM_HZ
        hold = max(0.0, duration)
        watchdog = self._settings.unitree_control_watchdog_seconds
        zero_count = self._settings.unitree_zero_frame_count
        loop = asyncio.get_event_loop()
        deadline = loop.time() + hold
        last_send_mono = 0.0
        end_reason = MotionEndReason.COMPLETED

        try:
            while True:
                if gen != self._motion_gen:
                    end_reason = MotionEndReason.PREEMPTED
                    break
                if not self._connected:
                    end_reason = MotionEndReason.DISCONNECT
                    break
                now_mono = time.monotonic()
                if last_send_mono > 0 and (now_mono - last_send_mono) > watchdog:
                    end_reason = MotionEndReason.WATCHDOG
                    break

                if use_move:
                    self._publish_drive_velocity(pub_sub, vx, vy, vyaw)
                else:
                    self._publish_joystick_velocity(pub_sub, vx, vy, vyaw)
                last_send_mono = time.monotonic()

                if loop.time() >= deadline:
                    break
                await asyncio.sleep(period)
        except asyncio.CancelledError:
            end_reason = MotionEndReason.CANCELLED
            raise
        except Exception:
            end_reason = MotionEndReason.TRANSPORT_ERROR
            raise
        finally:
            self._last_drive_end_reason = end_reason
            await self._send_drive_zeros(count=zero_count, pub_sub=pub_sub, period=period)

    async def _send_drive_zeros(
        self,
        count: int | None = None,
        *,
        pub_sub: Any = None,
        period: float | None = None,
    ) -> None:
        """Zero velocity on active drive channel(s) after a move command."""
        if self._conn is None:
            return
        pub = pub_sub or self._conn.datachannel.pub_sub
        interval = period if period is not None else (1.0 / _JOY_STREAM_HZ)
        frames = count if count is not None else self._settings.unitree_zero_frame_count
        use_move = self._settings.unitree_webrtc_drive_via_move

        async def _zeros() -> None:
            for _ in range(frames):
                if use_move:
                    self._publish_move_velocity(pub, 0.0, 0.0, 0.0)
                self._publish_joystick_velocity(pub, 0.0, 0.0, 0.0)
                await asyncio.sleep(interval)

        if pub_sub is not None:
            await _zeros()
        else:
            await self._run_on_conn_loop(_zeros(), timeout=frames * interval + 5.0)

    def _map_state(self, sport: dict[str, Any], low: dict[str, Any] | None) -> UnitreeState:
        """Map WebRTC sport mode state dict to UnitreeState."""
        try:
            sport = _unwrap_topic_payload(sport)
            if low is not None and isinstance(low, dict):
                low = _unwrap_topic_payload(low)

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

            # Mode — Go2 SportModeState enum (not the older SDK passive/stand_up scale).
            mode = int(sport.get("mode", 0))
            is_standing = mode not in _SPORT_MODE_DRIVE_BLOCKED

            # Velocity
            vel = sport.get("velocity", [0, 0, 0])
            if isinstance(vel, list) and len(vel) >= 2:
                speed = math.sqrt(float(vel[0]) ** 2 + float(vel[1]) ** 2)
            else:
                speed = 0.0
            is_moving = speed > 0.01

            error_code = _parse_sport_error_code(sport)

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
                sport_mode=mode,
                velocity=(float(vel[0]), float(vel[1]), float(vel[2]) if len(vel) >= 3 else 0.0),
                imu_rpy=(float(rpy[0]), float(rpy[1]), float(rpy[2]) if len(rpy) >= 3 else 0.0),
            )
        except Exception as exc:
            logger.warning("WebRTC state mapping error: %s", exc)
            return UnitreeState(connected=True, error_code=-1)


def create_webrtc_transport(settings: Settings) -> UnitreeWebRTCTransport:
    """Factory function for the WebRTC transport."""
    return UnitreeWebRTCTransport(settings)
