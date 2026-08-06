"""Unitree Go2 transport via WebRTC data channel (LAN or cloud Remote mode).

Uses unitree-webrtc-connect to communicate with Go2 over the local network
(router/STA mode). This works when Go2 and the dev machine are on the same
LAN — no direct Wi-Fi hotspot connection required.

This iteration supports posture/stop commands (StandUp, StandDown, Sit,
BalanceStand, RecoveryStand, Damp, StopMove) plus velocity teleop ("drive")
over the WIRELESS_CONTROLLER joystick channel — the same path the Unitree app
and dimos's keyboard control use, which drives the robot reliably even when the
high-level sport (SPORT_MOD) controller ignores posture commands. All motion
requires RDB_UNITREE_ENABLE_MOTION=true (stop is always allowed when connected).

Resilience (2026-06):
- Smart retry: tight 2s loop for slot-occupied, exponential backoff for network
  errors, immediate fail for auth errors.
- SIGTERM/SIGINT handler + atexit: guaranteed graceful disconnect on any normal
  exit, releasing the Go2 WebRTC slot immediately.
- Connection state machine: CONNECTING / CONNECTED / DISCONNECTING / DISCONNECTED
  with a condition variable so callers can await state transitions.
- Background keepalive: after initial connect, if the background loop detects
  connection loss it auto-reconnects internally (keeping the same transport
  object alive), so a transient network blip does not require a full gateway
  restart.
"""
from __future__ import annotations

import asyncio
import atexit
import json
import logging
import math
import os
import socket
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass, replace
from enum import Enum
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
    "speed_level": 1015,     # SpeedLevel — gait speed (1=slow … 5=fast)
    "hello": 1016,           # Hello — built-in front-leg wave gesture
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

# ---------------------------------------------------------------------------
# Smart retry strategy
# ---------------------------------------------------------------------------
# Go2 firmware releases a stale WebRTC slot after ~60s.  When we detect
# "slot occupied" (ICE completes but DTLS/data-channel times out), we retry
# at a tight 2s interval for up to 90s to grab the slot as soon as it frees.
_SLOT_OCCUPIED_RETRY_INTERVAL = 2.0   # seconds
_SLOT_OCCUPIED_MAX_WAIT = 90.0         # seconds
# For network-level errors (port unreachable, connection refused), back off.
_NETWORK_RETRY_BASE = 1.0
_NETWORK_RETRY_MAX = 30.0
_NETWORK_RETRY_BACKOFF = 2.0

# ---------------------------------------------------------------------------
# Connection state
# ---------------------------------------------------------------------------


class ConnectionState(str, Enum):
    DISCONNECTED = "disconnected"
    CONNECTING = "connecting"
    CONNECTED = "connected"
    DISCONNECTING = "disconnecting"


# ---------------------------------------------------------------------------
# Error classification
# ---------------------------------------------------------------------------

class SlotOccupiedError(ConnectionError):
    """Go2 WebRTC slot is held by another client — retry will succeed once freed."""


class AuthError(ConnectionError):
    """AES key or other auth material is wrong — retry will NOT help."""


class NetworkUnreachableError(ConnectionError):
    """Network-level failure — retry with backoff may help."""


def _classify_connect_error(
    exc: BaseException,
    aes_key_set: bool,
    *,
    remote: bool = False,
) -> ConnectionError:
    """Wrap a raw connect exception into a classified error for smart retry."""
    msg = str(exc)

    # -- Slot occupied: ICE ok but DTLS / data-channel never completed --
    if any(phrase in msg.lower() for phrase in (
        "timed out", "never reached", "dtls", "data channel",
    )):
        hint_parts = [
            "Go2 WebRTC slot occupied — ICE completed but DTLS/data-channel failed.",
        ]
        if not remote and not aes_key_set:
            hint_parts.append(
                "UNITREE_AES_128_KEY is NOT set. "
                "Go2 firmware >= 1.1.15 requires it for DTLS handshake."
            )
            hint_parts.append(
                "Get it: unitree-fetch-aes-key --email <account> --device-type Go2"
            )
            return AuthError("\n".join(hint_parts))
        hint_parts.append(
            "Another client holds the Go2 WebRTC slot (Unitree app, another script). "
            "Will retry every 2s until the slot frees (Go2 timeout ~60s)."
        )
        return SlotOccupiedError("\n".join(hint_parts))

    # -- Auth / permission --
    if any(phrase in msg.lower() for phrase in (
        "aes", "key", "unauthorized", "forbidden", "permission",
        "login", "password", "token", "account", "401", "403",
    )):
        prefix = "Unitree cloud login or device authorization failed" if remote else "AES key or auth failure"
        return AuthError(f"{prefix}: {msg[:300]}")

    # -- Network unreachable --
    if any(phrase in msg.lower() for phrase in (
        "cannot reach", "connection refused", "no route",
        "network", "unreachable", "name resolution",
    )):
        return NetworkUnreachableError(f"Network unreachable: {msg[:300]}")

    # -- Fallback: treat as slot-occupied (may recover on retry) --
    return SlotOccupiedError(f"Connection failed (will retry): {msg[:300]}")


# ---------------------------------------------------------------------------
# Shutdown coordination
# ---------------------------------------------------------------------------
# _shutdown_flag is a module-level event set once by the signal/atexit handler.
# All connect attempts check it and abort early.
_shutdown_flag = threading.Event()
_shutdown_handler_installed = False
_shutdown_handler_lock = threading.Lock()


def _install_shutdown_handlers() -> None:
    """Install atexit handler ONCE (idempotent, thread-safe).

    Only registers atexit — NOT signal handlers.  Signal handlers are the
    application's responsibility.  Installing a SIGINT handler from a library
    would suppress Python's normal KeyboardInterrupt and make Ctrl+C useless.
    """
    global _shutdown_handler_installed
    with _shutdown_handler_lock:
        if _shutdown_handler_installed:
            return
        _shutdown_handler_installed = True

    def _on_exit() -> None:
        """atexit handler: set the global shutdown flag so the background
        loop calls disconnect() and releases the Go2 WebRTC slot.
        """
        if not _shutdown_flag.is_set():
            _shutdown_flag.set()
            import sys
            print(
                "[unitree_webrtc] atexit — Go2 slot will be released",
                file=sys.stderr,
            )

    atexit.register(_on_exit)


# ---------------------------------------------------------------------------
# Connection health
# ---------------------------------------------------------------------------


@dataclass
class WebRTCHealth:
    """Snapshot of connection health for observability."""
    connected: bool = False
    state: ConnectionState = ConnectionState.DISCONNECTED
    reconnect_count: int = 0
    last_connect_attempt: float = 0.0
    last_state_update: float = 0.0
    sport_state_count: int = 0
    low_state_count: int = 0
    lidar_compressed_message_count: int = 0
    lidar_uncompressed_message_count: int = 0
    lidar_state_count: int = 0
    last_lidar_state: Any = None
    lidar_frame_count: int = 0
    last_lidar_update: float = 0.0
    lidar_timestamp_repair_count: int = 0
    odom_frame_count: int = 0
    last_odom_update: float = 0.0
    drive_streams_active: int = 0
    bridge_call_timeouts: int = 0


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


def _extract_ultrasonic_from_dict(low: dict[str, Any]) -> tuple[float, float, float, float] | None:
    """Extract ultrasonic distances (metres) from a WebRTC low-state dict.

    The Go2 WebRTC service serialises the DDS LowState_ as JSON.  The
    ``ultrasonic`` key holds an array of 4 integers in mm:
    [front, left, right, rear].  Falls back gracefully on missing / malformed
    data (older firmware, mock, or non-Go2 backends).
    """
    raw = low.get("ultrasonic")
    if raw is None or not isinstance(raw, (list, tuple)) or len(raw) < 4:
        return None
    try:
        def _to_m(v: Any) -> float | None:
            val = float(v)
            if val <= 0 or val >= 65000:
                return None
            return val / 1000.0

        front = _to_m(raw[0])
        left_ = _to_m(raw[1])
        right_ = _to_m(raw[2])
        rear = _to_m(raw[3])
        if all(v is None for v in (front, left_, right_, rear)):
            return None
        return (front or 0.0, left_ or 0.0, right_ or 0.0, rear or 0.0)
    except (TypeError, ValueError, IndexError):
        return None


def _parse_robot_odom(message: Any) -> dict[str, Any] | None:
    """Parse ``rt/utlidar/robot_pose`` into the shared Go2 odom frame."""
    if isinstance(message, str):
        try:
            message = json.loads(message)
        except json.JSONDecodeError:
            return None
    if not isinstance(message, dict):
        return None
    data = message.get("data", message)
    if not isinstance(data, dict):
        return None
    pose = data.get("pose")
    header = data.get("header", {})
    if not isinstance(pose, dict) or not isinstance(header, dict):
        return None
    position = pose.get("position")
    orientation = pose.get("orientation")
    if not isinstance(position, dict) or not isinstance(orientation, dict):
        return None
    try:
        x, y = float(position["x"]), float(position["y"])
        qx = float(orientation.get("x", 0.0))
        qy = float(orientation.get("y", 0.0))
        qz = float(orientation.get("z", 0.0))
        qw = float(orientation.get("w", 1.0))
        values = (x, y, qx, qy, qz, qw)
        if not all(math.isfinite(value) for value in values):
            return None
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if norm <= 1e-9:
            return None
        qx, qy, qz, qw = (value / norm for value in (qx, qy, qz, qw))
        siny = 2.0 * (qw * qz + qx * qy)
        cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
        stamp = header.get("stamp", {})
        sensor_timestamp = None
        if isinstance(stamp, dict):
            sensor_timestamp = float(stamp.get("sec", 0)) + float(stamp.get("nanosec", 0)) / 1e9
        return {
            "position": Position(x=x, y=y),
            "heading_degrees": math.degrees(math.atan2(siny, cosy)),
            "frame_id": str(header.get("frame_id") or "odom"),
            "timestamp": sensor_timestamp if sensor_timestamp and sensor_timestamp > 0 else None,
        }
    except (KeyError, TypeError, ValueError, OverflowError):
        return None
def _resolve_connect_target(settings: Settings) -> tuple[str | None, str | None, bool]:
    """Return (ip, serial, ip_is_default_fallback)."""
    ip = settings.unitree_robot_ip or None
    serial = settings.unitree_serial or None
    if not ip and not serial:
        return _AP_MODE_DEFAULT_IP, None, True
    return ip, serial, False


def _resolve_connection_mode(settings: Settings) -> str:
    """Resolve auto/local/remote without exposing cloud credentials."""
    mode = settings.unitree_webrtc_connection_mode.strip().lower()
    if mode not in {"auto", "local", "remote"}:
        raise ValueError(
            "RDB_UNITREE_WEBRTC_CONNECTION_MODE must be auto, local, or remote"
        )
    if mode != "auto":
        return mode
    if settings.unitree_robot_ip:
        return "local"
    if (
        settings.unitree_serial
        and settings.unitree_cloud_username
        and settings.unitree_cloud_password
    ):
        return "remote"
    return "local"


def _connection_target(settings: Settings) -> str:
    """Return a secret-free connection description for logs and errors."""
    mode = _resolve_connection_mode(settings)
    if mode == "remote":
        serial = settings.unitree_serial or "missing"
        region = settings.unitree_cloud_region or "global"
        return f"cloud serial={serial}, region={region}"
    ip, serial, _ = _resolve_connect_target(settings)
    return f"serial={serial}" if serial and not ip else f"ip={ip}"


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


# ---------------------------------------------------------------------------
# Connect helpers (factored out for reconnection logic)
# ---------------------------------------------------------------------------


async def _safe_disconnect_conn(conn: Any) -> None:
    """Best-effort disconnect so a failed handshake releases the Go2 WebRTC slot."""
    if conn is None or not hasattr(conn, "disconnect"):
        return
    try:
        await conn.disconnect()
        logger.info("Cleaned up failed Go2 WebRTC handshake (slot released)")
    except Exception as exc:
        logger.warning("Go2 WebRTC cleanup after failed connect: %s", exc)


async def _do_connect(
    settings: Settings,
) -> tuple[Any, dict[str, Any]]:
    """Create and connect a UnitreeWebRTCConnection.  Must run on the background event loop."""
    import inspect

    UnitreeWebRTCConnection, WebRTCConnectionMethod, RTC_TOPIC = _import_webrtc()

    mode = _resolve_connection_mode(settings)
    ip, serial, ip_is_default = _resolve_connect_target(settings)
    aes_key = _aes_key_from_env()

    if mode == "local" and ip and not _robot_port_reachable(ip):
        raise NetworkUnreachableError(
            _connect_hint(ip, ip_is_default=ip_is_default, aes_key=aes_key)
        )

    init_params = inspect.signature(UnitreeWebRTCConnection.__init__).parameters
    kwargs: dict[str, Any]
    if mode == "remote":
        if not serial:
            raise ValueError("RDB_UNITREE_SERIAL is required for Unitree cloud Remote mode")
        if not settings.unitree_cloud_username or not settings.unitree_cloud_password:
            raise ValueError(
                "RDB_UNITREE_CLOUD_USERNAME and RDB_UNITREE_CLOUD_PASSWORD are required "
                "for Unitree cloud Remote mode"
            )
        kwargs = {
            "connectionMethod": WebRTCConnectionMethod.Remote,
            "serialNumber": serial,
            "username": settings.unitree_cloud_username,
            "password": settings.unitree_cloud_password,
            "region": settings.unitree_cloud_region,
            "device_type": settings.unitree_cloud_device_type,
        }
    else:
        kwargs = {"connectionMethod": WebRTCConnectionMethod.LocalSTA}
    if serial and "serialNumber" in init_params:
        kwargs["serialNumber"] = serial
    if mode == "local" and ip:
        kwargs["ip"] = ip
    if mode == "local" and aes_key and "aes_128_key" in init_params:
        kwargs["aes_128_key"] = aes_key

    conn = UnitreeWebRTCConnection(**kwargs)
    try:
        await conn.connect()
    except Exception as exc:
        await _safe_disconnect_conn(conn)
        raise _classify_connect_error(
            exc, aes_key_set=bool(aes_key), remote=mode == "remote"
        ) from exc

    media_started = False
    try:
        # Media relay / gateway need track consumers; pure control removes them.
        # With media_on_demand, skip ffmpeg until ensure_media_relays().
        if not settings.unitree_dry_run:
            if settings.unitree_gateway:
                # Match gRPC connect priming (outbound audio + video consumers) but
                # stay in-process — no ffmpeg / UDP :5000/:5005/:5010.
                from robot_brain.media.go2_audio_relay import prime_go2_audio_for_connect
                from robot_brain.media.go2_video_relay import prime_go2_video_for_connect

                prime_go2_video_for_connect(conn)
                prime_go2_audio_for_connect(conn)
                media_started = True
                logger.info("Go2 WebRTC ready (gateway mode)")
            elif settings.unitree_media_on_demand:
                logger.info(
                    "Go2 WebRTC connected; ffmpeg media relays deferred "
                    "(RDB_UNITREE_MEDIA_ON_DEMAND=true)"
                )
            else:
                if settings.unitree_video_relay:
                    from robot_brain.media.go2_video_relay import start_go2_video_relay

                    start_go2_video_relay(
                        conn,
                        host=settings.unitree_video_relay_host,
                        port=settings.unitree_video_relay_port,
                    )
                    media_started = True
                if settings.unitree_audio_relay:
                    from robot_brain.media.go2_audio_relay import start_go2_audio_relay

                    start_go2_audio_relay(
                        conn,
                        relay_host=settings.unitree_audio_relay_host,
                        relay_port=settings.unitree_audio_relay_port,
                        ingress_host=settings.unitree_audio_ingress_host,
                        ingress_port=settings.unitree_audio_ingress_port,
                    )
                    media_started = True

        needs_media = (
            not settings.unitree_dry_run
            and (
                settings.unitree_gateway
                or (
                    not settings.unitree_media_on_demand
                    and (settings.unitree_video_relay or settings.unitree_audio_relay)
                )
            )
        )
        if hasattr(conn, "pc") and not needs_media:
            conn.pc.remove_all_listeners("track")

        await conn.datachannel.disableTrafficSaving(True)
        conn.datachannel.set_decoder(decoder_type="native")

        lidar_requested = (
            settings.unitree_lidar_stream
            or settings.navigation_backend in {"direct_go2", "native_go2"}
        )
        lidar_switch_topic = RTC_TOPIC.get("ULIDAR_SWITCH")
        if lidar_requested and lidar_switch_topic:
            # Unitree's WebRTC bridge does not emit voxel_map_compressed until
            # this sensor-stream switch is explicitly enabled.
            conn.datachannel.pub_sub.publish_without_callback(
                lidar_switch_topic,
                "on",
            )
            logger.info("Go2 built-in LiDAR stream requested (%s)", lidar_switch_topic)

        conn_info = {
            "mode": mode,
            "ip": ip if mode == "local" else None,
            "serial": serial,
            "aes_key_set": bool(aes_key) if mode == "local" else False,
            "target": _connection_target(settings),
            "ip_is_default": ip_is_default if mode == "local" else False,
            "media_relays_started": media_started,
            "media_on_demand": bool(settings.unitree_media_on_demand),
        }

        # Check current motion mode
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

        if settings.unitree_enable_motion and current_mode != settings.unitree_motion_mode:
            select = await pub_sub.publish_request_new(
                RTC_TOPIC["MOTION_SWITCHER"],
                {"api_id": 1002, "parameter": {"name": settings.unitree_motion_mode}},
            )
            logger.info(
                "MotionSwitcher select '%s' (api_id=1002) -> code=%s (current=%s)",
                settings.unitree_motion_mode,
                _extract_status_code(select),
                current_mode,
            )

        return conn, conn_info
    except Exception:
        await _safe_disconnect_conn(conn)
        raise


class UnitreeWebRTCTransport(UnitreeTransport):
    """Real transport using WebRTC data channel for Go2 in STA/LAN mode.

    Subscribes to sport mode state and low state via WebRTC pub/sub, and sends
    posture/stop commands over SPORT_MOD plus velocity teleop ("drive") over the
    WIRELESS_CONTROLLER joystick channel.

    Resilience features:
    - Graceful shutdown: SIGTERM/SIGINT/atexit → guaranteed disconnect
    - Smart retry: tight loop for slot-occupied, backoff for network errors,
      immediate fail for auth errors
    - Connection state machine with condition variable for awaiting transitions
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
        self._last_lidar = None
        self._last_lidar_sensor_stamp: float | None = None
        self._last_robot_odom: dict[str, Any] | None = None
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
        self._last_sport_api: dict[str, Any] = {}
        # Health / observability
        self._health = WebRTCHealth()
        # Connection state machine
        self._conn_state = ConnectionState.DISCONNECTED
        self._conn_state_cond = threading.Condition(threading.Lock())
        # Shutdown coordination
        self._transport_shutdown = False
        # On-connect callbacks (gateway re-attaches media after reconnect)
        self._on_connect_callbacks: list[Callable[[Any], None]] = []
        # ffmpeg RTP relays (deferred when unitree_media_on_demand)
        self._media_relays_started = False

        _install_shutdown_handlers()

    # ------------------------------------------------------------------ health
    @property
    def health(self) -> WebRTCHealth:
        """Snapshot of connection health for observability."""
        with self._state_lock:
            h = WebRTCHealth(
                connected=self._connected,
                state=self._conn_state,
                reconnect_count=self._health.reconnect_count,
                last_connect_attempt=self._health.last_connect_attempt,
                last_state_update=self._last_sport_state_mono,
                sport_state_count=self._health.sport_state_count,
                low_state_count=self._health.low_state_count,
                lidar_compressed_message_count=(
                    self._health.lidar_compressed_message_count
                ),
                lidar_uncompressed_message_count=(
                    self._health.lidar_uncompressed_message_count
                ),
                lidar_state_count=self._health.lidar_state_count,
                last_lidar_state=self._health.last_lidar_state,
                lidar_frame_count=self._health.lidar_frame_count,
                last_lidar_update=self._health.last_lidar_update,
                lidar_timestamp_repair_count=self._health.lidar_timestamp_repair_count,
                odom_frame_count=self._health.odom_frame_count,
                last_odom_update=self._health.last_odom_update,
                bridge_call_timeouts=self._health.bridge_call_timeouts,
            )
        return h

    @property
    def connection_state(self) -> ConnectionState:
        return self._conn_state

    @property
    def connection(self) -> Any:
        """Expose the connected media session to service-owned read-only taps."""
        return self._conn

    def add_on_connect(self, callback: Callable[[Any], None]) -> None:
        """Register a callback invoked after each successful (re)connect with the conn."""
        self._on_connect_callbacks.append(callback)

    @property
    def media_relays_started(self) -> bool:
        return self._media_relays_started

    def ensure_media_relays(self) -> dict[str, object]:
        """Start deferred ffmpeg video/audio RTP relays (idempotent).

        Call when an operator actually needs topsun/browser media. Safe no-op if
        relays are disabled in settings, already started, or not connected.
        """
        if self._settings.unitree_dry_run:
            return {"started": False, "reason": "dry_run"}
        if self._settings.unitree_gateway:
            return {"started": False, "reason": "gateway_uses_in_process_media"}
        if self._media_relays_started:
            return {"started": True, "reason": "already_running"}
        conn = self._conn
        if conn is None or not self._connected:
            return {"started": False, "reason": "not_connected"}

        started_video = False
        started_audio = False
        if self._settings.unitree_video_relay:
            from robot_brain.media.go2_video_relay import start_go2_video_relay

            start_go2_video_relay(
                conn,
                host=self._settings.unitree_video_relay_host,
                port=self._settings.unitree_video_relay_port,
            )
            started_video = True
        if self._settings.unitree_audio_relay:
            from robot_brain.media.go2_audio_relay import start_go2_audio_relay

            start_go2_audio_relay(
                conn,
                relay_host=self._settings.unitree_audio_relay_host,
                relay_port=self._settings.unitree_audio_relay_port,
                ingress_host=self._settings.unitree_audio_ingress_host,
                ingress_port=self._settings.unitree_audio_ingress_port,
            )
            started_audio = True
        if not started_video and not started_audio:
            return {
                "started": False,
                "reason": "relays_disabled_in_settings",
                "video": False,
                "audio": False,
            }
        self._media_relays_started = True
        logger.info(
            "Go2 media relays started on demand (video=%s audio=%s)",
            started_video,
            started_audio,
        )
        return {
            "started": True,
            "reason": "started",
            "video": started_video,
            "audio": started_audio,
        }

    @property
    def last_drive_end_reason(self) -> MotionEndReason | None:
        return self._last_drive_end_reason

    def state_age_seconds(self) -> float:
        """Seconds since the last sport-state callback, or inf if never received."""
        if self._last_sport_state_mono <= 0.0:
            return float("inf")
        return time.monotonic() - self._last_sport_state_mono

    def lidar_age_seconds(self) -> float:
        """Seconds since the last valid built-in LiDAR frame."""
        updated = self._health.last_lidar_update
        return float("inf") if updated <= 0.0 else time.monotonic() - updated

    def odometry_age_seconds(self) -> float:
        updated = self._health.last_odom_update
        return self.state_age_seconds() if updated <= 0.0 else time.monotonic() - updated

    def read_lidar_snapshot(self):
        """Return the latest immutable built-in LiDAR frame, if available."""
        with self._state_lock:
            return self._last_lidar

    @property
    def is_connected(self) -> bool:
        return self._connected

    # --------------------------------------------------------------- connection
    async def connect(self) -> None:
        if self._connected:
            return

        if self._injected_conn is not None:
            self._conn = self._injected_conn
            self._connected = True
            self._set_conn_state(ConnectionState.CONNECTED)
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

        _install_benign_noise_filter()

        mode = _resolve_connection_mode(self._settings)
        target = _connection_target(self._settings)

        # --- Smart retry loop ---
        slot_retry_deadline = 0.0
        attempt = 0
        last_error: BaseException | None = None

        while not _shutdown_flag.is_set() and not self._transport_shutdown:
            self._set_conn_state(ConnectionState.CONNECTING)
            self._health.last_connect_attempt = time.monotonic()
            aes_set = bool(_aes_key_from_env())

            logger.info(
                "Connecting to Go2 via WebRTC (%s, mode=%s, attempt=%d, aes_key=%s)...",
                target, mode, attempt,
                "n/a" if mode == "remote" else ("set" if aes_set else "none"),
            )

            try:
                await self._connect_once(target)
                self._health.reconnect_count = attempt
                self._set_conn_state(ConnectionState.CONNECTED)
                logger.info(
                    "UnitreeWebRTCTransport connected via WebRTC (%s, attempt=%d)",
                    target, attempt,
                )
                return
            except AuthError:
                await self._cleanup_failed_attempt()
                self._set_conn_state(ConnectionState.DISCONNECTED)
                raise
            except SlotOccupiedError as exc:
                last_error = exc
                await self._cleanup_failed_attempt()
                # Tight retry: every 2s until Go2 releases the slot (~60s).
                if slot_retry_deadline == 0.0:
                    slot_retry_deadline = time.monotonic() + _SLOT_OCCUPIED_MAX_WAIT
                if time.monotonic() >= slot_retry_deadline:
                    logger.error(
                        "Go2 slot still occupied after %.0fs — giving up. "
                        "Power-cycle the robot or kill the other client.",
                        _SLOT_OCCUPIED_MAX_WAIT,
                    )
                    break
                remaining = slot_retry_deadline - time.monotonic()
                detail = str(exc).split("\n")[0]
                logger.warning(
                    "Go2 slot occupied (attempt %d): %s. Retrying every %.0fs "
                    "for up to %.0fs more — slot frees when the other client "
                    "disconnects or the Go2 firmware timeout expires (~60s).",
                    attempt, detail, _SLOT_OCCUPIED_RETRY_INTERVAL, remaining,
                )
                await asyncio.sleep(_SLOT_OCCUPIED_RETRY_INTERVAL)
            except NetworkUnreachableError as exc:
                last_error = exc
                await self._cleanup_failed_attempt()
                delay = min(
                    _NETWORK_RETRY_MAX,
                    _NETWORK_RETRY_BASE * (_NETWORK_RETRY_BACKOFF ** attempt),
                )
                logger.warning(
                    "Go2 network unreachable (attempt %d). Backing off %.0fs.",
                    attempt, delay,
                )
                await asyncio.sleep(delay)
            except ConnectionError as exc:
                last_error = exc
                await self._cleanup_failed_attempt()
                delay = min(30.0, 1.0 * (2.0 ** attempt))
                logger.warning(
                    "Go2 connection failed (attempt %d): %s. Retrying in %.0fs.",
                    attempt, exc, delay,
                )
                await asyncio.sleep(delay)
            except Exception as exc:
                last_error = exc
                await self._cleanup_failed_attempt()
                logger.error(
                    "Unexpected error during Go2 connect (attempt %d): %s",
                    attempt, exc,
                )
                delay = min(30.0, 1.0 * (2.0 ** attempt))
                await asyncio.sleep(delay)

            attempt += 1

        self._set_conn_state(ConnectionState.DISCONNECTED)
        if _shutdown_flag.is_set():
            raise ConnectionError("Connection aborted — process is shutting down")
        raise ConnectionError(
            f"Failed to connect via WebRTC to Go2 ({target}) after "
            f"{attempt + 1} attempts. Last error: {last_error}"
        ) from last_error

    async def _connect_once(self, target: str) -> None:
        """Single connection attempt — spawns background loop and waits for ready."""
        connection_error: list[BaseException] = []
        ready_event = threading.Event()

        def run_bg_loop() -> None:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            self._bg_loop = loop

            def _ignore_media_stream_errors(
                loop_: asyncio.AbstractEventLoop, context: dict[str, Any]
            ) -> None:
                exc = context.get("exception")
                if exc is not None and type(exc).__name__ == "MediaStreamError":
                    return
                loop_.default_exception_handler(context)

            loop.set_exception_handler(_ignore_media_stream_errors)

            async def async_connect() -> None:
                try:
                    conn, conn_info = await _do_connect(self._settings)
                    self._conn = conn

                    RTC_TOPIC = _import_webrtc()[2]
                    # Subscribe to state topics
                    conn.datachannel.pub_sub.subscribe(
                        RTC_TOPIC["LF_SPORT_MOD_STATE"],
                        self._on_sport_state,
                    )
                    conn.datachannel.pub_sub.subscribe(
                        RTC_TOPIC["LOW_STATE"],
                        self._on_low_state,
                    )
                    lidar_topic = RTC_TOPIC.get("ULIDAR_ARRAY")
                    if lidar_topic:
                        conn.datachannel.pub_sub.subscribe(
                            lidar_topic,
                            self._on_lidar,
                        )
                    raw_lidar_topic = RTC_TOPIC.get("ULIDAR")
                    if (
                        self._settings.unitree_lidar_allow_uncompressed
                        and raw_lidar_topic
                        and raw_lidar_topic != lidar_topic
                    ):
                        conn.datachannel.pub_sub.subscribe(
                            raw_lidar_topic,
                            self._on_uncompressed_lidar,
                        )
                    lidar_state_topic = RTC_TOPIC.get("ULIDAR_STATE")
                    if lidar_state_topic:
                        conn.datachannel.pub_sub.subscribe(
                            lidar_state_topic,
                            self._on_lidar_state,
                        )
                    odom_topic = RTC_TOPIC.get("ROBOTODOM")
                    if odom_topic:
                        conn.datachannel.pub_sub.subscribe(
                            odom_topic,
                            self._on_robot_odom,
                        )

                    self._connected = True
                    self._media_relays_started = bool(
                        conn_info.get("media_relays_started")
                    )
                    ready_event.set()

                    # Fire on-connect callbacks (gateway re-attaches media)
                    for cb in self._on_connect_callbacks:
                        try:
                            cb(conn)
                        except Exception as exc:
                            logger.warning("on-connect callback error: %s", exc)

                    # Keepalive loop — check for shutdown flag
                    while self._connected and not _shutdown_flag.is_set():
                        await asyncio.sleep(1)

                    # Shutdown requested: disconnect cleanly to release Go2 slot.
                    if self._connected:
                        logger.info("Shutdown flag set — disconnecting from Go2...")
                        await self._do_disconnect_on_bg_loop()
                except BaseException as exc:
                    connection_error.append(exc)
                    ready_event.set()

            loop.create_task(async_connect())
            try:
                loop.run_forever()
            finally:
                pending = asyncio.all_tasks(loop)
                for task in pending:
                    task.cancel()
                if pending:
                    loop.run_until_complete(
                        asyncio.gather(*pending, return_exceptions=True)
                    )
                loop.close()

        self._bg_thread = threading.Thread(target=run_bg_loop, daemon=True)
        self._bg_thread.start()

        timeout = self._settings.unitree_webrtc_connect_timeout
        if not ready_event.wait(timeout=timeout):
            self._connected = False
            raise SlotOccupiedError(
                f"WebRTC connection timed out after {timeout:.0f}s ({target}). "
                f"Check: robot powered on, connection settings/serial, and no other "
                f"WebRTC client (Unitree app)."
            )

        if connection_error:
            self._connected = False
            err = connection_error[0]
            raise _classify_connect_error(
                err,
                aes_key_set=bool(_aes_key_from_env()),
                remote=_resolve_connection_mode(self._settings) == "remote",
            ) from err

    async def _cleanup_failed_attempt(self) -> None:
        """Stop the background loop from a failed connect attempt.

        When _connect_once fails, its background thread keeps running
        loop.run_forever().  If we retry without cleaning up, Go2 sees
        two overlapping WebRTC handshakes from the same IP and rejects both.

        This method is safe to call even if there's no active background
        loop (e.g. first attempt, or already cleaned up).
        """
        # 1. Disconnect any stale connection (best-effort, on the bg loop).
        conn = self._conn
        self._conn = None
        loop = self._bg_loop
        if conn is not None and loop is not None and loop.is_running():
            try:
                if hasattr(conn, "disconnect"):
                    fut = asyncio.run_coroutine_threadsafe(
                        conn.disconnect(), loop
                    )
                    try:
                        fut.result(timeout=3.0)
                    except Exception:
                        pass
            except Exception:
                pass

        # 2. Stop the event loop.
        self._bg_loop = None
        if loop is not None:
            try:
                if loop.is_running():
                    loop.call_soon_threadsafe(loop.stop)
            except Exception:
                pass

        # 3. Wait for the background thread to finish.
        thread = self._bg_thread
        self._bg_thread = None
        if thread is not None and thread.is_alive():
            thread.join(timeout=5.0)
            if thread.is_alive():
                logger.warning("Background thread did not stop within 5s")

        # Let Go2 firmware release the slot before the next handshake.
        await asyncio.sleep(1.0)

    async def _do_disconnect_on_bg_loop(self) -> None:
        """Disconnect from Go2 on the background loop (called from bg loop).

        Idempotent — safe to call even if disconnect() already ran from the
        main thread (e.g. during shutdown both atexit and finally fire).
        """
        conn = self._conn
        self._conn = None  # prevent double-disconnect
        if conn is not None and hasattr(conn, "disconnect"):
            try:
                await conn.disconnect()
                logger.info("Go2 WebRTC disconnected cleanly (slot released)")
            except Exception as exc:
                logger.warning("WebRTC disconnect error: %s", exc)

    async def disconnect(self) -> None:
        """External disconnect — stop motion, release Go2 slot.

        Idempotent: safe to call multiple times (concurrent bg-loop and main-thread
        shutdown paths both trigger this during process exit).
        """
        self._transport_shutdown = True
        self._media_relays_started = False
        self._set_conn_state(ConnectionState.DISCONNECTING)

        # A dry-run/read-only session must not publish zero joystick frames or
        # StopMove during teardown. Live sessions retain the defensive halt.
        if self._connected and (
            not self._settings.unitree_dry_run or not self._drive_idle.is_set()
        ):
            try:
                await self._halt_motion(MotionEndReason.DISCONNECT, send_stopmove=True)
            except Exception as exc:
                logger.warning("Pre-disconnect halt failed: %s", exc)

        self._connected = False

        conn = self._conn
        self._conn = None  # prevent double-disconnect from bg loop
        if conn is not None and hasattr(conn, "disconnect"):
            try:
                if self._bg_loop is not None and self._bg_loop.is_running():
                    future = asyncio.run_coroutine_threadsafe(
                        conn.disconnect(), self._bg_loop
                    )
                    future.result(timeout=5.0)
            except Exception as exc:
                logger.warning("WebRTC disconnect error: %s", exc)
        self._last_sport_state = None
        self._last_low_state = None
        self._last_lidar = None
        self._last_lidar_sensor_stamp = None
        self._last_robot_odom = None
        loop = self._bg_loop
        self._bg_loop = None
        if loop is not None and loop.is_running():
            loop.call_soon_threadsafe(loop.stop)
        thread = self._bg_thread
        self._bg_thread = None
        if thread is not None and thread.is_alive():
            await asyncio.to_thread(thread.join, 3.0)
            if thread.is_alive():
                logger.warning("WebRTC background thread did not stop within 3s")
        self._set_conn_state(ConnectionState.DISCONNECTED)
        logger.info("UnitreeWebRTCTransport disconnected")

    def _set_conn_state(self, state: ConnectionState) -> None:
        with self._conn_state_cond:
            self._conn_state = state
            self._conn_state_cond.notify_all()

    # --------------------------------------------------------------- state callbacks
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
            self._health.sport_state_count += 1

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
            self._health.low_state_count += 1

    def _on_lidar(self, msg: Any) -> None:
        """Capture a decoded Go2 built-in LiDAR frame for read-only consumers."""
        with self._state_lock:
            self._health.lidar_compressed_message_count += 1
        self._decode_lidar(msg)

    def _on_uncompressed_lidar(self, msg: Any) -> None:
        """Capture firmware variants that publish the non-compressed topic."""
        with self._state_lock:
            self._health.lidar_uncompressed_message_count += 1
        self._decode_lidar(msg)

    def _on_lidar_state(self, msg: Any) -> None:
        """Record LiDAR service state independently from point-cloud delivery."""
        with self._state_lock:
            self._health.lidar_state_count += 1
            self._health.last_lidar_state = msg

    def _decode_lidar(self, msg: Any) -> None:
        """Decode a frame received from either built-in LiDAR topic."""
        from robot_brain.perception.pointcloud import pointcloud_from_unitree_webrtc

        received = time.monotonic()
        snapshot = pointcloud_from_unitree_webrtc(msg, received_monotonic=received)
        if snapshot is None:
            return
        with self._state_lock:
            stamp = snapshot.sensor_timestamp
            if (
                stamp is not None
                and self._last_lidar_sensor_stamp is not None
                and stamp <= self._last_lidar_sensor_stamp
            ):
                snapshot = replace(
                    snapshot,
                    sensor_timestamp=None,
                    timestamp_valid=False,
                )
                self._health.lidar_timestamp_repair_count += 1
            elif stamp is not None:
                self._last_lidar_sensor_stamp = stamp
            self._last_lidar = snapshot
            self._health.last_lidar_update = received
            self._health.lidar_frame_count += 1

    def _on_robot_odom(self, msg: Any) -> None:
        parsed = _parse_robot_odom(msg)
        if parsed is None:
            return
        with self._state_lock:
            self._last_robot_odom = parsed
            self._health.last_odom_update = time.monotonic()
            self._health.odom_frame_count += 1

    # --------------------------------------------------------------- state reading
    async def read_state(self) -> UnitreeState:
        if not self._connected:
            raise ConnectionError("UnitreeWebRTCTransport not connected")

        with self._state_lock:
            sport = self._last_sport_state
            low = self._last_low_state
            odom = self._last_robot_odom

        if sport is None:
            # State may take a moment to arrive after subscription
            for attempt in range(3):
                wait_sec = 2.0 * (attempt + 1)
                logger.info("Waiting for sport state (attempt %d, %.0fs)...", attempt + 1, wait_sec)
                await asyncio.sleep(wait_sec)
                with self._state_lock:
                    sport = self._last_sport_state
                    low = self._last_low_state
                    odom = self._last_robot_odom
                if sport is not None:
                    break
            if sport is None:
                raise ConnectionError(
                    "No state received from robot via WebRTC after 12s. "
                    "Check: robot powered on, same network, correct IP."
                )

        return self._map_state(sport, low, odom)

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

    # --------------------------------------------------------------- commands
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

    # --------------------------------------------------------------- sport publish
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
        self._last_sport_api = {
            "api_id": api_id,
            "status_code": code,
            "parameter": parameter,
        }
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

    # --------------------------------------------------------------- velocity helpers
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
        """Pick Move for pure translation/rotation; joystick only for motion arcs."""
        if not self._settings.unitree_webrtc_drive_via_move:
            return True
        # MCF Move(1008) handles vx, vy and pure yaw.  The virtual joystick is
        # only needed for simultaneous translation+yaw arcs; on Remote 4G its
        # short pure-yaw pulses can be acknowledged without sustained motion.
        if vyaw != 0.0 and (vx != 0.0 or vy != 0.0):
            return True
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
        """After FreeWalk: enable joystick/Move control and set gait SpeedLevel."""
        level = max(1, min(5, int(getattr(self._settings, "unitree_speed_level", 2))))
        await self._publish_sport(
            _SPORT_API_ID["switch_joystick"], parameter={"data": True}
        )
        await self._publish_sport(
            _SPORT_API_ID["speed_level"], parameter={"data": level}
        )
        logger.info(
            "WebRTC omni teleop enabled (SwitchJoystick + SpeedLevel=%s)", level
        )

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

    # --------------------------------------------------------------- motion control
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

    # --------------------------------------------------------------- drive (one-shot)
    async def _publish_drive(
        self, vx: float, vy: float, vyaw: float, duration: float
    ) -> None:
        """Acquire motion lease and stream joystick velocity, then auto-stop."""
        if self._conn is None:
            raise ConnectionError("No active WebRTC connection")

        gen = self._bump_motion_generation()
        await self._wait_drive_idle()

        self._drive_idle.clear()
        self._health.drive_streams_active += 1
        try:
            await self._run_on_conn_loop(
                self._velocity_stream(
                    gen=gen,
                    get_velocity=lambda: (vx, vy, vyaw),
                    deadline=None,
                    duration=duration,
                ),
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
            self._health.drive_streams_active -= 1

    # --------------------------------------------------------------- stream_hold (continuous)
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
        self._health.drive_streams_active += 1
        timeout = max(0.0, session_deadline - time.time()) + 15.0
        try:
            await self._run_on_conn_loop(
                self._velocity_stream(
                    gen=gen,
                    get_velocity=get_velocity,
                    deadline=session_deadline,
                    duration=None,
                ),
                timeout=timeout,
            )
            if gen == self._motion_gen:
                self._last_drive_end_reason = MotionEndReason.COMPLETED
        finally:
            self._drive_idle.set()
            self._health.drive_streams_active -= 1
            if zero_on_exit and self._conn is not None:
                await self._send_drive_zeros()

    # --------------------------------------------------------------- velocity stream (common core)
    async def _velocity_stream(
        self,
        *,
        gen: int,
        get_velocity: Callable[[], tuple[float, float, float]],
        deadline: float | None,
        duration: float | None,
    ) -> None:
        """Common core for both drive (one-shot) and hold (continuous) velocity streaming.

        - **deadline** (absolute monotonic loop time): continuous loop until exceeded.
        - **duration** (seconds from now): one-shot loop — ignored if deadline is set.

        Exactly one of deadline/duration must be provided.
        """
        pub_sub = self._conn.datachannel.pub_sub
        use_move = self._settings.unitree_webrtc_drive_via_move
        period = 1.0 / _JOY_STREAM_HZ
        watchdog = self._settings.unitree_control_watchdog_seconds
        zero_count = self._settings.unitree_zero_frame_count
        loop = asyncio.get_event_loop()

        # Resolve stop condition
        if deadline is not None and duration is not None:
            raise ValueError("_velocity_stream: set deadline or duration, not both")
        if deadline is None and duration is not None:
            deadline = loop.time() + max(0.0, duration)
        elif deadline is None:
            deadline = float("inf")

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

                vx, vy, vyaw = get_velocity()
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
            # Only send zeros for one-shot drives; continuous holds manage their own
            if duration is not None:
                await self._send_drive_zeros(
                    count=zero_count, pub_sub=pub_sub, period=period
                )

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

    # --------------------------------------------------------------- event-loop bridging
    async def run_on_conn_loop(self, coro: Any, *, timeout: float = 30.0) -> Any:
        """Schedule *coro* on the Go2 WebRTC background loop (gateway media bridge)."""
        return await self._run_on_conn_loop(coro, timeout=timeout)

    @property
    def webrtc_conn(self) -> Any:
        return self._conn

    async def _run_on_conn_loop(
        self, coro: Any, *, timeout: float
    ) -> Any:
        if self._bg_loop is not None and self._bg_loop.is_running():
            fut = asyncio.run_coroutine_threadsafe(coro, self._bg_loop)
            # wrap_future so Ctrl+C / task cancel can abort promptly; to_thread(fut.result)
            # only cancels the waiter and leaves the bg velocity stream running.
            afut = asyncio.wrap_future(fut)
            try:
                return await asyncio.wait_for(afut, timeout=timeout)
            except asyncio.CancelledError:
                fut.cancel()
                with self._motion_lock:
                    self._motion_gen += 1
                raise
            except TimeoutError:
                fut.cancel()
                with self._motion_lock:
                    self._motion_gen += 1
                self._health.bridge_call_timeouts += 1
                raise
        return await coro

    # --------------------------------------------------------------- state mapping
    def _map_state(
        self,
        sport: dict[str, Any],
        low: dict[str, Any] | None,
        odom: dict[str, Any] | None = None,
    ) -> UnitreeState:
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
            pose_frame_id = "world"
            pose_timestamp = None
            pose_source = "sport_state"
            if odom is not None:
                position = odom["position"].model_copy(deep=True)
                heading = float(odom["heading_degrees"])
                pose_frame_id = str(odom["frame_id"])
                pose_timestamp = odom["timestamp"]
                pose_source = "unitree_robotodom"

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
            ultrasonic = None
            if low is not None:
                # power_v voltage → percentage (8S LiPo: 24V empty, 33.6V full)
                voltage = float(low.get("power_v", 0) or 0)
                if voltage > 0:
                    battery = max(0.0, min(100.0, (voltage - 24.0) / (33.6 - 24.0) * 100.0))
                ultrasonic = _extract_ultrasonic_from_dict(low)

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
                ultrasonic=ultrasonic,
                pose_frame_id=pose_frame_id,
                pose_timestamp=pose_timestamp,
                pose_source=pose_source,
            )
        except Exception as exc:
            logger.warning("WebRTC state mapping error: %s", exc)
            return UnitreeState(connected=True, error_code=-1)


def _aes_key_from_env() -> str | None:
    return os.environ.get("UNITREE_AES_128_KEY") or os.environ.get("UNITREE_AES_KEY") or None


def create_webrtc_transport(settings: Settings) -> UnitreeWebRTCTransport:
    """Factory function for the WebRTC transport."""
    return UnitreeWebRTCTransport(settings)
