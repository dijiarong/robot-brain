"""Global settings for selecting adapters and safety thresholds."""
from __future__ import annotations

import os
import re
from dataclasses import dataclass, field
from urllib.parse import urlparse


def _env_bool(key: str, default: bool) -> bool:
    val = os.getenv(key)
    if val is None:
        return default
    return val.strip().lower() in {"1", "true", "yes", "on"}


@dataclass
class Settings:
    # Adapter implementations: mock | openai | future robot SDK adapters.
    llm_backend: str = field(default_factory=lambda: os.getenv("RDB_LLM", "mock"))
    robot_backend: str = field(default_factory=lambda: os.getenv("RDB_ROBOT", "mock"))
    perception_backend: str = field(default_factory=lambda: os.getenv("RDB_PERCEPTION", "mock"))

    # Dual-system thresholds.
    low_battery_threshold: float = 25.0
    critical_battery_threshold: float = 10.0

    # Safety constraints.
    max_linear_speed: float = 1.5
    max_step_distance: float = 30.0
    require_confirmation_for: tuple[str, ...] = (
        "follow", "nudge", "scan", "retreat", "explore", "go2_local_nav",
    )
    object_ttl_seconds: float = 30.0
    obstacle_proximity_threshold: float = 0.3  # metres — ultrasonic proximity alert

    # Runtime.
    max_loop_iterations: int = 50
    default_task_max_attempts: int = 2
    enable_verbose_log: bool = field(default_factory=lambda: _env_bool("RDB_VERBOSE", True))
    max_conversation_context: int = 10  # max messages fetched from DB for LLM context

    # Local persistence.
    memory_db_path: str = field(default_factory=lambda: os.getenv("RDB_MEMORY_DB", "data/robot_brain.sqlite3"))

    # Used only by the optional OpenAI adapter.
    openai_model: str = field(default_factory=lambda: os.getenv("RDB_OPENAI_MODEL", "gpt-4o-mini"))

    # Unitree robot adapter.
    unitree_transport: str = field(default_factory=lambda: os.getenv("RDB_UNITREE_TRANSPORT", "fake"))
    unitree_model: str = field(default_factory=lambda: os.getenv("RDB_UNITREE_MODEL", "go2"))
    # CycloneDDS network interface name for SDK transport (e.g. "en0"), NOT robot IP.
    unitree_net_iface: str = field(default_factory=lambda: os.getenv("RDB_UNITREE_NET_IFACE", ""))
    # Robot IP for WebRTC LocalSTA (router/LAN or AP mode).
    # Also reads UNITREE_ROBOT_IP, DIMOS_ROBOT_IP, ROBOT_IP (DimOS convention).
    unitree_robot_ip: str = field(
        default_factory=lambda: (
            os.getenv("RDB_UNITREE_ROBOT_IP")
            or os.getenv("UNITREE_ROBOT_IP")
            or os.getenv("DIMOS_ROBOT_IP")
            or os.getenv("ROBOT_IP", "")
        )
    )
    # Optional Go2 serial for WebRTC multicast discovery when IP is unknown.
    unitree_serial: str = field(default_factory=lambda: os.getenv("RDB_UNITREE_SERIAL", ""))
    unitree_dry_run: bool = field(default_factory=lambda: _env_bool("RDB_UNITREE_DRY_RUN", True))
    # Hard gate for real motion on live transports (webrtc/sdk). Even with
    # dry_run disabled, posture/stop commands are only sent when this is true.
    unitree_enable_motion: bool = field(
        default_factory=lambda: _env_bool("RDB_UNITREE_ENABLE_MOTION", False)
    )
    # Motion controller mode selected via MOTION_SWITCHER on connect (webrtc).
    # For the standard Go2 the sport controller is named "mcf" (the wheeled
    # Go2-W uses "ai-w"); "normal"/"ai" are not valid names on this firmware and
    # a switch attempt is rejected with code 7004. The robot already boots into
    # "mcf", so by default we match it and skip any (invalid) switch.
    unitree_motion_mode: str = field(
        default_factory=lambda: os.getenv("RDB_UNITREE_MOTION_MODE", "mcf")
    )
    unitree_max_speed: float = field(
        default_factory=lambda: float(os.getenv("RDB_UNITREE_MAX_SPEED", "0.2"))
    )
    unitree_max_step: float = field(
        default_factory=lambda: float(os.getenv("RDB_UNITREE_MAX_STEP", "2.0"))
    )
    # Velocity-teleop (joystick "drive") safety clamps.
    unitree_max_yaw_speed: float = field(
        default_factory=lambda: float(os.getenv("RDB_UNITREE_MAX_YAW_SPEED", "0.3"))
    )
    unitree_max_drive_duration: float = field(
        default_factory=lambda: float(os.getenv("RDB_UNITREE_MAX_DRIVE_DURATION", "0.5"))
    )
    # Live control loop (iteration 9): watchdog, state freshness, zero-frame count.
    unitree_control_watchdog_seconds: float = field(
        default_factory=lambda: float(os.getenv("RDB_UNITREE_CONTROL_WATCHDOG_SECONDS", "0.25"))
    )
    unitree_zero_frame_count: int = field(
        default_factory=lambda: int(os.getenv("RDB_UNITREE_ZERO_FRAME_COUNT", "5"))
    )
    unitree_state_max_age_seconds: float = field(
        default_factory=lambda: float(os.getenv("RDB_UNITREE_STATE_MAX_AGE_SECONDS", "2.0"))
    )
    unitree_post_drive_stop_timeout: float = field(
        default_factory=lambda: float(os.getenv("RDB_UNITREE_POST_DRIVE_STOP_TIMEOUT", "3.0"))
    )
    unitree_webrtc_connect_timeout: float = field(
        default_factory=lambda: float(os.getenv("RDB_UNITREE_WEBRTC_CONNECT_TIMEOUT", "30.0"))
    )
    unitree_webrtc_connect_retries: int = field(
        default_factory=lambda: int(os.getenv("RDB_UNITREE_WEBRTC_CONNECT_RETRIES", "3"))
    )
    # MCF firmware: omni vx/vy/vyaw via sport Move(1008); joystick alone often only drives ly.
    unitree_webrtc_drive_via_move: bool = field(
        default_factory=lambda: os.getenv("RDB_UNITREE_WEBRTC_DRIVE_VIA_MOVE", "true").lower()
        in ("1", "true", "yes")
    )
    # Forward Go2 front camera to local RTP (topsun_robot_service UDP :5000).
    unitree_video_relay: bool = field(
        default_factory=lambda: os.getenv("RDB_UNITREE_VIDEO_RELAY", "true").lower()
        in ("1", "true", "yes")
    )
    unitree_video_relay_host: str = field(
        default_factory=lambda: os.getenv("RDB_UNITREE_VIDEO_RELAY_HOST", "127.0.0.1")
    )
    unitree_video_relay_port: int = field(
        default_factory=lambda: int(os.getenv("RDB_UNITREE_VIDEO_RELAY_PORT", "5000"))
    )
    # Go2 audio ↔ topsun_robot_service (Opus RTP :5005 out, :5010 in from browser mic).
    unitree_audio_relay: bool = field(
        default_factory=lambda: os.getenv("RDB_UNITREE_AUDIO_RELAY", "true").lower()
        in ("1", "true", "yes")
    )
    unitree_audio_relay_host: str = field(
        default_factory=lambda: os.getenv("RDB_UNITREE_AUDIO_RELAY_HOST", "127.0.0.1")
    )
    unitree_audio_relay_port: int = field(
        default_factory=lambda: int(os.getenv("RDB_UNITREE_AUDIO_RELAY_PORT", "5005"))
    )
    unitree_audio_ingress_host: str = field(
        default_factory=lambda: os.getenv("RDB_UNITREE_AUDIO_INGRESS_HOST", "127.0.0.1")
    )
    unitree_audio_ingress_port: int = field(
        default_factory=lambda: int(os.getenv("RDB_UNITREE_AUDIO_INGRESS_PORT", "5010"))
    )
    # Unified gateway: browser WebRTC + Go2 WebRTC in one process (no UDP/ffmpeg relay).
    unitree_gateway: bool = field(
        default_factory=lambda: _env_bool("RDB_UNITREE_GATEWAY", False)
    )
    gateway_signaling_url: str = field(
        default_factory=lambda: os.getenv("RDB_GATEWAY_SIGNALING_URL", "ws://127.0.0.1:9999/ws")
    )
    gateway_robot_id: str = field(
        default_factory=lambda: os.getenv("RDB_GATEWAY_ROBOT_ID", "robot_001")
    )
    gateway_turn_url: str = field(
        default_factory=lambda: os.getenv("RDB_GATEWAY_TURN_URL", "")
    )
    gateway_turn_user: str = field(
        default_factory=lambda: os.getenv("RDB_GATEWAY_TURN_USER", "")
    )
    gateway_turn_pass: str = field(
        default_factory=lambda: os.getenv("RDB_GATEWAY_TURN_PASS", "")
    )
    # Trust self-signed TLS on wss:// signaling (cloud dev cert). Disable in production.
    gateway_signaling_insecure_ssl: bool = field(
        default_factory=lambda: _env_bool("RDB_GATEWAY_SIGNALING_INSECURE_SSL", True)
    )

    # Bounded Explore — composite skill limits.
    explore_max_steps: int = field(
        default_factory=lambda: int(os.getenv("RDB_EXPLORE_MAX_STEPS", "5"))
    )
    explore_max_duration: float = field(
        default_factory=lambda: float(os.getenv("RDB_EXPLORE_MAX_DURATION", "120"))
    )
    explore_step_cm: float = field(
        default_factory=lambda: float(os.getenv("RDB_EXPLORE_STEP_CM", "20"))
    )
    explore_scan_deg: float = field(
        default_factory=lambda: float(os.getenv("RDB_EXPLORE_SCAN_DEG", "45"))
    )
    # Iteration 18: explore stop protection (no SLAM; behavior-trace based).
    # Consecutive steps without a successful forward nudge -> no_progress stop.
    explore_no_progress_steps: int = field(
        default_factory=lambda: int(os.getenv("RDB_EXPLORE_NO_PROGRESS_STEPS", "3"))
    )
    # Alternating scan_alt_left/right count that triggers a ping-pong stop.
    explore_ping_pong_steps: int = field(
        default_factory=lambda: int(os.getenv("RDB_EXPLORE_PING_PONG_STEPS", "4"))
    )
    # Consecutive vlm_hold (VLM 'stop') steps that trigger a semantic_hold stop.
    explore_max_holds: int = field(
        default_factory=lambda: int(os.getenv("RDB_EXPLORE_MAX_HOLDS", "2"))
    )
    # Iteration 19: odometry-backed local progress thresholds.
    odom_progress_min_m: float = field(
        default_factory=lambda: float(os.getenv("RDB_ODOM_PROGRESS_MIN_M", "0.03"))
    )
    odom_progress_min_yaw_deg: float = field(
        default_factory=lambda: float(os.getenv("RDB_ODOM_PROGRESS_MIN_YAW_DEG", "3.0"))
    )
    odom_max_age_seconds: float = field(
        default_factory=lambda: float(os.getenv("RDB_ODOM_MAX_AGE_SECONDS", "1.0"))
    )

    # Go2 FastReflex — consecutive non-zero error_code reads before triggering stop.
    go2_reflex_error_debounce: int = field(
        default_factory=lambda: int(os.getenv("RDB_GO2_REFLEX_ERROR_DEBOUNCE", "1"))
    )

    # Local VLM passability hint (iteration 17). Default off - does not affect
    # mock/CI. When enabled, ExploreSkill asks a LAN Qwen3-VL for a soft
    # direction suggestion; ultrasonic remains the hard safety gate.
    vlm_enabled: bool = field(default_factory=lambda: _env_bool("RDB_VLM_ENABLED", False))
    vlm_base_url: str = field(
        default_factory=lambda: os.getenv("RDB_VLM_BASE_URL", "http://10.10.197.175:8080")
    )
    vlm_model: str = field(
        default_factory=lambda: os.getenv("RDB_VLM_MODEL", "/Users/dijia/models/Qwen3-VL-8B-4bit")
    )
    vlm_api_key: str = field(default_factory=lambda: os.getenv("RDB_VLM_API_KEY", "vlm"))
    vlm_max_edge: int = field(default_factory=lambda: int(os.getenv("RDB_VLM_MAX_EDGE", "768")))
    vlm_timeout: float = field(default_factory=lambda: float(os.getenv("RDB_VLM_TIMEOUT", "30")))
    vlm_min_interval: float = field(
        default_factory=lambda: float(os.getenv("RDB_VLM_MIN_INTERVAL", "2.0"))
    )
    vlm_confidence_min: float = field(
        default_factory=lambda: float(os.getenv("RDB_VLM_CONFIDENCE_MIN", "0.5"))
    )
    # Optional still-image path for mock/file VLM smoke tests (no live camera).
    vlm_frame_path: str = field(default_factory=lambda: os.getenv("RDB_VLM_FRAME_PATH", ""))
    # Iteration 18: explicit frame-source selection and video-consumer priority.
    # vlm_frame_source: auto (frame_path -> go2_tap on unitree+webrtc -> null) |
    # file | go2_tap | none.
    vlm_frame_source: str = field(
        default_factory=lambda: os.getenv("RDB_VLM_FRAME_SOURCE", "auto")
    )
    # When both the RTP relay and the VLM tap could read the same Go2 video
    # track, which consumer wins. vlm = VLM tap (relay may starve);
    # relay = keep RTP relay (no VLM tap); manual = caller wires the tap.
    vlm_video_priority: str = field(
        default_factory=lambda: os.getenv("RDB_VLM_VIDEO_PRIORITY", "vlm")
    )

    # Teleop session (remote velocity control ingress).
    # Deadman: if no setpoint arrives within this window, drive auto-stops.
    teleop_deadman_ms: int = field(
        default_factory=lambda: int(os.getenv("RDB_TELEOP_DEADMAN_MS", "300"))
    )
    # Lease TTL: a held control lease expires this long after the last renewal
    # (each accepted setpoint renews it).
    teleop_lease_ttl_ms: int = field(
        default_factory=lambda: int(os.getenv("RDB_TELEOP_LEASE_TTL_MS", "2000"))
    )
    # Max seconds per internal stream_hold chunk before the loop re-checks
    # lease/deadman state.
    teleop_chunk_seconds: float = field(
        default_factory=lambda: float(os.getenv("RDB_TELEOP_CHUNK_SECONDS", "1.0"))
    )

    def __post_init__(self) -> None:
        self._apply_gateway_signaling_defaults()
        self._apply_gateway_turn_defaults()

    def _apply_gateway_signaling_defaults(self) -> None:
        """Remote cloud signaling now uses WSS; upgrade legacy ws:// URLs."""
        url = self.gateway_signaling_url
        parsed = urlparse(url)
        host = parsed.hostname or ""
        if not host or host in ("127.0.0.1", "localhost"):
            return
        if parsed.scheme == "ws":
            upgraded = url.replace("ws://", "wss://", 1)
            object.__setattr__(self, "gateway_signaling_url", upgraded)

    def _apply_gateway_turn_defaults(self) -> None:
        """Match cloud test page TURN when signaling points at a remote host."""
        if self.gateway_turn_url:
            return
        host = urlparse(self.gateway_signaling_url).hostname or ""
        if not host or host in ("127.0.0.1", "localhost"):
            return
        if not re.match(r"^\d+\.\d+\.\d+\.\d+$", host) and "." not in host:
            return
        object.__setattr__(self, "gateway_turn_url", f"turn:{host}:3478")
        if not self.gateway_turn_user:
            object.__setattr__(self, "gateway_turn_user", "test")
        if not self.gateway_turn_pass:
            object.__setattr__(self, "gateway_turn_pass", "123456")


SETTINGS = Settings()
