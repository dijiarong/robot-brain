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

    # External navigation provider. auto = Fake on mock, disabled on real robot;
    # nav2 = connect to the topsun-bot/Navigation ROS2 graph lazily;
    # direct_go2 = straight bounded local goals; native_go2 = robot-brain-owned
    # costmap + A* + replanning using built-in odom and LiDAR.
    navigation_backend: str = field(
        default_factory=lambda: os.getenv("RDB_NAVIGATION_BACKEND", "auto")
    )
    nav2_action_name: str = field(
        default_factory=lambda: os.getenv("RDB_NAV2_ACTION_NAME", "/navigate_to_pose")
    )
    nav2_odom_topic: str = field(
        default_factory=lambda: os.getenv("RDB_NAV2_ODOM_TOPIC", "/odom")
    )
    nav2_goal_frame: str = field(
        default_factory=lambda: os.getenv("RDB_NAV2_GOAL_FRAME", "odom")
    )
    nav2_map_frame: str = field(
        default_factory=lambda: os.getenv("RDB_NAV2_MAP_FRAME", "map")
    )
    nav2_base_frame: str = field(
        default_factory=lambda: os.getenv("RDB_NAV2_BASE_FRAME", "base_link")
    )
    nav2_map_id: str = field(
        default_factory=lambda: os.getenv("RDB_NAV2_MAP_ID", "")
    )
    nav2_map_version: str | None = field(
        default_factory=lambda: os.getenv("RDB_NAV2_MAP_VERSION") or None
    )
    nav2_server_timeout_s: float = field(
        default_factory=lambda: float(os.getenv("RDB_NAV2_SERVER_TIMEOUT_S", "2.0"))
    )
    nav2_pose_timeout_s: float = field(
        default_factory=lambda: float(os.getenv("RDB_NAV2_POSE_TIMEOUT_S", "3.0"))
    )
    nav2_cancel_timeout_s: float = field(
        default_factory=lambda: float(os.getenv("RDB_NAV2_CANCEL_TIMEOUT_S", "2.0"))
    )
    direct_nav_pointcloud_max_age_s: float = field(
        default_factory=lambda: float(os.getenv("RDB_DIRECT_NAV_POINTCLOUD_MAX_AGE_S", "0.5"))
    )
    direct_nav_segment_duration_s: float = field(
        default_factory=lambda: float(os.getenv("RDB_DIRECT_NAV_SEGMENT_DURATION_S", "0.25"))
    )
    direct_nav_obstacle_stop_m: float = field(
        default_factory=lambda: float(os.getenv("RDB_DIRECT_NAV_OBSTACLE_STOP_M", "0.45"))
    )
    direct_nav_obstacle_half_width_m: float = field(
        default_factory=lambda: float(os.getenv("RDB_DIRECT_NAV_OBSTACLE_HALF_WIDTH_M", "0.28"))
    )
    direct_nav_no_progress_segments: int = field(
        default_factory=lambda: int(os.getenv("RDB_DIRECT_NAV_NO_PROGRESS_SEGMENTS", "4"))
    )
    direct_nav_odom_settle_s: float = field(
        default_factory=lambda: float(os.getenv("RDB_DIRECT_NAV_ODOM_SETTLE_S", "0.35"))
    )
    direct_nav_reach_tolerance_m: float = field(
        default_factory=lambda: float(os.getenv("RDB_DIRECT_NAV_REACH_TOLERANCE_M", "0.015"))
    )
    direct_nav_reach_tolerance_yaw_deg: float = field(
        default_factory=lambda: float(
            os.getenv("RDB_DIRECT_NAV_REACH_TOLERANCE_YAW_DEG", "5.0")
        )
    )
    direct_nav_require_robotodom: bool = field(
        default_factory=lambda: _env_bool("RDB_DIRECT_NAV_REQUIRE_ROBOTODOM", True)
    )
    native_nav_map_size_m: float = field(
        default_factory=lambda: float(os.getenv("RDB_NATIVE_NAV_MAP_SIZE_M", "7.0"))
    )
    native_nav_resolution_m: float = field(
        default_factory=lambda: float(os.getenv("RDB_NATIVE_NAV_RESOLUTION_M", "0.10"))
    )
    native_nav_robot_radius_m: float = field(
        default_factory=lambda: float(os.getenv("RDB_NATIVE_NAV_ROBOT_RADIUS_M", "0.30"))
    )
    native_nav_emergency_stop_m: float = field(
        default_factory=lambda: float(os.getenv("RDB_NATIVE_NAV_EMERGENCY_STOP_M", "0.25"))
    )
    native_nav_max_no_path_replans: int = field(
        # Remote WebRTC LiDAR can need several fresh frames after the posture
        # sequence before a narrow detour is consistently visible.  Retries
        # remain bounded and every no-path cycle commands stop.
        default_factory=lambda: int(os.getenv("RDB_NATIVE_NAV_MAX_NO_PATH_REPLANS", "10"))
    )
    native_nav_trace_path: str = field(
        default_factory=lambda: os.getenv("RDB_NATIVE_NAV_TRACE_PATH", "")
    )
    native_nav_replay_path: str = field(
        default_factory=lambda: os.getenv("RDB_NATIVE_NAV_REPLAY_PATH", "")
    )
    native_nav_map_path: str = field(
        default_factory=lambda: os.getenv("RDB_NATIVE_NAV_MAP_PATH", "")
    )
    native_nav_min_replan_interval_s: float = field(
        default_factory=lambda: float(os.getenv("RDB_NATIVE_NAV_MIN_REPLAN_INTERVAL_S", "0.10"))
    )
    native_nav_pose_graph_enabled: bool = field(
        default_factory=lambda: _env_bool("RDB_NATIVE_NAV_POSE_GRAPH_ENABLED", False)
    )
    native_nav_max_acceleration_mps2: float = field(
        default_factory=lambda: float(os.getenv("RDB_NATIVE_NAV_MAX_ACCELERATION_MPS2", "1.0"))
    )

    # Local persistence.
    memory_db_path: str = field(default_factory=lambda: os.getenv("RDB_MEMORY_DB", "data/robot_brain.sqlite3"))

    # Used only by the optional OpenAI adapter.
    openai_model: str = field(default_factory=lambda: os.getenv("RDB_OPENAI_MODEL", "gpt-4o-mini"))

    # Unitree robot adapter.
    unitree_transport: str = field(default_factory=lambda: os.getenv("RDB_UNITREE_TRANSPORT", "fake"))
    unitree_model: str = field(default_factory=lambda: os.getenv("RDB_UNITREE_MODEL", "go2"))
    # CycloneDDS network interface name for SDK transport (e.g. "en0"), NOT robot IP.
    unitree_net_iface: str = field(default_factory=lambda: os.getenv("RDB_UNITREE_NET_IFACE", ""))
    # WebRTC route: auto prefers an explicit LAN IP, otherwise uses Unitree
    # cloud when serial + account credentials are configured.
    unitree_webrtc_connection_mode: str = field(
        default_factory=lambda: os.getenv("RDB_UNITREE_WEBRTC_CONNECTION_MODE", "auto")
    )
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
    # Go2 serial: optional for LAN discovery, required for cloud Remote mode.
    unitree_serial: str = field(default_factory=lambda: os.getenv("RDB_UNITREE_SERIAL", ""))
    # Unitree cloud login for WebRTC Remote mode.  Keep the password in the
    # process environment; repr=False prevents accidental Settings dumps.
    unitree_cloud_username: str = field(
        default_factory=lambda: os.getenv("RDB_UNITREE_CLOUD_USERNAME", "")
    )
    unitree_cloud_password: str = field(
        default_factory=lambda: os.getenv("RDB_UNITREE_CLOUD_PASSWORD", ""),
        repr=False,
    )
    unitree_cloud_region: str = field(
        default_factory=lambda: os.getenv("RDB_UNITREE_CLOUD_REGION", "global")
    )
    unitree_cloud_device_type: str = field(
        default_factory=lambda: os.getenv("RDB_UNITREE_CLOUD_DEVICE_TYPE", "Go2")
    )
    # Request the built-in Go2 LiDAR stream. direct_go2 navigation enables it
    # automatically; this flag also allows sensor-only diagnostics.
    unitree_lidar_stream: bool = field(
        default_factory=lambda: _env_bool("RDB_UNITREE_LIDAR_STREAM", False)
    )
    # Raw voxel_map can be expensive over 4G. Keep it opt-in and use only to
    # diagnose firmware that does not publish voxel_map_compressed.
    unitree_lidar_allow_uncompressed: bool = field(
        default_factory=lambda: _env_bool(
            "RDB_UNITREE_LIDAR_ALLOW_UNCOMPRESSED", False
        )
    )
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
    # On service start (live webrtc/sdk transports only), run the wake sequence
    # stand_up → balance_stand → free_walk → SwitchJoystick so the Go2 stands up
    # even when it booted lying down. Only effective with motion enabled.
    unitree_auto_stand: bool = field(
        default_factory=lambda: _env_bool("RDB_UNITREE_AUTO_STAND", True)
    )
    unitree_max_speed: float = field(
        default_factory=lambda: float(os.getenv("RDB_UNITREE_MAX_SPEED", "0.35"))
    )
    unitree_max_step: float = field(
        default_factory=lambda: float(os.getenv("RDB_UNITREE_MAX_STEP", "2.0"))
    )
    # Velocity-teleop (joystick "drive") safety clamps.
    unitree_max_yaw_speed: float = field(
        default_factory=lambda: float(os.getenv("RDB_UNITREE_MAX_YAW_SPEED", "0.8"))
    )
    # Go2 Sport SpeedLevel after FreeWalk (1=slow … higher=faster). Dashboard WASD uses 2.
    unitree_speed_level: int = field(
        default_factory=lambda: max(1, min(5, int(os.getenv("RDB_UNITREE_SPEED_LEVEL", "2"))))
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
    # Defer ffmpeg video/audio relays until ensure_media_relays() / API click.
    # Prefer true when co-located with Mid-360 Nav2 or when brain is lean.
    unitree_media_on_demand: bool = field(
        default_factory=lambda: _env_bool("RDB_UNITREE_MEDIA_ON_DEMAND", False)
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
