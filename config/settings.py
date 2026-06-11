"""Global settings for selecting adapters and safety thresholds."""
from __future__ import annotations

import os
from dataclasses import dataclass, field


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
    require_confirmation_for: tuple[str, ...] = ("follow",)
    object_ttl_seconds: float = 30.0

    # Runtime.
    max_loop_iterations: int = 50
    default_task_max_attempts: int = 2
    enable_verbose_log: bool = field(default_factory=lambda: _env_bool("RDB_VERBOSE", True))

    # Local persistence.
    memory_db_path: str = field(default_factory=lambda: os.getenv("RDB_MEMORY_DB", "data/robot_brain.sqlite3"))

    # Used only by the optional OpenAI adapter.
    openai_model: str = field(default_factory=lambda: os.getenv("RDB_OPENAI_MODEL", "gpt-4o-mini"))

    # Unitree robot adapter.
    unitree_transport: str = field(default_factory=lambda: os.getenv("RDB_UNITREE_TRANSPORT", "fake"))
    unitree_model: str = field(default_factory=lambda: os.getenv("RDB_UNITREE_MODEL", "go2"))
    # CycloneDDS network interface name for SDK transport (e.g. "en0"), NOT robot IP.
    unitree_net_iface: str = field(default_factory=lambda: os.getenv("RDB_UNITREE_NET_IFACE", ""))
    # Robot IP for WebRTC LocalSTA (router/LAN or AP mode). Also reads UNITREE_ROBOT_IP.
    unitree_robot_ip: str = field(
        default_factory=lambda: os.getenv("RDB_UNITREE_ROBOT_IP")
        or os.getenv("UNITREE_ROBOT_IP", "")
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
        default_factory=lambda: float(os.getenv("RDB_UNITREE_MAX_SPEED", "0.5"))
    )
    unitree_max_step: float = field(
        default_factory=lambda: float(os.getenv("RDB_UNITREE_MAX_STEP", "2.0"))
    )
    # Velocity-teleop (joystick "drive") safety clamps.
    unitree_max_yaw_speed: float = field(
        default_factory=lambda: float(os.getenv("RDB_UNITREE_MAX_YAW_SPEED", "1.0"))
    )
    unitree_max_drive_duration: float = field(
        default_factory=lambda: float(os.getenv("RDB_UNITREE_MAX_DRIVE_DURATION", "2.0"))
    )


SETTINGS = Settings()
