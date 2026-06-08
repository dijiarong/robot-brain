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
    unitree_model: str = field(default_factory=lambda: os.getenv("RDB_UNITREE_MODEL", ""))
    unitree_net_iface: str = field(default_factory=lambda: os.getenv("RDB_UNITREE_NET_IFACE", ""))
    unitree_dry_run: bool = field(default_factory=lambda: _env_bool("RDB_UNITREE_DRY_RUN", True))
    unitree_max_speed: float = field(
        default_factory=lambda: float(os.getenv("RDB_UNITREE_MAX_SPEED", "0.5"))
    )
    unitree_max_step: float = field(
        default_factory=lambda: float(os.getenv("RDB_UNITREE_MAX_STEP", "2.0"))
    )


SETTINGS = Settings()
