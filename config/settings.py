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

    # Runtime.
    max_loop_iterations: int = 50
    enable_verbose_log: bool = field(default_factory=lambda: _env_bool("RDB_VERBOSE", True))

    # Used only by the optional OpenAI adapter.
    openai_model: str = field(default_factory=lambda: os.getenv("RDB_OPENAI_MODEL", "gpt-4o-mini"))


SETTINGS = Settings()
