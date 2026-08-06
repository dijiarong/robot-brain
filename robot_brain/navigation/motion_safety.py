"""Runtime safety signals and bounded velocity ramping for native navigation."""
from __future__ import annotations

from dataclasses import dataclass
import math
import time


@dataclass(frozen=True)
class NavigationMotionSafetySignal:
    speed_scale: float = 1.0
    stop_requested: bool = False
    reason: str = "external_safety"
    observed_monotonic: float = 0.0

    def __post_init__(self) -> None:
        if not math.isfinite(self.speed_scale) or not 0 <= self.speed_scale <= 1:
            raise ValueError("safety speed scale must be in [0, 1]")
        if not math.isfinite(self.observed_monotonic) or self.observed_monotonic < 0:
            raise ValueError("invalid safety signal timestamp")
        if not self.reason:
            raise ValueError("safety signal reason is required")

    @classmethod
    def now(cls, *, speed_scale: float = 1.0, stop_requested: bool = False,
            reason: str = "external_safety"):
        return cls(speed_scale, stop_requested, reason, time.monotonic())


class LinearVelocityRampLimiter:
    def __init__(self, max_acceleration_mps2: float) -> None:
        if not math.isfinite(max_acceleration_mps2) or max_acceleration_mps2 <= 0:
            raise ValueError("maximum acceleration must be positive and finite")
        self.max_acceleration_mps2 = max_acceleration_mps2
        self._vx = self._vy = 0.0

    def step(self, target_vx: float, target_vy: float, duration_s: float,
             *, speed_scale: float = 1.0) -> tuple[float, float]:
        if not all(math.isfinite(value) for value in (target_vx, target_vy, duration_s, speed_scale)):
            raise ValueError("velocity ramp values must be finite")
        if duration_s <= 0 or not 0 <= speed_scale <= 1:
            raise ValueError("invalid velocity ramp duration or scale")
        target_vx *= speed_scale
        target_vy *= speed_scale
        dx, dy = target_vx-self._vx, target_vy-self._vy
        delta = math.hypot(dx, dy)
        maximum = self.max_acceleration_mps2*duration_s
        if delta > maximum:
            ratio = maximum/delta
            dx, dy = dx*ratio, dy*ratio
        self._vx += dx
        self._vy += dy
        return self._vx, self._vy

    def reset(self) -> None:
        self._vx = self._vy = 0.0
