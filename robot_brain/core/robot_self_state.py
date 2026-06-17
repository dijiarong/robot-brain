"""Robot self-state model — shared by perception and world-state layers.

Defined in a separate module to avoid circular imports between
``perception.base`` and ``core.world_state``.
"""
from __future__ import annotations

from pydantic import BaseModel


class Velocity(BaseModel):
    """Body-frame velocity (m/s)."""
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0


class ImuRPY(BaseModel):
    """IMU orientation in degrees."""
    roll_deg: float = 0.0
    pitch_deg: float = 0.0
    yaw_deg: float = 0.0


class UltrasonicData(BaseModel):
    """Go2 ultrasonic distance readings in metres."""
    front_m: float | None = None
    rear_m: float | None = None
    left_m: float | None = None
    right_m: float | None = None


class RobotSelfState(BaseModel):
    """Robot-specific self-state reported by the perception adapter.

    Fields are optional by default — a non-Unitree backend leaves the
    entire ``self_state`` as ``None`` on ``Observation``.
    """
    source: str
    is_standing: bool | None = None
    is_moving: bool | None = None
    sport_mode: int | None = None
    error_code: int | None = None
    velocity: Velocity | None = None
    imu_rpy: ImuRPY | None = None
    state_age_seconds: float | None = None
    ultrasonic: UltrasonicData | None = None
