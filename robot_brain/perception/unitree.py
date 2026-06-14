"""Unitree Go2 perception adapter — exposes robot self-state to WorldState."""
from __future__ import annotations

import logging
import math

from robot_brain.actuation.unitree import UnitreeRobot, UnitreeState
from robot_brain.core.robot_self_state import ImuRPY, RobotSelfState, Velocity
from robot_brain.perception.base import Observation, PerceptionAdapter

logger = logging.getLogger(__name__)


def _build_self_state(raw: UnitreeState, age: float) -> RobotSelfState:
    """Map a raw UnitreeState into a RobotSelfState (IMU radians → degrees)."""
    return RobotSelfState(
        source="unitree_go2",
        is_standing=raw.is_standing,
        is_moving=raw.is_moving,
        sport_mode=raw.sport_mode,
        error_code=raw.error_code,
        velocity=Velocity(
            vx=raw.velocity[0],
            vy=raw.velocity[1] if len(raw.velocity) >= 2 else 0.0,
            vz=raw.velocity[2] if len(raw.velocity) >= 3 else 0.0,
        ),
        imu_rpy=ImuRPY(
            roll_deg=math.degrees(raw.imu_rpy[0]),
            pitch_deg=math.degrees(raw.imu_rpy[1]) if len(raw.imu_rpy) >= 2 else 0.0,
            yaw_deg=math.degrees(raw.imu_rpy[2]) if len(raw.imu_rpy) >= 3 else 0.0,
        ),
        state_age_seconds=age,
    )


class UnitreePerceptionAdapter(PerceptionAdapter):
    """Observes the Go2 via its robot interface and transport layer.

    Produces an ``Observation`` whose ``self_state`` carries Go2-specific
    data (sport mode, error code, velocity, IMU orientation, state freshness).
    Generic fields (position, heading, battery) come from the standard
    ``RobotState`` mapping.
    """

    def __init__(self, robot: UnitreeRobot) -> None:
        self._robot = robot

    async def observe(self) -> Observation:
        robot_state = await self._robot.get_state()

        try:
            raw = await self._robot.transport.read_state()
            age = self._robot.transport.state_age_seconds()
        except Exception as exc:
            logger.warning("UnitreePerceptionAdapter: transport read failed, "
                           "self_state will be degraded: %s", exc)
            return Observation(
                position=robot_state.position.model_copy(deep=True),
                heading_degrees=robot_state.heading_degrees,
                battery_level=robot_state.battery_level,
                payload=robot_state.payload,
                self_state=RobotSelfState(source="unitree_go2"),
            )

        return Observation(
            position=robot_state.position.model_copy(deep=True),
            heading_degrees=robot_state.heading_degrees,
            battery_level=robot_state.battery_level,
            payload=robot_state.payload,
            self_state=_build_self_state(raw, age),
        )
