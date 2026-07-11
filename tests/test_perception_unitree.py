"""Tests for UnitreePerceptionAdapter and WorldState robot_self_state bridge."""
from __future__ import annotations

import unittest


from config.settings import Settings
from robot_brain.actuation.unitree import (
    FakeUnitreeTransport,
    UnitreeRobot,
    UnitreeState,
)
from robot_brain.core.robot_self_state import ImuRPY, RobotSelfState, Velocity
from robot_brain.core.world_state import Position, WorldState
from robot_brain.perception.base import Observation
from robot_brain.perception.mock import MockPerception
from robot_brain.perception.unitree import UnitreePerceptionAdapter, _build_self_state


def _make_unitree_robot(
    *,
    unitree_dry_run: bool = True,
    unitree_enable_motion: bool = False,
    **state_overrides,
) -> UnitreeRobot:
    """Create a UnitreeRobot backed by a connected FakeUnitreeTransport."""
    settings = Settings(
        robot_backend="unitree",
        unitree_transport="fake",
        unitree_dry_run=unitree_dry_run,
        unitree_enable_motion=unitree_enable_motion,
        memory_db_path=":memory:",
    )
    state = UnitreeState(
        connected=True,
        is_standing=True,
        battery_level=85.0,
        position=Position(x=1.0, y=2.0),
        heading_degrees=45.0,
        sport_mode=3,
        error_code=0,
        velocity=(0.0, 0.0, 0.0),
        imu_rpy=(0.1, -0.05, 0.8),
    )
    for key, value in state_overrides.items():
        setattr(state, key, value)
    transport = FakeUnitreeTransport(initial_state=state)
    return UnitreeRobot(transport, settings)


async def _connect(robot: UnitreeRobot) -> None:
    await robot.transport.connect()


# ---------------------------------------------------------------------------
# _build_self_state
# ---------------------------------------------------------------------------
class BuildSelfStateTests(unittest.TestCase):
    def test_maps_all_fields(self) -> None:
        raw = UnitreeState(
            is_standing=True,
            is_moving=False,
            sport_mode=3,
            error_code=0,
            velocity=(0.3, 0.1, 0.0),
            imu_rpy=(0.0, 0.0, 1.57),
        )
        ss = _build_self_state(raw, age=0.05)
        self.assertEqual("unitree_go2", ss.source)
        self.assertTrue(ss.is_standing)
        self.assertFalse(ss.is_moving)
        self.assertEqual(3, ss.sport_mode)
        self.assertEqual(0, ss.error_code)
        self.assertIsNotNone(ss.velocity)
        self.assertAlmostEqual(0.3, ss.velocity.vx)  # type: ignore[union-attr]
        self.assertAlmostEqual(0.1, ss.velocity.vy)  # type: ignore[union-attr]
        self.assertAlmostEqual(0.0, ss.velocity.vz)  # type: ignore[union-attr]
        self.assertIsNotNone(ss.imu_rpy)
        self.assertAlmostEqual(0.0, ss.imu_rpy.roll_deg)  # type: ignore[union-attr]
        self.assertAlmostEqual(0.0, ss.imu_rpy.pitch_deg)  # type: ignore[union-attr]
        self.assertAlmostEqual(89.954, ss.imu_rpy.yaw_deg, places=2)  # type: ignore[union-attr]
        self.assertEqual(0.05, ss.state_age_seconds)

    def test_default_state_does_not_crash(self) -> None:
        ss = _build_self_state(UnitreeState(), age=0.0)
        self.assertEqual("unitree_go2", ss.source)


# ---------------------------------------------------------------------------
# UnitreePerceptionAdapter
# ---------------------------------------------------------------------------
class UnitreePerceptionAdapterTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self) -> None:
        self.robot = _make_unitree_robot()
        await _connect(self.robot)
        self.adapter = UnitreePerceptionAdapter(self.robot)

    # -- observe ----------------------------------------------------------
    async def test_observe_produces_self_state(self) -> None:
        obs = await self.adapter.observe()
        self.assertIsNotNone(obs.self_state)
        self.assertEqual("unitree_go2", obs.self_state.source)  # type: ignore[union-attr]
        self.assertTrue(obs.self_state.is_standing)  # type: ignore[union-attr]
        self.assertFalse(obs.self_state.is_moving)  # type: ignore[union-attr]

    async def test_observe_carries_generic_fields(self) -> None:
        obs = await self.adapter.observe()
        self.assertIsNotNone(obs.position)
        self.assertEqual(1.0, obs.position.x)  # type: ignore[union-attr]
        self.assertEqual(2.0, obs.position.y)  # type: ignore[union-attr]
        self.assertEqual(45.0, obs.heading_degrees)
        self.assertEqual(85.0, obs.battery_level)

    async def test_error_code_passed_through(self) -> None:
        self.robot.transport._state.error_code = 7004
        obs = await self.adapter.observe()
        self.assertEqual(7004, obs.self_state.error_code)  # type: ignore[union-attr]

    async def test_sport_mode_passed_through(self) -> None:
        self.robot.transport._state.sport_mode = 1  # balanceStand
        obs = await self.adapter.observe()
        self.assertEqual(1, obs.self_state.sport_mode)  # type: ignore[union-attr]

    async def test_velocity_passed_through(self) -> None:
        self.robot.transport._state.velocity = (0.3, -0.15, 0.0)
        obs = await self.adapter.observe()
        self.assertIsNotNone(obs.self_state.velocity)  # type: ignore[union-attr]
        self.assertAlmostEqual(0.3, obs.self_state.velocity.vx)  # type: ignore[union-attr]
        self.assertAlmostEqual(-0.15, obs.self_state.velocity.vy)  # type: ignore[union-attr]

    async def test_imu_rpy_radians_to_degrees(self) -> None:
        self.robot.transport._state.imu_rpy = (0.1, -0.05, 1.57)
        obs = await self.adapter.observe()
        self.assertIsNotNone(obs.self_state.imu_rpy)  # type: ignore[union-attr]
        self.assertAlmostEqual(5.729, obs.self_state.imu_rpy.roll_deg, places=2)  # type: ignore[union-attr]
        self.assertAlmostEqual(-2.864, obs.self_state.imu_rpy.pitch_deg, places=2)  # type: ignore[union-attr]
        self.assertAlmostEqual(89.954, obs.self_state.imu_rpy.yaw_deg, places=2)  # type: ignore[union-attr]

    async def test_state_age_reported(self) -> None:
        obs = await self.adapter.observe()
        # Fake transport always returns 0.0
        self.assertEqual(0.0, obs.self_state.state_age_seconds)  # type: ignore[union-attr]

    async def test_low_battery_in_both_generic_and_self_state(self) -> None:
        self.robot.transport._state.battery_level = 15.0
        obs = await self.adapter.observe()
        self.assertEqual(15.0, obs.battery_level)
        self.assertIsNotNone(obs.self_state)
        self.assertTrue(obs.self_state.is_standing)  # type: ignore[union-attr]

    # -- degraded path ----------------------------------------------------
    async def test_transport_read_failure_degrades_gracefully(self) -> None:
        await self.robot.transport.disconnect()
        obs = await self.adapter.observe()
        self.assertIsNotNone(obs.self_state)
        self.assertEqual("unitree_go2", obs.self_state.source)  # type: ignore[union-attr]
        # Generic fields still populated via get_state (which also fails →
        # RobotState(stopped=True), so position/battery are defaults)
        self.assertIsNone(obs.self_state.is_standing)  # type: ignore[union-attr]


# ---------------------------------------------------------------------------
# MockPerception backward compat
# ---------------------------------------------------------------------------
class MockPerceptionCompatTests(unittest.IsolatedAsyncioTestCase):
    async def test_mock_observation_has_no_self_state(self) -> None:
        robot = _make_unitree_robot()
        await _connect(robot)
        mp = MockPerception(robot)
        obs = await mp.observe()
        self.assertIsNone(obs.self_state)


# ---------------------------------------------------------------------------
# WorldState + self_state
# ---------------------------------------------------------------------------
class WorldStateSelfStateTests(unittest.TestCase):
    def test_apply_observation_merges_self_state(self) -> None:
        ws = WorldState()
        ss = RobotSelfState(
            source="unitree_go2",
            is_standing=True,
            is_moving=False,
            sport_mode=3,
            error_code=0,
        )
        obs = Observation(battery_level=90.0, self_state=ss)
        ws.apply_observation(obs)
        self.assertIsNotNone(ws.robot_self_state)
        self.assertTrue(ws.robot_self_state.is_standing)  # type: ignore[union-attr]
        self.assertEqual(3, ws.robot_self_state.sport_mode)  # type: ignore[union-attr]

    def test_apply_mock_observation_preserves_existing_self_state(self) -> None:
        ws = WorldState()
        existing = RobotSelfState(source="unitree_go2", error_code=7004)
        ws.robot_self_state = existing
        # Apply a mock observation (no self_state)
        obs = Observation(battery_level=50.0, self_state=None)
        ws.apply_observation(obs)
        self.assertIsNotNone(ws.robot_self_state)
        self.assertEqual(7004, ws.robot_self_state.error_code)  # type: ignore[union-attr]
        self.assertEqual(50.0, ws.battery_level)  # generic field still updated

    def test_query_state_age_seconds(self) -> None:
        ws = WorldState()
        self.assertIsNone(ws.state_age_seconds)
        ws.robot_self_state = RobotSelfState(source="u", state_age_seconds=0.25)
        self.assertEqual(0.25, ws.state_age_seconds)

    def test_query_robot_error_code(self) -> None:
        ws = WorldState()
        self.assertIsNone(ws.robot_error_code)
        ws.robot_self_state = RobotSelfState(source="u", error_code=42)
        self.assertEqual(42, ws.robot_error_code)

    def test_apply_null_self_state_does_not_overwrite(self) -> None:
        """Null self_state on Observation leaves existing WorldState.robot_self_state intact."""
        ws = WorldState()
        ws.robot_self_state = RobotSelfState(source="u", is_standing=True)
        obs = Observation(self_state=None)
        ws.apply_observation(obs)
        self.assertIsNotNone(ws.robot_self_state)
        self.assertTrue(ws.robot_self_state.is_standing)  # type: ignore[union-attr]

    def test_world_snapshot_includes_robot_self_state(self) -> None:
        ws = WorldState()
        ws.robot_self_state = RobotSelfState(source="unitree_go2", sport_mode=3)
        snap = ws.snapshot()
        self.assertIn("robot_self_state", snap)
        self.assertEqual("unitree_go2", snap["robot_self_state"]["source"])
        self.assertEqual(3, snap["robot_self_state"]["sport_mode"])


# ---------------------------------------------------------------------------
# AgentRuntime factory integration
# ---------------------------------------------------------------------------
class RuntimePerceptionBridgeTests(unittest.IsolatedAsyncioTestCase):
    async def test_runtime_create_with_unitree_perception(self) -> None:
        from robot_brain.runtime.loop import AgentRuntime

        settings = Settings(
            robot_backend="unitree",
            perception_backend="unitree",
            unitree_dry_run=True,
            memory_db_path=":memory:",
        )
        runtime = AgentRuntime.create(settings=settings)
        self.assertIsInstance(runtime.context.perception, UnitreePerceptionAdapter)
        # World should be fresh
        self.assertIsNone(runtime.context.world.robot_self_state)

    async def test_refresh_world_populates_robot_self_state(self) -> None:
        from robot_brain.runtime.loop import AgentRuntime

        settings = Settings(
            robot_backend="unitree",
            perception_backend="unitree",
            unitree_dry_run=True,
            memory_db_path=":memory:",
        )
        runtime = AgentRuntime.create(settings=settings)
        await runtime.context.robot.transport.connect()
        await runtime.refresh_world(reason="test")
        ws = runtime.context.world
        self.assertIsNotNone(ws.robot_self_state)
        self.assertEqual("unitree_go2", ws.robot_self_state.source)  # type: ignore[union-attr]

    async def test_mock_perception_does_not_set_robot_self_state(self) -> None:
        from robot_brain.runtime.loop import AgentRuntime

        settings = Settings(
            robot_backend="unitree",
            perception_backend="mock",
            unitree_dry_run=True,
            memory_db_path=":memory:",
        )
        runtime = AgentRuntime.create(settings=settings)
        await runtime.context.robot.transport.connect()
        await runtime.refresh_world(reason="test")
        self.assertIsNone(runtime.context.world.robot_self_state)


# ---------------------------------------------------------------------------
# Model serialisation round-trip
# ---------------------------------------------------------------------------
class ModelRoundTripTests(unittest.TestCase):
    def test_velocity_round_trip(self) -> None:
        v = Velocity(vx=0.5, vy=-0.2, vz=0.0)
        reloaded = Velocity.model_validate(v.model_dump(mode="json"))
        self.assertEqual(v, reloaded)

    def test_imu_rpy_round_trip(self) -> None:
        imu = ImuRPY(roll_deg=5.0, pitch_deg=-2.0, yaw_deg=90.0)
        reloaded = ImuRPY.model_validate(imu.model_dump(mode="json"))
        self.assertEqual(imu, reloaded)

    def test_robot_self_state_round_trip(self) -> None:
        ss = RobotSelfState(
            source="unitree_go2",
            is_standing=True,
            is_moving=False,
            sport_mode=3,
            error_code=0,
            velocity=Velocity(vx=0.1),
            imu_rpy=ImuRPY(yaw_deg=45.0),
            state_age_seconds=0.05,
        )
        reloaded = RobotSelfState.model_validate(ss.model_dump(mode="json"))
        self.assertEqual(ss, reloaded)

    def test_robot_self_state_minimal_round_trip(self) -> None:
        ss = RobotSelfState(source="unitree_go2")
        reloaded = RobotSelfState.model_validate(ss.model_dump(mode="json"))
        self.assertEqual(ss, reloaded)


if __name__ == "__main__":
    unittest.main()
