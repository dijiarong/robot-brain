"""Process-wide motion authority shares one TeleopSession across ingresses."""
from __future__ import annotations

import unittest

from config.settings import Settings
from robot_brain.actuation.unitree import FakeUnitreeTransport, UnitreeRobot, UnitreeState
from robot_brain.control.authority import (
    get_motion_session,
    install_motion_session,
    reset_motion_authority_for_tests,
    session_or_create,
)
from robot_brain.teleop.session import TeleopSession


class MotionAuthorityTests(unittest.IsolatedAsyncioTestCase):
    def setUp(self) -> None:
        reset_motion_authority_for_tests()
        self.settings = Settings(
            robot_backend="unitree",
            unitree_dry_run=False,
            unitree_enable_motion=True,
            teleop_deadman_ms=120,
            teleop_lease_ttl_ms=400,
            teleop_chunk_seconds=0.05,
            memory_db_path=":memory:",
        )
        self.transport = FakeUnitreeTransport(
            UnitreeState(connected=True, is_standing=True)
        )
        self.robot = UnitreeRobot(self.transport, self.settings)

    def tearDown(self) -> None:
        reset_motion_authority_for_tests()

    def test_session_or_create_is_singleton(self) -> None:
        first = session_or_create(self.robot, self.settings)
        second = session_or_create(self.robot, self.settings)
        self.assertIs(first, second)
        self.assertIs(get_motion_session(), first)

    def test_install_rejects_different_session(self) -> None:
        session_or_create(self.robot, self.settings)
        other = TeleopSession(self.robot, self.settings)
        with self.assertRaises(RuntimeError):
            install_motion_session(other)

    async def test_dashboard_sees_prior_lease_on_shared_session(self) -> None:
        await self.transport.connect()
        session = session_or_create(self.robot, self.settings)
        lease = await session.acquire_lease("grpc-op")
        self.assertTrue(lease.granted)

        from robot_brain.runtime.loop import AgentRuntime
        from robot_brain.runtime.scheduler import AgentScheduler
        from robot_brain.service.runner import AgentService

        runtime = AgentRuntime.create(settings=self.settings, robot=self.robot)
        service = AgentService(
            AgentScheduler(runtime),
            poll_interval=0.01,
            close_runtime_on_stop=False,
        )
        result = await service.set_web_teleop(0.1, 0.0, 0.0)
        self.assertFalse(result["accepted"])
        self.assertIn("grpc-op", result.get("reason", ""))
        await service.stop()
        runtime.close()


class MediaOnDemandSettingsTests(unittest.TestCase):
    def test_media_on_demand_defaults_false(self) -> None:
        settings = Settings(memory_db_path=":memory:")
        self.assertFalse(settings.unitree_media_on_demand)

    def test_edge_profile_env_keys_exist(self) -> None:
        from pathlib import Path

        lean = Path("config/profiles/edge-brain-lean.env").read_text(encoding="utf-8")
        orin = Path("config/profiles/orin-nav-only.env").read_text(encoding="utf-8")
        self.assertIn("RDB_NAVIGATION_BACKEND=nav2", lean)
        self.assertIn("RDB_UNITREE_MEDIA_ON_DEMAND=true", lean)
        self.assertIn("RDB_UNITREE_VIDEO_RELAY=false", orin)


if __name__ == "__main__":
    unittest.main()
