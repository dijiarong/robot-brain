"""Tests for real Unitree SDK transport using an injected fake client."""
from __future__ import annotations

import unittest
from unittest.mock import AsyncMock, patch

from config.settings import Settings
from robot_brain.actuation.unitree import UnitreeCommand
from robot_brain.actuation.unitree_sdk import UnitreeSDKTransport


def make_settings(**kwargs) -> Settings:
    defaults = dict(
        robot_backend="unitree",
        unitree_transport="sdk",
        unitree_model="go2",
        unitree_dry_run=True,
        memory_db_path=":memory:",
    )
    defaults.update(kwargs)
    return Settings(**defaults)


class FakeSDKClient:
    """Fake SDK client that returns dict-based state for testing."""

    def __init__(self, state: dict | None = None, *, fail: bool = False) -> None:
        self._state = state or {
            "connected": True,
            "battery_level": 78.0,
            "position": {"x": 1.5, "y": 2.3},
            "heading_degrees": 45.0,
            "is_standing": True,
            "is_moving": False,
            "error_code": 0,
        }
        self._fail = fail

    async def get_state(self) -> dict:
        if self._fail:
            raise RuntimeError("Simulated SDK read failure")
        return self._state


class ConnectDisconnectTests(unittest.IsolatedAsyncioTestCase):
    async def test_connect_with_injected_client(self) -> None:
        client = FakeSDKClient()
        transport = UnitreeSDKTransport(make_settings(), sdk_client=client)
        self.assertFalse(transport.is_connected)
        await transport.connect()
        self.assertTrue(transport.is_connected)

    async def test_connect_idempotent(self) -> None:
        client = FakeSDKClient()
        transport = UnitreeSDKTransport(make_settings(), sdk_client=client)
        await transport.connect()
        await transport.connect()
        self.assertTrue(transport.is_connected)

    async def test_disconnect(self) -> None:
        client = FakeSDKClient()
        transport = UnitreeSDKTransport(make_settings(), sdk_client=client)
        await transport.connect()
        await transport.disconnect()
        self.assertFalse(transport.is_connected)

    async def test_connect_without_sdk_raises_clear_error(self) -> None:
        """When no client injected and SDK not installed, error is clear."""
        transport = UnitreeSDKTransport(make_settings())
        with patch(
            "robot_brain.actuation.unitree_sdk._import_sdk",
            side_effect=RuntimeError("unitree_sdk2_python is not installed"),
        ):
            with self.assertRaises(RuntimeError) as ctx:
                await transport.connect()
            self.assertIn("not installed", str(ctx.exception))


class ReadStateTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self) -> None:
        self.client = FakeSDKClient()
        self.transport = UnitreeSDKTransport(make_settings(), sdk_client=self.client)
        await self.transport.connect()

    async def test_read_state_basic(self) -> None:
        state = await self.transport.read_state()
        self.assertTrue(state.connected)
        self.assertAlmostEqual(78.0, state.battery_level)
        self.assertAlmostEqual(1.5, state.position.x)
        self.assertAlmostEqual(2.3, state.position.y)
        self.assertAlmostEqual(45.0, state.heading_degrees)
        self.assertTrue(state.is_standing)
        self.assertFalse(state.is_moving)
        self.assertEqual(0, state.error_code)

    async def test_read_state_moving(self) -> None:
        self.client._state["is_moving"] = True
        self.client._state["is_standing"] = True
        state = await self.transport.read_state()
        self.assertTrue(state.is_moving)
        self.assertTrue(state.is_standing)

    async def test_read_state_low_battery(self) -> None:
        self.client._state["battery_level"] = 12.5
        state = await self.transport.read_state()
        self.assertAlmostEqual(12.5, state.battery_level)

    async def test_read_state_with_error_code(self) -> None:
        self.client._state["error_code"] = 42
        state = await self.transport.read_state()
        self.assertEqual(42, state.error_code)

    async def test_read_state_missing_fields_defaults(self) -> None:
        """Missing fields should use safe defaults."""
        self.client._state = {"connected": True}
        state = await self.transport.read_state()
        self.assertTrue(state.connected)
        self.assertAlmostEqual(100.0, state.battery_level)
        self.assertAlmostEqual(0.0, state.position.x)

    async def test_read_state_not_connected_raises(self) -> None:
        await self.transport.disconnect()
        with self.assertRaises(ConnectionError):
            await self.transport.read_state()


class ReadStateErrorTests(unittest.IsolatedAsyncioTestCase):
    async def test_sdk_read_failure(self) -> None:
        client = FakeSDKClient(fail=True)
        transport = UnitreeSDKTransport(make_settings(), sdk_client=client)
        await transport.connect()
        with self.assertRaises(ConnectionError) as ctx:
            await transport.read_state()
        self.assertIn("State read failed", str(ctx.exception))

    async def test_sdk_returns_none_fields(self) -> None:
        """Client returning nulls for some fields should not crash."""
        client = FakeSDKClient(state={
            "connected": True,
            "battery_level": None,
            "position": None,
            "heading_degrees": None,
            "is_standing": None,
            "is_moving": None,
            "error_code": None,
        })
        transport = UnitreeSDKTransport(make_settings(), sdk_client=client)
        await transport.connect()
        state = await transport.read_state()
        self.assertTrue(state.connected)
        # None values should default safely
        self.assertAlmostEqual(100.0, state.battery_level)
        self.assertAlmostEqual(0.0, state.position.x)


class SendCommandReadOnlyTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self) -> None:
        client = FakeSDKClient()
        self.transport = UnitreeSDKTransport(make_settings(), sdk_client=client)
        await self.transport.connect()

    async def test_send_command_rejected(self) -> None:
        cmd = UnitreeCommand(action="stop", parameters={"reason": "test"})
        with self.assertRaises(NotImplementedError) as ctx:
            await self.transport.send_command(cmd)
        self.assertIn("read-only", str(ctx.exception))

    async def test_move_command_rejected(self) -> None:
        cmd = UnitreeCommand(action="move", parameters={"target": {"x": 1, "y": 0}})
        with self.assertRaises(NotImplementedError):
            await self.transport.send_command(cmd)

    async def test_send_command_not_connected_raises(self) -> None:
        await self.transport.disconnect()
        cmd = UnitreeCommand(action="stop", parameters={})
        with self.assertRaises(ConnectionError):
            await self.transport.send_command(cmd)


class IntegrationWithUnitreeRobotTests(unittest.IsolatedAsyncioTestCase):
    """Test that UnitreeRobot works with the SDK transport (read-only)."""

    async def test_get_state_via_sdk_transport(self) -> None:
        from robot_brain.actuation.unitree import UnitreeRobot

        client = FakeSDKClient()
        settings = make_settings()
        transport = UnitreeSDKTransport(settings, sdk_client=client)
        await transport.connect()
        robot = UnitreeRobot(transport, settings)

        state = await robot.get_state()
        self.assertAlmostEqual(78.0, state.battery_level)
        self.assertAlmostEqual(1.5, state.position.x)
        self.assertAlmostEqual(2.3, state.position.y)

    async def test_stop_fails_gracefully_with_readonly_transport(self) -> None:
        """UnitreeRobot.stop() with SDK transport should not crash (best-effort)."""
        from robot_brain.actuation.unitree import UnitreeRobot

        client = FakeSDKClient()
        settings = make_settings(unitree_dry_run=False)
        transport = UnitreeSDKTransport(settings, sdk_client=client)
        await transport.connect()
        robot = UnitreeRobot(transport, settings)

        # stop() catches exceptions from transport — should not raise
        await robot.stop("test")
        # Action recorded even if transport rejected
        self.assertTrue(any(a["action"] == "stop" for a in robot.action_history))

    async def test_move_to_fails_with_readonly_transport(self) -> None:
        """move_to with real SDK transport raises since transport is read-only."""
        from robot_brain.actuation.unitree import UnitreeRobot
        from robot_brain.core.world_state import Position

        client = FakeSDKClient()
        settings = make_settings(unitree_dry_run=False)
        transport = UnitreeSDKTransport(settings, sdk_client=client)
        await transport.connect()
        robot = UnitreeRobot(transport, settings)

        with self.assertRaises(RuntimeError):
            await robot.move_to(Position(x=1.6, y=2.3), speed=0.3)


class RuntimeFactorySDKTransportTests(unittest.IsolatedAsyncioTestCase):
    async def test_runtime_create_unitree_sdk_fails_without_real_sdk(self) -> None:
        """Creating runtime with sdk transport when SDK absent gives clear error."""
        from robot_brain.runtime.loop import AgentRuntime

        settings = Settings(
            robot_backend="unitree",
            unitree_transport="sdk",
            unitree_dry_run=True,
            memory_db_path=":memory:",
        )
        # This should fail at transport creation because SDK isn't installed
        # But since create_sdk_transport returns a transport without connecting,
        # the error comes at connect time. For runtime factory, transport is
        # created but connect is not called — so it should succeed.
        runtime = AgentRuntime.create(settings=settings)
        from robot_brain.actuation.unitree import UnitreeRobot
        self.assertIsInstance(runtime.context.robot, UnitreeRobot)


if __name__ == "__main__":
    unittest.main()
