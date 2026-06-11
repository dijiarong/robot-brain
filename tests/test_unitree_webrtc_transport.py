"""Tests for the WebRTC Unitree transport using an injected fake connection.

Covers posture/stop command mapping, the enable_motion safety gate, rejection
of translation commands, and integration with UnitreeRobot.set_posture.
"""
from __future__ import annotations

import unittest
from unittest.mock import AsyncMock, MagicMock

from config.settings import Settings
from robot_brain.actuation.unitree import UnitreeCommand, UnitreeRobot
from robot_brain.actuation.unitree_webrtc import (
    _GO2_JOY_FULL_LINEAR,
    _GO2_JOY_FULL_YAW,
    _SPORT_API_ID,
    _SPORT_REQUEST_TOPIC,
    _WIRELESS_CONTROLLER_TOPIC,
    UnitreeWebRTCTransport,
)


def make_settings(**kwargs) -> Settings:
    defaults = dict(
        robot_backend="unitree",
        unitree_transport="webrtc",
        unitree_model="go2",
        unitree_dry_run=True,
        unitree_enable_motion=False,
        memory_db_path=":memory:",
    )
    defaults.update(kwargs)
    return Settings(**defaults)


def make_fake_conn() -> MagicMock:
    """Build a fake UnitreeWebRTCConnection with an async publish_request_new."""
    conn = MagicMock()
    conn.datachannel.pub_sub.publish_request_new = AsyncMock(return_value={"ok": True})
    return conn


def connected_transport(**settings_kwargs) -> tuple[UnitreeWebRTCTransport, MagicMock]:
    conn = make_fake_conn()
    transport = UnitreeWebRTCTransport(make_settings(**settings_kwargs), webrtc_conn=conn)
    return transport, conn


class ConnectDisconnectTests(unittest.IsolatedAsyncioTestCase):
    async def test_connect_with_injected_conn(self) -> None:
        transport, _ = connected_transport()
        self.assertFalse(transport.is_connected)
        await transport.connect()
        self.assertTrue(transport.is_connected)

    async def test_disconnect_clears_connection(self) -> None:
        transport, _ = connected_transport()
        await transport.connect()
        await transport.disconnect()
        self.assertFalse(transport.is_connected)


class SendCommandPostureTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self) -> None:
        self.transport, self.conn = connected_transport(unitree_enable_motion=True)
        await self.transport.connect()
        self.publish = self.conn.datachannel.pub_sub.publish_request_new

    async def test_stand_up_maps_to_api_id(self) -> None:
        result = await self.transport.send_command(UnitreeCommand(action="stand_up"))
        self.assertTrue(result)
        self.publish.assert_awaited_once_with(
            _SPORT_REQUEST_TOPIC, {"api_id": _SPORT_API_ID["stand_up"]}
        )

    async def test_each_posture_maps_correctly(self) -> None:
        for action, api_id in _SPORT_API_ID.items():
            with self.subTest(action=action):
                self.publish.reset_mock()
                await self.transport.send_command(UnitreeCommand(action=action))
                self.publish.assert_awaited_once_with(
                    _SPORT_REQUEST_TOPIC, {"api_id": api_id}
                )

    async def test_unknown_action_rejected(self) -> None:
        with self.assertRaises(NotImplementedError):
            await self.transport.send_command(UnitreeCommand(action="backflip"))

    async def test_translation_commands_rejected(self) -> None:
        for action in ("move", "turn"):
            with self.subTest(action=action):
                with self.assertRaises(NotImplementedError):
                    await self.transport.send_command(UnitreeCommand(action=action))

    async def test_publish_failure_wrapped_as_runtime_error(self) -> None:
        self.publish.side_effect = Exception("channel closed")
        with self.assertRaises(RuntimeError) as ctx:
            await self.transport.send_command(UnitreeCommand(action="stand_up"))
        self.assertIn("failed", str(ctx.exception))

    async def test_response_status_code_zero_is_accepted(self) -> None:
        self.publish.return_value = {"data": {"header": {"status": {"code": 0}}}}
        result = await self.transport.send_command(UnitreeCommand(action="stand_up"))
        self.assertTrue(result)

    async def test_nonzero_status_code_rejected_as_runtime_error(self) -> None:
        self.publish.return_value = {"data": {"header": {"status": {"code": 7002}}}}
        with self.assertRaises(RuntimeError) as ctx:
            await self.transport.send_command(UnitreeCommand(action="stand_up"))
        self.assertIn("7002", str(ctx.exception))

    async def test_status_code_in_json_string_data(self) -> None:
        import json as _json

        self.publish.return_value = {"data": _json.dumps({"header": {"status": {"code": 3203}}})}
        with self.assertRaises(RuntimeError):
            await self.transport.send_command(UnitreeCommand(action="stand_up"))


class MotionGateTests(unittest.IsolatedAsyncioTestCase):
    async def test_posture_blocked_when_motion_disabled(self) -> None:
        transport, conn = connected_transport(unitree_enable_motion=False)
        await transport.connect()
        with self.assertRaises(PermissionError):
            await transport.send_command(UnitreeCommand(action="stand_up"))
        conn.datachannel.pub_sub.publish_request_new.assert_not_awaited()

    async def test_stop_allowed_even_when_motion_disabled(self) -> None:
        transport, conn = connected_transport(unitree_enable_motion=False)
        await transport.connect()
        result = await transport.send_command(UnitreeCommand(action="stop"))
        self.assertTrue(result)
        conn.datachannel.pub_sub.publish_request_new.assert_awaited_once_with(
            _SPORT_REQUEST_TOPIC, {"api_id": _SPORT_API_ID["stop"]}
        )

    async def test_command_not_connected_raises(self) -> None:
        transport, _ = connected_transport(unitree_enable_motion=True)
        with self.assertRaises(ConnectionError):
            await transport.send_command(UnitreeCommand(action="stand_up"))


class DriveJoystickTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self) -> None:
        self.transport, self.conn = connected_transport(unitree_enable_motion=True)
        await self.transport.connect()
        self.joy = self.conn.datachannel.pub_sub.publish_without_callback

    async def test_drive_blocked_when_motion_disabled(self) -> None:
        transport, conn = connected_transport(unitree_enable_motion=False)
        await transport.connect()
        with self.assertRaises(PermissionError):
            await transport.send_command(
                UnitreeCommand(action="drive", parameters={"vx": 0.3, "duration": 0.0})
            )
        conn.datachannel.pub_sub.publish_without_callback.assert_not_called()

    async def test_drive_streams_joystick_on_wireless_topic(self) -> None:
        result = await self.transport.send_command(
            UnitreeCommand(action="drive", parameters={"vx": 0.3, "duration": 0.0})
        )
        self.assertTrue(result)
        self.assertGreaterEqual(self.joy.call_count, 1)
        topic, kwargs = self.joy.call_args_list[0].args[0], self.joy.call_args_list[0].kwargs
        self.assertEqual(topic, _WIRELESS_CONTROLLER_TOPIC)
        self.assertAlmostEqual(kwargs["data"]["ly"], 0.3 / _GO2_JOY_FULL_LINEAR, places=4)

    async def test_drive_yaw_maps_to_inverted_right_stick(self) -> None:
        await self.transport.send_command(
            UnitreeCommand(action="drive", parameters={"vyaw": 0.5, "duration": 0.0})
        )
        first = self.joy.call_args_list[0].kwargs["data"]
        self.assertAlmostEqual(first["rx"], -0.5 / _GO2_JOY_FULL_YAW, places=4)

    async def test_drive_always_ends_with_zero_sample(self) -> None:
        await self.transport.send_command(
            UnitreeCommand(action="drive", parameters={"vx": 0.3, "duration": 0.0})
        )
        last = self.joy.call_args_list[-1].kwargs["data"]
        self.assertEqual(last, {"lx": 0.0, "ly": 0.0, "rx": 0.0, "ry": 0.0})

    async def test_robot_drive_clamps_to_max_speed(self) -> None:
        settings = make_settings(unitree_enable_motion=True, unitree_dry_run=False)
        robot = UnitreeRobot(self.transport, settings)
        await robot.drive(vx=99.0, duration=0.0)
        first = self.joy.call_args_list[0].kwargs["data"]
        # vx clamped to unitree_max_speed (0.5) -> ly = 0.5 / 1.5
        self.assertAlmostEqual(
            first["ly"], settings.unitree_max_speed / _GO2_JOY_FULL_LINEAR, places=4
        )
        entry = next(a for a in robot.action_history if a["action"] == "drive")
        self.assertLessEqual(entry["vx"], settings.unitree_max_speed)

    async def test_robot_drive_dry_run_does_not_send(self) -> None:
        settings = make_settings(unitree_enable_motion=True, unitree_dry_run=True)
        robot = UnitreeRobot(self.transport, settings)
        await robot.drive(vx=0.3, duration=0.0)
        self.joy.assert_not_called()
        self.assertTrue(any(a["action"] == "drive" for a in robot.action_history))


class IntegrationWithUnitreeRobotTests(unittest.IsolatedAsyncioTestCase):
    async def test_set_posture_dry_run_does_not_send(self) -> None:
        transport, conn = connected_transport(unitree_enable_motion=True, unitree_dry_run=True)
        await transport.connect()
        robot = UnitreeRobot(transport, make_settings(unitree_enable_motion=True))
        await robot.set_posture("stand_up")
        conn.datachannel.pub_sub.publish_request_new.assert_not_awaited()
        self.assertTrue(any(a["action"] == "set_posture" for a in robot.action_history))

    async def test_set_posture_live_sends_command(self) -> None:
        settings = make_settings(unitree_enable_motion=True, unitree_dry_run=False)
        transport, conn = connected_transport(unitree_enable_motion=True, unitree_dry_run=False)
        await transport.connect()
        robot = UnitreeRobot(transport, settings)
        await robot.set_posture("balance_stand")
        conn.datachannel.pub_sub.publish_request_new.assert_awaited_once_with(
            _SPORT_REQUEST_TOPIC, {"api_id": _SPORT_API_ID["balance_stand"]}
        )

    async def test_set_posture_unsupported_raises(self) -> None:
        settings = make_settings(unitree_dry_run=False)
        transport, _ = connected_transport(unitree_dry_run=False)
        await transport.connect()
        robot = UnitreeRobot(transport, settings)
        with self.assertRaises(ValueError):
            await robot.set_posture("moonwalk")

    async def test_set_posture_blocked_by_gate_best_effort_stop(self) -> None:
        """With motion disabled, set_posture fails but stop (safety) still sends."""
        settings = make_settings(unitree_enable_motion=False, unitree_dry_run=False)
        transport, conn = connected_transport(unitree_enable_motion=False, unitree_dry_run=False)
        await transport.connect()
        robot = UnitreeRobot(transport, settings)
        with self.assertRaises(RuntimeError):
            await robot.set_posture("stand_up")
        # Best-effort stop was issued and reached the transport (stop bypasses gate).
        conn.datachannel.pub_sub.publish_request_new.assert_awaited_once_with(
            _SPORT_REQUEST_TOPIC, {"api_id": _SPORT_API_ID["stop"]}
        )

    async def test_stop_via_robot_sends_stopmove(self) -> None:
        settings = make_settings(unitree_enable_motion=False, unitree_dry_run=False)
        transport, conn = connected_transport(unitree_enable_motion=False, unitree_dry_run=False)
        await transport.connect()
        robot = UnitreeRobot(transport, settings)
        await robot.stop("test")
        conn.datachannel.pub_sub.publish_request_new.assert_awaited_once_with(
            _SPORT_REQUEST_TOPIC, {"api_id": _SPORT_API_ID["stop"]}
        )


if __name__ == "__main__":
    unittest.main()
