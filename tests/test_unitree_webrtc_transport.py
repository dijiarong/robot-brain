"""Tests for the WebRTC Unitree transport using an injected fake connection.

Covers posture/stop command mapping, the enable_motion safety gate, rejection
of translation commands, motion lease/preemption, and integration with UnitreeRobot.
"""
from __future__ import annotations

import asyncio
import json
import time
import unittest
from unittest.mock import AsyncMock, MagicMock, patch

from config.settings import Settings
from robot_brain.actuation.unitree import UnitreeCommand, UnitreeRobot, UnitreeState
from robot_brain.actuation.unitree_motion import MotionEndReason
from robot_brain.actuation.unitree_webrtc import (
    _GO2_JOY_FULL_LINEAR,
    _GO2_JOY_FULL_YAW,
    _SPORT_API_ID,
    _SPORT_REQUEST_TOPIC,
    _WIRELESS_CONTROLLER_TOPIC,
    _ZERO_STICK,
    _looks_like_sport_state,
    _parse_sport_error_code,
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


def _move_pub_calls(pub_cb) -> list:
    out: list = []
    for c in pub_cb.call_args_list:
        if c.args[0] != _SPORT_REQUEST_TOPIC or not isinstance(c.args[1], dict):
            continue
        payload = c.args[1]
        api_id = payload.get("header", {}).get("identity", {}).get("api_id")
        if api_id == _SPORT_API_ID["sport_move"]:
            out.append(c)
    return out


def _move_params_from_call(call) -> dict[str, float]:
    raw = call.args[1].get("parameter", "{}")
    return json.loads(raw) if isinstance(raw, str) else raw


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
        skip = {"sport_move", "switch_joystick", "speed_level"}
        for action, api_id in _SPORT_API_ID.items():
            if action in skip:
                continue
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
        self.transport, self.conn = connected_transport(
            unitree_enable_motion=True,
            unitree_webrtc_drive_via_move=False,
        )
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
        settings = make_settings(
            unitree_enable_motion=True,
            unitree_dry_run=False,
            unitree_max_speed=0.5,
        )
        robot = UnitreeRobot(self.transport, settings)
        await robot.drive(vx=99.0, duration=0.0)
        first = self.joy.call_args_list[0].kwargs["data"]
        self.assertAlmostEqual(
            first["ly"], 0.5 / _GO2_JOY_FULL_LINEAR, places=4
        )
        entry = next(a for a in robot.action_history if a["action"] == "drive")
        self.assertLessEqual(entry["vx"], 0.5)

    async def test_robot_drive_dry_run_does_not_send(self) -> None:
        settings = make_settings(unitree_enable_motion=True, unitree_dry_run=True)
        robot = UnitreeRobot(self.transport, settings)
        await robot.drive(vx=0.3, duration=0.0)
        self.joy.assert_not_called()
        self.assertTrue(any(a["action"] == "drive" for a in robot.action_history))


class DriveMoveTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self) -> None:
        self.transport, self.conn = connected_transport(
            unitree_enable_motion=True,
            unitree_webrtc_drive_via_move=True,
        )
        await self.transport.connect()
        self.pub = self.conn.datachannel.pub_sub.publish_without_callback

    async def test_drive_streams_move_api_for_strafe(self) -> None:
        await self.transport.send_command(
            UnitreeCommand(action="drive", parameters={"vy": -0.35, "duration": 0.0})
        )
        move_calls = _move_pub_calls(self.pub)
        self.assertGreaterEqual(len(move_calls), 1)
        param = _move_params_from_call(move_calls[0])
        self.assertAlmostEqual(param["y"], -0.35, places=4)
        self.assertAlmostEqual(param["x"], 0.0, places=4)

    async def test_drive_move_ends_with_zero_velocity(self) -> None:
        await self.transport.send_command(
            UnitreeCommand(action="drive", parameters={"vx": 0.2, "duration": 0.0})
        )
        move_calls = _move_pub_calls(self.pub)
        last = _move_params_from_call(move_calls[-1])
        self.assertEqual(last, {"x": 0.0, "y": 0.0, "z": 0.0})

    async def test_enable_omni_teleop_sends_switch_and_speed(self) -> None:
        await self.transport.enable_omni_teleop()
        pub = self.conn.datachannel.pub_sub.publish_request_new
        ids = [c.args[1]["api_id"] for c in pub.await_args_list]
        self.assertIn(_SPORT_API_ID["switch_joystick"], ids)
        self.assertIn(_SPORT_API_ID["speed_level"], ids)

    async def test_forward_plus_turn_uses_joystick_not_move(self) -> None:
        await self.transport.send_command(
            UnitreeCommand(
                action="drive",
                parameters={"vx": 0.3, "vyaw": -0.3, "duration": 0.0},
            )
        )
        move_calls = _move_pub_calls(self.pub)
        joy_calls = [
            c
            for c in self.pub.call_args_list
            if c.args and c.args[0] == _WIRELESS_CONTROLLER_TOPIC
        ]
        non_zero_sticks = [
            c.kwargs["data"]
            for c in joy_calls
            if c.kwargs["data"] != _ZERO_STICK
        ]
        self.assertGreaterEqual(len(non_zero_sticks), 1)
        last_stick = non_zero_sticks[-1]
        self.assertNotAlmostEqual(last_stick["ly"], 0.0)
        self.assertNotAlmostEqual(last_stick["rx"], 0.0)
        non_zero_moves = [
            c for c in move_calls if _move_params_from_call(c) != {"x": 0.0, "y": 0.0, "z": 0.0}
        ]
        self.assertEqual(non_zero_moves, [])


class MotionLeaseTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self) -> None:
        self.transport, self.conn = connected_transport(
            unitree_enable_motion=True,
            unitree_zero_frame_count=3,
            unitree_webrtc_drive_via_move=False,
        )
        await self.transport.connect()
        self.joy = self.conn.datachannel.pub_sub.publish_without_callback

    async def test_stop_preempts_active_drive(self) -> None:
        task = asyncio.create_task(
            self.transport.send_command(
                UnitreeCommand(action="drive", parameters={"vx": 0.2, "duration": 1.0})
            )
        )
        await asyncio.sleep(0.05)
        await self.transport.send_command(UnitreeCommand(action="stop"))
        await task
        self.assertEqual(
            self.transport.last_drive_end_reason,
            MotionEndReason.OPERATOR_STOP,
        )
        for call in self.joy.call_args_list[-3:]:
            self.assertEqual(call.kwargs["data"], _ZERO_STICK)

    async def test_new_drive_preempts_old_drive(self) -> None:
        first = asyncio.create_task(
            self.transport.send_command(
                UnitreeCommand(action="drive", parameters={"vx": 0.2, "duration": 1.0})
            )
        )
        await asyncio.sleep(0.05)
        await self.transport.send_command(
            UnitreeCommand(action="drive", parameters={"vx": -0.2, "duration": 0.0})
        )
        await first
        non_zero_after_preempt = [
            c for c in self.joy.call_args_list
            if c.kwargs["data"]["ly"] > 0 and c is not self.joy.call_args_list[0]
        ]
        # Second drive uses negative vx — should see negative ly, not positive from first.
        last_nonzero = next(
            (c for c in reversed(self.joy.call_args_list) if c.kwargs["data"] != _ZERO_STICK),
            None,
        )
        self.assertIsNotNone(last_nonzero)
        self.assertLess(last_nonzero.kwargs["data"]["ly"], 0)

    async def test_stale_state_rejects_drive(self) -> None:
        self.transport._last_sport_state_mono = time.monotonic() - 10.0
        settings = make_settings(unitree_enable_motion=True, unitree_dry_run=False)
        robot = UnitreeRobot(self.transport, settings)
        with self.assertRaises(RuntimeError) as ctx:
            await robot.drive(vx=0.1, duration=0.1)
        self.assertIn("stale", str(ctx.exception).lower())

    async def test_not_standing_rejects_drive(self) -> None:
        self.transport._on_sport_state(
            {
                "mode": 5,  # lieDown
                "error_code": 0,
                "velocity": [0.0, 0.0, 0.0],
                "position": [0.0, 0.0, 0.0],
                "imu_state": {"rpy": [0.0, 0.0, 0.0]},
            }
        )
        settings = make_settings(unitree_enable_motion=True, unitree_dry_run=False)
        robot = UnitreeRobot(self.transport, settings)
        with self.assertRaises(RuntimeError) as ctx:
            await robot.drive(vx=0.1, duration=0.1)
        self.assertIn("not ready", str(ctx.exception).lower())

    async def test_drive_records_end_reason_in_history(self) -> None:
        settings = make_settings(unitree_enable_motion=True, unitree_dry_run=False)
        robot = UnitreeRobot(self.transport, settings)
        await robot.drive(vx=0.1, duration=0.0)
        entry = next(a for a in robot.action_history if a["action"] == "drive")
        self.assertIn("end_reason", entry)
        self.assertIn("elapsed", entry)
        self.assertTrue(entry.get("success"))

    async def test_api_echo_error_code_allows_drive(self) -> None:
        """MCF firmware may put last sport api_id (e.g. 1002) in error_code — not a fault."""
        self.transport._on_sport_state(
            {
                "mode": 1,
                "error_code": 1002,
                "velocity": [0.0, 0.0, 0.0],
                "position": [0.0, 0.0, 0.0],
                "imu_state": {"rpy": [0.0, 0.0, 0.0]},
            }
        )
        settings = make_settings(unitree_enable_motion=True, unitree_dry_run=False)
        robot = UnitreeRobot(self.transport, settings)
        await robot.drive(vx=0.1, duration=0.0)
        state = await self.transport.read_state()
        self.assertEqual(0, state.error_code)

    async def test_sport_api_response_does_not_overwrite_state(self) -> None:
        self.transport._on_sport_state(
            {
                "mode": 1,
                "error_code": 0,
                "velocity": [0.0, 0.0, 0.0],
                "position": [0.0, 0.0, 0.0],
                "imu_state": {"rpy": [0.0, 0.0, 0.0]},
            }
        )
        self.transport._on_sport_state(
            {
                "type": "res",
                "topic": "rt/api/sport/response",
                "data": {
                    "header": {
                        "identity": {"api_id": 1002, "id": 1},
                        "status": {"code": 0},
                    },
                    "data": "",
                },
            }
        )
        state = await self.transport.read_state()
        self.assertEqual(1, state.sport_mode)
        self.assertEqual(0, state.error_code)

    async def test_watchdog_stops_stream_and_zeros(self) -> None:
        transport, conn = connected_transport(
            unitree_enable_motion=True,
            unitree_control_watchdog_seconds=0.05,
            unitree_zero_frame_count=2,
            unitree_webrtc_drive_via_move=False,
        )
        await transport.connect()
        joy = conn.datachannel.pub_sub.publish_without_callback

        real_sleep = asyncio.sleep
        sleep_calls = 0

        async def slow_first_period(delay: float) -> None:
            nonlocal sleep_calls
            sleep_calls += 1
            if sleep_calls == 1:
                await real_sleep(0.12)
            else:
                await real_sleep(delay)

        with patch(
            "robot_brain.actuation.unitree_webrtc.asyncio.sleep",
            side_effect=slow_first_period,
        ):
            await transport.send_command(
                UnitreeCommand(action="drive", parameters={"vx": 0.2, "duration": 2.0})
            )

        self.assertEqual(
            transport.last_drive_end_reason,
            MotionEndReason.WATCHDOG,
        )
        for call in joy.call_args_list[-2:]:
            self.assertEqual(call.kwargs["data"], _ZERO_STICK)

    async def test_drive_cancel_sends_zeros(self) -> None:
        task = asyncio.create_task(
            self.transport.send_command(
                UnitreeCommand(action="drive", parameters={"vx": 0.2, "duration": 5.0})
            )
        )
        await asyncio.sleep(0.05)
        task.cancel()
        with self.assertRaises(asyncio.CancelledError):
            await task
        self.assertEqual(
            self.transport.last_drive_end_reason,
            MotionEndReason.CANCELLED,
        )
        for call in self.joy.call_args_list[-3:]:
            self.assertEqual(call.kwargs["data"], _ZERO_STICK)

    async def test_publish_error_sends_zeros(self) -> None:
        def fail_nonzero(*args: object, **kwargs: object) -> None:
            data = kwargs.get("data")
            if data != _ZERO_STICK:
                raise RuntimeError("channel closed")

        self.joy.side_effect = fail_nonzero
        with self.assertRaises(RuntimeError):
            await self.transport.send_command(
                UnitreeCommand(action="drive", parameters={"vx": 0.2, "duration": 1.0})
            )
        self.assertEqual(
            self.transport.last_drive_end_reason,
            MotionEndReason.TRANSPORT_ERROR,
        )
        for call in self.joy.call_args_list[-3:]:
            self.assertEqual(call.kwargs["data"], _ZERO_STICK)

    async def test_disconnect_during_drive_halts_and_zeros(self) -> None:
        task = asyncio.create_task(
            self.transport.send_command(
                UnitreeCommand(action="drive", parameters={"vx": 0.2, "duration": 5.0})
            )
        )
        await asyncio.sleep(0.05)
        await self.transport.disconnect()
        await task
        self.assertIn(
            self.transport.last_drive_end_reason,
            (MotionEndReason.DISCONNECT, MotionEndReason.PREEMPTED),
        )
        self.assertFalse(self.transport.is_connected)
        for call in self.joy.call_args_list[-3:]:
            self.assertEqual(call.kwargs["data"], _ZERO_STICK)

    async def test_post_drive_still_moving_upgrades_stop(self) -> None:
        self.transport._on_sport_state(
            {
                "mode": 1,
                "error_code": 0,
                "velocity": [0.5, 0.0, 0.0],
                "position": [0.0, 0.0, 0.0],
                "imu_state": {"rpy": [0.0, 0.0, 0.0]},
            }
        )
        settings = make_settings(unitree_enable_motion=True, unitree_dry_run=False)
        robot = UnitreeRobot(self.transport, settings)
        with self.assertRaises(RuntimeError) as ctx:
            await robot.drive(vx=0.1, duration=0.0)
        self.assertIn("still reports motion", str(ctx.exception))
        entry = next(a for a in robot.action_history if a["action"] == "drive")
        self.assertFalse(entry.get("success"))
        self.assertEqual(entry.get("end_reason"), "post_drive_still_moving")
        pub = self.conn.datachannel.pub_sub.publish_request_new
        stop_calls = [
            c for c in pub.await_args_list
            if c.args and c.args[1].get("api_id") == _SPORT_API_ID["stop"]
        ]
        self.assertGreaterEqual(len(stop_calls), 1)


class SportStateParsingTests(unittest.TestCase):
    def test_parse_sport_error_code_ignores_api_echo(self) -> None:
        self.assertEqual(0, _parse_sport_error_code({"error_code": 1002}))
        self.assertEqual(0, _parse_sport_error_code({"error_code": 1004}))
        self.assertEqual(42, _parse_sport_error_code({"error_code": 42}))

    def test_looks_like_sport_state_rejects_api_response(self) -> None:
        self.assertFalse(
            _looks_like_sport_state(
                {"header": {"identity": {"api_id": 1002}, "status": {"code": 0}}}
            )
        )
        self.assertTrue(_looks_like_sport_state({"mode": 1, "velocity": [0, 0, 0]}))


class ReleaseDriveTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self) -> None:
        self.transport, self.conn = connected_transport(
            unitree_enable_motion=True,
            unitree_webrtc_drive_via_move=True,
        )
        await self.transport.connect()
        self.pub = self.conn.datachannel.pub_sub.publish_without_callback

    async def test_release_zeros_without_stopmove(self) -> None:
        await self.transport.send_command(UnitreeCommand(action="release"))
        pub = self.conn.datachannel.pub_sub.publish_request_new
        stop_calls = [
            c for c in pub.await_args_list
            if c.args and c.args[1].get("api_id") == _SPORT_API_ID["stop"]
        ]
        self.assertEqual([], stop_calls)
        move_zeros = [
            c
            for c in _move_pub_calls(self.pub)
            if _move_params_from_call(c) == {"x": 0.0, "y": 0.0, "z": 0.0}
        ]
        self.assertTrue(move_zeros)

    async def test_free_walk_sends_api_2045(self) -> None:
        settings = make_settings(unitree_enable_motion=True, unitree_dry_run=False)
        robot = UnitreeRobot(self.transport, settings)
        await robot.set_posture("free_walk")
        self.conn.datachannel.pub_sub.publish_request_new.assert_awaited_with(
            _SPORT_REQUEST_TOPIC, {"api_id": _SPORT_API_ID["free_walk"]}
        )


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
