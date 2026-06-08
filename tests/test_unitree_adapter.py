"""Tests for the Unitree robot adapter using fake transport."""
from __future__ import annotations

import unittest

from config.settings import Settings
from robot_brain.actuation.unitree import (
    FakeUnitreeTransport,
    UnitreeRobot,
    UnitreeState,
)
from robot_brain.core.world_state import Position


def make_robot(
    *,
    dry_run: bool = True,
    max_speed: float = 0.5,
    max_step: float = 2.0,
    initial_state: UnitreeState | None = None,
) -> tuple[UnitreeRobot, FakeUnitreeTransport]:
    settings = Settings(
        robot_backend="unitree",
        unitree_dry_run=dry_run,
        unitree_max_speed=max_speed,
        unitree_max_step=max_step,
        memory_db_path=":memory:",
    )
    transport = FakeUnitreeTransport(initial_state or UnitreeState(connected=True, is_standing=True))
    robot = UnitreeRobot(transport, settings)
    return robot, transport


class GetStateTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self) -> None:
        self.robot, self.transport = make_robot()
        await self.transport.connect()

    async def test_read_state_returns_robot_state(self) -> None:
        state = await self.robot.get_state()
        self.assertEqual(100.0, state.battery_level)
        self.assertFalse(state.docked)

    async def test_read_state_with_custom_values(self) -> None:
        self.transport._state.battery_level = 42.0
        self.transport._state.position = Position(x=3.0, y=4.0)
        state = await self.robot.get_state()
        self.assertEqual(42.0, state.battery_level)
        self.assertEqual(3.0, state.position.x)
        self.assertEqual(4.0, state.position.y)

    async def test_read_state_failure_returns_stopped(self) -> None:
        await self.transport.disconnect()
        state = await self.robot.get_state()
        self.assertTrue(state.stopped)

    async def test_read_state_records_action_history(self) -> None:
        await self.robot.get_state()
        self.assertEqual(1, len(self.robot.action_history))
        self.assertEqual("get_state", self.robot.action_history[0]["action"])


class StopTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self) -> None:
        self.robot, self.transport = make_robot(dry_run=False)
        await self.transport.connect()

    async def test_stop_sends_command(self) -> None:
        await self.robot.stop("test reason")
        self.assertEqual(1, len(self.transport.command_log))
        self.assertEqual("stop", self.transport.command_log[0].action)
        self.assertEqual("test reason", self.transport.command_log[0].parameters["reason"])

    async def test_stop_idempotent(self) -> None:
        await self.robot.stop("first")
        await self.robot.stop("second")
        await self.robot.stop("third")
        self.assertEqual(3, len(self.transport.command_log))

    async def test_stop_dry_run_no_command(self) -> None:
        robot, transport = make_robot(dry_run=True)
        await transport.connect()
        await robot.stop("dry run test")
        self.assertEqual(0, len(transport.command_log))
        self.assertEqual(1, len(robot.action_history))

    async def test_stop_transport_error_does_not_raise(self) -> None:
        self.transport.fail_next = True
        await self.robot.stop("should not crash")
        self.assertEqual(1, len(self.robot.action_history))


class MoveToTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self) -> None:
        self.robot, self.transport = make_robot(dry_run=False, max_speed=0.5, max_step=2.0)
        await self.transport.connect()

    async def test_move_within_limits(self) -> None:
        target = Position(x=1.0, y=0.0)
        await self.robot.move_to(target, speed=0.3)
        self.assertEqual(1, len(self.transport.command_log))
        cmd = self.transport.command_log[0]
        self.assertEqual("move", cmd.action)
        self.assertAlmostEqual(0.3, cmd.parameters["speed"])

    async def test_speed_clamped_to_max(self) -> None:
        target = Position(x=0.5, y=0.0)
        await self.robot.move_to(target, speed=5.0)
        cmd = self.transport.command_log[0]
        self.assertAlmostEqual(0.5, cmd.parameters["speed"])

    async def test_distance_exceeds_max_step_rejected(self) -> None:
        target = Position(x=10.0, y=10.0)
        with self.assertRaises(ValueError) as ctx:
            await self.robot.move_to(target, speed=0.3)
        self.assertIn("exceeds max step", str(ctx.exception))
        self.assertEqual(0, len(self.transport.command_log))

    async def test_transport_failure_triggers_stop(self) -> None:
        self.transport.fail_next = True
        target = Position(x=0.5, y=0.0)
        with self.assertRaises(RuntimeError):
            await self.robot.move_to(target, speed=0.3)
        # Best-effort stop should have been sent
        stop_cmds = [c for c in self.transport.command_log if c.action == "stop"]
        self.assertGreaterEqual(len(stop_cmds), 1)

    async def test_dry_run_no_command(self) -> None:
        robot, transport = make_robot(dry_run=True)
        await transport.connect()
        target = Position(x=0.5, y=0.0)
        await robot.move_to(target, speed=0.3)
        self.assertEqual(0, len(transport.command_log))

    async def test_cannot_read_state_before_move(self) -> None:
        await self.transport.disconnect()
        target = Position(x=0.5, y=0.0)
        with self.assertRaises(RuntimeError) as ctx:
            await self.robot.move_to(target, speed=0.3)
        self.assertIn("cannot read state", str(ctx.exception))


class TurnTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self) -> None:
        self.robot, self.transport = make_robot(dry_run=False)
        await self.transport.connect()

    async def test_turn_within_limits(self) -> None:
        await self.robot.turn(30.0)
        self.assertEqual(1, len(self.transport.command_log))
        cmd = self.transport.command_log[0]
        self.assertEqual("turn", cmd.action)
        self.assertAlmostEqual(30.0, cmd.parameters["heading_degrees"])

    async def test_turn_clamped_to_max(self) -> None:
        await self.robot.turn(90.0)
        cmd = self.transport.command_log[0]
        self.assertAlmostEqual(45.0, cmd.parameters["heading_degrees"])

    async def test_turn_clamped_negative(self) -> None:
        await self.robot.turn(-90.0)
        cmd = self.transport.command_log[0]
        self.assertAlmostEqual(-45.0, cmd.parameters["heading_degrees"])

    async def test_turn_transport_failure_triggers_stop(self) -> None:
        self.transport.fail_next = True
        with self.assertRaises(RuntimeError):
            await self.robot.turn(10.0)
        stop_cmds = [c for c in self.transport.command_log if c.action == "stop"]
        self.assertGreaterEqual(len(stop_cmds), 1)

    async def test_turn_dry_run(self) -> None:
        robot, transport = make_robot(dry_run=True)
        await transport.connect()
        await robot.turn(20.0)
        self.assertEqual(0, len(transport.command_log))


class UnsupportedMethodTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self) -> None:
        self.robot, self.transport = make_robot()
        await self.transport.connect()

    async def test_dock_raises_not_implemented(self) -> None:
        with self.assertRaises(NotImplementedError):
            await self.robot.dock("home")

    async def test_follow_raises_not_implemented(self) -> None:
        with self.assertRaises(NotImplementedError):
            await self.robot.follow("person-1", 2.0)

    async def test_report_does_not_raise(self) -> None:
        await self.robot.report("test message", "info")
        self.assertEqual(1, len(self.robot.action_history))
        self.assertEqual("report", self.robot.action_history[0]["action"])


class ActionHistoryTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self) -> None:
        self.robot, self.transport = make_robot(dry_run=False)
        await self.transport.connect()

    async def test_all_actions_recorded(self) -> None:
        await self.robot.get_state()
        await self.robot.stop("test")
        await self.robot.turn(10.0)
        await self.robot.move_to(Position(x=0.5, y=0), speed=0.3)
        await self.robot.report("hi", "info")
        self.assertEqual(5, len(self.robot.action_history))
        actions = [entry["action"] for entry in self.robot.action_history]
        self.assertEqual(["get_state", "stop", "turn", "move_to", "report"], actions)

    async def test_history_includes_timestamps(self) -> None:
        await self.robot.stop("test")
        entry = self.robot.action_history[0]
        self.assertIn("timestamp", entry)
        self.assertIsInstance(entry["timestamp"], float)


class RuntimeFactoryUnitreeTests(unittest.IsolatedAsyncioTestCase):
    async def test_runtime_create_unitree_backend(self) -> None:
        from robot_brain.runtime.loop import AgentRuntime

        settings = Settings(
            robot_backend="unitree",
            unitree_dry_run=True,
            memory_db_path=":memory:",
        )
        runtime = AgentRuntime.create(settings=settings)
        # Should not crash, and should use UnitreeRobot
        from robot_brain.actuation.unitree import UnitreeRobot
        self.assertIsInstance(runtime.context.robot, UnitreeRobot)

    async def test_runtime_unitree_can_run_command(self) -> None:
        from robot_brain.runtime.loop import AgentRuntime

        settings = Settings(
            robot_backend="unitree",
            unitree_dry_run=True,
            memory_db_path=":memory:",
        )
        runtime = AgentRuntime.create(settings=settings)
        result = await runtime.run_command("stop now")
        self.assertEqual("completed", result.status)


if __name__ == "__main__":
    unittest.main()
