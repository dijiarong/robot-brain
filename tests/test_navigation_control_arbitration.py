from __future__ import annotations

import unittest

from config.settings import Settings
from robot_brain.actuation.unitree import FakeUnitreeTransport, UnitreeRobot
from robot_brain.navigation import FakeNavigationClient, NavigationStatus, RelativeNavigationGoal
from robot_brain.teleop.session import ControlEventType, TeleopSession


class _BrokenCancelNavigation(FakeNavigationClient):
    async def cancel(self, goal_id=None):
        raise RuntimeError("cancel channel down")


async def _session(navigation):
    settings = Settings(
        robot_backend="unitree", unitree_dry_run=False,
        unitree_enable_motion=True, memory_db_path=":memory:",
    )
    transport = FakeUnitreeTransport()
    await transport.connect()
    robot = UnitreeRobot(transport, settings)
    return TeleopSession(robot, settings, navigation), robot


class NavigationControlArbitrationTests(unittest.IsolatedAsyncioTestCase):
    async def test_teleop_lease_preempts_active_navigation(self) -> None:
        navigation = FakeNavigationClient(outcomes=[NavigationStatus.ACTIVE])
        handle = await navigation.set_relative_goal(RelativeNavigationGoal(forward_m=0.5))
        session, _ = await _session(navigation)

        lease = await session.acquire_lease("operator")
        state = await navigation.get_state()
        event = await session.events.get()

        self.assertTrue(lease.granted)
        self.assertEqual(NavigationStatus.CANCELED, state.status)
        self.assertEqual(ControlEventType.PREEMPTED, event.type)
        self.assertIn(handle.goal_id, event.message)

    async def test_preemption_failure_denies_teleop_and_stops_robot(self) -> None:
        navigation = _BrokenCancelNavigation(outcomes=[NavigationStatus.ACTIVE])
        await navigation.set_relative_goal(RelativeNavigationGoal(forward_m=0.5))
        session, robot = await _session(navigation)

        lease = await session.acquire_lease("operator")

        self.assertFalse(lease.granted)
        self.assertIn("preemption failed", lease.reason)
        self.assertTrue(any(
            row["action"] == "stop" and "preempt" in row.get("reason", "")
            for row in robot.action_history
        ))

    async def test_estop_cancels_navigation_without_active_teleop_lease(self) -> None:
        navigation = FakeNavigationClient(outcomes=[NavigationStatus.ACTIVE])
        await navigation.set_relative_goal(RelativeNavigationGoal(forward_m=0.5))
        session, _ = await _session(navigation)

        await session.emergency_stop("operator estop")

        self.assertEqual(NavigationStatus.CANCELED, (await navigation.get_state()).status)
