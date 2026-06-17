"""Tests for the transport-agnostic TeleopSession (lease + deadman watchdog)."""
from __future__ import annotations

import asyncio
import unittest

from config.settings import Settings
from robot_brain.actuation.unitree import FakeUnitreeTransport, UnitreeRobot, UnitreeState
from robot_brain.teleop.session import ControlEventType, TeleopSession


def make_session(
    *,
    deadman_ms: int = 120,
    lease_ttl_ms: int = 400,
    chunk_seconds: float = 0.05,
) -> tuple[TeleopSession, UnitreeRobot, FakeUnitreeTransport]:
    settings = Settings(
        robot_backend="unitree",
        unitree_dry_run=True,
        teleop_deadman_ms=deadman_ms,
        teleop_lease_ttl_ms=lease_ttl_ms,
        teleop_chunk_seconds=chunk_seconds,
        memory_db_path=":memory:",
    )
    transport = FakeUnitreeTransport(UnitreeState(connected=True, is_standing=True))
    robot = UnitreeRobot(transport, settings)
    return TeleopSession(robot, settings), robot, transport


def _actions(robot: UnitreeRobot) -> list[str]:
    return [entry["action"] for entry in robot.action_history]


async def _drain(session: TeleopSession) -> list[ControlEventType]:
    events: list[ControlEventType] = []
    while not session.events.empty():
        events.append((await session.events.get()).type)
    return events


class LeaseTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self) -> None:
        self.session, self.robot, self.transport = make_session()
        await self.transport.connect()

    async def asyncTearDown(self) -> None:
        await self.session.emergency_stop("teardown")

    async def test_acquire_grants_lease(self) -> None:
        result = await self.session.acquire_lease("op-1")
        self.assertTrue(result.granted)
        self.assertTrue(result.lease_id)

    async def test_second_operator_rejected_while_held(self) -> None:
        await self.session.acquire_lease("op-1")
        result = await self.session.acquire_lease("op-2")
        self.assertFalse(result.granted)
        self.assertIn("op-1", result.reason)

    async def test_release_allows_new_holder(self) -> None:
        first = await self.session.acquire_lease("op-1")
        self.assertTrue(await self.session.release_lease(first.lease_id))
        second = await self.session.acquire_lease("op-2")
        self.assertTrue(second.granted)

    async def test_release_with_stale_lease_id_is_noop(self) -> None:
        await self.session.acquire_lease("op-1")
        self.assertFalse(await self.session.release_lease("not-the-lease"))


class SetpointTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self) -> None:
        self.session, self.robot, self.transport = make_session()
        await self.transport.connect()

    async def asyncTearDown(self) -> None:
        await self.session.emergency_stop("teardown")

    async def test_setpoint_without_lease_rejected(self) -> None:
        result = await self.session.set_velocity("ghost", 0.1, 0.0, 0.0)
        self.assertFalse(result.accepted)
        self.assertIn("no active lease", result.reason)

    async def test_setpoint_wrong_lease_rejected(self) -> None:
        await self.session.acquire_lease("op-1")
        result = await self.session.set_velocity("wrong-id", 0.1, 0.0, 0.0)
        self.assertFalse(result.accepted)

    async def test_setpoint_accepted_starts_drive(self) -> None:
        lease = await self.session.acquire_lease("op-1")
        result = await self.session.set_velocity(lease.lease_id, 0.1, 0.0, 0.0)
        self.assertTrue(result.accepted)
        await asyncio.sleep(0.02)
        self.assertIn("stream_hold", _actions(self.robot))
        self.assertIn(ControlEventType.ACCEPTED, await _drain(self.session))

    async def test_zero_setpoint_releases_drive(self) -> None:
        lease = await self.session.acquire_lease("op-1")
        await self.session.set_velocity(lease.lease_id, 0.1, 0.0, 0.0)
        await asyncio.sleep(0.02)
        await self.session.set_velocity(lease.lease_id, 0.0, 0.0, 0.0)
        self.assertIn("release_drive", _actions(self.robot))


class WatchdogTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self) -> None:
        self.session, self.robot, self.transport = make_session(
            deadman_ms=80, chunk_seconds=0.04
        )
        await self.transport.connect()

    async def asyncTearDown(self) -> None:
        await self.session.emergency_stop("teardown")

    async def test_deadman_stops_when_no_setpoint(self) -> None:
        lease = await self.session.acquire_lease("op-1")
        await self.session.set_velocity(lease.lease_id, 0.15, 0.0, 0.0)
        # No further setpoints: deadman must fire and end the drive loop.
        await asyncio.sleep(0.3)
        self.assertIn(ControlEventType.WATCHDOG, await _drain(self.session))
        self.assertIn("release_drive", _actions(self.robot))


class LeaseExpiryTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self) -> None:
        self.session, self.robot, self.transport = make_session(
            deadman_ms=1000, lease_ttl_ms=80
        )
        await self.transport.connect()

    async def asyncTearDown(self) -> None:
        await self.session.emergency_stop("teardown")

    async def test_setpoint_rejected_after_lease_expiry(self) -> None:
        lease = await self.session.acquire_lease("op-1")
        await asyncio.sleep(0.15)  # exceed lease TTL without renewing
        result = await self.session.set_velocity(lease.lease_id, 0.1, 0.0, 0.0)
        self.assertFalse(result.accepted)
        self.assertIn("no active lease", result.reason)


class EmergencyStopTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self) -> None:
        self.session, self.robot, self.transport = make_session()
        await self.transport.connect()

    async def test_estop_stops_and_invalidates_lease(self) -> None:
        lease = await self.session.acquire_lease("op-1")
        await self.session.set_velocity(lease.lease_id, 0.2, 0.0, 0.0)
        await asyncio.sleep(0.02)
        await self.session.emergency_stop("operator estop")
        self.assertIn("stop", _actions(self.robot))
        # Lease invalidated: further setpoints rejected.
        result = await self.session.set_velocity(lease.lease_id, 0.1, 0.0, 0.0)
        self.assertFalse(result.accepted)


if __name__ == "__main__":
    unittest.main()
