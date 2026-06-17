"""End-to-end test for the gRPC control plane over a dry-run UnitreeRobot.

Boots the real aio server on an ephemeral port and drives it with a generated
client stub: acquire lease -> stream setpoints -> emergency stop. grpc is an
optional dependency, so the whole module is skipped when it (or the generated
stubs) cannot be imported.
"""
from __future__ import annotations

import asyncio
import unittest

from config.settings import Settings
from robot_brain.actuation.unitree import FakeUnitreeTransport, UnitreeRobot, UnitreeState

try:
    import grpc

    from robot_brain.control.generated import control_pb2 as pb
    from robot_brain.control.generated import control_pb2_grpc as pb_grpc
    from robot_brain.control.server import build_server

    _HAVE_GRPC = True
except ImportError:  # pragma: no cover - exercised only without grpc installed
    _HAVE_GRPC = False


def _make_robot() -> tuple[UnitreeRobot, FakeUnitreeTransport, Settings]:
    settings = Settings(
        robot_backend="unitree",
        unitree_dry_run=True,
        teleop_deadman_ms=120,
        teleop_lease_ttl_ms=400,
        teleop_chunk_seconds=0.05,
        memory_db_path=":memory:",
    )
    transport = FakeUnitreeTransport(UnitreeState(connected=True, is_standing=True))
    return UnitreeRobot(transport, settings), transport, settings


def _actions(robot: UnitreeRobot) -> list[str]:
    return [entry["action"] for entry in robot.action_history]


@unittest.skipUnless(_HAVE_GRPC, "grpcio / generated stubs not available")
class ControlGrpcTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self) -> None:
        self.robot, self.transport, settings = _make_robot()
        await self.transport.connect()
        self.server, bound = build_server(self.robot, settings, "127.0.0.1:0")
        await self.server.start()
        self.channel = grpc.aio.insecure_channel(bound)
        self.stub = pb_grpc.RobotControlStub(self.channel)

    async def asyncTearDown(self) -> None:
        await self.channel.close()
        await self.server.stop(grace=None)

    async def test_acquire_lease_grants(self) -> None:
        reply = await self.stub.AcquireLease(pb.AcquireLeaseRequest(operator_id="web-1"))
        self.assertTrue(reply.granted)
        self.assertTrue(reply.lease_id)

    async def test_second_operator_rejected(self) -> None:
        await self.stub.AcquireLease(pb.AcquireLeaseRequest(operator_id="web-1"))
        reply = await self.stub.AcquireLease(pb.AcquireLeaseRequest(operator_id="web-2"))
        self.assertFalse(reply.granted)
        self.assertIn("web-1", reply.reason)

    async def test_teleop_stream_drives_and_reports_accepted(self) -> None:
        lease = await self.stub.AcquireLease(
            pb.AcquireLeaseRequest(operator_id="web-1")
        )

        async def setpoints():
            yield pb.MoveSetpoint(lease_id=lease.lease_id, vx=0.2, vy=0.0, vyaw=0.0, seq=1)
            await asyncio.sleep(0.1)

        stream = self.stub.Teleop(setpoints())
        first = await stream.read()
        self.assertEqual(first.type, pb.ControlEvent.ACCEPTED)
        await asyncio.sleep(0.02)
        self.assertIn("stream_hold", _actions(self.robot))
        stream.cancel()

    async def test_emergency_stop(self) -> None:
        lease = await self.stub.AcquireLease(
            pb.AcquireLeaseRequest(operator_id="web-1")
        )
        await self.stub.AcquireLease(pb.AcquireLeaseRequest(operator_id="web-1"))
        ack = await self.stub.EmergencyStop(pb.EstopRequest(reason="panic"))
        self.assertTrue(ack.ok)
        self.assertIn("stop", _actions(self.robot))
        # Lease invalidated after estop.
        sp = pb.MoveSetpoint(lease_id=lease.lease_id, vx=0.1, vy=0.0, vyaw=0.0)
        stream = self.stub.Teleop(_single(sp))
        # Server accepts the stream but rejects the setpoint internally; the
        # drive must not start.
        await asyncio.sleep(0.05)
        stream.cancel()
        self.assertNotIn("stream_hold", _actions(self.robot)[-1:])


async def _single(msg):
    yield msg


if __name__ == "__main__":
    unittest.main()
