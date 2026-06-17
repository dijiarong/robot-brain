"""gRPC (asyncio) servicer wrapping a single :class:`TeleopSession`.

Responsibilities are deliberately thin:

* ``AcquireLease`` / ``ReleaseLease`` / ``EmergencyStop`` map 1:1 onto the
  session methods.
* ``Teleop`` is a bidirectional stream: inbound ``MoveSetpoint`` messages are
  applied as velocity setpoints (renewing the lease + deadman), while session
  ``ControlEvent``s are streamed back to the operator.

All robot safety (clamping, dry-run, lease, deadman) lives below this layer in
``TeleopSession`` / ``UnitreeRobot`` — the transport adds none of its own.
"""
from __future__ import annotations

import asyncio
import logging

import grpc

from config.settings import Settings
from robot_brain.actuation.unitree import UnitreeRobot
from robot_brain.control.generated import control_pb2 as pb
from robot_brain.control.generated import control_pb2_grpc as pb_grpc
from robot_brain.teleop.session import ControlEvent, ControlEventType, TeleopSession

logger = logging.getLogger(__name__)

_EVENT_TYPE_TO_PROTO = {
    ControlEventType.ACCEPTED: pb.ControlEvent.ACCEPTED,
    ControlEventType.REJECTED: pb.ControlEvent.REJECTED,
    ControlEventType.STOPPED: pb.ControlEvent.STOPPED,
    ControlEventType.PREEMPTED: pb.ControlEvent.PREEMPTED,
    ControlEventType.WATCHDOG: pb.ControlEvent.WATCHDOG,
    ControlEventType.OBSTACLE: pb.ControlEvent.OBSTACLE,
}


def _to_proto_event(event: ControlEvent) -> pb.ControlEvent:
    return pb.ControlEvent(
        type=_EVENT_TYPE_TO_PROTO[event.type],
        message=event.message,
    )


class RobotControlServicer(pb_grpc.RobotControlServicer):
    """Bridges the gRPC control plane to one TeleopSession."""

    def __init__(self, session: TeleopSession) -> None:
        self._session = session

    async def AcquireLease(
        self, request: pb.AcquireLeaseRequest, context: grpc.aio.ServicerContext
    ) -> pb.LeaseReply:
        ttl_ms = request.ttl_ms or None
        result = await self._session.acquire_lease(request.operator_id, ttl_ms)
        return pb.LeaseReply(
            granted=result.granted,
            lease_id=result.lease_id,
            expires_at=result.expires_at,
            reason=result.reason,
        )

    async def ReleaseLease(
        self, request: pb.LeaseRef, context: grpc.aio.ServicerContext
    ) -> pb.Ack:
        ok = await self._session.release_lease(request.lease_id)
        return pb.Ack(ok=ok, message="" if ok else "stale lease_id")

    async def EmergencyStop(
        self, request: pb.EstopRequest, context: grpc.aio.ServicerContext
    ) -> pb.Ack:
        await self._session.emergency_stop(request.reason)
        return pb.Ack(ok=True, message="stopped")

    async def Teleop(
        self,
        request_iterator: grpc.aio.MessageIterator,
        context: grpc.aio.ServicerContext,
    ):
        """Apply inbound setpoints; stream session events back to the operator."""
        reader = asyncio.create_task(self._consume_setpoints(request_iterator))
        try:
            while True:
                event_task = asyncio.create_task(self._session.events.get())
                done, _ = await asyncio.wait(
                    {event_task, reader},
                    return_when=asyncio.FIRST_COMPLETED,
                )
                if event_task in done:
                    yield _to_proto_event(event_task.result())
                else:
                    # Inbound stream ended (client closed / cancelled): drain any
                    # event that arrived in the same tick, then finish.
                    event_task.cancel()
                    while not self._session.events.empty():
                        yield _to_proto_event(self._session.events.get_nowait())
                    break
        finally:
            if not reader.done():
                reader.cancel()

    async def _consume_setpoints(
        self, request_iterator: grpc.aio.MessageIterator
    ) -> None:
        async for setpoint in request_iterator:
            await self._session.set_velocity(
                setpoint.lease_id,
                setpoint.vx,
                setpoint.vy,
                setpoint.vyaw,
            )


def build_server(
    robot: UnitreeRobot,
    settings: Settings,
    address: str = "127.0.0.1:50071",
) -> tuple[grpc.aio.Server, str]:
    """Create (but do not start) an aio gRPC server bound to *address*.

    Returns the server and the resolved bind address (with the chosen port,
    useful when *address* ends in ``:0``).
    """
    session = TeleopSession(robot, settings)
    server = grpc.aio.server()
    pb_grpc.add_RobotControlServicer_to_server(RobotControlServicer(session), server)
    bound_port = server.add_insecure_port(address)
    host = address.rsplit(":", 1)[0]
    return server, f"{host}:{bound_port}"


async def serve(
    robot: UnitreeRobot,
    settings: Settings,
    address: str = "127.0.0.1:50071",
) -> None:
    """Run the control server until cancelled."""
    server, bound = build_server(robot, settings, address)
    await server.start()
    logger.info("robot-brain control plane listening on %s", bound)
    try:
        await server.wait_for_termination()
    except asyncio.CancelledError:
        await server.stop(grace=1.0)
        raise
