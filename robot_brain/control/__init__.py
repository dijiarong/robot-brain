"""gRPC control-plane shell over the transport-agnostic TeleopSession.

The package keeps the wire layer (gRPC) separate from robot logic: the
servicer only translates protobuf messages into ``TeleopSession`` calls and
forwards ``ControlEvent``s back. Importing this package does not require the
generated stubs until :func:`build_server` / :class:`RobotControlServicer`
are actually used, so the rest of robot-brain still runs without grpc.
"""

from robot_brain.control.server import RobotControlServicer, build_server, serve

__all__ = ["RobotControlServicer", "build_server", "serve"]
