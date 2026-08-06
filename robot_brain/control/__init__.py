"""gRPC control-plane shell over the transport-agnostic TeleopSession.

The package keeps the wire layer (gRPC) separate from robot logic: the
servicer only translates protobuf messages into ``TeleopSession`` calls and
forwards ``ControlEvent``s back. Importing this package does not require the
generated stubs until :func:`build_server` / :class:`RobotControlServicer`
are actually used, so the rest of robot-brain still runs without grpc.
"""
from __future__ import annotations

from typing import Any

__all__ = ["RobotControlServicer", "build_server", "serve"]


def __getattr__(name: str) -> Any:
    if name in __all__:
        from robot_brain.control import server as _server

        return getattr(_server, name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
