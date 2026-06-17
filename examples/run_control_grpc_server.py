"""Run the robot-brain gRPC control plane over a Unitree Go2 (fake or webrtc).

This is the Stage-2 ingress: a thin gRPC shell over TeleopSession (lease +
deadman watchdog). A LAN/cloud gateway (e.g. AvatarRobot-Service) connects as a
client and forwards "前后左右" velocity setpoints; all safety still lives below
in TeleopSession / UnitreeRobot.

Requires the optional grpc dependency:  pip install -e '.[grpc]'

Usage:
    python -m examples.run_control_grpc_server --transport fake
    RDB_UNITREE_ENABLE_MOTION=true python -m examples.run_control_grpc_server \\
        --transport webrtc --live --address 0.0.0.0:50071
"""
from __future__ import annotations

import argparse
import asyncio
import logging
import sys

from config.settings import Settings
from examples.run_unitree_smoke import create_transport
from examples.run_unitree_teleop import TELEOP_CONFIRMATION, prep_locomotion
from robot_brain.actuation.unitree import UnitreeRobot
from robot_brain.control.server import build_server

# Continuous teleop streams in chunks; allow a long single drive window.
_MAX_DRIVE_SECONDS = 5.0


async def main() -> None:
    logging.basicConfig(level=logging.WARNING)
    logging.getLogger("robot_brain").setLevel(logging.INFO)

    parser = argparse.ArgumentParser(description="robot-brain gRPC control server")
    parser.add_argument("--transport", choices=["fake", "webrtc"], default="fake")
    parser.add_argument("--live", action="store_true")
    parser.add_argument("--robot-ip", default="")
    parser.add_argument("--address", default="127.0.0.1:50071")
    parser.add_argument("--prep-stand", action="store_true", default=None)
    parser.add_argument("--no-prep-stand", action="store_true")
    args = parser.parse_args()

    if args.no_prep_stand:
        prep_stand_flag = False
    elif args.prep_stand:
        prep_stand_flag = True
    else:
        prep_stand_flag = args.live

    if args.transport == "webrtc" and not args.live:
        print("[ERROR] webrtc control server requires --live")
        sys.exit(1)

    settings_kwargs: dict = dict(
        robot_backend="unitree",
        unitree_transport=args.transport,
        unitree_dry_run=not args.live,
    )
    if args.robot_ip:
        settings_kwargs["unitree_robot_ip"] = args.robot_ip
    if args.live:
        settings_kwargs["unitree_max_drive_duration"] = _MAX_DRIVE_SECONDS
    settings = Settings(**settings_kwargs)

    if args.transport == "webrtc" and args.live and not settings.unitree_enable_motion:
        print("[ERROR] Live webrtc requires RDB_UNITREE_ENABLE_MOTION=true")
        sys.exit(1)

    if args.live:
        print("[WARNING] Real motion — flat ground, clear space, keep e-stop ready.")
        confirm = input(f"Type '{TELEOP_CONFIRMATION}' to proceed: ").strip()
        if confirm != TELEOP_CONFIRMATION:
            print("[ABORT]")
            sys.exit(1)

    transport = None
    robot = None
    try:
        try:
            transport = await create_transport(args.transport, settings)
        except (RuntimeError, ConnectionError, TimeoutError) as exc:
            print(f"[ERROR] Connect failed: {exc}")
            sys.exit(1)

        robot = UnitreeRobot(transport, settings)
        if args.live and prep_stand_flag:
            await prep_locomotion(robot)

        server, bound = build_server(robot, settings, args.address)
        await server.start()
        print(f"[Control] gRPC RobotControl listening on {bound} (transport={args.transport})")
        try:
            await server.wait_for_termination()
        except (KeyboardInterrupt, asyncio.CancelledError):
            pass
        finally:
            await server.stop(grace=1.0)
    finally:
        if robot is not None:
            await robot.stop("control server shutdown")
        if transport is not None:
            await transport.disconnect()
        print("[OK] Control server stopped.")


if __name__ == "__main__":
    asyncio.run(main())
