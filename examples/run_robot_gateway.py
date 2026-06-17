"""Unified robot gateway: Go2 WebRTC + cloud signaling + browser WebRTC (no bin/robot).

Replaces the two-process stack (robot-brain gRPC + topsun bin/robot) for teleop:
  browser ←WebRTC→ this process ←WebRTC→ Go2

Control goes through TeleopSession (lease + deadman). All A/V is in-process WebRTC
(Go2 ↔ gateway ↔ browser) — no ffmpeg, no UDP :5000/:5005/:5010.

Requires:
  pip install -e '.[unitree-webrtc,grpc]'

Usage:
  export UNITREE_AES_128_KEY=<32hex>
  export RDB_UNITREE_ENABLE_MOTION=true
  export RDB_GATEWAY_SIGNALING_URL=wss://111.229.166.203:9999/ws
  export RDB_GATEWAY_TURN_URL=turn:111.229.166.203:3478
  export RDB_GATEWAY_TURN_USER=test
  export RDB_GATEWAY_TURN_PASS=123456
  python -m examples.run_robot_gateway --live --robot-ip 10.10.197.28
"""
from __future__ import annotations

import argparse
import asyncio
import logging
import sys

from config.settings import Settings
from examples.run_unitree_smoke import create_transport
from examples.run_unitree_teleop import prep_locomotion
from robot_brain.actuation.unitree import UnitreeRobot

# NOTE: Do NOT import robot_brain.gateway (aiortc) before Go2 WebRTC connect.
# gRPC entry does not pre-load aiortc; loading it here on the main thread before
# the Go2 handshake runs on a background event loop breaks DTLS on Python 3.13
# (ICE completes, peer never reaches "connected", 15 s datachannel timeout).

_MAX_DRIVE_SECONDS = 5.0
logger = logging.getLogger(__name__)


async def main() -> None:
    logging.basicConfig(level=logging.WARNING)
    logging.getLogger("robot_brain").setLevel(logging.INFO)

    parser = argparse.ArgumentParser(description="robot-brain unified gateway (Go2 + browser WebRTC)")
    parser.add_argument("--live", action="store_true")
    parser.add_argument("--robot-ip", default="")
    parser.add_argument("--signaling-url", default="")
    parser.add_argument("--robot-id", default="")
    parser.add_argument("--prep-stand", action="store_true", default=None)
    parser.add_argument("--no-prep-stand", action="store_true")
    args = parser.parse_args()

    if not args.live:
        print("[ERROR] gateway requires --live")
        sys.exit(1)

    prep_stand = False if args.no_prep_stand else (True if args.prep_stand else True)

    # In-process Go2 media (no ffmpeg/UDP relay). Do NOT import gateway/aiortc before connect.
    settings_kwargs: dict = dict(
        robot_backend="unitree",
        unitree_transport="webrtc",
        unitree_dry_run=False,
        unitree_gateway=True,
        unitree_video_relay=False,
        unitree_audio_relay=False,
        unitree_max_drive_duration=_MAX_DRIVE_SECONDS,
    )
    if args.robot_ip:
        settings_kwargs["unitree_robot_ip"] = args.robot_ip
    if args.signaling_url:
        settings_kwargs["gateway_signaling_url"] = args.signaling_url
    if args.robot_id:
        settings_kwargs["gateway_robot_id"] = args.robot_id
    settings = Settings(**settings_kwargs)

    if not settings.unitree_enable_motion:
        print("[ERROR] Live gateway requires RDB_UNITREE_ENABLE_MOTION=true")
        sys.exit(1)

    transport = None
    robot = None
    try:
        try:
            transport = await create_transport("webrtc", settings)
        except (RuntimeError, ConnectionError, TimeoutError) as exc:
            print(f"[ERROR] Connect failed: {exc}")
            sys.exit(1)

        # Import browser gateway stack only AFTER Go2 WebRTC is up.
        from robot_brain.gateway.gateway import RobotGateway
        from robot_brain.teleop.session import TeleopSession

        robot = UnitreeRobot(transport, settings)
        if prep_stand:
            await prep_locomotion(robot)

        session = TeleopSession(robot, settings)
        gateway = RobotGateway(transport, session, settings)

        print(
            f"[Gateway] Go2 connected; signaling={settings.gateway_signaling_url} "
            f"robot_id={settings.gateway_robot_id}"
        )
        if settings.gateway_turn_url:
            print(
                f"[Gateway] TURN {settings.gateway_turn_url} "
                f"(user={settings.gateway_turn_user or '(none)'})"
            )
        else:
            print("[Gateway] WARNING: no TURN — browser must be on same LAN as this Mac.")
        print(
            "[Gateway] Open https://<cloud-host>:9999/ (trust self-signed cert), "
            "connect signaling, call robot — no bin/robot needed."
        )

        await gateway.run()
    except (KeyboardInterrupt, asyncio.CancelledError):
        pass
    except Exception as exc:
        logger.exception("[Gateway] fatal: %s", exc)
    finally:
        if robot is not None:
            await robot.stop("gateway shutdown")
        if transport is not None:
            await transport.disconnect()
        print("[OK] Gateway stopped. Go2 slot released.", file=sys.stderr)


if __name__ == "__main__":
    asyncio.run(main())
