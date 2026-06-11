"""Unitree smoke test — validate adapter connectivity and basic actions.

Usage:
    python -m examples.run_unitree_smoke                                   # state-only, fake transport
    python -m examples.run_unitree_smoke --state-only --transport sdk      # real SDK, read-only
    python -m examples.run_unitree_smoke --state-only --transport webrtc   # real WebRTC, read-only
    python -m examples.run_unitree_smoke --actions --transport fake        # dry-run action sequence
    python -m examples.run_unitree_smoke --actions --live                  # fake real actions (confirmation)
    # Real posture/stop on Go2 (no translation), requires explicit motion gate:
    RDB_UNITREE_ENABLE_MOTION=true python -m examples.run_unitree_smoke \\
        --transport webrtc --actions --live
    # Real velocity teleop nudges on Go2 (joystick channel, the robot WILL move):
    RDB_UNITREE_ENABLE_MOTION=true python -m examples.run_unitree_smoke \\
        --transport webrtc --drive --live
"""
from __future__ import annotations

import argparse
import asyncio
import logging
import sys

from config.settings import Settings
from robot_brain.actuation.unitree import (
    FakeUnitreeTransport,
    UnitreeRobot,
    UnitreeState,
)
from robot_brain.core.world_state import Position


CONFIRMATION_PHRASE = "I_UNDERSTAND_UNITREE_MOVE"
POSTURE_CONFIRMATION_PHRASE = "I_UNDERSTAND_UNITREE_POSTURE"
DRIVE_CONFIRMATION_PHRASE = "I_UNDERSTAND_UNITREE_DRIVE"


def print_state(label: str, state: object) -> None:
    print(f"\n{'='*50}")
    print(f"  {label}")
    print(f"{'='*50}")
    if hasattr(state, "model_dump"):
        for key, value in state.model_dump(mode="json").items():
            print(f"  {key}: {value}")
    else:
        print(f"  {state}")
    print()


async def run_state_only(robot: UnitreeRobot) -> None:
    print("[Phase] Reading robot state...")
    state = await robot.get_state()
    print_state("Robot State", state)
    print("[OK] State read complete.")


async def run_action_sequence(robot: UnitreeRobot) -> None:
    print("[Phase] Minimal action sequence:")
    print("  1. Read state")
    print("  2. Stop")
    print("  3. Turn 15 degrees")
    print("  4. Move forward 0.5m")
    print("  5. Stop")
    print("  6. Read state")
    print()

    if not robot.dry_run:
        print(f"[WARNING] DRY-RUN IS OFF. Real actions will be sent to the robot.")
        print(f"[WARNING] Ensure: open space, safe distance, operator has e-stop ready.")
        print()
        confirmation = input(f"Type '{CONFIRMATION_PHRASE}' to proceed: ").strip()
        if confirmation != CONFIRMATION_PHRASE:
            print("[ABORT] Confirmation not received. Exiting.")
            sys.exit(1)
        print()

    print("[1/6] Reading initial state...")
    state = await robot.get_state()
    print_state("Initial State", state)

    print("[2/6] Issuing stop...")
    await robot.stop("smoke test: initial stop")
    print("  Done.")

    print("[3/6] Turning 15 degrees...")
    await robot.turn(15.0)
    print("  Done.")

    print("[4/6] Moving forward 0.5m...")
    target = Position(x=state.position.x + 0.5, y=state.position.y)
    try:
        await robot.move_to(target, speed=0.3)
        print("  Done.")
    except (ValueError, RuntimeError) as exc:
        print(f"  Move rejected/failed: {exc}")

    print("[5/6] Issuing final stop...")
    await robot.stop("smoke test: final stop")
    print("  Done.")

    print("[6/6] Reading final state...")
    final_state = await robot.get_state()
    print_state("Final State", final_state)

    print("\n[Action History]")
    for i, entry in enumerate(robot.action_history, 1):
        action = entry.get("action", "?")
        ts = entry.get("timestamp", "")
        display = {k: v for k, v in entry.items() if k not in ("action", "timestamp")}
        print(f"  {i}. {action} {display}")

    print("\n[OK] Action sequence complete.")


async def run_posture_sequence(robot: UnitreeRobot) -> None:
    """Real posture/stop sequence for live WebRTC transport (no translation).

    The robot stays in place — it only changes posture. Run only on flat ground
    with clear space around the robot and an operator ready to power off.
    """
    print("[Phase] Posture sequence (no translation):")
    print("  1. Read state")
    print("  2. Stand up (wake from rest)")
    print("  3. Balance stand")
    print("  4. Stand down (lie)")
    print("  5. Recovery stand (get up)")
    print("  6. Balance stand")
    print("  7. Read state")
    print()

    print(f"[WARNING] Real posture commands will be sent to the robot.")
    print(f"[WARNING] Ensure: flat ground, clear space, operator can power off.")
    print()
    confirmation = input(f"Type '{POSTURE_CONFIRMATION_PHRASE}' to proceed: ").strip()
    if confirmation != POSTURE_CONFIRMATION_PHRASE:
        print("[ABORT] Confirmation not received. Exiting.")
        sys.exit(1)
    print()

    print("[1/7] Reading initial state...")
    print_state("Initial State", await robot.get_state())

    # Mirror dimos's proven wake-up recipe: StandUp first (this is what actually
    # lifts a resting Go2), then BalanceStand, before any other posture.
    steps = [
        ("2/7", "stand_up", 3.0),
        ("3/7", "balance_stand", 3.0),
        ("4/7", "stand_down", 4.0),
        ("5/7", "recovery_stand", 4.0),
        ("6/7", "balance_stand", 3.0),
    ]
    for label, posture, settle in steps:
        print(f"[{label}] {posture}...")
        await robot.set_posture(posture)
        await asyncio.sleep(settle)
        print("  Done.")

    print("[7/7] Reading final state...")
    print_state("Final State", await robot.get_state())

    print("\n[Action History]")
    for i, entry in enumerate(robot.action_history, 1):
        action = entry.get("action", "?")
        display = {k: v for k, v in entry.items() if k not in ("action", "timestamp")}
        print(f"  {i}. {action} {display}")

    print("\n[OK] Posture sequence complete.")


async def run_drive_sequence(robot: UnitreeRobot) -> None:
    """Minimal velocity-teleop demo over the joystick channel (WebRTC).

    Sends small, short, auto-stopping nudges — the same channel dimos's qweasd
    keyboard control uses. Run only on flat ground with clear space and an
    operator ready to power off.
    """
    print("[Phase] Drive nudge demo (joystick velocity teleop):")
    print("  1. Read state")
    print("  2. Forward nudge  (vx=+0.3 m/s, 0.6s)")
    print("  3. Backward nudge (vx=-0.3 m/s, 0.6s)")
    print("  4. Yaw left nudge (vyaw=+0.5 rad/s, 0.6s)")
    print("  5. Read state")
    print()

    print("[WARNING] Real movement commands will be sent — the robot WILL move.")
    print("[WARNING] Ensure: flat ground, >1m clear space, operator can power off.")
    print()
    confirmation = input(f"Type '{DRIVE_CONFIRMATION_PHRASE}' to proceed: ").strip()
    if confirmation != DRIVE_CONFIRMATION_PHRASE:
        print("[ABORT] Confirmation not received. Exiting.")
        sys.exit(1)
    print()

    print("[1/5] Reading initial state...")
    print_state("Initial State", await robot.get_state())

    nudges = [
        ("2/5", "forward",  {"vx": 0.3, "duration": 0.6}),
        ("3/5", "backward", {"vx": -0.3, "duration": 0.6}),
        ("4/5", "yaw left", {"vyaw": 0.5, "duration": 0.6}),
    ]
    for label, name, kw in nudges:
        print(f"[{label}] {name} nudge...")
        await robot.drive(**kw)
        await asyncio.sleep(1.0)
        print("  Done.")

    print("[5/5] Reading final state...")
    print_state("Final State", await robot.get_state())

    print("\n[Action History]")
    for i, entry in enumerate(robot.action_history, 1):
        action = entry.get("action", "?")
        display = {k: v for k, v in entry.items() if k not in ("action", "timestamp")}
        print(f"  {i}. {action} {display}")

    print("\n[OK] Drive demo complete.")


async def create_transport(transport_type: str, settings: Settings):
    """Create and connect the appropriate transport."""
    if transport_type == "sdk":
        from robot_brain.actuation.unitree_sdk import create_sdk_transport

        transport = create_sdk_transport(settings)
        print("[Transport] Connecting to real Unitree Go2 via SDK (DDS)...")
        await transport.connect()
        print("[Transport] Connected.")
        return transport
    elif transport_type == "webrtc":
        from robot_brain.actuation.unitree_webrtc import create_webrtc_transport

        transport = create_webrtc_transport(settings)
        print("[Transport] Connecting to real Unitree Go2 via WebRTC...")
        await transport.connect()
        print("[Transport] Connected.")
        return transport
    else:
        transport = FakeUnitreeTransport(
            initial_state=UnitreeState(
                connected=True,
                is_standing=True,
                battery_level=85.0,
                position=Position(x=0, y=0),
            )
        )
        await transport.connect()
        return transport


async def main() -> None:
    # Show INFO from our own modules (e.g. sport command responses) without
    # amplifying the verbose library logs, which go to the root logger.
    logging.basicConfig(level=logging.WARNING)
    logging.getLogger("robot_brain").setLevel(logging.INFO)

    parser = argparse.ArgumentParser(description="Unitree robot smoke test")
    parser.add_argument("--state-only", action="store_true", default=True, help="Only read state (default)")
    parser.add_argument("--actions", action="store_true", help="Run minimal action sequence")
    parser.add_argument(
        "--drive", action="store_true",
        help="Run a minimal velocity-teleop nudge demo (webrtc only, joystick channel)",
    )
    parser.add_argument("--live", action="store_true", help="Disable dry-run (send real commands)")
    parser.add_argument(
        "--transport", choices=["fake", "sdk", "webrtc"], default="fake",
        help="Transport to use: fake (default), sdk (DDS direct), or webrtc (LAN/STA mode)",
    )
    args = parser.parse_args()

    # Safety: sdk transport is still read-only in this iteration.
    if args.transport == "sdk" and (args.actions or args.live):
        print("[ERROR] sdk transport is read-only in this iteration.")
        print("        Use --transport fake for action sequences, or --transport webrtc for posture.")
        sys.exit(1)

    # webrtc supports posture/stop actions only, and only when live + motion enabled.
    if args.transport == "webrtc" and args.actions and not args.live:
        print("[ERROR] webrtc posture sequence requires --live (real commands).")
        print("        Use --transport fake --actions for a dry-run sequence.")
        sys.exit(1)

    # --drive (velocity teleop) is webrtc-only and requires live + motion gate.
    if args.drive and args.transport != "webrtc":
        print("[ERROR] --drive (velocity teleop) is only supported on --transport webrtc.")
        sys.exit(1)
    if args.drive and not args.live:
        print("[ERROR] --drive requires --live (real movement commands).")
        sys.exit(1)

    settings = Settings(
        robot_backend="unitree",
        unitree_transport=args.transport,
        unitree_dry_run=not args.live,
    )

    if args.transport == "webrtc" and (args.actions or args.drive) and not settings.unitree_enable_motion:
        kind = "posture sequence" if args.actions else "drive demo"
        flag = "--actions" if args.actions else "--drive"
        print(f"[ERROR] webrtc {kind} requires RDB_UNITREE_ENABLE_MOTION=true.")
        print("        This is a hard safety gate for real motion. Export it explicitly:")
        print(f"        RDB_UNITREE_ENABLE_MOTION=true ... --transport webrtc {flag} --live")
        sys.exit(1)

    print(f"[Config]")
    print(f"  Backend:       unitree")
    print(f"  Transport:     {args.transport}")
    print(f"  Model:         {settings.unitree_model or '(not set)'}")
    if args.transport == "sdk":
        print(f"  Net iface:     {settings.unitree_net_iface or '(auto)'}")
    if args.transport == "webrtc":
        print(f"  Robot IP:      {settings.unitree_robot_ip or '(default 192.168.123.161)'}")
        if settings.unitree_serial:
            print(f"  Serial:        {settings.unitree_serial}")
    print(f"  Dry-run:       {settings.unitree_dry_run}")
    print(f"  Enable motion: {settings.unitree_enable_motion}")
    print(f"  Motion mode:   {settings.unitree_motion_mode}")
    print(f"  Max speed:     {settings.unitree_max_speed} m/s")
    print(f"  Max step:      {settings.unitree_max_step} m")
    print()

    try:
        transport = await create_transport(args.transport, settings)
    except (RuntimeError, ConnectionError, TimeoutError) as exc:
        print(f"\n[ERROR] Transport initialization failed: {exc}")
        if args.transport in ("sdk", "webrtc"):
            print("[HINT] Check: dependencies installed, robot powered on, same network.")
            if args.transport == "sdk":
                print("[HINT] SDK needs: pip install git+https://github.com/unitreerobotics/unitree_sdk2_python.git")
                print("[HINT] Wi-Fi direct to Go2 hotspot; set RDB_UNITREE_NET_IFACE to your Wi-Fi interface (e.g. en0).")
            else:
                print("[HINT] WebRTC needs: pip install unitree-webrtc-connect")
                print("[HINT] Set RDB_UNITREE_ROBOT_IP to robot LAN IP (Unitree app), or RDB_UNITREE_SERIAL for discovery.")
                print("[HINT] Firmware >= 1.1.15 also needs UNITREE_AES_128_KEY.")
            print("[HINT] To run without real hardware: --transport fake")
        sys.exit(1)

    robot = UnitreeRobot(transport, settings)

    try:
        if args.drive:
            await run_drive_sequence(robot)
        elif args.actions:
            if args.transport == "webrtc":
                await run_posture_sequence(robot)
            else:
                await run_action_sequence(robot)
        else:
            await run_state_only(robot)
    except ConnectionError as exc:
        print(f"\n[ERROR] Connection failed: {exc}")
        print("[HINT] Check: robot powered on, Wi-Fi connected, correct network interface.")
        sys.exit(1)
    except RuntimeError as exc:
        print(f"\n[ERROR] Runtime error: {exc}")
        sys.exit(1)
    finally:
        # Leave the robot stopped after any action run (StopMove is always allowed).
        if args.actions or args.drive:
            await robot.stop("smoke test: shutdown")
        await transport.disconnect()


if __name__ == "__main__":
    asyncio.run(main())
