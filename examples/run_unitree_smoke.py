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
    # Graded live acceptance (webrtc, levels 0-5):
    RDB_UNITREE_ENABLE_MOTION=true python -m examples.run_unitree_smoke \\
        --transport webrtc --graded --live --level 0
"""
from __future__ import annotations

import argparse
import asyncio
import json
import logging
import sys
import time

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
        print("[WARNING] DRY-RUN IS OFF. Real actions will be sent to the robot.")
        print("[WARNING] Ensure: open space, safe distance, operator has e-stop ready.")
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

    print("[WARNING] Real posture commands will be sent to the robot.")
    print("[WARNING] Ensure: flat ground, clear space, operator can power off.")
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


GRADED_CONFIRMATION = "I_UNDERSTAND_UNITREE_GRADED_ACCEPTANCE"


class AcceptanceSummary:
    """Collect graded acceptance results for optional export."""

    def __init__(self) -> None:
        self.levels: list[dict[str, object]] = []
        self.started_at: float = time.time()

    def record(
        self,
        level: int,
        name: str,
        passed: bool,
        detail: str = "",
        *,
        pre_state: dict[str, object] | None = None,
        post_state: dict[str, object] | None = None,
        audit: list[dict[str, object]] | None = None,
    ) -> None:
        self.levels.append(
            {
                "level": level,
                "name": name,
                "passed": passed,
                "detail": detail,
                "timestamp": time.time(),
                "pre_state": pre_state,
                "post_state": post_state,
                "audit": audit or [],
            }
        )
        status = "PASS" if passed else "FAIL"
        print(f"\n[Level {level}] {name}: {status}")
        if detail:
            print(f"  {detail}")

    def to_dict(self) -> dict[str, object]:
        passed = all(row["passed"] for row in self.levels) if self.levels else False
        return {
            "started_at": self.started_at,
            "finished_at": time.time(),
            "passed": passed,
            "levels": self.levels,
        }

    def print_summary(self) -> None:
        print("\n" + "=" * 50)
        print("  Graded Acceptance Summary")
        print("=" * 50)
        for row in self.levels:
            mark = "OK" if row["passed"] else "FAIL"
            print(f"  L{row['level']} {row['name']}: {mark}")
        print()


async def _robot_state_snapshot(robot: UnitreeRobot) -> dict[str, object]:
    state = await robot.get_state()
    return state.model_dump(mode="json")


def _audit_since(robot: UnitreeRobot, since: float) -> list[dict[str, object]]:
    return [
        {k: v for k, v in entry.items()}
        for entry in robot.action_history
        if entry.get("timestamp", 0) >= since
    ]


async def run_graded_level(
    level: int,
    robot: UnitreeRobot,
    summary: AcceptanceSummary,
    *,
    level0_seconds: float = 60.0,
) -> bool:
    """Run a single acceptance level. Returns False on failure (stops progression)."""
    level_start = time.time()
    pre_state = await _robot_state_snapshot(robot)

    if level == 0:
        print(f"[Level 0] Read-only stability ({level0_seconds:.0f}s polling)...")
        start = time.time()
        reads = 0
        while time.time() - start < level0_seconds:
            state = await robot.get_state()
            reads += 1
            await asyncio.sleep(2.0)
        post_state = state.model_dump(mode="json")
        summary.record(
            0,
            "read-only 60s",
            True,
            f"{reads} state reads",
            pre_state=pre_state,
            post_state=post_state,
            audit=_audit_since(robot, level_start),
        )
        return True

    if level == 1:
        print("[Level 1] Stop-only (repeat 3x)...")
        for i in range(3):
            await robot.stop(f"graded L1 stop {i + 1}")
            await asyncio.sleep(0.5)
        summary.record(
            1,
            "stop-only",
            True,
            "3x stop issued",
            pre_state=pre_state,
            post_state=await _robot_state_snapshot(robot),
            audit=_audit_since(robot, level_start),
        )
        return True

    if level == 2:
        print("[Level 2] Posture sequence + omni teleop enable...")
        for posture in ("stand_up", "balance_stand", "free_walk"):
            await robot.set_posture(posture)
            await asyncio.sleep(2.0)
        await robot.enable_omni_teleop()
        await asyncio.sleep(1.0)
        summary.record(
            2,
            "posture",
            True,
            "stand_up + balance_stand + free_walk + SwitchJoystick",
            pre_state=pre_state,
            post_state=await _robot_state_snapshot(robot),
            audit=_audit_since(robot, level_start),
        )
        return True

    if level == 3:
        print("[Level 3] In-place yaw nudges...")
        await robot.drive(vyaw=0.3, duration=0.5)
        await asyncio.sleep(1.0)
        await robot.drive(vyaw=-0.3, duration=0.5)
        summary.record(
            3,
            "yaw nudge",
            True,
            "±0.3 rad/s × 0.5s",
            pre_state=pre_state,
            post_state=await _robot_state_snapshot(robot),
            audit=_audit_since(robot, level_start),
        )
        return True

    if level == 4:
        print("[Level 4] Forward/back nudges...")
        await robot.drive(vx=0.2, duration=0.5)
        await asyncio.sleep(1.0)
        await robot.drive(vx=-0.2, duration=0.5)
        summary.record(
            4,
            "linear nudge",
            True,
            "±0.2 m/s × 0.5s",
            pre_state=pre_state,
            post_state=await _robot_state_snapshot(robot),
            audit=_audit_since(robot, level_start),
        )
        return True

    if level == 5:
        print("[Level 5] Stop during motion...")
        task = asyncio.create_task(robot.drive(vx=0.2, duration=1.0))
        await asyncio.sleep(0.2)
        await robot.stop("graded L5 preempt")
        await task
        summary.record(
            5,
            "stop preempt",
            True,
            "stop during active drive",
            pre_state=pre_state,
            post_state=await _robot_state_snapshot(robot),
            audit=_audit_since(robot, level_start),
        )
        return True

    print(f"[ERROR] Unknown level {level}")
    return False


async def run_graded_acceptance(
    robot: UnitreeRobot,
    max_level: int,
    *,
    level0_seconds: float = 60.0,
    output_json: str | None = None,
) -> None:
    summary = AcceptanceSummary()
    print("[Phase] Graded live acceptance (webrtc)")
    print(f"  Levels 0..{max_level}")
    print()

    confirmation = input(f"Type '{GRADED_CONFIRMATION}' to proceed: ").strip()
    if confirmation != GRADED_CONFIRMATION:
        print("[ABORT] Confirmation not received.")
        sys.exit(1)

    for level in range(max_level + 1):
        try:
            ok = await run_graded_level(
                level, robot, summary, level0_seconds=level0_seconds
            )
        except Exception as exc:
            summary.record(
                level,
                f"level-{level}",
                False,
                str(exc),
                post_state=await _robot_state_snapshot(robot),
                audit=_audit_since(robot, summary.started_at),
            )
            print(f"\n[HALT] Level {level} failed: {exc}")
            break
        if not ok:
            break
    else:
        print("\n[OK] All requested levels passed.")

    summary.print_summary()
    payload = summary.to_dict()
    print(json.dumps(payload, indent=2))
    if output_json:
        with open(output_json, "w", encoding="utf-8") as fh:
            json.dump(payload, fh, indent=2)
        print(f"\n[Saved] Graded summary written to {output_json}")


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
    parser.add_argument(
        "--graded", action="store_true",
        help="Run graded live acceptance levels (webrtc + live + motion gate)",
    )
    parser.add_argument(
        "--level", type=int, default=5,
        help="Highest graded level to run (0-5, default 5)",
    )
    parser.add_argument(
        "--level0-seconds", type=float, default=60.0,
        help="Level 0 read-only polling duration (default 60)",
    )
    parser.add_argument(
        "--output-json",
        metavar="PATH",
        help="Write graded acceptance summary JSON to PATH",
    )
    args = parser.parse_args()

    if args.graded:
        args.state_only = False
        if args.transport != "webrtc":
            print("[ERROR] --graded requires --transport webrtc")
            sys.exit(1)
        if not args.live:
            print("[ERROR] --graded requires --live")
            sys.exit(1)
        if args.level < 0 or args.level > 5:
            print("[ERROR] --level must be 0-5")
            sys.exit(1)

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

    if args.transport == "webrtc" and (args.actions or args.drive or args.graded) and not settings.unitree_enable_motion:
        kind = "posture sequence" if args.actions else "drive demo" if args.drive else "graded acceptance"
        flag = "--actions" if args.actions else "--drive" if args.drive else "--graded"
        print(f"[ERROR] webrtc {kind} requires RDB_UNITREE_ENABLE_MOTION=true.")
        print("        This is a hard safety gate for real motion. Export it explicitly:")
        print(f"        RDB_UNITREE_ENABLE_MOTION=true ... --transport webrtc {flag} --live")
        sys.exit(1)

    print("[Config]")
    print("  Backend:       unitree")
    print(f"  Transport:     {args.transport}")
    print(f"  Model:         {settings.unitree_model or '(not set)'}")
    if args.transport == "sdk":
        print(f"  Net iface:     {settings.unitree_net_iface or '(auto)'}")
    if args.transport == "webrtc":
        connection_mode = settings.unitree_webrtc_connection_mode.lower()
        if connection_mode == "remote" or (
            connection_mode == "auto"
            and not settings.unitree_robot_ip
            and settings.unitree_cloud_username
            and settings.unitree_cloud_password
        ):
            print(f"  Connection:    cloud remote ({settings.unitree_cloud_region})")
        else:
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
                print("[HINT] LAN: set RDB_UNITREE_ROBOT_IP; cloud: set Remote mode, serial, account, password, and region.")
                print("[HINT] Firmware >= 1.1.15 also needs UNITREE_AES_128_KEY.")
            print("[HINT] To run without real hardware: --transport fake")
        sys.exit(1)

    robot = UnitreeRobot(transport, settings)

    try:
        if args.graded:
            await run_graded_acceptance(
                robot,
                args.level,
                level0_seconds=args.level0_seconds,
                output_json=args.output_json,
            )
        elif args.drive:
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
        if args.actions or args.drive or args.graded:
            await robot.stop("smoke test: shutdown")
        await transport.disconnect()


if __name__ == "__main__":
    asyncio.run(main())
