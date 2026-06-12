"""Operator teleop for Unitree Go2 — discrete low-speed nudges only.

Not exposed to LLM, skills, or the service API. Each command is a fixed
duration nudge within configured safety clamps.

Usage:
    python -m examples.run_unitree_teleop --transport fake
    RDB_UNITREE_ENABLE_MOTION=true python -m examples.run_unitree_teleop \\
        --transport webrtc --live --robot-ip 192.168.x.x
"""
from __future__ import annotations

import argparse
import asyncio
import logging
import sys

from config.settings import Settings
from examples.run_unitree_smoke import create_transport
from robot_brain.actuation.unitree import UnitreeRobot

TELEOP_CONFIRMATION = "I_UNDERSTAND_UNITREE_TELEOP"

# Default nudge recipe (iteration-9 live limits: 0.2 m/s, 0.3 rad/s, 0.5 s).
# Omni: A/D strafe, Q/E yaw.
_DEFAULT_NUDGES: dict[str, dict[str, float]] = {
    "w": {"vx": 0.2, "duration": 0.5},
    "s": {"vx": -0.2, "duration": 0.5},
    "a": {"vy": 0.2, "duration": 0.5},
    "d": {"vy": -0.2, "duration": 0.5},
    "q": {"vyaw": 0.3, "duration": 0.5},
    "e": {"vyaw": -0.3, "duration": 0.5},
}
_STRONG_NUDGES: dict[str, dict[str, float]] = {
    "w": {"vx": 0.35, "duration": 0.8},
    "s": {"vx": -0.35, "duration": 0.8},
    "a": {"vy": 0.35, "duration": 0.8},
    "d": {"vy": -0.35, "duration": 0.8},
    "q": {"vyaw": 0.35, "duration": 0.8},
    "e": {"vyaw": -0.35, "duration": 0.8},
}
# Car-style (web default): A/D turn, Q/E strafe — W+A / W+D arc while moving.
_CAR_NUDGES: dict[str, dict[str, float]] = {
    "w": {"vx": 0.2, "duration": 0.5},
    "s": {"vx": -0.2, "duration": 0.5},
    "a": {"vyaw": 0.3, "duration": 0.5},
    "d": {"vyaw": -0.3, "duration": 0.5},
    "q": {"vy": 0.2, "duration": 0.5},
    "e": {"vy": -0.2, "duration": 0.5},
}
_CAR_STRONG_NUDGES: dict[str, dict[str, float]] = {
    "w": {"vx": 0.35, "duration": 0.8},
    "s": {"vx": -0.35, "duration": 0.8},
    "a": {"vyaw": 0.35, "duration": 0.8},
    "d": {"vyaw": -0.35, "duration": 0.8},
    "q": {"vy": 0.35, "duration": 0.8},
    "e": {"vy": -0.35, "duration": 0.8},
}

_SPORT_MODE_LABELS = {
    0: "idle/stand",
    1: "balanceStand",
    3: "locomotion",
    5: "lieDown",
    7: "damping",
}

POSTURES: dict[str, str] = {
    "u": "stand_up",
    "b": "balance_stand",
    "f": "free_walk",
    "l": "stand_down",
}


async def prep_locomotion(robot: UnitreeRobot) -> None:
    """DimOS-style wake: stand_up → balance_stand → free_walk → omni teleop enable."""
    print("[Prep] stand_up → balance_stand → free_walk → SwitchJoystick ...")
    await robot.set_posture("stand_up")
    await asyncio.sleep(3.0)
    await robot.set_posture("balance_stand")
    await asyncio.sleep(2.0)
    await robot.set_posture("free_walk")
    await asyncio.sleep(2.0)
    await robot.enable_omni_teleop()
    await asyncio.sleep(1.0)
    await robot.get_state()
    raw = robot.action_history[-1].get("raw", {})
    mode = raw.get("sport_mode")
    label = _SPORT_MODE_LABELS.get(mode, "?") if mode is not None else "?"
    print(f"[Prep] ready — sport_mode={mode} ({label})")


def print_banner(
    settings: Settings, transport: str, live: bool, *, strong: bool, prep_stand: bool
) -> None:
    print("[Unitree Teleop]")
    print(f"  Transport:     {transport}")
    print(f"  Robot IP:      {settings.unitree_robot_ip or '(default)'}")
    print(f"  Dry-run:       {settings.unitree_dry_run}")
    print(f"  Enable motion: {settings.unitree_enable_motion}")
    print(f"  Nudge profile: {'strong (0.35 / 0.8s)' if strong else 'default (0.2 / 0.5s)'}")
    print(f"  Max speed:     {settings.unitree_max_speed} m/s")
    print(f"  Max yaw:       {settings.unitree_max_yaw_speed} rad/s")
    print(f"  Max duration:  {settings.unitree_max_drive_duration} s")
    print(f"  Prep stand:    {prep_stand}")
    print()
    print("Keys: w/s forward/back  a/d strafe  q/e yaw  u/b/f stand  l lie  i=status")
    print("      space=STOP  x=quit  (try w before d — forward is easier to see)")
    if live:
        print("[WARNING] Live mode — robot WILL move on nudge keys.")
    print()


async def repl(robot: UnitreeRobot, nudges: dict[str, dict[str, float]]) -> None:
    loop = asyncio.get_event_loop()
    while True:
        try:
            key = await loop.run_in_executor(None, lambda: input("> ").strip().lower())
        except (EOFError, KeyboardInterrupt):
            key = "x"
        if key in ("x", "quit", "exit"):
            break
        if key in ("", "h", "?"):
            print("w/s/a/d/q/e = nudge  u/b/f = posture  i = status  space = stop  x = quit")
            continue
        if key == "i":
            state = await robot.get_state()
            raw = robot.action_history[-1].get("raw", {}) if robot.action_history else {}
            mode = raw.get("sport_mode")
            mode_label = _SPORT_MODE_LABELS.get(mode, "?") if mode is not None else "?"
            print(
                f"  battery={state.battery_level:.0f}% moving={not state.stopped} "
                f"sport_mode={mode} ({mode_label}) "
                f"standing={raw.get('is_standing')} error_code={raw.get('error_code', 0)}"
            )
            continue
        if key in POSTURES:
            posture = POSTURES[key]
            try:
                await robot.set_posture(posture)
                print(f"  [posture] {posture} sent")
            except Exception as exc:
                print(f"  [posture failed] {exc}")
            continue
        if key in (" ", "stop"):
            await robot.stop("operator teleop stop")
            print("  [stop sent]")
            continue
        if key not in nudges:
            print(f"  unknown key: {key!r}")
            continue
        try:
            await robot.drive(**nudges[key])
            entry = robot.action_history[-1]
            reason = entry.get("end_reason", "?")
            elapsed = entry.get("elapsed", "?")
            if reason == "dry_run":
                print(f"  [dry-run] vx={entry.get('vx')} dur={entry.get('duration')}s (no command sent)")
            else:
                print(f"  [nudge ok] end_reason={reason} elapsed={elapsed:.3f}s")
        except Exception as exc:
            print(f"  [nudge failed] {exc}")


async def main() -> None:
    logging.basicConfig(level=logging.WARNING)
    logging.getLogger("robot_brain").setLevel(logging.INFO)

    parser = argparse.ArgumentParser(description="Unitree Go2 operator teleop (discrete nudges)")
    parser.add_argument(
        "--transport", choices=["fake", "webrtc"], default="fake",
        help="fake (default) or webrtc for real robot",
    )
    parser.add_argument("--live", action="store_true", help="Disable dry-run (real commands)")
    parser.add_argument("--robot-ip", default="", help="Go2 LAN IP (RDB_UNITREE_ROBOT_IP)")
    parser.add_argument(
        "--prep-stand", action="store_true", default=None,
        help="On connect: stand_up then balance_stand (default for --live)",
    )
    parser.add_argument(
        "--no-prep-stand", action="store_true",
        help="Skip automatic stand_up → balance_stand on connect",
    )
    parser.add_argument(
        "--strong", action="store_true",
        help="Use stronger nudges (0.35 m/s / 0.8 s) for clearer real-hardware motion",
    )
    args = parser.parse_args()

    if args.no_prep_stand:
        prep_stand = False
    elif args.prep_stand:
        prep_stand = True
    else:
        prep_stand = args.live

    if args.transport == "webrtc" and not args.live:
        print("[ERROR] webrtc teleop requires --live")
        sys.exit(1)

    settings_kwargs: dict = dict(
        robot_backend="unitree",
        unitree_transport=args.transport,
        unitree_dry_run=not args.live,
    )
    if args.robot_ip:
        settings_kwargs["unitree_robot_ip"] = args.robot_ip
    if args.strong and args.live:
        settings_kwargs.update(
            unitree_max_speed=0.35,
            unitree_max_yaw_speed=0.35,
            unitree_max_drive_duration=0.8,
        )
    settings = Settings(**settings_kwargs)
    nudges = _STRONG_NUDGES if args.strong else _DEFAULT_NUDGES

    if args.transport == "webrtc" and args.live and not settings.unitree_enable_motion:
        print("[ERROR] Live webrtc teleop requires RDB_UNITREE_ENABLE_MOTION=true")
        sys.exit(1)

    if args.live:
        print("[WARNING] Real motion enabled. Flat ground, clear space, operator ready to stop.")
        confirm = input(f"Type '{TELEOP_CONFIRMATION}' to proceed: ").strip()
        if confirm != TELEOP_CONFIRMATION:
            print("[ABORT]")
            sys.exit(1)

    print_banner(settings, args.transport, args.live, strong=args.strong, prep_stand=prep_stand)

    try:
        transport = await create_transport(args.transport, settings)
    except (RuntimeError, ConnectionError, TimeoutError) as exc:
        print(f"[ERROR] Connect failed: {exc}")
        if args.transport == "webrtc":
            print("[HINT] Find robot IP: Unitree App, or `dimos go2tool discover`")
            print("[HINT] export RDB_UNITREE_ROBOT_IP=<ip>  (also reads DIMOS_ROBOT_IP / ROBOT_IP)")
            print("[HINT] New firmware: export UNITREE_AES_128_KEY=<32-hex>")
        sys.exit(1)

    robot = UnitreeRobot(transport, settings)
    if args.live and prep_stand:
        await prep_locomotion(robot)
    try:
        await repl(robot, nudges)
    finally:
        await robot.stop("teleop shutdown")
        await transport.disconnect()
        print("[OK] Teleop session ended.")


if __name__ == "__main__":
    asyncio.run(main())
