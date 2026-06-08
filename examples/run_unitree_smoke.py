"""Unitree smoke test — validate adapter connectivity and basic actions.

Usage:
    python examples/run_unitree_smoke.py                 # state-only (default)
    python examples/run_unitree_smoke.py --state-only    # only read state
    python examples/run_unitree_smoke.py --actions       # run minimal action sequence (dry-run)
    python examples/run_unitree_smoke.py --actions --live # run real actions (requires confirmation)
"""
from __future__ import annotations

import argparse
import asyncio
import sys

from config.settings import Settings
from robot_brain.actuation.unitree import (
    FakeUnitreeTransport,
    UnitreeRobot,
    UnitreeState,
)
from robot_brain.core.world_state import Position


CONFIRMATION_PHRASE = "I_UNDERSTAND_UNITREE_MOVE"


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
        action = entry.pop("action", "?")
        ts = entry.pop("timestamp", "")
        print(f"  {i}. {action} {entry}")

    print("\n[OK] Action sequence complete.")


async def main() -> None:
    parser = argparse.ArgumentParser(description="Unitree robot smoke test")
    parser.add_argument("--state-only", action="store_true", default=True, help="Only read state (default)")
    parser.add_argument("--actions", action="store_true", help="Run minimal action sequence")
    parser.add_argument("--live", action="store_true", help="Disable dry-run (send real commands)")
    args = parser.parse_args()

    settings = Settings(
        robot_backend="unitree",
        unitree_dry_run=not args.live,
    )

    print(f"[Config]")
    print(f"  Backend:    unitree")
    print(f"  Model:      {settings.unitree_model or '(not set)'}")
    print(f"  Dry-run:    {settings.unitree_dry_run}")
    print(f"  Max speed:  {settings.unitree_max_speed} m/s")
    print(f"  Max step:   {settings.unitree_max_step} m")
    print()

    # Use FakeTransport for now — replace with real SDK transport when available
    transport = FakeUnitreeTransport(
        initial_state=UnitreeState(
            connected=True,
            is_standing=True,
            battery_level=85.0,
            position=Position(x=0, y=0),
        )
    )
    await transport.connect()
    robot = UnitreeRobot(transport, settings)

    try:
        if args.actions:
            await run_action_sequence(robot)
        else:
            await run_state_only(robot)
    finally:
        await robot.stop("smoke test: shutdown")
        await transport.disconnect()


if __name__ == "__main__":
    asyncio.run(main())
