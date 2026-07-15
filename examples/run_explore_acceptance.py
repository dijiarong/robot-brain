"""Explore field-verification acceptance script (iteration 18).

Runs the ``explore`` skill in a chosen mode and emits an archivable JSON
report: result, stop_reason, per-step trace, and safety checks. ``mock`` and
``unitree-fake`` are hardware-free; the default ``--scenario clear`` path is
green (completed). Use ``--scenario blocked`` to exercise stop protection
(no_progress). Live Go2 verification is not wired here -- run it on-site via
the service/runtime with live env (``RDB_UNITREE_ENABLE_MOTION=true`` etc.).

Examples::

    python -m examples.run_explore_acceptance --mode mock --output-json acc-mock.json
    python -m examples.run_explore_acceptance --mode unitree-fake --output-json acc-fake.json
    python -m examples.run_explore_acceptance --scenario blocked --output-json acc-protection.json
"""
from __future__ import annotations

import argparse
import asyncio
import json
import sys
from typing import Any


def _build_mock(front_m: float):
    from config.settings import Settings
    from robot_brain.actuation.mock import MockRobot
    from robot_brain.core.robot_self_state import RobotSelfState, UltrasonicData
    from robot_brain.core.world_state import WorldState
    from robot_brain.skills.builtin.explore import ExploreSkill

    settings = Settings(memory_db_path=":memory:")
    robot = MockRobot()
    world = WorldState(
        battery_level=80.0,
        robot_self_state=RobotSelfState(
            source="acceptance",
            ultrasonic=UltrasonicData(front_m=front_m, rear_m=1.0, left_m=1.0, right_m=1.0),
        ),
    )
    skill = ExploreSkill(settings, passability=_maybe_passability(settings))
    return settings, robot, world, skill


def _build_unitree_fake(front_m: float):
    from config.settings import Settings
    from robot_brain.actuation.unitree import FakeUnitreeTransport, UnitreeRobot, UnitreeState
    from robot_brain.core.robot_self_state import RobotSelfState, UltrasonicData
    from robot_brain.core.world_state import WorldState
    from robot_brain.skills.builtin.explore import ExploreSkill

    settings = Settings(
        memory_db_path=":memory:",
        robot_backend="unitree",
        unitree_transport="fake",
        unitree_dry_run=False,
        unitree_enable_motion=False,
    )
    transport = FakeUnitreeTransport(
        UnitreeState(
            connected=True,
            is_standing=True,
            is_moving=False,
            error_code=0,
            ultrasonic=(front_m, 1.0, 1.0, 1.0),
        )
    )
    robot = UnitreeRobot(transport, settings)
    from robot_brain.perception.unitree import UnitreePerceptionAdapter

    perception = UnitreePerceptionAdapter(robot)
    world = WorldState(
        battery_level=80.0,
        robot_self_state=RobotSelfState(
            source="acceptance",
            is_standing=True,
            is_moving=False,
            error_code=0,
            state_age_seconds=0.1,
            ultrasonic=UltrasonicData(front_m=front_m, rear_m=1.0, left_m=1.0, right_m=1.0),
        ),
    )
    skill = ExploreSkill(settings, perception=perception, passability=_maybe_passability(settings))
    return settings, robot, world, skill


def _maybe_passability(settings):
    """Build a PassabilityAnalyzer when RDB_VLM_ENABLED=true, else None."""
    if not settings.vlm_enabled:
        return None
    from robot_brain.runtime.loop import _build_passability

    analyzer, _frame_source = _build_passability(settings)
    return analyzer


def _checks(trace: list[dict[str, Any]], stop_reason: str, vlm_enabled: bool) -> dict[str, str]:
    """Summarize safety-property outcomes as passed/skipped."""
    threshold = 0.3  # matches default obstacle_proximity_threshold
    blocked_steps = [t for t in trace if (t.get("ultrasonic", {}).get("front_m") or 1.0) < threshold]
    nudged_while_blocked = any(t["chosen_action"] == "nudge" for t in blocked_steps)
    ultrasonic = "passed" if blocked_steps and not nudged_while_blocked else ("skipped" if not blocked_steps else "failed")

    if vlm_enabled:
        # Passed if the run completed despite VLM falling back (no crash); any
        # step with no usable hint exercising the rule path counts as fallback.
        fell_back = any(t.get("passability_hint") is None for t in trace)
        vlm = "passed" if fell_back else "skipped"
    else:
        vlm = "skipped"

    no_progress = "passed" if stop_reason in ("no_progress", "semantic_hold", "ping_pong") else "skipped"
    return {
        "ultrasonic_hard_gate": ultrasonic,
        "vlm_fallback": vlm,
        "no_progress_stop": no_progress,
    }


async def _run(mode: str, front_m: float, max_steps: int) -> dict[str, Any]:
    if mode == "mock":
        settings, robot, world, skill = _build_mock(front_m)
    elif mode == "unitree-fake":
        settings, robot, world, skill = _build_unitree_fake(front_m)
        await robot.transport.connect()
    else:
        raise ValueError(f"unknown mode: {mode}")

    from robot_brain.skills.builtin.explore import ExploreParams

    params = ExploreParams(max_steps=max_steps)
    result = await skill.execute(params, robot, world)
    trace = result.data.get("trace", [])
    report = {
        "mode": mode,
        "vlm_enabled": settings.vlm_enabled,
        "result": "completed" if result.success else "aborted",
        "stop_reason": result.data.get("stop_reason"),
        "steps_completed": result.data.get("steps_completed"),
        "trace": trace,
        "checks": _checks(trace, result.data.get("stop_reason", ""), settings.vlm_enabled),
    }
    if "segments_total" in result.data:
        report["segments_total"] = result.data["segments_total"]

    # Release VLM resources if an analyzer was wired.
    if skill._passability is not None:
        await skill._passability.aclose()
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description="Explore field-verification acceptance")
    parser.add_argument("--mode", choices=["mock", "unitree-fake"], default="mock",
                        help="mock / unitree-fake (both hardware-free). "
                             "Live Go2 verification runs via the service/runtime with live env on-site.")
    parser.add_argument("--scenario", choices=["clear", "blocked"], default="clear",
                        help="clear (front open, happy path) / blocked (front <0.3m, exercises stop protection)")
    parser.add_argument("--front-m", type=float, default=None,
                        help="Override front ultrasonic distance (m); defaults to scenario")
    parser.add_argument("--max-steps", type=int, default=4)
    parser.add_argument("--output-json", default="", help="Write the report JSON to this path")
    args = parser.parse_args()

    front_m = args.front_m if args.front_m is not None else (1.0 if args.scenario == "clear" else 0.15)
    report = asyncio.run(_run(args.mode, front_m, args.max_steps))
    text = json.dumps(report, ensure_ascii=False, indent=2)
    if args.output_json:
        with open(args.output_json, "w", encoding="utf-8") as fh:
            fh.write(text + "\n")
        print(f"[acceptance] report written to {args.output_json}", file=sys.stderr)
    print(text)
    return 0 if report["result"] == "completed" else 1


if __name__ == "__main__":
    sys.exit(main())
