#!/usr/bin/env python3
"""Create a non-executing, immutable field-acceptance run manifest."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import uuid

try:
    from scripts.audit_native_navigation_completion import DEFAULT_MATRIX, audit
except ModuleNotFoundError:  # direct ``python scripts/...`` execution
    from audit_native_navigation_completion import DEFAULT_MATRIX, audit


BATCHES = {
    "live_safety": {
        "gates": ["go2_read_only_sensor_report", "go2_live_motion_suite",
                  "go2_live_teleop_estop_arbitration"],
        "operator_setup": "clear motion area; soft obstacle; physical estop operator; six live scenarios",
        "references": ["docs/native-navigation-live-acceptance.md"],
    },
    "mapping_localization_loop": {
        "gates": ["go2_mapping_replay", "go2_relocalization_replay",
                  "go2_closed_loop_replay"],
        "operator_setup": "record mapped route, restart near known initial pose, then record >20 s closed loop",
        "references": ["docs/native-navigation-mapping-replay-acceptance.md"],
    },
    "exploration_patrol_visual": {
        "gates": ["go2_frontier_exploration_trace", "go2_four_strategy_patrol_traces",
                  "go2_visual_servo_trace"],
        "operator_setup": "partially mapped safe area; four patrol strategies; known-size visual target",
        "references": ["docs/native-navigation-exploration-patrol-acceptance.md",
                       "docs/native-navigation-visual-servo-acceptance.md"],
    },
    "terrain_tare": {
        "gates": ["go2_mid360_terrain_replay", "go2_terrain_execution_trace",
                  "go2_tare_exploration_trace"],
        "operator_setup": "bounded non-flat area with measured step/slope and MCF motion mode",
        "references": ["docs/native-navigation-terrain3d-acceptance.md"],
    },
}

GATE_COMMANDS = {
    "go2_read_only_sensor_report": [
        "python scripts/verify_native_go2_navigation.py --report-path <RUN_DIR>/read-only.json",
        "python scripts/register_native_navigation_evidence.py go2_read_only_sensor_report <RUN_DIR>/read-only.json --output <RUN_DIR>/go2_read_only_sensor_report-registered.json",
    ],
    "go2_live_motion_suite": [
        "run the five --live scenarios in docs/native-navigation-live-acceptance.md into <RUN_DIR>",
        "python scripts/summarize_native_go2_acceptance.py <SIX_REPORTS> --output <RUN_DIR>/motion-suite.json",
        "python scripts/register_native_navigation_evidence.py go2_live_motion_suite <RUN_DIR>/motion-suite.json <SIX_REPORTS> --output <RUN_DIR>/go2_live_motion_suite-registered.json",
    ],
    "go2_live_teleop_estop_arbitration": [
        "python scripts/verify_native_go2_arbitration.py --live --confirm I_UNDERSTAND_GO2_CONTROL_ARBITRATION --output <RUN_DIR>/arbitration.json",
        "python scripts/register_native_navigation_evidence.py go2_live_teleop_estop_arbitration <RUN_DIR>/arbitration.json --output <RUN_DIR>/go2_live_teleop_estop_arbitration-registered.json",
    ],
    "go2_mapping_replay": [
        "python scripts/verify_native_mapping_replay.py mapping <MAPPING_REPLAY> --output <RUN_DIR>/mapping.json",
        "python scripts/register_native_navigation_evidence.py go2_mapping_replay <RUN_DIR>/mapping.json <MAPPING_REPLAY> --output <RUN_DIR>/go2_mapping_replay-registered.json",
    ],
    "go2_relocalization_replay": [
        "python scripts/verify_native_mapping_replay.py relocalization <RELOCALIZATION_REPLAY> --map <MAP> --initial-x <X> --initial-y <Y> --initial-yaw <YAW> --output <RUN_DIR>/relocalization.json",
        "python scripts/register_native_navigation_evidence.py go2_relocalization_replay <RUN_DIR>/relocalization.json <RELOCALIZATION_REPLAY> <MAP> --output <RUN_DIR>/go2_relocalization_replay-registered.json",
    ],
    "go2_closed_loop_replay": [
        "python scripts/verify_native_mapping_replay.py loop_closure <CLOSED_LOOP_REPLAY> --output <RUN_DIR>/loop.json",
        "python scripts/register_native_navigation_evidence.py go2_closed_loop_replay <RUN_DIR>/loop.json <CLOSED_LOOP_REPLAY> --output <RUN_DIR>/go2_closed_loop_replay-registered.json",
    ],
    "go2_frontier_exploration_trace": [
        "python scripts/analyze_native_navigation.py <EXPLORATION_TRACE> --output <RUN_DIR>/exploration.json",
        "python scripts/register_native_navigation_evidence.py go2_frontier_exploration_trace <RUN_DIR>/exploration.json <EXPLORATION_TRACE> --output <RUN_DIR>/go2_frontier_exploration_trace-registered.json",
    ],
    "go2_four_strategy_patrol_traces": [
        "analyze four separate coverage/frontier/random/least_visited traces into four JSON reports",
        "python scripts/register_native_navigation_evidence.py go2_four_strategy_patrol_traces <FOUR_REPORTS> <FOUR_TRACES> --output <RUN_DIR>/go2_four_strategy_patrol_traces-registered.json",
    ],
    "go2_visual_servo_trace": [
        "python scripts/analyze_native_navigation.py <VISUAL_TRACE> --output <RUN_DIR>/visual.json",
        "python scripts/register_native_navigation_evidence.py go2_visual_servo_trace <RUN_DIR>/visual.json <VISUAL_TRACE> --output <RUN_DIR>/go2_visual_servo_trace-registered.json",
    ],
    "go2_mid360_terrain_replay": [
        "python scripts/verify_native_terrain3d.py <TERRAIN_REPLAY> --output <RUN_DIR>/terrain.json",
        "python scripts/register_native_navigation_evidence.py go2_mid360_terrain_replay <RUN_DIR>/terrain.json <TERRAIN_REPLAY> --output <RUN_DIR>/go2_mid360_terrain_replay-registered.json",
    ],
    "go2_terrain_execution_trace": [
        "python scripts/analyze_native_navigation.py <TERRAIN_EXECUTION_TRACE> --output <RUN_DIR>/terrain-execution.json",
        "python scripts/register_native_navigation_evidence.py go2_terrain_execution_trace <RUN_DIR>/terrain-execution.json <TERRAIN_EXECUTION_TRACE> --output <RUN_DIR>/go2_terrain_execution_trace-registered.json",
    ],
    "go2_tare_exploration_trace": [
        "python scripts/analyze_native_navigation.py <TARE_TRACE> --output <RUN_DIR>/tare.json",
        "python scripts/register_native_navigation_evidence.py go2_tare_exploration_trace <RUN_DIR>/tare.json <TARE_TRACE> --output <RUN_DIR>/go2_tare_exploration_trace-registered.json",
    ],
}


def build_manifest(*, evidence_dir: Path | None = None, run_id: str | None = None):
    completion = audit(DEFAULT_MATRIX, evidence_dir)
    pending = sorted({row.split(":", 1)[1] for row in completion["external_pending"]})
    batches = []
    assigned = set()
    for batch_id, template in BATCHES.items():
        gates = [gate for gate in template["gates"] if gate in pending]
        if not gates:
            continue
        assigned.update(gates)
        batches.append({"id": batch_id, "status": "pending", "gates": gates,
                        "operator_setup": template["operator_setup"],
                        "references": template["references"]})
    unassigned = sorted(set(pending)-assigned)
    return {
        "schema_version": 1,
        "run_id": run_id or f"native-nav-{datetime.now(timezone.utc):%Y%m%dT%H%M%SZ}-{uuid.uuid4().hex[:8]}",
        "created_at": datetime.now(timezone.utc).isoformat(),
        "executes_motion": False,
        "matrix_sha256": completion["matrix_sha256"],
        "pending_gate_references": len(completion["external_pending"]),
        "unique_pending_gates": pending,
        "batches": batches, "unassigned_gates": unassigned,
        "gate_commands": {gate: GATE_COMMANDS[gate] for gate in pending},
        "completion_command": (
            "python scripts/audit_native_navigation_completion.py "
            "--run-verifiers --external-evidence-dir <RUN_DIR>"
        ),
    }


def _runbook(manifest) -> str:
    lines = ["# Native navigation field acceptance", "",
             f"Run ID: `{manifest['run_id']}`", "",
             "This manifest does not execute motion. Follow each referenced runbook; every live command still requires its explicit confirmation and motion gate.", ""]
    for index, batch in enumerate(manifest["batches"], 1):
        lines += [f"## {index}. {batch['id']}", "", batch["operator_setup"], "",
                  "Gates:", ""]
        lines += [f"- `{gate}`" for gate in batch["gates"]]
        lines += ["", "Commands / evidence registration:", ""]
        for gate in batch["gates"]:
            lines.append(f"- `{gate}`")
            lines += [f"  - `{command}`" for command in manifest["gate_commands"][gate]]
        lines += ["", "References:", ""]
        lines += [f"- `{path}`" for path in batch["references"]]
        lines.append("")
    lines += ["## Final audit", "", f"`{manifest['completion_command']}`", ""]
    return "\n".join(lines)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("output_dir", type=Path)
    parser.add_argument("--existing-evidence-dir", type=Path)
    args = parser.parse_args()
    manifest = build_manifest(evidence_dir=args.existing_evidence_dir)
    args.output_dir.mkdir(parents=True, exist_ok=False)
    (args.output_dir/"manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2, sort_keys=True)+"\n"
    )
    (args.output_dir/"RUNBOOK.md").write_text(_runbook(manifest))
    print(json.dumps({"ok": not manifest["unassigned_gates"],
                      "run_id": manifest["run_id"],
                      "output_dir": str(args.output_dir),
                      "batches": len(manifest["batches"]),
                      "unique_pending_gates": len(manifest["unique_pending_gates"])},
                     ensure_ascii=False, sort_keys=True))
    raise SystemExit(0 if not manifest["unassigned_gates"] else 2)


if __name__ == "__main__":
    main()
