#!/usr/bin/env python3
"""Semantically validate external reports and bind them to a completion gate."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

try:
    from scripts.audit_native_navigation_completion import DEFAULT_MATRIX
except ModuleNotFoundError:  # direct ``python scripts/...`` execution
    from audit_native_navigation_completion import DEFAULT_MATRIX


def validate_gate(gate: str, reports: list[dict[str, object]]) -> tuple[bool, list[str]]:
    failures = []
    top_ok = [row for row in reports if row.get("ok") is True]

    def nested(key):
        return [row[key] for row in reports
                if isinstance(row.get(key), dict) and row[key].get("ok") is True]
    if gate == "go2_read_only_sensor_report":
        valid = any(row.get("mode") == "read_only" and row.get("stop_reason") == "read_only_complete"
                    and isinstance(row.get("sensors"), dict) and row["sensors"].get("ready") is True
                    for row in top_ok)
    elif gate == "go2_live_motion_suite":
        required = {"read_only", "straight", "obstacle", "cancel", "sudden_block", "stuck"}
        valid = any(set(row.get("scenarios", {})) == required for row in top_ok)
    elif gate == "go2_mapping_replay":
        valid = any(row.get("reason") == "mapped" and row.get("frames_integrated", 0) > 0
                    and row.get("voxel_count", 0) > 0 for row in top_ok)
    elif gate == "go2_relocalization_replay":
        valid = any(row.get("reason") == "accepted" and row.get("fitness", 0) > 0
                    and row.get("rmse_m") is not None and isinstance(row.get("map_identity"), dict)
                    for row in top_ok)
    elif gate == "go2_closed_loop_replay":
        valid = any(row.get("accepted_loops", 0) > 0 and row.get("reason") == "accepted_loop_observed"
                    for row in top_ok)
    elif gate == "go2_frontier_exploration_trace":
        valid = bool(nested("exploration"))
    elif gate == "go2_four_strategy_patrol_traces":
        valid = {row.get("strategy") for row in nested("patrol")} == {
            "coverage", "frontier", "random", "least_visited",
        }
    elif gate == "go2_visual_servo_trace":
        valid = bool(nested("visual_servo"))
    elif gate == "go2_mid360_terrain_replay":
        valid = any(row.get("input_kind") == "native_navigation_replay"
                    and isinstance(row.get("safety"), dict) and row["safety"].get("ok") is True
                    for row in top_ok)
    elif gate == "go2_terrain_execution_trace":
        valid = bool(nested("terrain_execution"))
    elif gate == "go2_tare_exploration_trace":
        valid = bool(nested("terrain_exploration"))
    elif gate == "go2_live_teleop_estop_arbitration":
        valid = any(row.get("mode") == "live" and all(row.get(key) is True for key in (
            "navigation_preempted", "teleop_lease_granted", "estop_stopped",
            "lease_invalidated", "motion_rejected_during_estop",
        )) for row in top_ok)
    elif gate == "native_service_integration_report":
        valid = any(row.get("provider") == "native_go2" and all(
            row.get(key) is True for key in (
                "service_health", "skill_invocation", "motion_gate_enforced",
            )) for row in top_ok)
    else:
        valid = False
        failures.append("unsupported_gate")
    if not valid and not failures:
        failures.append("gate_semantics_not_satisfied")
    return valid, failures


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("gate_id")
    parser.add_argument("artifacts", nargs="+", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    matrix = json.loads(DEFAULT_MATRIX.read_text())
    allowed = {gate for row in matrix["capabilities"] for gate in row["external_gates"]}
    if args.gate_id not in allowed:
        parser.error("gate_id is not declared in the completion matrix")
    reports, artifact_rows = [], []
    for path in args.artifacts:
        raw = path.read_bytes()
        artifact_rows.append({"path": str(path.resolve()),
                              "sha256": hashlib.sha256(raw).hexdigest()})
        if path.suffix == ".json":
            try:
                reports.append(json.loads(raw))
            except (UnicodeDecodeError, json.JSONDecodeError):
                pass
    ok, failures = validate_gate(args.gate_id, reports)
    result = {"schema_version": 1, "gate_id": args.gate_id, "ok": ok,
              "artifacts": artifact_rows, "failures": failures}
    encoded = json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True) + "\n"
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x", encoding="utf-8") as stream:
        stream.write(encoded)
    print(encoded, end="")
    raise SystemExit(0 if ok else 3)


if __name__ == "__main__":
    main()
