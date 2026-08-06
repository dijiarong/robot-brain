#!/usr/bin/env python3
"""Audit the complete extraction matrix without confusing offline readiness with completion."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MATRIX = ROOT/"docs/evidence/native-navigation-completion-matrix.json"


def audit(matrix_path: Path, evidence_dir: Path | None = None) -> dict[str, object]:
    raw = matrix_path.read_bytes()
    matrix = json.loads(raw)
    failures: list[str] = []
    capabilities = matrix.get("capabilities")
    if matrix.get("schema_version") != 1 or not isinstance(capabilities, list):
        raise ValueError("unsupported completion matrix schema")
    ids = [row.get("id") for row in capabilities]
    if len(ids) != len(set(ids)) or any(not value for value in ids):
        failures.append("capability ids must be unique and non-empty")
    rows = []
    all_pending = []
    for item in capabilities:
        missing = []
        for field in ("implementation", "tests", "acceptance"):
            values = item.get(field)
            if not isinstance(values, list) or not values:
                missing.append(f"{field}:empty")
                continue
            missing.extend(path for path in values if not (ROOT/path).exists())
        gates = item.get("external_gates")
        if not isinstance(gates, list):
            missing.append("external_gates:not_list")
            gates = []
        satisfied = []
        pending = []
        for gate in gates:
            matches = sorted(evidence_dir.glob(f"{gate}*.json")) if evidence_dir else []
            valid = []
            for path in matches:
                if _valid_external_gate_report(path, gate, evidence_dir):
                    valid.append(str(path))
            (satisfied if valid else pending).append({"gate": gate, "reports": valid})
        all_pending.extend(f"{item['id']}:{row['gate']}" for row in pending)
        if missing:
            failures.extend(f"{item['id']}:{value}" for value in missing)
        rows.append({
            "id": item["id"], "requirement": item.get("requirement"),
            "offline_artifacts_ok": not missing,
            "external_satisfied": satisfied, "external_pending": pending,
            "status": "external_pending" if pending else "evidence_ready",
        })
    return {
        "schema_version": 1,
        "matrix_sha256": hashlib.sha256(raw).hexdigest(),
        "offline_artifacts_ok": not failures,
        "complete": not failures and not all_pending,
        "capabilities": rows, "failures": failures,
        "external_pending": all_pending,
    }


def _valid_external_gate_report(path: Path, gate: str, evidence_dir: Path) -> bool:
    try:
        report = json.loads(path.read_text())
        if (report.get("schema_version") != 1 or report.get("gate_id") != gate
                or report.get("ok") is not True):
            return False
        artifacts = report.get("artifacts")
        if not isinstance(artifacts, list) or not artifacts:
            return False
        for item in artifacts:
            if not isinstance(item, dict) or not isinstance(item.get("path"), str):
                return False
            artifact = Path(item["path"])
            if not artifact.is_absolute():
                artifact = evidence_dir/artifact
            if not artifact.is_file():
                return False
            if hashlib.sha256(artifact.read_bytes()).hexdigest() != item.get("sha256"):
                return False
        return True
    except Exception:
        return False


def _run(command: list[str]) -> dict[str, object]:
    completed = subprocess.run(command, cwd=ROOT, text=True, capture_output=True)
    return {"command": command, "ok": completed.returncode == 0,
            "returncode": completed.returncode,
            "stdout_tail": completed.stdout[-2000:], "stderr_tail": completed.stderr[-2000:]}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--matrix", type=Path, default=DEFAULT_MATRIX)
    parser.add_argument("--external-evidence-dir", type=Path)
    parser.add_argument("--run-verifiers", action="store_true")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    report = audit(args.matrix, args.external_evidence_dir)
    if args.run_verifiers:
        checks = [
            _run([sys.executable, "-m", "pytest", "-q"]),
            _run([sys.executable, "scripts/verify_native_navigation_offline.py"]),
            _run([sys.executable, "scripts/audit_dimos_navigation_extraction.py",
                  "--dimos-root", "/Users/dijia/project/topsun_dimos", "--robot-root", str(ROOT)]),
        ]
        report["verifiers"] = checks
        report["offline_artifacts_ok"] = bool(report["offline_artifacts_ok"] and all(row["ok"] for row in checks))
        report["complete"] = bool(report["offline_artifacts_ok"] and not report["external_pending"])
    encoded = json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open("x", encoding="utf-8") as stream:
            stream.write(encoded)
    print(encoded, end="")
    raise SystemExit(0 if report["offline_artifacts_ok"] else 2)


if __name__ == "__main__":
    main()
