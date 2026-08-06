#!/usr/bin/env python3
"""Validate and summarize immutable native Go2 live-acceptance reports."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path


REQUIRED = frozenset({"read_only", "straight", "obstacle", "cancel", "sudden_block", "stuck"})


def summarize(paths: list[Path]) -> dict[str, object]:
    scenarios: dict[str, dict[str, object]] = {}
    failures: list[str] = []
    for path in paths:
        raw = path.read_bytes()
        try:
            report = json.loads(raw)
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            failures.append(f"{path}: invalid JSON: {exc}")
            continue
        scenario = "read_only" if report.get("mode") == "read_only" else report.get("scenario")
        if scenario not in REQUIRED:
            failures.append(f"{path}: unknown scenario {scenario!r}")
            continue
        if scenario in scenarios:
            failures.append(f"duplicate scenario: {scenario}")
            continue
        item = {
            "path": str(path),
            "sha256": hashlib.sha256(raw).hexdigest(),
            "ok": report.get("ok") is True,
            "provider": report.get("provider"),
            "mode": report.get("mode"),
            "stop_reason": report.get("stop_reason"),
            "started_at": report.get("started_at"),
            "acceptance_failures": (report.get("acceptance") or {}).get("failures", []),
        }
        scenarios[str(scenario)] = item
        if item["provider"] != "native_go2":
            failures.append(f"{scenario}: provider is not native_go2")
        if not item["ok"]:
            failures.append(f"{scenario}: report did not pass")
        if scenario == "read_only":
            sensors = report.get("sensors") or {}
            if not sensors.get("ready") or report.get("stop_reason") != "read_only_complete":
                failures.append("read_only: sensor readiness evidence missing")
        elif item["mode"] != "live":
            failures.append(f"{scenario}: report is not live")
        elif item["acceptance_failures"]:
            failures.append(f"{scenario}: acceptance failures are not empty")
    missing = sorted(REQUIRED-scenarios.keys())
    if missing:
        failures.append("missing scenarios: " + ", ".join(missing))
    return {
        "schema_version": 1,
        "ok": not failures,
        "required_scenarios": sorted(REQUIRED),
        "scenarios": scenarios,
        "failures": failures,
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("reports", nargs="+", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = summarize(args.reports)
    encoded = json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open("x", encoding="utf-8") as stream:
            stream.write(encoded)
    print(encoded, end="")
    raise SystemExit(0 if result["ok"] else 3)


if __name__ == "__main__":
    main()
