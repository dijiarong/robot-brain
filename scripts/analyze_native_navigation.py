#!/usr/bin/env python3
"""Summarize a native navigation JSONL trace for acceptance/debugging."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from robot_brain.navigation.diagnostics import (
    load_navigation_trace,
    build_navigation_report,
)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("trace_path")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    events = load_navigation_trace(args.trace_path)
    report = build_navigation_report(events)
    encoded = json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True,
                         allow_nan=False) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open("x", encoding="utf-8") as stream:
            stream.write(encoded)
    print(encoded, end="")


if __name__ == "__main__":
    main()
