#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path

from robot_brain.navigation.extraction_audit import audit


def main() -> int:
    parser = argparse.ArgumentParser(description="Audit complete DIMOS navigation extraction coverage")
    parser.add_argument("--dimos-root", type=Path, default=Path("../topsun_dimos"))
    parser.add_argument("--robot-root", type=Path, default=Path("."))
    args = parser.parse_args()
    result = audit(args.dimos_root.resolve(), args.robot_root.resolve())
    print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False))
    return 0 if result["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
