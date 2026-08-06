#!/usr/bin/env python3
"""Verify native relocalization or loop closure from a recorded sensor replay."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from robot_brain.core.robot_self_state import RobotPose
from robot_brain.navigation.map_store import SparseVoxelMap
from robot_brain.navigation.replay import (
    evaluate_replay_mapping, evaluate_replay_pose_graph, evaluate_replay_relocalization,
    load_navigation_replay,
)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("mode", choices=("mapping", "relocalization", "loop_closure"))
    parser.add_argument("replay", type=Path)
    parser.add_argument("--map", dest="map_path", type=Path)
    parser.add_argument("--initial-x", type=float)
    parser.add_argument("--initial-y", type=float)
    parser.add_argument("--initial-yaw", type=float, default=0.0)
    parser.add_argument("--global-fallback", action="store_true")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    frames = load_navigation_replay(args.replay)
    if args.mode == "mapping":
        result = evaluate_replay_mapping(frames)
    elif args.mode == "relocalization":
        if args.map_path is None:
            parser.error("relocalization requires --map")
        initial = None
        if args.initial_x is not None or args.initial_y is not None:
            if args.initial_x is None or args.initial_y is None:
                parser.error("--initial-x and --initial-y must be supplied together")
            initial = RobotPose(
                x_m=args.initial_x, y_m=args.initial_y,
                yaw_deg=args.initial_yaw, frame_id="map",
            )
        result = evaluate_replay_relocalization(
            frames, SparseVoxelMap.load(args.map_path), initial_map_pose=initial,
            allow_global_fallback=args.global_fallback,
        )
    else:
        result = evaluate_replay_pose_graph(frames)
    encoded = json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True,
                         allow_nan=False) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open("x", encoding="utf-8") as stream:
            stream.write(encoded)
    print(encoded, end="")
    raise SystemExit(0 if result.get("ok") else 3)


if __name__ == "__main__":
    main()
