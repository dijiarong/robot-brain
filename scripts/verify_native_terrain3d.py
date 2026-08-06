#!/usr/bin/env python3
"""Offline MLS verification against ASCII/binary PLY or binary/ascii PCD maps."""
from __future__ import annotations

import argparse
import builtins
import hashlib
import json
import math
from pathlib import Path
import struct
import sys
import time

FORBIDDEN = ("dimos", "rclpy", "open3d", "reactivex")
_original_import = builtins.__import__


def _blocked_import(name, globals=None, locals=None, fromlist=(), level=0):
    if name.split(".", 1)[0] in FORBIDDEN:
        raise ImportError(f"forbidden dependency requested: {name}")
    return _original_import(name, globals, locals, fromlist, level)


builtins.__import__ = _blocked_import

from robot_brain.navigation.terrain3d import build_surface_graph, plan_surface_path  # noqa: E402
from robot_brain.navigation.replay import load_navigation_replay  # noqa: E402

_PLY_TYPES = {
    "char": "b", "uchar": "B", "int8": "b", "uint8": "B",
    "short": "h", "ushort": "H", "int16": "h", "uint16": "H",
    "int": "i", "uint": "I", "int32": "i", "uint32": "I",
    "float": "f", "float32": "f", "double": "d", "float64": "d",
}


def load_xyz(path: Path, *, max_points: int = 2_000_000) -> tuple[tuple[float, float, float], ...]:
    with path.open("rb") as stream:
        first = stream.readline().decode("ascii", "strict").strip().lower()
        stream.seek(0)
        if first == "ply":
            return _load_ply(stream, max_points)
        if first.startswith("# .pcd"):
            return _load_pcd(stream, max_points)
    raise ValueError("only PLY and PCD point clouds are supported")


def load_input_xyz(path: Path, *, max_points: int = 2_000_000):
    if path.name.endswith((".jsonl.gz", ".replay.gz")):
        points = []
        for frame in load_navigation_replay(path):
            yaw = math.radians(frame.pose.yaw_deg)
            cosine, sine = math.cos(yaw), math.sin(yaw)
            for x, y, z in frame.points_xyz:
                points.append((frame.pose.x_m+x*cosine-y*sine,
                               frame.pose.y_m+x*sine+y*cosine,
                               frame.pose.z_m+z))
        step = max(1, math.ceil(len(points)/max_points))
        return tuple(points[::step]), "native_navigation_replay"
    return load_xyz(path, max_points=max_points), "pointcloud_file"


def _path_safety(path, *, max_step_m: float, max_slope_degrees: float,
                 required_clearance_m: float):
    steps, slopes = [], []
    for left, right in zip(path.nodes, path.nodes[1:]):
        horizontal = math.hypot(right.x_m-left.x_m, right.y_m-left.y_m)
        vertical = abs(right.z_m-left.z_m)
        steps.append(vertical)
        slopes.append(math.degrees(math.atan2(vertical, horizontal)) if horizontal else 90.0)
    finite_costs = all(math.isfinite(node.traversal_cost) for node in path.nodes)
    clearance = path.minimum_clearance_m
    clearance_ok = not math.isfinite(clearance) or clearance > required_clearance_m
    failures = []
    if steps and max(steps) > max_step_m+1e-9:
        failures.append("step_limit_exceeded")
    if slopes and max(slopes) > max_slope_degrees+1e-9:
        failures.append("slope_limit_exceeded")
    if not finite_costs:
        failures.append("non_traversable_path_node")
    if not clearance_ok:
        failures.append("wall_clearance_not_strictly_satisfied")
    return {
        "ok": not failures, "failures": failures,
        "maximum_step_m": max(steps, default=0.0),
        "maximum_slope_degrees": max(slopes, default=0.0),
        "minimum_clearance_m": None if not math.isfinite(clearance) else clearance,
        "finite_traversal_costs": finite_costs,
    }


def _load_ply(stream, maximum):
    header = []
    while True:
        line = stream.readline()
        if not line:
            raise ValueError("truncated PLY header")
        text = line.decode("ascii", "strict").strip()
        header.append(text)
        if text == "end_header":
            break
    fmt = next(row.split()[1] for row in header if row.startswith("format "))
    count = int(next(row.split()[2] for row in header if row.startswith("element vertex ")))
    vertex_at = next(i for i, row in enumerate(header) if row.startswith("element vertex "))
    properties = []
    for row in header[vertex_at + 1:]:
        if row.startswith("element ") or row == "end_header":
            break
        parts = row.split()
        if parts[0] == "property" and parts[1] != "list":
            properties.append((parts[2], parts[1]))
    names = [name for name, _ in properties]
    indices = [names.index(axis) for axis in ("x", "y", "z")]
    step = max(1, (count + maximum - 1) // maximum)
    points = []
    if fmt == "ascii":
        for index in range(count):
            row = stream.readline().split()
            if index % step == 0 and row:
                points.append(tuple(float(row[i]) for i in indices))
    elif fmt in {"binary_little_endian", "binary_big_endian"}:
        endian = "<" if fmt == "binary_little_endian" else ">"
        unpacker = struct.Struct(endian + "".join(_PLY_TYPES[kind] for _, kind in properties))
        for index in range(count):
            row = stream.read(unpacker.size)
            if len(row) != unpacker.size:
                raise ValueError("truncated PLY vertex data")
            if index % step == 0:
                values = unpacker.unpack(row)
                points.append(tuple(float(values[i]) for i in indices))
    else:
        raise ValueError(f"unsupported PLY encoding: {fmt}")
    return tuple(points)


def _load_pcd(stream, maximum):
    header = {}
    while True:
        line = stream.readline()
        if not line:
            raise ValueError("truncated PCD header")
        text = line.decode("ascii", "strict").strip()
        if text and not text.startswith("#"):
            key, *values = text.split()
            header[key.upper()] = values
        if text.upper().startswith("DATA "):
            break
    fields = header["FIELDS"]
    sizes = [int(v) for v in header["SIZE"]]
    kinds = header["TYPE"]
    counts = [int(v) for v in header.get("COUNT", ["1"] * len(fields))]
    total = int(header["POINTS"][0])
    if any(count != 1 for count in counts):
        raise ValueError("PCD vector fields are not supported")
    codes = {("F", 4): "f", ("F", 8): "d", ("I", 4): "i", ("U", 4): "I"}
    unpacker = struct.Struct("<" + "".join(codes[(kind, size)] for kind, size in zip(kinds, sizes)))
    indices = [fields.index(axis) for axis in ("x", "y", "z")]
    step = max(1, (total + maximum - 1) // maximum)
    points = []
    encoding = header["DATA"][0].lower()
    for index in range(total):
        if encoding == "binary":
            row = stream.read(unpacker.size)
            values = unpacker.unpack(row)
        elif encoding == "ascii":
            values = tuple(float(v) for v in stream.readline().split())
        else:
            raise ValueError(f"unsupported PCD encoding: {encoding}")
        if index % step == 0:
            points.append(tuple(float(values[i]) for i in indices))
    return tuple(points)


def _component_endpoints(graph, *, max_step_m=.16, max_slope_degrees=25.0):
    by_xy = {}
    for node in graph.values():
        if not math.isfinite(node.traversal_cost):
            continue
        by_xy.setdefault((node.cell[0], node.cell[1]), []).append(node)
    unseen = {node.cell for nodes in by_xy.values() for node in nodes}
    largest = set()
    while unseen:
        seed = unseen.pop()
        component = {seed}
        queue = [seed]
        while queue:
            cell = queue.pop()
            node = graph[cell]
            x, y, _ = cell
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    if dx == 0 and dy == 0:
                        continue
                    for candidate in by_xy.get((x + dx, y + dy), ()):
                        key = candidate.cell
                        horizontal = math.hypot(candidate.x_m-node.x_m, candidate.y_m-node.y_m)
                        vertical = abs(candidate.z_m-node.z_m)
                        slope = math.degrees(math.atan2(vertical, horizontal)) if horizontal else 90.0
                        if (key in unseen and vertical <= max_step_m
                                and slope <= max_slope_degrees):
                            unseen.remove(key)
                            component.add(key)
                            queue.append(key)
        if len(component) > len(largest):
            largest = component
    if len(largest) < 2:
        raise ValueError("no connected surface region with at least two cells")
    start_key = min(largest)
    start = graph[start_key]
    goal = max((graph[key] for key in largest),
               key=lambda node: (node.x_m - start.x_m) ** 2 + (node.y_m - start.y_m) ** 2)
    return (start.x_m, start.y_m, start.z_m), (goal.x_m, goal.y_m, goal.z_m), len(largest)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("pointcloud", type=Path)
    parser.add_argument("--resolution", type=float, default=0.20)
    parser.add_argument("--max-points", type=int, default=2_000_000)
    parser.add_argument("--robot-height", type=float, default=0.30)
    parser.add_argument("--max-step", type=float, default=0.16)
    parser.add_argument("--max-slope", type=float, default=25.0)
    parser.add_argument("--wall-clearance", type=float, default=0.10)
    parser.add_argument("--wall-buffer", type=float, default=0.75)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    started = time.perf_counter()
    points, input_kind = load_input_xyz(args.pointcloud, max_points=args.max_points)
    loaded = time.perf_counter()
    graph = build_surface_graph(
        points, resolution_m=args.resolution, layer_height_m=args.resolution,
        robot_height_m=args.robot_height, wall_clearance_m=args.wall_clearance,
        wall_buffer_m=args.wall_buffer, wall_buffer_weight=100.0,
        surface_closing_radius_m=.30,
    )
    built = time.perf_counter()
    start, goal, component_size = _component_endpoints(
        graph, max_step_m=args.max_step, max_slope_degrees=args.max_slope,
    )
    selected = time.perf_counter()
    path = plan_surface_path(
        graph, start, goal, max_step_height_m=args.max_step,
        max_slope_degrees=args.max_slope,
        max_endpoint_distance_m=args.resolution,
    )
    done = time.perf_counter()
    forbidden = sorted(name for name in sys.modules if name.split(".", 1)[0] in FORBIDDEN)
    safety = (_path_safety(path, max_step_m=args.max_step,
                           max_slope_degrees=args.max_slope,
                           required_clearance_m=args.wall_clearance)
              if path is not None else {"ok": False, "failures": ["no_surface_path"]})
    report = {
        "ok": path is not None and len(path.nodes) >= 2 and not forbidden and safety["ok"],
        "input": str(args.pointcloud.resolve()), "input_points": len(points),
        "input_kind": input_kind,
        "input_sha256": hashlib.sha256(args.pointcloud.read_bytes()).hexdigest(),
        "surface_nodes": len(graph), "largest_xy_component": component_size,
        "start_xyz": start, "goal_xyz": goal,
        "path_nodes": len(path.nodes) if path else 0,
        "path_length_m": path.length_m if path else None,
        "elevation_gain_m": path.elevation_gain_m if path else None,
        "safety": safety,
        "timings_ms": {"load": (loaded-started)*1000, "build": (built-loaded)*1000,
                       "component_select": (selected-built)*1000,
                       "plan": (done-selected)*1000},
        "loaded_forbidden_modules": forbidden,
    }
    encoded = json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open("x", encoding="utf-8") as stream:
            stream.write(encoded)
    print(encoded, end="")
    raise SystemExit(0 if report["ok"] else 1)


if __name__ == "__main__":
    main()
