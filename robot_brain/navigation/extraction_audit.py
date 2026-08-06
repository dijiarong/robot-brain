"""Machine-checkable DIMOS navigation extraction inventory."""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
from pathlib import Path


@dataclass(frozen=True)
class EvidenceGroup:
    implementation: tuple[str, ...]
    tests: tuple[str, ...]


EVIDENCE = {
    "contracts": EvidenceGroup(("robot_brain/navigation/base.py",), ("tests/test_navigation_capability.py",)),
    "sensors": EvidenceGroup(("robot_brain/navigation/sensors.py", "robot_brain/actuation/unitree_webrtc.py"), ("tests/test_navigation_sensors.py",)),
    "mapping": EvidenceGroup(("robot_brain/navigation/grid.py", "robot_brain/navigation/map_store.py"), ("tests/test_native_map_store.py", "tests/test_native_navigation.py")),
    "relocalization": EvidenceGroup(("robot_brain/navigation/relocalization.py",), ("tests/test_native_relocalization.py",)),
    "planning_control": EvidenceGroup(("robot_brain/navigation/planner.py", "robot_brain/navigation/native_go2.py", "robot_brain/navigation/motion_safety.py"), ("tests/test_native_navigation.py", "tests/test_native_motion_safety.py")),
    "arbitration": EvidenceGroup(("robot_brain/teleop/session.py",), ("tests/test_navigation_control_arbitration.py",)),
    "exploration_patrol": EvidenceGroup(("robot_brain/navigation/frontier.py", "robot_brain/navigation/exploration.py", "robot_brain/navigation/patrol.py", "robot_brain/navigation/terrain_exploration.py"), ("tests/test_native_exploration.py", "tests/test_native_patrol.py", "tests/test_native_terrain_exploration.py")),
    "visual": EvidenceGroup(("robot_brain/navigation/visual_navigation.py", "robot_brain/navigation/visual_controller.py"), ("tests/test_native_visual_navigation.py", "tests/test_native_visual_controller.py")),
    "terrain3d": EvidenceGroup(("robot_brain/navigation/terrain3d.py", "robot_brain/navigation/terrain_controller.py"), ("tests/test_native_terrain3d.py", "tests/test_native_terrain_controller.py")),
    "pose_graph": EvidenceGroup(("robot_brain/navigation/pose_graph.py",), ("tests/test_native_pose_graph.py",)),
    "diagnostics": EvidenceGroup(("robot_brain/navigation/diagnostics.py", "robot_brain/navigation/replay.py"), ("tests/test_navigation_diagnostics.py", "tests/test_navigation_replay.py")),
    "topology": EvidenceGroup(("robot_brain/navigation/topology.py",), ("tests/test_native_topology.py",)),
    "integration": EvidenceGroup(("robot_brain/runtime/loop.py", "robot_brain/skills/builtin/native_navigation.py", "robot_brain/tools/builtin/native_navigation.py"), ("tests/test_native_navigation_skills.py", "tests/test_native_navigation_tools.py")),
}


def classify_source(relative: str) -> tuple[str | None, str | None]:
    """Return (evidence group, exclusion reason); neither means audit failure."""
    path = relative.replace("\\", "/")
    name = path.rsplit("/", 1)[-1]
    if name == "__init__.py" or name.startswith("test_") or "/tests/" in path or name == "conftest.py":
        return None, "test_or_package_scaffolding"
    if path.startswith("dimos/navigation/diagnostics/") or "/nav_record/" in path:
        return "diagnostics", None
    if path.startswith("dimos/navigation/replanning_a_star/") or "/basic_path_follower/" in path:
        return "planning_control", None
    if any(token in path for token in ("/frontier_exploration/", "/patrolling/", "/tare_planner/")):
        return "exploration_patrol", None
    if any(token in path for token in ("/visual_servoing/", "/navigation/visual/")) or path.endswith("bbox_navigation.py") or path.endswith("visual_navigation_skills.py"):
        return "visual", None
    if "/movement_manager/" in path or "/click_start_goal_router/" in path:
        return "arbitration", None
    if path.endswith("navigation/topology.py"):
        return "topology", None
    if "/modules/pgo/" in path or path.startswith("dimos/mapping/loop_closure/"):
        if path.endswith(("eval.py", "markers_rrd.py")):
            return None, "offline_visualization_or_evaluation"
        return "pose_graph", None
    if path.startswith("dimos/navigation/nav_3d/") or any(token in path for token in (
        "/far_planner/", "/simple_planner/", "/local_planner/", "/path_follower/",
        "/terrain_analysis/", "/terrain_map_ext/",
    )):
        if path.endswith("plan_rrd.py"):
            return None, "offline_visualization_or_evaluation"
        return "terrain3d", None
    if path in {"dimos/navigation/base.py", "dimos/navigation/navigation_spec.py",
                "dimos/navigation/cmu_nav/frames.py", "dimos/navigation/cmu_nav/main.py"}:
        return "contracts", None
    if path.startswith("dimos/mapping/relocalization/"):
        return "relocalization", None
    if path.startswith(("dimos/mapping/occupancy/", "dimos/mapping/pointclouds/",
                        "dimos/mapping/ray_tracing/")) or path in {
        "dimos/mapping/voxels.py", "dimos/mapping/costmapper.py", "dimos/mapping/models.py",
        "dimos/mapping/utils/distance.py",
    }:
        if any(token in path for token in ("visual", "/demo.py", "/tool_")):
            return None, "offline_visualization_or_benchmark"
        return "mapping", None
    if path in {"dimos/mapping/utils/cli/replay.py", "dimos/mapping/utils/cli/summary.py"}:
        return "diagnostics", None
    if path.startswith("dimos/mapping/utils/cli/") or path == "dimos/mapping/tool_voxels.py":
        return None, "operator_dataset_tooling"
    if path.startswith(("dimos/mapping/google_maps/", "dimos/mapping/osm/")):
        return None, "geospatial_service_not_in_go2_navigation_blueprint"
    if "/blueprints/" in path:
        return "integration", None
    if path.startswith("dimos/robot/unitree/go2/"):
        if path in {
            "dimos/robot/unitree/go2/connection.py",
            "dimos/robot/unitree/go2/connection_spec.py",
            "dimos/robot/unitree/go2/go2_mid360_static_transforms.py",
            "dimos/robot/unitree/go2/dds/extrinsics.py",
        }:
            return "sensors", None
        return None, "transport_fleet_or_recording_tool_outside_navigation_algorithm"
    if path.startswith("dimos/agents/skills/navigation.py"):
        return "integration", None
    return None, None


def audit(dimos_root: Path, robot_root: Path) -> dict:
    candidates = _source_candidates(dimos_root)
    groups = {name: [] for name in EVIDENCE}
    excluded, unclassified = [], []
    digest = hashlib.sha256()
    for source in candidates:
        relative = source.relative_to(dimos_root).as_posix()
        group, reason = classify_source(relative)
        digest.update(relative.encode()+b"\0"+source.read_bytes()+b"\0")
        if group is not None:
            groups[group].append(relative)
        elif reason is not None:
            excluded.append({"source": relative, "reason": reason})
        else:
            unclassified.append(relative)
    evidence = {}
    missing = []
    for name, expected in EVIDENCE.items():
        implementations = [{"path": path, "exists": (robot_root/path).is_file()}
                           for path in expected.implementation]
        tests = [{"path": path, "exists": (robot_root/path).is_file()} for path in expected.tests]
        if not groups[name]:
            missing.append(f"no DIMOS source classified for {name}")
        missing.extend(item["path"] for item in implementations+tests if not item["exists"])
        evidence[name] = {"source_files": sorted(groups[name]),
                          "implementation": implementations, "tests": tests}
    forbidden = _forbidden_imports(robot_root)
    ok = not unclassified and not missing and not forbidden
    return {"ok": ok, "dimos_source_digest": digest.hexdigest(),
            "source_file_count": len(candidates), "groups": evidence,
            "excluded": sorted(excluded, key=lambda item: item["source"]),
            "unclassified": sorted(unclassified), "missing_evidence": sorted(set(missing)),
            "forbidden_runtime_imports": forbidden}


def _source_candidates(root: Path) -> list[Path]:
    roots = [root/"dimos/navigation", root/"dimos/mapping",
             root/"dimos/robot/unitree/go2"]
    files = set()
    for base in roots:
        if base.is_dir():
            files.update(base.rglob("*.py"))
    files.update(path for path in (
        root/"dimos/skills/visual_navigation_skills.py",
        root/"dimos/agents/skills/navigation.py",
    ) if path.is_file())
    return sorted(files)


def _forbidden_imports(root: Path) -> list[dict[str, str]]:
    forbidden = ("dimos", "reactivex", "rclpy", "open3d")
    findings = []
    for group in EVIDENCE.values():
        for relative in group.implementation:
            text = (root/relative).read_text(encoding="utf-8")
            for line in text.splitlines():
                stripped = line.strip()
                if any(stripped.startswith(f"import {name}") or stripped.startswith(f"from {name}")
                       for name in forbidden):
                    findings.append({"path": relative, "line": stripped})
    return findings


def json_report(dimos_root: Path, robot_root: Path) -> str:
    return json.dumps(audit(dimos_root, robot_root), indent=2, sort_keys=True,
                      allow_nan=False)+"\n"
