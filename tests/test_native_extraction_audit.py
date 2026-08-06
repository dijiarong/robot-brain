from __future__ import annotations

import unittest
from pathlib import Path

from robot_brain.navigation.extraction_audit import audit, classify_source


class NativeExtractionAuditTests(unittest.TestCase):
    def test_classifies_named_navigation_branches(self) -> None:
        expected = {
            "dimos/navigation/replanning_a_star/module.py": "planning_control",
            "dimos/navigation/cmu_nav/modules/tare_planner/tare_planner.py": "exploration_patrol",
            "dimos/navigation/cmu_nav/modules/pgo/pgo.py": "pose_graph",
            "dimos/navigation/visual_servoing/detection_navigation.py": "visual",
            "dimos/mapping/relocalization/relocalize.py": "relocalization",
            "dimos/robot/unitree/go2/connection.py": "sensors",
            "dimos/robot/unitree/go2/dds/extrinsics.py": "sensors",
            "dimos/navigation/nav_3d/evaluator/evaluator.py": "terrain3d",
            "dimos/mapping/utils/cli/replay.py": "diagnostics",
        }
        for path, group in expected.items():
            self.assertEqual(group, classify_source(path)[0])
        self.assertIsNone(classify_source(
            "dimos/robot/unitree/go2/fleet_connection.py"
        )[0])

    def test_current_workspace_inventory_has_no_silent_unclassified_source(self) -> None:
        robot = Path(__file__).resolve().parents[1]
        dimos = robot.parent/"topsun_dimos"
        if not dimos.is_dir():
            self.skipTest("topsun_dimos checkout is not available")
        result = audit(dimos, robot)
        self.assertEqual([], result["unclassified"], result["unclassified"])
        self.assertEqual([], result["missing_evidence"], result["missing_evidence"])
        self.assertEqual([], result["forbidden_runtime_imports"])
        self.assertTrue(result["ok"])


if __name__ == "__main__":
    unittest.main()
