from __future__ import annotations

import unittest

from scripts.register_native_navigation_evidence import validate_gate


class NativeEvidenceRegistrationTests(unittest.TestCase):
    def test_read_only_requires_ready_sensor_semantics(self) -> None:
        good = {"ok": True, "mode": "read_only", "stop_reason": "read_only_complete",
                "sensors": {"ready": True}}
        self.assertTrue(validate_gate("go2_read_only_sensor_report", [good])[0])
        self.assertFalse(validate_gate("go2_read_only_sensor_report", [{"ok": True}])[0])

    def test_patrol_gate_requires_all_four_distinct_strategies(self) -> None:
        reports = [{"patrol": {"ok": True, "strategy": value}} for value in
                   ("coverage", "frontier", "random", "least_visited")]
        self.assertTrue(validate_gate("go2_four_strategy_patrol_traces", reports)[0])
        self.assertFalse(validate_gate("go2_four_strategy_patrol_traces", reports[:3])[0])

    def test_mapping_and_loop_reports_require_domain_fields(self) -> None:
        self.assertTrue(validate_gate("go2_mapping_replay", [{
            "ok": True, "reason": "mapped", "frames_integrated": 3, "voxel_count": 20,
        }])[0])
        self.assertTrue(validate_gate("go2_closed_loop_replay", [{
            "ok": True, "reason": "accepted_loop_observed", "accepted_loops": 1,
        }])[0])
        self.assertFalse(validate_gate("go2_closed_loop_replay", [{"ok": True}])[0])

    def test_generic_ok_cannot_close_specialized_gate(self) -> None:
        for gate in ("go2_visual_servo_trace", "native_service_integration_report",
                     "go2_live_teleop_estop_arbitration", "go2_mid360_terrain_replay"):
            self.assertFalse(validate_gate(gate, [{"ok": True}])[0], gate)
