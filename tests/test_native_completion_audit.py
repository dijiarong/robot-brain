from __future__ import annotations

import json
import hashlib
from pathlib import Path
import tempfile
import unittest

from scripts.audit_native_navigation_completion import DEFAULT_MATRIX, audit


class NativeCompletionAuditTests(unittest.TestCase):
    def test_current_matrix_has_all_offline_artifacts_but_keeps_external_gates_open(self) -> None:
        report = audit(DEFAULT_MATRIX)
        self.assertTrue(report["offline_artifacts_ok"], report["failures"])
        self.assertFalse(report["complete"])
        self.assertTrue(report["external_pending"])
        self.assertTrue(all(row["status"] in {"evidence_ready", "external_pending"}
                            for row in report["capabilities"]))

    def test_valid_external_reports_can_close_every_declared_gate(self) -> None:
        matrix = json.loads(DEFAULT_MATRIX.read_text())
        with tempfile.TemporaryDirectory() as raw:
            directory = Path(raw)
            for capability in matrix["capabilities"]:
                for gate in capability["external_gates"]:
                    artifact = directory/f"{gate}.raw"
                    artifact.write_text(gate)
                    (directory/f"{gate}-report.json").write_text(json.dumps({
                        "schema_version": 1, "gate_id": gate, "ok": True,
                        "artifacts": [{"path": artifact.name,
                                       "sha256": hashlib.sha256(artifact.read_bytes()).hexdigest()}],
                    }))
            report = audit(DEFAULT_MATRIX, directory)
        self.assertTrue(report["complete"], report)
        self.assertFalse(report["external_pending"])

    def test_false_or_malformed_external_report_never_closes_gate(self) -> None:
        with tempfile.TemporaryDirectory() as raw:
            directory = Path(raw)
            (directory/"go2_read_only_sensor_report-failed.json").write_text('{"ok": false}')
            (directory/"go2_live_motion_suite-bad.json").write_text("not json")
            report = audit(DEFAULT_MATRIX, directory)
        self.assertIn("sensors_transport:go2_read_only_sensor_report",
                      report["external_pending"])
        self.assertIn("sensors_transport:go2_live_motion_suite",
                      report["external_pending"])

    def test_wrong_gate_or_artifact_hash_cannot_close_gate(self) -> None:
        with tempfile.TemporaryDirectory() as raw:
            directory = Path(raw)
            artifact = directory/"sensor.raw"
            artifact.write_text("real evidence")
            (directory/"go2_read_only_sensor_report-forged.json").write_text(json.dumps({
                "schema_version": 1, "gate_id": "different_gate", "ok": True,
                "artifacts": [{"path": artifact.name, "sha256": "0"*64}],
            }))
            report = audit(DEFAULT_MATRIX, directory)
        self.assertIn("sensors_transport:go2_read_only_sensor_report",
                      report["external_pending"])
