from __future__ import annotations

import unittest

from scripts.prepare_native_navigation_acceptance import build_manifest


class NativeAcceptancePreparationTests(unittest.TestCase):
    def test_all_unique_pending_gates_are_assigned_once_without_motion(self) -> None:
        from pathlib import Path
        manifest = build_manifest(evidence_dir=Path("docs/evidence"), run_id="test-run")
        assigned = [gate for batch in manifest["batches"] for gate in batch["gates"]]
        self.assertFalse(manifest["executes_motion"])
        self.assertFalse(manifest["unassigned_gates"])
        self.assertEqual(set(manifest["unique_pending_gates"]), set(assigned))
        self.assertEqual(len(assigned), len(set(assigned)))
        self.assertEqual(12, len(assigned))
        self.assertEqual(set(assigned), set(manifest["gate_commands"]))
        self.assertTrue(all(commands for commands in manifest["gate_commands"].values()))

    def test_registered_service_gate_is_not_scheduled_again(self) -> None:
        from pathlib import Path
        manifest = build_manifest(evidence_dir=Path("docs/evidence"), run_id="test-run")
        self.assertNotIn("native_service_integration_report", manifest["unique_pending_gates"])
        self.assertEqual(12, len(manifest["unique_pending_gates"]))
