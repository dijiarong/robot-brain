from __future__ import annotations

import json
from pathlib import Path
import tempfile
import unittest

from scripts.summarize_native_go2_acceptance import summarize


class NativeLiveAcceptanceSummaryTests(unittest.TestCase):
    def _reports(self, directory: Path) -> list[Path]:
        paths = []
        for scenario in ("read_only", "straight", "obstacle", "cancel", "sudden_block", "stuck"):
            report = {
                "provider": "native_go2", "ok": True, "started_at": 1,
                "mode": "read_only" if scenario == "read_only" else "live",
                "scenario": "straight" if scenario == "read_only" else scenario,
                "stop_reason": "read_only_complete" if scenario == "read_only" else "goal_reached",
                "sensors": {"ready": True},
                "acceptance": {"failures": []},
            }
            path = directory/f"{scenario}.json"
            path.write_text(json.dumps(report))
            paths.append(path)
        return paths

    def test_complete_suite_passes_and_hashes_every_report(self) -> None:
        with tempfile.TemporaryDirectory() as raw:
            result = summarize(self._reports(Path(raw)))
        self.assertTrue(result["ok"])
        self.assertEqual(6, len(result["scenarios"]))
        self.assertTrue(all(len(row["sha256"]) == 64 for row in result["scenarios"].values()))

    def test_missing_duplicate_or_failed_report_fails_suite(self) -> None:
        with tempfile.TemporaryDirectory() as raw:
            paths = self._reports(Path(raw))
            failed = json.loads(paths[1].read_text())
            failed["ok"] = False
            paths[1].write_text(json.dumps(failed))
            result = summarize(paths[:-1] + [paths[1]])
        self.assertFalse(result["ok"])
        self.assertTrue(any("missing scenarios" in row for row in result["failures"]))
        self.assertTrue(any("duplicate scenario" in row for row in result["failures"]))
        self.assertTrue(any("did not pass" in row for row in result["failures"]))
