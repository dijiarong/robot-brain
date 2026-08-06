from __future__ import annotations

import json
from pathlib import Path
import subprocess
import sys
import unittest


class NativeDependencyIsolationTests(unittest.TestCase):
    def test_offline_navigation_runs_with_external_stacks_blocked(self) -> None:
        root = Path(__file__).resolve().parents[1]
        completed = subprocess.run(
            [sys.executable, "scripts/verify_native_navigation_offline.py"],
            cwd=root, capture_output=True, text=True, timeout=15, check=False,
        )
        self.assertEqual(0, completed.returncode, completed.stderr or completed.stdout)
        report = json.loads(completed.stdout)
        self.assertTrue(report["ok"])
        self.assertEqual("native_go2", report["provider"])
        self.assertEqual([], report["loaded_forbidden_modules"])
