from __future__ import annotations

import unittest

from scripts.verify_native_go2_arbitration import verify


class NativeArbitrationVerifierTests(unittest.IsolatedAsyncioTestCase):
    async def test_dry_run_exercises_full_control_ownership_state_machine(self) -> None:
        report = await verify(live=False)
        self.assertTrue(report["ok"], report)
        self.assertEqual("dry_run", report["mode"])
        self.assertTrue(report["navigation_preempted"])
        self.assertTrue(report["lease_invalidated"])
        self.assertTrue(report["motion_rejected_during_estop"])
