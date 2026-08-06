from __future__ import annotations

import unittest

from scripts.verify_native_service_integration import verify


class NativeServiceIntegrationVerifierTests(unittest.IsolatedAsyncioTestCase):
    async def test_production_native_client_is_wired_through_service_with_motion_closed(self) -> None:
        report = await verify()
        self.assertTrue(report["ok"], report)
        self.assertEqual("native_go2", report["provider"])
        self.assertTrue(report["service_health"])
        self.assertTrue(report["skill_invocation"])
        self.assertTrue(report["motion_gate_enforced"])
        self.assertIn("nav_cancel", report["registered_skills"])
        self.assertIn("nav_get_state", report["registered_tools"])
