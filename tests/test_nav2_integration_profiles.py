"""Offline checks for Nav2 control-surface recoverability (no ROS required)."""
from __future__ import annotations

import unittest
from pathlib import Path


class Nav2ControlSurfaceScriptTests(unittest.TestCase):
    def test_control_surface_script_exists(self) -> None:
        path = Path("scripts/verify_nav2_control_surface.py")
        text = path.read_text(encoding="utf-8")
        self.assertIn("cancel", text)
        self.assertIn("second_goal", text)
        self.assertIn("get_state", text)

    def test_baseline_script_expects_no_ffmpeg(self) -> None:
        path = Path("scripts/collect_orin_nav_baseline.sh")
        text = path.read_text(encoding="utf-8")
        self.assertIn("ffmpeg_detected", text)
        self.assertIn("robot_brain_detected", text)
        self.assertIn("go2_webrtc_detected", text)
        self.assertIn("navigation_stack_detected", text)

    def test_run_with_profile_loads_env(self) -> None:
        path = Path("scripts/run_with_profile.sh")
        self.assertTrue(path.is_file())
        self.assertIn("config/profiles", path.read_text(encoding="utf-8"))


if __name__ == "__main__":
    unittest.main()
