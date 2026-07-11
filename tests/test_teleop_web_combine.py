"""Tests for web teleop chord (multi-key) velocity combining."""
from __future__ import annotations

import unittest

from examples.run_unitree_teleop import _CAR_NUDGES, _DEFAULT_NUDGES
from examples.run_unitree_teleop_web import combine_nudge_keys, drive_channel_label


class CombineNudgeKeysTests(unittest.TestCase):
    def test_single_key(self) -> None:
        vx, vy, vyaw = combine_nudge_keys({"w"}, _DEFAULT_NUDGES)
        self.assertAlmostEqual(vx, 0.2)
        self.assertAlmostEqual(vy, 0.0)
        self.assertAlmostEqual(vyaw, 0.0)

    def test_omni_forward_and_strafe(self) -> None:
        vx, vy, vyaw = combine_nudge_keys({"w", "d"}, _DEFAULT_NUDGES)
        self.assertAlmostEqual(vx, 0.2)
        self.assertAlmostEqual(vy, -0.2)
        self.assertAlmostEqual(vyaw, 0.0)

    def test_car_forward_and_turn(self) -> None:
        vx, vy, vyaw = combine_nudge_keys({"w", "d"}, _CAR_NUDGES)
        self.assertAlmostEqual(vx, 0.2)
        self.assertAlmostEqual(vy, 0.0)
        self.assertAlmostEqual(vyaw, -0.3)
        self.assertEqual(
            drive_channel_label(vx, vy, vyaw, omni=False),
            "joystick (arc)",
        )

    def test_opposite_keys_cancel(self) -> None:
        vx, vy, vyaw = combine_nudge_keys({"w", "s"}, _DEFAULT_NUDGES)
        self.assertAlmostEqual(vx, 0.0)
        self.assertAlmostEqual(vy, 0.0)
        self.assertAlmostEqual(vyaw, 0.0)


if __name__ == "__main__":
    unittest.main()
