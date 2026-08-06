from __future__ import annotations

import unittest

from robot_brain.navigation import LinearVelocityRampLimiter, NavigationMotionSafetySignal


class NativeMotionSafetyTests(unittest.TestCase):
    def test_vector_acceleration_is_bounded_and_resettable(self) -> None:
        limiter = LinearVelocityRampLimiter(1.0)
        self.assertEqual((.1, 0.0), limiter.step(1.0, 0.0, .1))
        vx, vy = limiter.step(1.0, 1.0, .1)
        self.assertAlmostEqual(.1, ((vx-.1)**2+vy**2)**.5)
        limiter.reset()
        self.assertEqual((0.0, 0.0), limiter.step(1.0, 0.0, .1, speed_scale=0))

    def test_invalid_safety_signal_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            NavigationMotionSafetySignal(speed_scale=1.1)
        with self.assertRaises(ValueError):
            LinearVelocityRampLimiter(0)


if __name__ == "__main__":
    unittest.main()
