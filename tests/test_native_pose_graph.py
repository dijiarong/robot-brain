from __future__ import annotations

import unittest

from robot_brain.navigation import (
    PlanarPose, PlanarPoseGraph, compose_pose, inverse_pose, relative_pose,
    OnlinePoseGraphTracker, PoseGraphTrackerConfig, verify_loop_constraint,
)


class NativePoseGraphTests(unittest.TestCase):
    def test_pose_composition_and_relative_round_trip(self) -> None:
        source = PlanarPose(2.0, -1.0, 90.0)
        target = PlanarPose(2.0, 1.0, -170.0)
        rebuilt = compose_pose(source, relative_pose(source, target))
        self.assertAlmostEqual(target.x_m, rebuilt.x_m)
        self.assertAlmostEqual(target.y_m, rebuilt.y_m)
        self.assertAlmostEqual(target.yaw_degrees, rebuilt.yaw_degrees)

    def test_verified_loop_reduces_square_trajectory_drift(self) -> None:
        graph = PlanarPoseGraph()
        for timestamp, pose in enumerate((
            PlanarPose(0.0, 0.0, 0.0), PlanarPose(2.0, 0.0, 90.0),
            PlanarPose(2.1, 2.0, 180.0), PlanarPose(0.2, 2.1, -90.0),
            PlanarPose(0.45, 0.25, 5.0),
        )):
            graph.add_keyframe(float(timestamp), pose)
        graph.add_loop_constraint(0, 4, PlanarPose(0.0, 0.0, 0.0), confidence=0.95)
        result = graph.optimize(max_iterations=200)
        self.assertTrue(result.accepted, result)
        self.assertLess(result.optimized_rmse, result.initial_rmse)
        self.assertLess(result.optimized[-1].x_m, 0.45)

    def test_without_verified_loop_fails_closed(self) -> None:
        graph = PlanarPoseGraph()
        graph.add_keyframe(1.0, PlanarPose(0, 0, 0))
        graph.add_keyframe(2.0, PlanarPose(1, 0, 0))
        result = graph.optimize()
        self.assertFalse(result.accepted)
        self.assertEqual("no_verified_loop_constraints", result.reason)
        with self.assertRaisesRegex(ValueError, "accepted"):
            graph.correction(result, 1.5)

    def test_correction_interpolates_and_obeys_quality_gate(self) -> None:
        graph = PlanarPoseGraph()
        graph.add_keyframe(0.0, PlanarPose(0, 0, 0))
        graph.add_keyframe(1.0, PlanarPose(1, 0, 0))
        graph.add_keyframe(2.0, PlanarPose(2, 0.5, 10))
        graph.add_loop_constraint(0, 2, PlanarPose(2, 0, 0), confidence=1.0)
        result = graph.optimize(max_iterations=150)
        self.assertTrue(result.accepted, result)
        self.assertLess(graph.correction(result, 1.5).y_m, 0.0)
        rejected = graph.optimize(max_translation_correction_m=0.001)
        self.assertFalse(rejected.accepted)
        self.assertEqual("translation_correction_exceeds_limit", rejected.reason)

    def test_rejects_invalid_graph_inputs(self) -> None:
        graph = PlanarPoseGraph()
        graph.add_keyframe(1.0, PlanarPose(0, 0, 0))
        with self.assertRaises(ValueError):
            graph.add_keyframe(1.0, PlanarPose(1, 0, 0))
        with self.assertRaises(ValueError):
            graph.add_loop_constraint(0, 2, PlanarPose(0, 0, 0), confidence=1.0)

    def test_scan_verified_loop_recovers_relative_transform(self) -> None:
        source = tuple((x, y, 0.5) for x, y in (
            (0, 0), (0.2, 0.1), (0.4, -0.1), (0.7, 0.3), (1.0, 0),
            (0.1, 0.8), (0.5, 1.1), (1.2, 0.7), (1.5, 0.2), (1.4, 1.3),
            (0.3, 1.7), (1.0, 1.9), (1.8, 1.7), (2.0, 0.8), (2.2, 0.1),
        ))
        expected = PlanarPose(0.3, -0.2, 10.0)
        target_from_source = inverse_pose(expected)
        target = tuple((*_point_transform(target_from_source, point[:2]), point[2])
                       for point in source)
        result = verify_loop_constraint(
            source, target, PlanarPose(0.25, -0.15, 8.0),
            translation_radius_m=0.2, yaw_radius_degrees=5.0,
            translation_step_m=0.05, yaw_step_degrees=1.0,
            minimum_score_margin=0.0,
        )
        self.assertTrue(result.accepted, result)
        self.assertAlmostEqual(expected.x_m, result.relative.x_m, delta=0.06)
        self.assertAlmostEqual(expected.y_m, result.relative.y_m, delta=0.06)

    def test_scan_loop_budget_and_ambiguity_fail_closed(self) -> None:
        repeated = tuple((float(i % 5), float(i // 5), 0.5) for i in range(25))
        budget = verify_loop_constraint(repeated, repeated, PlanarPose(0, 0, 0), max_candidates=1)
        self.assertEqual("loop_search_budget_exceeded", budget.reason)
        ambiguous = verify_loop_constraint(
            repeated, repeated, PlanarPose(0, 0, 0),
            translation_radius_m=0.2, yaw_radius_degrees=5,
            minimum_score_margin=0.5,
        )
        self.assertFalse(ambiguous.accepted)

    def test_online_tracker_adds_only_scan_verified_old_loop(self) -> None:
        config = PoseGraphTrackerConfig(
            keyframe_translation_m=0.1, keyframe_yaw_degrees=5,
            loop_search_radius_m=0.8, minimum_loop_age_s=3,
            minimum_keyframes_for_loop=4,
        )
        tracker = OnlinePoseGraphTracker(config)
        landmarks = tuple((x, y, 0.5) for x, y in (
            (0, 0), (.2, .1), (.4, -.1), (.7, .3), (1, 0), (.1, .8),
            (.5, 1.1), (1.2, .7), (1.5, .2), (1.4, 1.3), (.3, 1.7),
            (1, 1.9), (1.8, 1.7), (2, .8), (2.2, .1),
        ))
        poses = (
            PlanarPose(0, 0, 0), PlanarPose(2, 0, 90),
            PlanarPose(2, 2, 180), PlanarPose(0, 2, -90),
            PlanarPose(.3, .2, 4),
        )
        update = None
        for timestamp, raw in enumerate(poses):
            # Final scan represents a true revisit even though raw odometry drifted.
            scan_pose = PlanarPose(0, 0, 0) if timestamp == 4 else raw
            scan = tuple((*_point_transform(inverse_pose(scan_pose), point[:2]), point[2])
                         for point in landmarks)
            update = tracker.process(
                float(timestamp), raw, scan,
                translation_radius_m=.5, yaw_radius_degrees=10,
                translation_step_m=.1, yaw_step_degrees=2,
                minimum_score_margin=0,
            )
        self.assertTrue(update.loop_added, update)
        self.assertEqual(0, update.loop_source_index)
        self.assertTrue(update.graph_result.accepted)
        self.assertLess(update.corrected_pose.x_m, poses[-1].x_m)

    def test_online_tracker_threshold_and_capacity_are_bounded(self) -> None:
        tracker = OnlinePoseGraphTracker(PoseGraphTrackerConfig(
            keyframe_translation_m=1, keyframe_yaw_degrees=30,
            minimum_loop_age_s=1, max_keyframes=1,
        ))
        scan = ((0, 0, .5),)*20
        self.assertTrue(tracker.process(0, PlanarPose(0, 0, 0), scan).keyframe_added)
        below = tracker.process(1, PlanarPose(.1, 0, 0), scan)
        self.assertEqual("below_keyframe_threshold", below.reason)
        capacity = tracker.process(2, PlanarPose(2, 0, 0), scan)
        self.assertEqual("keyframe_capacity_reached", capacity.reason)


def _point_transform(pose: PlanarPose, point):
    transformed = compose_pose(pose, PlanarPose(point[0], point[1], 0))
    return transformed.x_m, transformed.y_m


if __name__ == "__main__":
    unittest.main()
