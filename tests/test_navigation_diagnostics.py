from __future__ import annotations

import tempfile
import unittest
import json
import subprocess
import sys
import time
from pathlib import Path

from robot_brain.navigation.diagnostics import (
    NavigationTraceWriter,
    build_navigation_report,
    load_navigation_trace,
    navigation_trajectory_metrics,
    summarize_navigation_trace,
)


class NavigationDiagnosticsTests(unittest.TestCase):
    def test_trace_writer_refuses_to_mix_sessions_in_existing_artifact(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"trace.jsonl"
            first = NavigationTraceWriter(path, provider="native_go2")
            first.close()
            with self.assertRaises(FileExistsError):
                NavigationTraceWriter(path, provider="native_go2")

    def test_analyzer_output_is_strict_json_and_refuses_overwrite(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            trace = Path(directory)/"trace.jsonl"
            output = Path(directory)/"report.json"
            trace.write_text(json.dumps({"schema_version": 1, "event": "finished",
                                         "status": "succeeded"})+"\n")
            command = [sys.executable, "scripts/analyze_native_navigation.py",
                       str(trace), "--output", str(output)]
            first = subprocess.run(command, capture_output=True, text=True)
            second = subprocess.run(command, capture_output=True, text=True)
            self.assertEqual(0, first.returncode, first.stderr)
            self.assertNotEqual(0, second.returncode)
            self.assertEqual(1, json.loads(output.read_text())["schema_version"])

    def test_jsonl_trace_round_trip_and_summary(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "navigation.jsonl"
            writer = NavigationTraceWriter(path, provider="native_go2", config={"speed": 0.2})
            writer.record("goal_accepted", target_x_m=1.0)
            writer.record("plan", replan_count=1, path_points=4)
            writer.record("plan_failed", replan_count=2)
            writer.record("emergency_stop", replan_count=2)
            writer.record("finished", status="failed", stop_reason="no_path", replan_count=2)
            writer.close()
            events = load_navigation_trace(path)
            summary = summarize_navigation_trace(events)
        self.assertEqual(1, summary.sessions)
        self.assertEqual(1, summary.goals)
        self.assertEqual(1, summary.plans)
        self.assertEqual(1, summary.failed_plans)
        self.assertEqual(1, summary.emergency_stops)
        self.assertEqual({"failed": 1}, summary.terminal_statuses)
        self.assertEqual({"no_path": 1}, summary.stop_reasons)
        self.assertEqual(2, summary.maximum_replan_count)
        self.assertEqual(0, summary.recovered_open_sessions)
        self.assertEqual(0, summary.dropped_events)

    def test_malformed_rows_are_reported_not_silently_dropped(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bad.jsonl"
            path.write_text('{"event":"goal_accepted"}\nnot-json\n')
            summary = summarize_navigation_trace(load_navigation_trace(path))
        self.assertEqual(1, summary.goals)
        self.assertEqual(1, summary.malformed_rows)

    def test_open_session_is_reported_as_recovered_process_exit_evidence(self) -> None:
        summary = summarize_navigation_trace((
            {"event": "session_started", "session_id": "nav-open"},
            {"event": "goal_accepted", "session_id": "nav-open"},
        ))
        self.assertEqual(1, summary.recovered_open_sessions)

    def test_manifest_redacts_secrets_and_report_computes_trajectory_metrics(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "trace.jsonl"
            writer = NavigationTraceWriter(
                path, provider="native_go2",
                config={"resolution": 0.1, "api_token": "do-not-leak"},
            )
            writer.record("goal_accepted", goal_id="g1")
            writer.record("plan_geometry", goal_id="g1", path_xy=[[0, 0], [1, 0]])
            writer.record("command", goal_id="g1", vx_mps=0.2)
            writer.record("motion_sample", goal_id="g1", x_m=0.0, y_m=0.1)
            writer.record("motion_sample", goal_id="g1", x_m=1.1, y_m=0.0)
            writer.record("finished", goal_id="g1", status="succeeded",
                          stop_reason="goal_reached")
            writer.close()
            manifest_text = writer.manifest_path.read_text()
            manifest = json.loads(manifest_text)
            report = build_navigation_report(load_navigation_trace(path))
        self.assertEqual("<redacted>", manifest["config"]["api_token"])
        self.assertNotIn("do-not-leak", manifest_text)
        self.assertEqual(2, report["trajectory"]["samples"])
        self.assertAlmostEqual(0.1, report["trajectory"]["cross_track_max_m"])
        self.assertEqual(1, report["trajectory"]["command_count"])

    def test_trace_enqueue_is_bounded_and_non_blocking(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            writer = NavigationTraceWriter(
                Path(directory) / "bounded.jsonl", provider="native_go2", queue_items=2,
            )
            started = time.monotonic()
            for index in range(20_000):
                writer.record("hot_path", index=index)
            elapsed = time.monotonic() - started
            writer.close()
        self.assertLess(elapsed, 1.0)
        self.assertGreater(writer.dropped_events, 0)

    def test_non_finite_diagnostic_values_are_strict_json_null(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "finite.jsonl"
            writer = NavigationTraceWriter(path, provider="native_go2")
            writer.record("terrain_plan", clearance=float("inf"), score=float("nan"))
            writer.close()
            raw = path.read_text()
            rows = load_navigation_trace(path)
        self.assertNotIn("Infinity", raw)
        self.assertNotIn("NaN", raw)
        event = next(row for row in rows if row["event"] == "terrain_plan")
        self.assertIsNone(event["clearance"])
        self.assertIsNone(event["score"])

    def test_trajectory_reports_tangent_overshoot_snaking_and_response_lag(self) -> None:
        events = [{"event": "plan_geometry", "monotonic": 0.0,
                   "path_xy": [[0, 0], [1, 0]]}]
        ys = (.1, -.1, .1, -.1, .1)
        yaws = (0, -10, 0, -10, 0)
        commands = (.5, -.5, .5, -.5, .5)
        for index, (y, yaw, command) in enumerate(zip(ys, yaws, commands)):
            base = index*.2
            events.append({"event": "command", "monotonic": base+.05,
                           "yaw_rps": command})
            events.append({"event": "motion_sample", "monotonic": base+.1,
                           "x_m": index*.3, "y_m": y,
                           "yaw_degrees": yaw})
        metrics = navigation_trajectory_metrics(events)
        self.assertAlmostEqual(.2, metrics.overshoot_m)
        self.assertEqual(4, metrics.angular_flip_count)
        self.assertEqual(1, len(metrics.snake_candidates))
        self.assertEqual("CORRELATED", metrics.response_lag_evidence)
        self.assertIsNotNone(metrics.response_lag_median_s)
        self.assertIsNotNone(metrics.response_lag_p95_s)

    def test_latest_planned_length_is_not_summed_across_replans(self) -> None:
        metrics = navigation_trajectory_metrics((
            {"event": "plan_geometry", "path_xy": [[0, 0], [2, 0]]},
            {"event": "motion_sample", "x_m": 0, "y_m": 0},
            {"event": "motion_sample", "x_m": 1, "y_m": 0},
            {"event": "plan_geometry", "path_xy": [[1, 0], [1.5, 0]]},
            {"event": "motion_sample", "x_m": 1, "y_m": 0},
            {"event": "motion_sample", "x_m": 1.5, "y_m": 0},
        ))
        self.assertAlmostEqual(.5, metrics.planned_path_length_m)
        self.assertAlmostEqual(1.5, metrics.odom_path_length_m)
