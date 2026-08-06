"""Durable JSONL navigation traces and small dependency-free summaries."""
from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import platform
import queue
import re
import threading
import time
from typing import Any, Iterable


class NavigationTraceWriter:
    def __init__(self, path: str | Path, *, provider: str,
                 config: dict[str, Any] | None = None, queue_items: int = 2048) -> None:
        if queue_items <= 0:
            raise ValueError("navigation trace queue size must be positive")
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.provider = provider
        self.session_id = f"nav-{time.time_ns()}"
        self._queue: queue.Queue[str | None] = queue.Queue(maxsize=queue_items)
        self._dropped = 0
        self._closed = False
        self._writer_error: str | None = None
        self.manifest_path = self.path.with_suffix(self.path.suffix + ".manifest.json")
        descriptor = os.open(self.path, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o600)
        os.close(descriptor)
        try:
            self._write_manifest(config or {})
        except Exception:
            # Roll back only the empty trace reserved by this constructor.
            self.path.unlink(missing_ok=True)
            raise
        self._thread = threading.Thread(
            target=self._writer_loop, name=f"native-nav-trace-{os.getpid()}", daemon=True,
        )
        self._thread.start()
        self.record("session_started", config=config or {})

    def _write_manifest(self, config: dict[str, Any]) -> None:
        payload = {
            "schema_version": 1, "manifest_kind": "native_navigation_run",
            "session_id": self.session_id, "provider": self.provider,
            "created_wall_time": datetime.now(timezone.utc).isoformat(),
            "runtime": {"python": platform.python_version(), "platform": platform.platform(),
                        "pid": os.getpid(), "monotonic_resolution_s": time.get_clock_info("monotonic").resolution},
            "config": _redact(config),
            "notes": {"odom_is_external_ground_truth": False,
                      "command_acceptance_is_not_motion_proof": True},
        }
        descriptor = os.open(self.manifest_path, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o600)
        try:
            os.write(descriptor, (json.dumps(payload, ensure_ascii=False, sort_keys=True,
                                             allow_nan=False, default=str) + "\n").encode())
        finally:
            os.close(descriptor)

    def record(self, event: str, **fields: object) -> None:
        if self._closed:
            return
        row = {
            "schema_version": 1,
            "session_id": self.session_id,
            "provider": self.provider,
            "event": event,
            "wall_time": datetime.now(timezone.utc).isoformat(),
            "monotonic_ns": time.monotonic_ns(),
            **fields,
        }
        encoded = json.dumps(
            _json_safe(row), ensure_ascii=False, sort_keys=True,
            default=str, allow_nan=False,
        ) + "\n"
        try:
            self._queue.put_nowait(encoded)
        except queue.Full:
            self._dropped += 1

    @property
    def dropped_events(self) -> int:
        return self._dropped

    @property
    def writer_error(self) -> str | None:
        return self._writer_error

    def flush(self, timeout_s: float = 1.0) -> bool:
        deadline = time.monotonic() + max(0.0, timeout_s)
        while self._queue.unfinished_tasks and time.monotonic() < deadline:
            time.sleep(0.002)
        return self._queue.unfinished_tasks == 0

    def close(self, timeout_s: float = 1.0) -> None:
        if self._closed:
            return
        self.record("session_ended", terminal="writer_closed", reason="normal_close")
        self.record("trace_writer_closed", dropped_events=self._dropped,
                    writer_error=self._writer_error)
        self.flush(timeout_s * 0.75)
        self._closed = True
        try:
            self._queue.put_nowait(None)
        except queue.Full:
            self._dropped += 1
            return
        self._thread.join(timeout=max(0.0, timeout_s * 0.25))

    def _writer_loop(self) -> None:
        descriptor = None
        try:
            descriptor = os.open(self.path, os.O_APPEND | os.O_WRONLY)
            while True:
                encoded = self._queue.get()
                try:
                    if encoded is None:
                        return
                    os.write(descriptor, encoded.encode("utf-8"))
                finally:
                    self._queue.task_done()
        except OSError as exc:
            self._writer_error = f"{type(exc).__name__}: {exc}"
            while True:
                try:
                    self._queue.get_nowait()
                    self._queue.task_done()
                except queue.Empty:
                    break
        finally:
            if descriptor is not None:
                os.close(descriptor)


@dataclass(frozen=True)
class NavigationTraceSummary:
    sessions: int
    goals: int
    plans: int
    failed_plans: int
    emergency_stops: int
    terminal_statuses: dict[str, int]
    stop_reasons: dict[str, int]
    maximum_replan_count: int
    malformed_rows: int
    recovered_open_sessions: int
    dropped_events: int
    writer_errors: tuple[str, ...]


@dataclass(frozen=True)
class NavigationTrajectoryMetrics:
    samples: int
    planned_path_length_m: float
    odom_path_length_m: float
    cross_track_rms_m: float | None
    cross_track_p95_m: float | None
    cross_track_max_m: float | None
    overshoot_m: float | None
    command_count: int
    angular_flip_count: int = 0
    snake_candidates: tuple[dict[str, Any], ...] = ()
    response_lag_median_s: float | None = None
    response_lag_p95_s: float | None = None
    response_lag_evidence: str = "UNKNOWN"
    evidence: str = "OBSERVED_ODOM"


def load_navigation_trace(path: str | Path) -> list[dict[str, Any]]:
    events: list[dict[str, Any]] = []
    for line_number, line in enumerate(Path(path).read_text(encoding="utf-8").splitlines(), 1):
        if not line.strip():
            continue
        try:
            row = json.loads(line)
        except json.JSONDecodeError as exc:
            events.append({"event": "malformed", "line_number": line_number, "error": str(exc)})
            continue
        if isinstance(row, dict):
            events.append(row)
        else:
            events.append({"event": "malformed", "line_number": line_number, "error": "not an object"})
    return events


def summarize_navigation_trace(events: Iterable[dict[str, Any]]) -> NavigationTraceSummary:
    sessions: set[str] = set()
    started: set[str] = set()
    ended: set[str] = set()
    dropped_events = 0
    writer_errors: list[str] = []
    goals = plans = failed_plans = emergency_stops = malformed = maximum_replans = 0
    statuses: dict[str, int] = {}
    reasons: dict[str, int] = {}
    for row in events:
        event = row.get("event")
        session_id = row.get("session_id")
        if isinstance(session_id, str):
            sessions.add(session_id)
            if event == "session_started":
                started.add(session_id)
            elif event == "session_ended":
                ended.add(session_id)
        if event == "goal_accepted":
            goals += 1
        elif event == "plan":
            plans += 1
        elif event == "plan_failed":
            failed_plans += 1
        elif event == "emergency_stop":
            emergency_stops += 1
        elif event == "malformed":
            malformed += 1
        elif event == "trace_writer_closed":
            try:
                dropped_events = max(dropped_events, int(row.get("dropped_events", 0)))
            except (TypeError, ValueError):
                malformed += 1
            if row.get("writer_error"):
                writer_errors.append(str(row["writer_error"]))
        if event == "finished":
            status = str(row.get("status") or "unknown")
            reason = str(row.get("stop_reason") or "none")
            statuses[status] = statuses.get(status, 0) + 1
            reasons[reason] = reasons.get(reason, 0) + 1
        try:
            maximum_replans = max(maximum_replans, int(row.get("replan_count", 0)))
        except (TypeError, ValueError):
            malformed += 1
    return NavigationTraceSummary(
        sessions=len(sessions), goals=goals, plans=plans,
        failed_plans=failed_plans, emergency_stops=emergency_stops,
        terminal_statuses=statuses, stop_reasons=reasons,
        maximum_replan_count=maximum_replans, malformed_rows=malformed,
        recovered_open_sessions=len(started-ended), dropped_events=dropped_events,
        writer_errors=tuple(writer_errors),
    )


def navigation_trajectory_metrics(
    events: Iterable[dict[str, Any]],
) -> NavigationTrajectoryMetrics:
    rows = list(events)
    segments: list[dict[str, Any]] = []
    current: dict[str, Any] | None = None
    for row in rows:
        if row.get("event") in {"plan", "plan_geometry"} and isinstance(row.get("path_xy"), list):
            candidate = _xy_rows(row["path_xy"])
            if len(candidate) >= 2:
                current = {"path": candidate, "samples": [], "sample_ts": [],
                           "sample_yaw": [], "command_ts": [], "command_yaw": []}
                segments.append(current)
        elif row.get("event") == "motion_sample" and current is not None:
            try:
                point = (float(row["x_m"]), float(row["y_m"]))
            except (KeyError, TypeError, ValueError):
                continue
            if not all(math.isfinite(value) for value in point):
                continue
            current["samples"].append(point)
            current["sample_ts"].append(_event_time_s(row))
            try:
                yaw = float(row.get("yaw_degrees", 0.0))
            except (TypeError, ValueError):
                yaw = 0.0
            current["sample_yaw"].append(yaw if math.isfinite(yaw) else 0.0)
        elif row.get("event") == "command" and current is not None:
            current["command_ts"].append(_event_time_s(row))
            try:
                yaw = float(row.get("yaw_rps", 0.0))
            except (TypeError, ValueError):
                yaw = 0.0
            current["command_yaw"].append(yaw if math.isfinite(yaw) else 0.0)
    all_errors: list[float] = []
    planned_lengths, odom_length, overshoot, commands, flips = [], 0.0, 0.0, 0, 0
    snakes: list[dict[str, Any]] = []
    response_lags: list[float] = []
    sample_count = 0
    for segment in segments:
        path, samples = segment["path"], segment["samples"]
        planned_lengths.append(_polyline_length(path))
        odom_length += _polyline_length(samples)
        signed = [_signed_point_polyline_distance(sample, path) for sample in samples]
        all_errors.extend(abs(value) for value in signed)
        sample_count += len(samples)
        overshoot = max(overshoot, _tangent_overshoot(path, samples))
        command_yaw = segment["command_yaw"]
        commands += len(command_yaw)
        flips += _sign_flip_count(command_yaw)
        yaw_rates = _yaw_rates(segment["sample_ts"], segment["sample_yaw"])
        candidate = _snake_candidate(segment["sample_ts"], signed, yaw_rates)
        if candidate is not None:
            snakes.append(candidate)
        response_lags.extend(_response_lags(
            segment["command_ts"], command_yaw,
            segment["sample_ts"], yaw_rates,
        ))
    path_length = planned_lengths[-1] if planned_lengths else 0.0
    errors = all_errors
    rms = math.sqrt(sum(value * value for value in errors) / len(errors)) if errors else None
    ordered = sorted(errors)
    p95 = ordered[min(len(ordered)-1, math.ceil(len(ordered)*0.95)-1)] if ordered else None
    maximum = max(errors) if errors else None
    return NavigationTrajectoryMetrics(
        samples=sample_count, planned_path_length_m=path_length,
        odom_path_length_m=odom_length, cross_track_rms_m=rms,
        cross_track_p95_m=p95, cross_track_max_m=maximum,
        overshoot_m=overshoot if segments else None, command_count=commands,
        angular_flip_count=flips, snake_candidates=tuple(snakes),
        response_lag_median_s=_percentile(response_lags, 50),
        response_lag_p95_s=_percentile(response_lags, 95),
        response_lag_evidence="CORRELATED" if response_lags else "UNKNOWN",
    )


def build_navigation_report(events: Iterable[dict[str, Any]]) -> dict[str, Any]:
    rows = list(events)
    from robot_brain.navigation.visual_controller import evaluate_visual_servo_trace
    from robot_brain.navigation.terrain_controller import evaluate_terrain_execution_trace
    from robot_brain.navigation.exploration import evaluate_exploration_trace
    from robot_brain.navigation.patrol_controller import evaluate_patrol_trace
    from robot_brain.navigation.terrain_exploration import evaluate_terrain_exploration_trace
    has_visual = any(str(row.get("event", "")).startswith("visual_servo_") for row in rows)
    has_terrain = any(str(row.get("event", "")).startswith("terrain_execution_") for row in rows)
    has_exploration = any(str(row.get("event", "")).startswith("exploration_") for row in rows)
    has_patrol = any(str(row.get("event", "")).startswith("patrol_") for row in rows)
    has_terrain_exploration = any(str(row.get("event", "")).startswith(
        "terrain_exploration_") for row in rows)
    return {
        "schema_version": 1,
        "summary": summarize_navigation_trace(rows).__dict__,
        "trajectory": navigation_trajectory_metrics(rows).__dict__,
        "visual_servo": evaluate_visual_servo_trace(rows) if has_visual else None,
        "terrain_execution": evaluate_terrain_execution_trace(rows) if has_terrain else None,
        "exploration": evaluate_exploration_trace(rows) if has_exploration else None,
        "patrol": evaluate_patrol_trace(rows) if has_patrol else None,
        "terrain_exploration": (evaluate_terrain_exploration_trace(rows)
                                if has_terrain_exploration else None),
        "evidence_notes": {
            "pose_source": "reported odometry; not external ground truth",
            "terminal_status": "provider-observed",
            "command_events": "issued commands; not proof of physical execution",
        },
    }


def _xy_rows(value: object) -> list[tuple[float, float]]:
    result = []
    if not isinstance(value, list):
        return result
    for row in value:
        if isinstance(row, (list, tuple)) and len(row) >= 2:
            try:
                point = (float(row[0]), float(row[1]))
            except (TypeError, ValueError):
                continue
            if all(math.isfinite(axis) for axis in point):
                result.append(point)
    return result


def _polyline_length(points: list[tuple[float, float]]) -> float:
    return sum(math.hypot(b[0]-a[0], b[1]-a[1]) for a, b in zip(points, points[1:]))


def _point_polyline_distance(point: tuple[float, float], path: list[tuple[float, float]]) -> float:
    best = math.inf
    for start, end in zip(path, path[1:]):
        dx, dy = end[0]-start[0], end[1]-start[1]
        denominator = dx*dx + dy*dy
        ratio = 0.0 if denominator <= 1e-12 else max(0.0, min(1.0,
            ((point[0]-start[0])*dx + (point[1]-start[1])*dy) / denominator))
        projected = (start[0]+ratio*dx, start[1]+ratio*dy)
        best = min(best, math.hypot(point[0]-projected[0], point[1]-projected[1]))
    return best


def _signed_point_polyline_distance(point, path) -> float:
    best_distance, best_signed = math.inf, 0.0
    for start, end in zip(path, path[1:]):
        dx, dy = end[0]-start[0], end[1]-start[1]
        length_sq = dx*dx+dy*dy
        if length_sq <= 1e-12:
            continue
        ratio = max(0.0, min(1.0, ((point[0]-start[0])*dx+(point[1]-start[1])*dy)/length_sq))
        px, py = start[0]+ratio*dx, start[1]+ratio*dy
        distance = math.hypot(point[0]-px, point[1]-py)
        if distance < best_distance:
            best_distance = distance
            cross = dx*(point[1]-py)-dy*(point[0]-px)
            best_signed = math.copysign(distance, cross) if distance else 0.0
    return best_signed


def _tangent_overshoot(path, samples) -> float:
    if len(path) < 2 or not samples:
        return 0.0
    dx, dy = path[-1][0]-path[-2][0], path[-1][1]-path[-2][1]
    length = math.hypot(dx, dy)
    if length <= 1e-12:
        return 0.0
    return max(0.0, max((x-path[-1][0])*dx/length+(y-path[-1][1])*dy/length
                        for x, y in samples))


def _event_time_s(row) -> float:
    for key, scale in (("monotonic_ns", 1e-9), ("monotonic", 1.0)):
        try:
            value = float(row[key])*scale
        except (KeyError, TypeError, ValueError):
            continue
        if math.isfinite(value):
            return value
    return 0.0


def _sign_flip_indices(values, deadband=.05):
    result, previous = [], 0
    for index, value in enumerate(values):
        sign = 1 if value > deadband else -1 if value < -deadband else 0
        if sign and previous and sign != previous:
            result.append(index)
        if sign:
            previous = sign
    return result


def _sign_flip_count(values, deadband=.05) -> int:
    return len(_sign_flip_indices(values, deadband))


def _yaw_rates(timestamps, yaw_degrees):
    if not timestamps:
        return []
    rates = [0.0]
    for index in range(1, len(timestamps)):
        elapsed = timestamps[index]-timestamps[index-1]
        delta = (yaw_degrees[index]-yaw_degrees[index-1]+180)%360-180
        rates.append(0.0 if elapsed <= 0 else math.radians(delta)/elapsed)
    return rates


def _snake_candidate(timestamps, signed, angular):
    if len(timestamps) < 4 or len(signed) != len(angular):
        return None
    amplitude = _percentile([abs(value) for value in signed], 95) or 0.0
    crossings = _sign_flip_indices(signed, max(.0125, amplitude/4))
    angular_flips = _sign_flip_indices(angular, .05)
    if amplitude < .05 or len(crossings) < 3 or len(angular_flips) < 3:
        return None
    cycles = [timestamps[b]-timestamps[a] for a, b in zip(crossings, crossings[2:])]
    return {"start_monotonic_s": timestamps[crossings[0]],
            "end_monotonic_s": timestamps[crossings[-1]],
            "amplitude_m": amplitude, "period_s": _percentile(cycles, 50),
            "cross_track_zero_crossings": len(crossings),
            "angular_flips": len(angular_flips), "evidence": "CORRELATED"}


def _response_lags(command_ts, commands, odom_ts, yaw_rates, maximum=2.0):
    lags = []
    for index in _sign_flip_indices(commands):
        target = 1 if commands[index] > 0 else -1
        for timestamp, rate in zip(odom_ts, yaw_rates):
            lag = timestamp-command_ts[index]
            if 0 <= lag <= maximum and ((rate > .05) - (rate < -.05)) == target:
                lags.append(lag)
                break
    return lags


def _percentile(values, percentile):
    if not values:
        return None
    ordered = sorted(values)
    index = min(len(ordered)-1, max(0, math.ceil(len(ordered)*percentile/100)-1))
    return ordered[index]


_SENSITIVE = re.compile(r"password|passwd|token|api.?key|secret|credential|username|serial", re.I)


def _redact(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(key): "<redacted>" if _SENSITIVE.search(str(key)) else _redact(child)
                for key, child in value.items()}
    if isinstance(value, (list, tuple)):
        return [_redact(child) for child in value]
    return value


def _json_safe(value: Any) -> Any:
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {str(key): _json_safe(child) for key, child in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(child) for child in value]
    return value
