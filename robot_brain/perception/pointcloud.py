"""Backend-neutral point-cloud snapshots for navigation perception."""
from __future__ import annotations

from dataclasses import dataclass
import math
import time
from typing import Any


@dataclass(frozen=True)
class PointCloudSnapshot:
    """One read-only point-cloud frame with explicit timing semantics."""

    points_xyz: tuple[tuple[float, float, float], ...]
    frame_id: str
    sensor_timestamp: float | None
    received_monotonic: float
    source: str
    timestamp_valid: bool
    origin_xyz: tuple[float, float, float] | None = None

    @property
    def point_count(self) -> int:
        return len(self.points_xyz)

    def age_seconds(self, *, now_monotonic: float | None = None) -> float:
        now = time.monotonic() if now_monotonic is None else now_monotonic
        return max(0.0, now - self.received_monotonic)


def pointcloud_from_unitree_webrtc(
    message: Any,
    *,
    received_monotonic: float | None = None,
) -> PointCloudSnapshot | None:
    """Parse the native-decoder ULIDAR_ARRAY envelope without NumPy/Open3D.

    Unitree firmware variants wrap the decoded payload differently.  The
    accepted shape is the common ``data.data.points`` form, with a fallback to
    a top-level ``points`` list.  Malformed frames are rejected rather than
    being exposed to navigation as an empty obstacle observation.
    """
    if not isinstance(message, dict):
        return None
    payload = message.get("data", message)
    if isinstance(payload, str):
        return None
    if not isinstance(payload, dict):
        return None
    nested = payload.get("data")
    raw_points = nested.get("points") if isinstance(nested, dict) else None
    if raw_points is None:
        raw_points = payload.get("points")
    if raw_points is None:
        return None

    try:
        points: list[tuple[float, float, float]] = []
        for item in raw_points:
            if isinstance(item, (str, bytes)) or not hasattr(item, "__len__") or len(item) < 3:
                return None
            xyz = (float(item[0]), float(item[1]), float(item[2]))
            if all(math.isfinite(value) for value in xyz):
                points.append(xyz)
    except (TypeError, ValueError, OverflowError):
        return None
    if not points:
        return None

    stamp_raw = payload.get("stamp")
    try:
        stamp = float(stamp_raw) if stamp_raw is not None else None
    except (TypeError, ValueError, OverflowError):
        stamp = None
    stamp_valid = stamp is not None and math.isfinite(stamp) and stamp > 0.0
    origin = None
    raw_origin = payload.get("origin")
    try:
        if raw_origin is not None and len(raw_origin) >= 3:
            candidate = (float(raw_origin[0]), float(raw_origin[1]), float(raw_origin[2]))
            if all(math.isfinite(value) for value in candidate):
                origin = candidate
    except (TypeError, ValueError, OverflowError):
        origin = None
    return PointCloudSnapshot(
        points_xyz=tuple(points),
        frame_id=str(payload.get("frame_id") or "unitree_lidar"),
        sensor_timestamp=stamp if stamp_valid else None,
        received_monotonic=(
            time.monotonic() if received_monotonic is None else received_monotonic
        ),
        source="unitree_webrtc_ulidar",
        timestamp_valid=stamp_valid,
        origin_xyz=origin,
    )
