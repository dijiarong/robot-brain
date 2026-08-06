"""Camera-model based visual navigation primitives with bounded outputs."""
from __future__ import annotations

from dataclasses import dataclass
import math

from robot_brain.navigation.base import RelativeNavigationGoal


@dataclass(frozen=True)
class CameraIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int

    def __post_init__(self) -> None:
        if self.fx <= 0 or self.fy <= 0 or self.width <= 0 or self.height <= 0:
            raise ValueError("invalid camera intrinsics")


@dataclass(frozen=True)
class VisualServoCommand:
    forward_mps: float
    yaw_rps: float
    estimated_distance_m: float | None
    valid: bool
    reason: str = ""


def bbox_to_relative_goal(
    bbox: tuple[float, float, float, float],
    camera: CameraIntrinsics,
    *,
    goal_distance_m: float = 1.0,
    max_lateral_m: float = 1.0,
    max_duration_s: float = 30.0,
) -> RelativeNavigationGoal:
    x1, y1, x2, y2 = _validated_bbox(bbox, camera)
    center_x = (x1 + x2) / 2.0
    lateral = -((center_x - camera.cx) / camera.fx) * goal_distance_m
    return RelativeNavigationGoal(
        forward_m=min(3.0, max(0.0, goal_distance_m)),
        left_m=max(-max_lateral_m, min(max_lateral_m, lateral)),
        max_duration_s=max_duration_s,
    )


def compute_visual_servo(
    bbox: tuple[float, float, float, float],
    camera: CameraIntrinsics,
    *,
    assumed_object_width_m: float = 0.45,
    target_distance_m: float = 1.5,
    minimum_distance_m: float = 0.8,
    max_linear_mps: float = 0.5,
    max_yaw_rps: float = 0.8,
    linear_gain: float = 0.8,
    yaw_gain: float = 1.0,
) -> VisualServoCommand:
    try:
        x1, _, x2, _ = _validated_bbox(bbox, camera)
    except ValueError as exc:
        return VisualServoCommand(0.0, 0.0, None, False, str(exc))
    pixel_width = x2 - x1
    distance = assumed_object_width_m * camera.fx / pixel_width
    center_x = (x1 + x2) / 2.0
    normalized_x = (center_x - camera.cx) / camera.fx
    yaw = _clamp(-normalized_x * yaw_gain, -max_yaw_rps, max_yaw_rps)
    if distance < minimum_distance_m:
        forward = -0.6 * max_linear_mps
    else:
        turn_factor = 1.0 - min(abs(normalized_x) * 2.0, 0.7)
        forward = _clamp(
            (distance - target_distance_m) * linear_gain * turn_factor,
            -max_linear_mps, max_linear_mps,
        )
    if not all(math.isfinite(value) for value in (distance, forward, yaw)):
        return VisualServoCommand(0.0, 0.0, None, False, "non_finite_result")
    return VisualServoCommand(forward, yaw, distance, True)


def robust_target_from_points(
    points_xyz: tuple[tuple[float, float, float], ...],
    *,
    floor_height_m: float = 0.30,
    front_quantile: float = 0.25,
    min_points: int = 3,
) -> tuple[float, float, float] | None:
    points = [point for point in points_xyz if all(math.isfinite(value) for value in point)]
    elevated = [point for point in points if point[2] > floor_height_m]
    candidates = elevated if len(elevated) >= min_points else points
    if len(candidates) < min_points:
        return None
    ranked = sorted(candidates, key=lambda point: math.hypot(point[0], point[1]))
    count = max(min_points, math.ceil(len(ranked) * front_quantile))
    selected = ranked[:count]
    return tuple(sum(point[index] for point in selected) / len(selected) for index in range(3))  # type: ignore[return-value]


def compute_visual_servo_3d(
    target_body_xyz: tuple[float, float, float], *,
    target_distance_m: float = 1.5, minimum_distance_m: float = 0.8,
    max_linear_mps: float = 0.5, max_yaw_rps: float = 0.8,
    linear_gain: float = 0.8, yaw_gain: float = 1.5,
) -> VisualServoCommand:
    if len(target_body_xyz) != 3 or not all(math.isfinite(value) for value in target_body_xyz):
        return VisualServoCommand(0.0, 0.0, None, False, "invalid_3d_target")
    x, y, _ = target_body_xyz
    distance = math.hypot(x, y)
    if distance <= 1e-6:
        return VisualServoCommand(0.0, 0.0, distance, False, "degenerate_3d_target")
    angle = math.atan2(y, x)
    yaw = _clamp(angle * yaw_gain, -max_yaw_rps, max_yaw_rps)
    if distance < minimum_distance_m:
        forward = -0.6 * max_linear_mps
    else:
        turn_factor = 1.0 - min(abs(angle) / math.pi, 0.7)
        forward = _clamp((distance-target_distance_m) * linear_gain * turn_factor,
                         -max_linear_mps, max_linear_mps)
    return VisualServoCommand(forward, yaw, distance, True)


def detection_label_matches(label: str, query: str) -> bool:
    """Conservative multilingual target match used for VLM detections."""
    label, query = label.casefold().strip(), query.casefold().strip()
    if not query:
        return True
    if not label:
        return False
    if label == query or query in label or label in query:
        return True
    if query.replace(" ", "").isascii():
        return bool(set(label.split()) & set(query.split()))
    # CJK tokenization without a language model is ambiguous. Substring matches
    # above are safe; shared color/adjective characters are not sufficient for
    # selecting a physical motion target.
    return False


def detection_bbox_to_pixels(
    bbox: tuple[float, float, float, float], camera: CameraIntrinsics,
) -> tuple[float, float, float, float]:
    """Normalize fraction, Qwen-1000, pixel, or inferred-scale detections."""
    if len(bbox) != 4:
        raise ValueError("invalid_bbox")
    values = tuple(float(value) for value in bbox)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("invalid_bbox")
    x1, y1, x2, y2 = values
    if min(values) < 0 or x2 <= x1 or y2 <= y1:
        raise ValueError("invalid_bbox")
    maximum = max(values)
    if maximum <= 1.0:
        pixels = (x1*camera.width, y1*camera.height,
                  x2*camera.width, y2*camera.height)
    elif maximum <= 1000.0:
        pixels = (x1/1000*camera.width, y1/1000*camera.height,
                  x2/1000*camera.width, y2/1000*camera.height)
    elif x2 <= camera.width+1 and y2 <= camera.height+1:
        pixels = values
    else:
        scale = maximum/max(camera.width, camera.height)
        if not math.isfinite(scale) or scale <= 0:
            raise ValueError("invalid_bbox_scale")
        pixels = tuple(value/scale for value in values)
    return _validated_bbox(pixels, camera)


def _validated_bbox(bbox, camera):
    if len(bbox) != 4 or not all(math.isfinite(float(value)) for value in bbox):
        raise ValueError("invalid_bbox")
    x1, y1, x2, y2 = (float(value) for value in bbox)
    if x2 <= x1 or y2 <= y1:
        raise ValueError("empty_bbox")
    if x2 < 0 or y2 < 0 or x1 > camera.width or y1 > camera.height:
        raise ValueError("bbox_outside_image")
    return x1, y1, x2, y2


def _clamp(value, lower, upper):
    return max(lower, min(upper, value))
