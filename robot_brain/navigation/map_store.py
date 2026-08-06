"""Sparse native voxel map with deterministic, atomic persistence."""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile
from typing import Any
from uuid import uuid4

from robot_brain.core.robot_self_state import RobotPose
from robot_brain.navigation.grid import OccupancyGrid2D
from robot_brain.perception.pointcloud import PointCloudSnapshot

Voxel = tuple[int, int, int]


@dataclass(frozen=True)
class NativeMapIdentity:
    map_id: str
    version: str
    revision: str
    resolution_m: float
    voxel_count: int


class SparseVoxelMap:
    """Accumulate trusted body-frame observations in an odom/map frame.

    Counts suppress isolated noise and preserve enough information for later
    carving/decay work without importing Open3D or DIMOS message types.
    """

    SCHEMA_VERSION = 2

    def __init__(
        self, *, resolution_m: float = 0.10, map_id: str | None = None,
        map_version: str | None = None,
        max_voxels: int = 500_000,
    ) -> None:
        if not math.isfinite(resolution_m) or resolution_m <= 0:
            raise ValueError("map resolution must be positive")
        self.resolution_m = resolution_m
        self.map_id = map_id or f"native-map-{uuid4().hex}"
        self.map_version = map_version or f"v1-{uuid4().hex}"
        if max_voxels <= 0:
            raise ValueError("max_voxels must be positive")
        self.max_voxels = max_voxels
        self._hits: dict[Voxel, int] = {}
        self._misses: dict[Voxel, int] = {}
        self._known_free_xy: set[tuple[int, int]] = set()
        self._generation = 0

    @property
    def voxel_count(self) -> int:
        return len(self._hits)

    @property
    def generation(self) -> int:
        """Cheap monotonic revision for viewer caching."""
        return self._generation

    def integrate(
        self,
        cloud: PointCloudSnapshot,
        pose: RobotPose,
        *,
        min_z_m: float = -0.20,
        max_z_m: float = 1.80,
        carve_free_space: bool = False,
        carve_misses: int = 3,
    ) -> int:
        if cloud.frame_id not in {"base", "base_link", "unitree_lidar", "utlidar"}:
            raise ValueError(f"cannot integrate non-body cloud: {cloud.frame_id}")
        yaw = math.radians(pose.yaw_deg)
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
        touched: set[Voxel] = set()
        observed_rays: list[tuple[Voxel, set[Voxel]]] = []
        for x, y, z in cloud.points_xyz:
            if not all(math.isfinite(value) for value in (x, y, z)):
                continue
            if not min_z_m <= z <= max_z_m:
                continue
            wx = pose.x_m + x * cos_yaw - y * sin_yaw
            wy = pose.y_m + x * sin_yaw + y * cos_yaw
            wz = pose.z_m + z
            voxel = self._voxel(wx, wy, wz)
            if carve_free_space:
                ray = set(self._ray_voxels(
                    (pose.x_m, pose.y_m, pose.z_m), (wx, wy, wz)
                ))
                ray.discard(voxel)
                observed_rays.append((voxel, ray))
            if voxel in touched:
                continue
            touched.add(voxel)
            self._hits[voxel] = self._hits.get(voxel, 0) + 1
            self._misses.pop(voxel, None)
        if carve_free_space:
            free_voxels = set().union(*(ray for _, ray in observed_rays)) if observed_rays else set()
            free_voxels.difference_update(touched)
            self._known_free_xy.update((voxel[0], voxel[1]) for voxel in free_voxels)
            for voxel in free_voxels:
                if voxel not in self._hits:
                    continue
                misses = self._misses.get(voxel, 0) + 1
                if misses >= carve_misses:
                    self._hits.pop(voxel, None)
                    self._misses.pop(voxel, None)
                else:
                    self._misses[voxel] = misses
        self._enforce_limit()
        if touched:
            self._generation += 1
        return len(touched)

    def points(self, *, min_hits: int = 1) -> tuple[tuple[float, float, float], ...]:
        return tuple(
            self._center(voxel)
            for voxel, hits in sorted(self._hits.items())
            if hits >= min_hits
        )

    def points_in_cylinder(
        self, *, center_x_m: float, center_y_m: float, radius_m: float,
        z_min_m: float, z_max_m: float, min_hits: int = 1,
        max_points: int = 120_000,
    ) -> tuple[tuple[float, float, float], ...]:
        """Return a bounded local 3-D slice without sorting the global map."""
        if radius_m <= 0 or z_max_m < z_min_m or max_points <= 0:
            raise ValueError("invalid voxel-map cylinder bounds")
        radius_squared = radius_m * radius_m
        result: list[tuple[float, float, float]] = []
        for voxel, hits in self._hits.items():
            if hits < min_hits:
                continue
            point = self._center(voxel)
            if not z_min_m <= point[2] <= z_max_m:
                continue
            if (point[0]-center_x_m) ** 2 + (point[1]-center_y_m) ** 2 > radius_squared:
                continue
            result.append(point)
            if len(result) > max_points:
                raise ValueError("local terrain point budget exceeded")
        return tuple(result)

    def viewer_points_in_cylinder(
        self, *, center_x_m: float, center_y_m: float, radius_m: float,
        z_min_m: float, z_max_m: float, min_hits: int = 1,
        max_points: int = 6_000,
    ) -> tuple[tuple[float, float, float], ...]:
        """Return a deterministic, capped local sample for visualization.

        Navigation planning deliberately rejects oversized point sets.  A viewer
        should instead remain responsive, so this method keeps the closest
        voxels and thins the remainder without changing the planning contract.
        """
        if radius_m <= 0 or z_max_m < z_min_m or max_points <= 0:
            raise ValueError("invalid voxel-map cylinder bounds")
        radius_squared = radius_m * radius_m
        candidates: list[tuple[float, Voxel]] = []
        for voxel, hits in self._hits.items():
            if hits < min_hits:
                continue
            point = self._center(voxel)
            distance_squared = (
                (point[0] - center_x_m) ** 2 + (point[1] - center_y_m) ** 2
            )
            if distance_squared > radius_squared or not z_min_m <= point[2] <= z_max_m:
                continue
            candidates.append((distance_squared, voxel))
        candidates.sort(key=lambda item: (item[0], item[1]))
        return tuple(self._center(voxel) for _, voxel in candidates[:max_points])

    def viewer_overview_points(
        self, *, center_x_m: float, center_y_m: float,
        z_min_m: float, z_max_m: float, max_points: int = 1_800,
        local_radius_m: float = 2.5, far_lod_factor: int = 3,
    ) -> tuple[tuple[float, float, float, float], ...]:
        """Return the whole map with detailed nearby and merged distant voxels."""
        if max_points <= 0 or local_radius_m <= 0 or far_lod_factor < 2:
            raise ValueError("invalid viewer overview bounds")
        radius_squared = local_radius_m * local_radius_m
        local: list[tuple[float, Voxel]] = []
        distant: dict[Voxel, tuple[float, Voxel]] = {}
        for voxel, hits in self._hits.items():
            if hits < 1:
                continue
            point = self._center(voxel)
            if not z_min_m <= point[2] <= z_max_m:
                continue
            distance_squared = (
                (point[0]-center_x_m) ** 2 + (point[1]-center_y_m) ** 2
            )
            if distance_squared <= radius_squared:
                local.append((distance_squared, voxel))
                continue
            bucket = (
                voxel[0] // far_lod_factor,
                voxel[1] // far_lod_factor,
                voxel[2] // max(2, far_lod_factor // 2),
            )
            previous = distant.get(bucket)
            if previous is None or hits > self._hits[previous[1]]:
                distant[bucket] = (distance_squared, voxel)
        local.sort(key=lambda item: (item[0], item[1]))
        local_budget = min(len(local), max_points * 2 // 3)
        remaining = max_points-local_budget
        far = sorted(distant.values(), key=lambda item: (item[0], item[1]))[:remaining]
        return tuple(
            (*self._center(voxel), self.resolution_m)
            for _, voxel in local[:local_budget]
        ) + tuple(
            (*self._center(voxel), self.resolution_m*far_lod_factor)
            for _, voxel in far
        )

    def body_cloud(
        self,
        pose: RobotPose,
        *,
        radius_m: float,
        min_hits: int = 1,
    ) -> PointCloudSnapshot:
        yaw = math.radians(pose.yaw_deg)
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
        radius_squared = radius_m * radius_m
        body: list[tuple[float, float, float]] = []
        for wx, wy, wz in self.points(min_hits=min_hits):
            dx, dy = wx - pose.x_m, wy - pose.y_m
            if dx * dx + dy * dy > radius_squared:
                continue
            body.append((
                dx * cos_yaw + dy * sin_yaw,
                -dx * sin_yaw + dy * cos_yaw,
                wz - pose.z_m,
            ))
        return PointCloudSnapshot(
            points_xyz=tuple(body), frame_id="base_link", sensor_timestamp=pose.timestamp,
            received_monotonic=0.0, source=f"native_voxel_map:{self.map_id}",
            timestamp_valid=pose.timestamp is not None, origin_xyz=(0.0, 0.0, 0.0),
        )

    def identity(self) -> NativeMapIdentity:
        payload = self._payload(include_hash=False)
        revision = _payload_hash(payload)
        return NativeMapIdentity(
            map_id=self.map_id, version=self.map_version, revision=revision,
            resolution_m=self.resolution_m, voxel_count=self.voxel_count,
        )

    def occupancy_grid(
        self,
        *,
        center_x_m: float,
        center_y_m: float,
        size_m: float,
        obstacle_min_z_m: float = 0.05,
        obstacle_max_z_m: float = 1.20,
        robot_radius_m: float = 0.0,
        frame_id: str = "map",
    ) -> OccupancyGrid2D:
        cells = max(3, math.ceil(size_m / self.resolution_m))
        origin_x = math.floor(
            (center_x_m - cells * self.resolution_m / 2.0) / self.resolution_m
        ) * self.resolution_m
        origin_y = math.floor(
            (center_y_m - cells * self.resolution_m / 2.0) / self.resolution_m
        ) * self.resolution_m

        def local_cell(global_xy: tuple[int, int]) -> tuple[int, int] | None:
            wx = (global_xy[0] + 0.5) * self.resolution_m
            wy = (global_xy[1] + 0.5) * self.resolution_m
            col = math.floor((wx - origin_x) / self.resolution_m)
            row = math.floor((wy - origin_y) / self.resolution_m)
            return (col, row) if 0 <= col < cells and 0 <= row < cells else None

        occupied_global = {
            (voxel[0], voxel[1])
            for voxel in self._hits
            if obstacle_min_z_m <= (voxel[2] + 0.5) * self.resolution_m <= obstacle_max_z_m
        }
        occupied_raw = {
            cell for global_cell in occupied_global
            if (cell := local_cell(global_cell)) is not None
        }
        inflation = math.ceil(max(0.0, robot_radius_m) / self.resolution_m)
        occupied = {
            (cell[0] + dx, cell[1] + dy)
            for cell in occupied_raw
            for dx in range(-inflation, inflation + 1)
            for dy in range(-inflation, inflation + 1)
            if dx * dx + dy * dy <= inflation * inflation
            and 0 <= cell[0] + dx < cells and 0 <= cell[1] + dy < cells
        }
        known_free = {
            cell for global_cell in self._known_free_xy
            if (cell := local_cell(global_cell)) is not None
        }
        known_free.difference_update(occupied)
        return OccupancyGrid2D(
            resolution_m=self.resolution_m, width=cells, height=cells,
            origin_x_m=origin_x, origin_y_m=origin_y,
            occupied=frozenset(occupied), raw_occupied=frozenset(occupied_raw),
            known_free=frozenset(known_free),
            frame_id=frame_id,
        )

    def save(self, path: str | Path) -> NativeMapIdentity:
        target = Path(path)
        target.parent.mkdir(parents=True, exist_ok=True)
        payload = self._payload(include_hash=False)
        payload["content_hash"] = _payload_hash(payload)
        encoded = json.dumps(payload, ensure_ascii=False, sort_keys=True, separators=(",", ":"))
        descriptor, temporary = tempfile.mkstemp(
            prefix=f".{target.name}.", suffix=".tmp", dir=target.parent
        )
        try:
            with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
                stream.write(encoded)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, target)
        except Exception:
            try:
                os.unlink(temporary)
            except FileNotFoundError:
                pass
            raise
        return self.identity()

    @classmethod
    def load(cls, path: str | Path) -> "SparseVoxelMap":
        payload = json.loads(Path(path).read_text(encoding="utf-8"))
        schema_version = payload.get("schema_version")
        if schema_version not in {1, cls.SCHEMA_VERSION}:
            raise ValueError("unsupported native map schema")
        expected = payload.pop("content_hash", None)
        actual = _payload_hash(payload)
        if not expected or expected != actual:
            raise ValueError("native map content hash mismatch")
        result = cls(
            resolution_m=float(payload["resolution_m"]), map_id=str(payload["map_id"]),
            map_version=(
                str(payload["map_version"])
                if schema_version == cls.SCHEMA_VERSION
                else f"legacy-{str(expected)[:16]}"
            ),
            max_voxels=max(int(payload.get("max_voxels", 500_000)), len(payload.get("voxels", []))),
        )
        for row in payload.get("voxels", []):
            if not isinstance(row, list) or len(row) != 4:
                raise ValueError("invalid native map voxel")
            voxel = (int(row[0]), int(row[1]), int(row[2]))
            hits = int(row[3])
            if hits <= 0:
                raise ValueError("invalid native map hit count")
            result._hits[voxel] = hits
        for row in payload.get("known_free_xy", []):
            if not isinstance(row, list) or len(row) != 2:
                raise ValueError("invalid native map free cell")
            result._known_free_xy.add((int(row[0]), int(row[1])))
        return result

    def _voxel(self, x: float, y: float, z: float) -> Voxel:
        return (
            math.floor(x / self.resolution_m),
            math.floor(y / self.resolution_m),
            math.floor(z / self.resolution_m),
        )

    def _center(self, voxel: Voxel) -> tuple[float, float, float]:
        return tuple((axis + 0.5) * self.resolution_m for axis in voxel)  # type: ignore[return-value]

    def _ray_voxels(
        self, start: tuple[float, float, float], end: tuple[float, float, float]
    ) -> list[Voxel]:
        distance = math.dist(start, end)
        steps = max(1, math.ceil(distance / (self.resolution_m * 0.5)))
        return [
            self._voxel(
                start[0] + (end[0] - start[0]) * index / steps,
                start[1] + (end[1] - start[1]) * index / steps,
                start[2] + (end[2] - start[2]) * index / steps,
            )
            for index in range(steps + 1)
        ]

    def _enforce_limit(self) -> None:
        excess = len(self._hits) - self.max_voxels
        if excess <= 0:
            return
        victims = sorted(self._hits, key=lambda voxel: (self._hits[voxel], voxel))[:excess]
        for voxel in victims:
            self._hits.pop(voxel, None)
            self._misses.pop(voxel, None)

    def _payload(self, *, include_hash: bool) -> dict[str, Any]:
        payload: dict[str, Any] = {
            "schema_version": self.SCHEMA_VERSION,
            "map_id": self.map_id,
            "map_version": self.map_version,
            "resolution_m": self.resolution_m,
            "frame_id": "map",
            "max_voxels": self.max_voxels,
            "voxels": [[*voxel, hits] for voxel, hits in sorted(self._hits.items())],
            "known_free_xy": [list(cell) for cell in sorted(self._known_free_xy)],
        }
        if include_hash:
            payload["content_hash"] = _payload_hash(payload)
        return payload


def _payload_hash(payload: dict[str, Any]) -> str:
    encoded = json.dumps(payload, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()
