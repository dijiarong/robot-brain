"""Map/session-bound room capture and memory-first object search."""
from __future__ import annotations

import asyncio
import math
import time

from pydantic import BaseModel, Field

from robot_brain.actuation.base import RobotInterface
from robot_brain.core.world_state import Position, WorldState
from robot_brain.memory.spatial import ObjectObservation, RoomMemory, SpatialMemoryStore
from robot_brain.navigation import (
    AbsoluteNavigationGoal,
    LocalizationState,
    NavigationClient,
    NavigationPose,
    NavigationStatus,
    RelativeNavigationGoal,
)
from robot_brain.skills.base import Skill, SkillResult
from robot_brain.tools.base import CapabilityMetadata, MotionKind, RiskLevel
from robot_brain.vlm.frame_source import FrameSource
from robot_brain.vlm.object_recognition import ObjectRecognizer, VisualObject


class RememberRoomParams(BaseModel):
    room_name: str = Field(min_length=1, max_length=80)


class FindObjectParams(BaseModel):
    object_name: str = Field(min_length=1, max_length=80)
    travel_check_interval_s: float = Field(default=2.0, ge=0.5, le=10.0)


class _NavigationLegError(RuntimeError):
    def __init__(self, reason: str, message: str) -> None:
        super().__init__(message)
        self.reason = reason


class _SpatialSkill(Skill):
    def __init__(
        self,
        store: SpatialMemoryStore,
        frames: FrameSource,
        recognizer: ObjectRecognizer,
        navigation: NavigationClient,
    ) -> None:
        self._store = store
        self._frames = frames
        self._recognizer = recognizer
        self._navigation = navigation

    async def _look(self, target: str | None = None) -> list[VisualObject]:
        frame = await self._frames.get_frame()
        return [] if not frame else await self._recognizer.recognize(frame, target)

    async def _localization(self) -> LocalizationState:
        state = await self._navigation.get_localization_state()
        if state.pose is None or state.map_identity is None:
            raise _NavigationLegError(
                "localization_unavailable",
                state.message or "navigation localization is unavailable",
            )
        if state.pose.frame_id != state.map_identity.frame_id:
            raise _NavigationLegError(
                "localization_frame_mismatch",
                "localization pose frame does not match map/session frame",
            )
        return state

    @staticmethod
    def _memory_context(localization: LocalizationState) -> dict[str, object]:
        identity = localization.map_identity
        assert identity is not None
        return {
            "map_id": identity.map_id,
            "map_version": identity.version,
            "frame_id": identity.frame_id,
            "session_id": None if identity.persistent else identity.map_id,
            "persistent_map": identity.persistent,
        }

    @staticmethod
    def _sync_world(world: WorldState, localization: LocalizationState) -> None:
        if localization.pose is None:
            return
        world.position = Position(x=localization.pose.x_m, y=localization.pose.y_m)
        world.heading_degrees = localization.pose.yaw_degrees

    async def _wait_goal(
        self,
        goal_id: str,
        *,
        timeout_s: float,
        target_name: str | None = None,
        check_interval_s: float = 0.5,
    ) -> VisualObject | None:
        deadline = time.monotonic() + timeout_s
        while True:
            if target_name is not None:
                found = _target(await self._look(target_name), target_name)
                if found is not None:
                    await self._navigation.cancel(goal_id)
                    return found
            state = await self._navigation.get_state()
            if state.status.terminal:
                if state.status == NavigationStatus.SUCCEEDED:
                    return None
                raise _NavigationLegError(
                    state.error_code or state.status.value,
                    state.message or f"navigation {state.status.value}",
                )
            if time.monotonic() >= deadline:
                await self._navigation.cancel(goal_id)
                raise _NavigationLegError("timed_out", "navigation timed out")
            await asyncio.sleep(min(check_interval_s, max(0.0, deadline - time.monotonic())))

    async def _relative(
        self,
        *,
        forward_m: float = 0.0,
        left_m: float = 0.0,
        yaw_degrees: float = 0.0,
        timeout_s: float = 12.0,
        target_name: str | None = None,
        check_interval_s: float = 0.5,
    ) -> VisualObject | None:
        handle = await self._navigation.set_relative_goal(RelativeNavigationGoal(
            forward_m=forward_m,
            left_m=left_m,
            yaw_degrees=yaw_degrees,
            max_duration_s=min(20.0, timeout_s),
        ))
        if not handle.accepted:
            raise _NavigationLegError("rejected", handle.message or "navigation rejected")
        return await self._wait_goal(
            handle.goal_id,
            timeout_s=timeout_s,
            target_name=target_name,
            check_interval_s=check_interval_s,
        )

    async def _turn_to(self, heading: float, world: WorldState) -> None:
        localization = await self._localization()
        current = localization.pose.yaw_degrees  # type: ignore[union-attr]
        delta = _normalize(heading - current)
        while abs(delta) >= 1.0:
            step = max(-90.0, min(90.0, delta))
            await self._relative(yaw_degrees=step)
            delta -= step
        after = await self._localization()
        self._sync_world(world, after)

    async def _travel_and_watch(
        self,
        room: RoomMemory,
        params: FindObjectParams,
        world: WorldState,
    ) -> VisualObject | None:
        localization = await self._localization()
        identity = localization.map_identity
        pose = localization.pose
        assert identity is not None and pose is not None
        if room.map_id != identity.map_id or room.frame_id != identity.frame_id:
            raise _NavigationLegError("map_mismatch", "room belongs to another map/session")
        if room.map_version is not None and room.map_version != identity.version:
            raise _NavigationLegError("map_version_mismatch", "room belongs to another map version")

        if identity.persistent:
            if not localization.usable_for_persistent_memory or not self._navigation.supports_absolute_goals:
                raise _NavigationLegError(
                    "persistent_navigation_unavailable",
                    "persistent room requires valid map localization and absolute navigation",
                )
            handle = await self._navigation.set_absolute_goal(AbsoluteNavigationGoal(
                pose=NavigationPose(
                    x_m=room.anchor.x,
                    y_m=room.anchor.y,
                    yaw_degrees=room.anchor_heading_degrees,
                    frame_id=room.frame_id,
                ),
                map_id=room.map_id,
                map_version=room.map_version,
                max_duration_s=60.0,
            ))
            if not handle.accepted:
                raise _NavigationLegError("rejected", handle.message or "room goal rejected")
            found = await self._wait_goal(
                handle.goal_id,
                timeout_s=60.0,
                target_name=params.object_name,
                check_interval_s=params.travel_check_interval_s,
            )
            self._sync_world(world, await self._localization())
            return found

        # Session-local odometry goals are split to respect the bounded relative
        # NavigationClient contract. Every chunk remains provider-controlled.
        while True:
            localization = await self._localization()
            pose = localization.pose
            assert pose is not None
            dx, dy = room.anchor.x - pose.x_m, room.anchor.y - pose.y_m
            if math.hypot(dx, dy) <= 0.05:
                self._sync_world(world, localization)
                return None
            yaw = math.radians(pose.yaw_degrees)
            forward = dx * math.cos(yaw) + dy * math.sin(yaw)
            left = -dx * math.sin(yaw) + dy * math.cos(yaw)
            scale = min(1.0, 1.0 / max(abs(forward), 1e-9), 0.5 / max(abs(left), 1e-9))
            found = await self._relative(
                forward_m=forward * scale,
                left_m=left * scale,
                target_name=params.object_name,
                check_interval_s=params.travel_check_interval_s,
            )
            self._sync_world(world, await self._localization())
            if found is not None:
                return found


class RememberRoomSkill(_SpatialSkill):
    name = "remember_room"
    description = "Capture a map/session-bound room observation by safely rotating through five views."
    params_model = RememberRoomParams
    capability_metadata = CapabilityMetadata(
        risk_level=RiskLevel.MEDIUM,
        motion_kind=MotionKind.YAW,
        requires_confirmation=True,
        planner_visible=True,
        tags=frozenset({"spatial_memory", "vision", "scan", "navigation"}),
    )

    async def execute(
        self, params: RememberRoomParams, robot: RobotInterface, world: WorldState
    ) -> SkillResult:
        try:
            localization = await self._localization()
            self._sync_world(world, localization)
            pose = localization.pose
            assert pose is not None
            context = self._memory_context(localization)
            room = RoomMemory(
                name=params.room_name,
                anchor=Position(x=pose.x_m, y=pose.y_m),
                anchor_heading_degrees=pose.yaw_degrees,
                **context,
            )
            observations: list[ObjectObservation] = []
            for _ in range(5):
                await self._relative(yaw_degrees=60.0)
                localization = await self._localization()
                self._sync_world(world, localization)
                pose = localization.pose
                assert pose is not None
                for item in await self._look():
                    observations.append(ObjectObservation(
                        room_name=room.name,
                        object_name=item.name,
                        position=Position(x=pose.x_m, y=pose.y_m),
                        heading_degrees=pose.yaw_degrees,
                        confidence=item.confidence,
                        bbox=item.bbox,
                        **context,
                    ))
            self._store.save_room(room)
            self._store.replace_room_observations(
                room.name, observations, map_id=room.map_id
            )
            return SkillResult(
                success=True,
                message=f"remembered room {room.name}",
                data={
                    "room": room.model_dump(mode="json"),
                    "photo_count": 5,
                    "objects": [item.model_dump(mode="json") for item in observations],
                },
            )
        except _NavigationLegError as exc:
            return SkillResult(
                success=False,
                message=f"remember room failed: {exc}",
                data={"stop_reason": exc.reason},
            )
        except Exception as exc:
            return SkillResult(
                success=False,
                message=f"remember room navigation error: {exc}",
                data={"stop_reason": "provider_error", "error": str(exc)},
            )


class FindObjectSkill(_SpatialSkill):
    name = "find_object"
    description = "Navigate through current-map observation points, recognize en route, and stop immediately on detection."
    params_model = FindObjectParams
    capability_metadata = CapabilityMetadata(
        risk_level=RiskLevel.MEDIUM,
        motion_kind=MotionKind.LINEAR,
        requires_confirmation=True,
        planner_visible=True,
        tags=frozenset({"spatial_memory", "vision", "navigation"}),
    )

    async def execute(
        self, params: FindObjectParams, robot: RobotInterface, world: WorldState
    ) -> SkillResult:
        try:
            localization = await self._localization()
            self._sync_world(world, localization)
            identity = localization.map_identity
            assert identity is not None
            memories = self._store.observations(params.object_name, map_id=identity.map_id)
            rooms = self._ordered_rooms(memories, identity.map_id)
            checked: list[str] = []
            for room in rooms:
                found = await self._travel_and_watch(room, params, world)
                if found:
                    return await self._finish(
                        found, params.object_name, "travel", checked, robot, world
                    )
                checked.append(room.name)
                headings = [
                    item.heading_degrees
                    for item in memories
                    if item.room_name == room.name
                ]
                headings += [
                    _normalize(room.anchor_heading_degrees + 60.0 * i)
                    for i in range(6)
                ]
                for heading in _unique_headings(headings):
                    await self._turn_to(heading, world)
                    found = _target(await self._look(params.object_name), params.object_name)
                    if found:
                        return await self._finish(
                            found, params.object_name, room.name, checked, robot, world
                        )
            return SkillResult(
                success=False,
                message=f"object not found: {params.object_name}",
                data={
                    "rooms_checked": checked,
                    "stop_reason": "exhausted_current_map_memory",
                    "map_id": identity.map_id,
                },
            )
        except _NavigationLegError as exc:
            return SkillResult(
                success=False,
                message=f"find object stopped: {exc}",
                data={"stop_reason": exc.reason},
            )
        except Exception as exc:
            return SkillResult(
                success=False,
                message=f"find object navigation error: {exc}",
                data={"stop_reason": "provider_error", "error": str(exc)},
            )

    def _ordered_rooms(
        self, memories: list[ObjectObservation], map_id: str
    ) -> list[RoomMemory]:
        rooms = self._store.rooms(map_id=map_id)
        rank = {
            name: index
            for index, name in enumerate(dict.fromkeys(item.room_name for item in memories))
        }
        return sorted(
            rooms,
            key=lambda room: (
                rank.get(room.name, len(rank)),
                -room.updated_at.timestamp(),
            ),
        )

    async def _finish(
        self,
        found: VisualObject,
        object_name: str,
        source: str,
        checked: list[str],
        robot: RobotInterface,
        world: WorldState,
    ) -> SkillResult:
        if found.bbox:
            center_x = (found.bbox[0] + found.bbox[2]) / 2.0
            localization = await self._localization()
            heading = _normalize(
                localization.pose.yaw_degrees + (center_x - 0.5) * 70.0  # type: ignore[union-attr]
            )
            await self._turn_to(heading, world)
        wave = getattr(robot, "wave", None)
        if callable(wave):
            await wave()
        else:
            await robot.report(f"wave at {object_name}", "info")
        return SkillResult(
            success=True,
            message=f"found {object_name} and waved",
            data={
                "found": found.model_dump(mode="json"),
                "source": source,
                "rooms_checked": checked,
                "final_heading": world.heading_degrees,
            },
        )


def _target(items: list[VisualObject], name: str) -> VisualObject | None:
    matches = [item for item in items if item.name.casefold() == name.casefold()]
    return max(matches, key=lambda item: item.confidence) if matches else None


def _normalize(degrees: float) -> float:
    return (degrees + 180.0) % 360.0 - 180.0


def _unique_headings(values: list[float]) -> list[float]:
    result: list[float] = []
    for value in values:
        value = _normalize(value)
        if not any(abs(_normalize(value - old)) < 1.0 for old in result):
            result.append(value)
    return result
