"""Room capture and memory-first cross-room object search."""
from __future__ import annotations

import math

from pydantic import BaseModel, Field

from robot_brain.actuation.base import RobotInterface
from robot_brain.core.world_state import Position, WorldState
from robot_brain.memory.spatial import ObjectObservation, RoomMemory, SpatialMemoryStore
from robot_brain.skills.base import Skill, SkillResult
from robot_brain.tools.base import CapabilityMetadata, MotionKind, RiskLevel
from robot_brain.vlm.frame_source import FrameSource
from robot_brain.vlm.object_recognition import ObjectRecognizer, VisualObject


class RememberRoomParams(BaseModel):
    room_name: str = Field(min_length=1, max_length=80)


class FindObjectParams(BaseModel):
    object_name: str = Field(min_length=1, max_length=80)
    travel_check_interval_s: float = Field(default=2.0, ge=0.5, le=10.0)


class _SpatialSkill(Skill):
    def __init__(self, store: SpatialMemoryStore, frames: FrameSource, recognizer: ObjectRecognizer) -> None:
        self._store, self._frames, self._recognizer = store, frames, recognizer

    async def _look(self, target: str | None = None) -> list[VisualObject]:
        frame = await self._frames.get_frame()
        return [] if not frame else await self._recognizer.recognize(frame, target)

    async def _turn_to(self, heading: float, robot: RobotInterface, world: WorldState) -> None:
        from robot_brain.actuation.unitree import UnitreeRobot

        if isinstance(robot, UnitreeRobot):
            from robot_brain.skills.builtin import go2_motion

            delta = math.radians(_normalize(heading - world.heading_degrees))
            if abs(delta) >= math.radians(1.0):
                speed = go2_motion.YAW_SPEED if delta > 0 else -go2_motion.YAW_SPEED
                durations = go2_motion.plan_yaw_segments(delta, robot._settings.unitree_max_drive_duration)
                result = await go2_motion.run_go2_drive_segments(robot, vyaw=speed, durations=durations)
                if not result["success"]:
                    raise RuntimeError("Go2 rotation failed during spatial search")
        else:
            await robot.turn(heading)
        world.heading_degrees = heading

    async def _move_to(self, point: Position, robot: RobotInterface, world: WorldState) -> None:
        from robot_brain.actuation.unitree import UnitreeRobot

        if not isinstance(robot, UnitreeRobot):
            await robot.move_to(point, speed=0.4)
            return
        from robot_brain.skills.builtin import go2_motion

        dx, dy = point.x - world.position.x, point.y - world.position.y
        yaw = math.radians(world.heading_degrees)
        forward = dx * math.cos(yaw) + dy * math.sin(yaw)
        left = -dx * math.sin(yaw) + dy * math.cos(yaw)
        for distance, axis in ((forward, "forward"), (left, "left")):
            if abs(distance) < 0.01:
                continue
            speed = math.copysign(go2_motion.LINEAR_SPEED, distance)
            durations = go2_motion.plan_linear_segments(abs(distance), robot._settings.unitree_max_drive_duration)
            kwargs = {"vx": speed} if axis == "forward" else {"vy": speed}
            result = await go2_motion.run_go2_drive_segments(robot, durations=durations, **kwargs)
            if not result["success"]:
                raise RuntimeError("Go2 travel failed during spatial search")


class RememberRoomSkill(_SpatialSkill):
    name = "remember_room"
    description = "Mark the current room, rotate 60 degrees five times, photograph it, and remember visible objects with pose."
    params_model = RememberRoomParams
    capability_metadata = CapabilityMetadata(risk_level=RiskLevel.MEDIUM,
        motion_kind=MotionKind.YAW, requires_confirmation=True, planner_visible=True,
        tags=frozenset({"spatial_memory", "vision", "scan"}))

    async def execute(self, params: RememberRoomParams, robot: RobotInterface, world: WorldState) -> SkillResult:
        room = RoomMemory(name=params.room_name, anchor=world.position.model_copy(deep=True),
                          anchor_heading_degrees=world.heading_degrees)
        observations: list[ObjectObservation] = []
        for _ in range(5):
            heading = _normalize(world.heading_degrees + 60.0)
            await self._turn_to(heading, robot, world)
            for item in await self._look():
                observations.append(ObjectObservation(room_name=room.name, object_name=item.name,
                    position=world.position.model_copy(deep=True), heading_degrees=world.heading_degrees,
                    confidence=item.confidence, bbox=item.bbox))
        self._store.save_room(room)
        self._store.replace_room_observations(room.name, observations)
        return SkillResult(success=True, message=f"remembered room {room.name}",
                           data={"room": room.model_dump(mode="json"), "photo_count": 5,
                                 "objects": [o.model_dump(mode="json") for o in observations]})


class FindObjectSkill(_SpatialSkill):
    name = "find_object"
    description = "Find an object using remembered room/heading first, then scan other rooms; recognizes during travel and waves when found."
    params_model = FindObjectParams
    capability_metadata = CapabilityMetadata(risk_level=RiskLevel.MEDIUM,
        motion_kind=MotionKind.LINEAR, requires_confirmation=True, planner_visible=True,
        tags=frozenset({"spatial_memory", "vision", "navigation"}))

    async def execute(self, params: FindObjectParams, robot: RobotInterface, world: WorldState) -> SkillResult:
        memories = self._store.observations(params.object_name)
        rooms = self._ordered_rooms(memories)
        checked: list[str] = []
        for room in rooms:
            found = await self._travel_and_watch(room.anchor, params, robot, world)
            if found:
                return await self._finish(found, params.object_name, "travel", checked, robot, world)
            checked.append(room.name)
            headings = [m.heading_degrees for m in memories if m.room_name == room.name]
            headings += [_normalize(room.anchor_heading_degrees + 60.0 * i) for i in range(6)]
            for heading in _unique_headings(headings):
                await self._turn_to(heading, robot, world)
                found = _target(await self._look(params.object_name), params.object_name)
                if found:
                    return await self._finish(found, params.object_name, room.name, checked, robot, world)
        return SkillResult(success=False, message=f"object not found: {params.object_name}",
                           data={"rooms_checked": checked, "stop_reason": "exhausted_memory"})

    def _ordered_rooms(self, memories: list[ObjectObservation]) -> list[RoomMemory]:
        rooms = self._store.rooms()
        rank = {name: i for i, name in enumerate(dict.fromkeys(m.room_name for m in memories))}
        return sorted(rooms, key=lambda r: (rank.get(r.name, len(rank)), -r.updated_at.timestamp()))

    async def _travel_and_watch(self, target: Position, params: FindObjectParams,
                                robot: RobotInterface, world: WorldState) -> VisualObject | None:
        distance = world.position.distance_to(target)
        # Search while travelling at the configured time cadence. Navigation
        # speed is 0.4 m/s, so each leg represents one recognition interval.
        steps = max(1, math.ceil(distance / (0.4 * params.travel_check_interval_s)))
        start = world.position.model_copy(deep=True)
        for i in range(1, steps + 1):
            ratio = i / steps
            point = Position(x=start.x + (target.x - start.x) * ratio,
                             y=start.y + (target.y - start.y) * ratio)
            await self._move_to(point, robot, world)
            world.position = point
            found = _target(await self._look(params.object_name), params.object_name)
            if found:
                await robot.stop("target detected during travel")
                return found
        return None

    async def _finish(self, found: VisualObject, object_name: str, source: str, checked: list[str],
                      robot: RobotInterface, world: WorldState) -> SkillResult:
        if found.bbox:
            center_x = (found.bbox[0] + found.bbox[2]) / 2.0
            heading = _normalize(world.heading_degrees + (center_x - 0.5) * 70.0)
            await self._turn_to(heading, robot, world)
        wave = getattr(robot, "wave", None)
        if callable(wave):
            await wave()
        else:
            await robot.report(f"wave at {object_name}", "info")
        return SkillResult(success=True, message=f"found {object_name} and waved",
                           data={"found": found.model_dump(mode="json"), "source": source,
                                 "rooms_checked": checked, "final_heading": world.heading_degrees})


def _target(items: list[VisualObject], name: str) -> VisualObject | None:
    matches = [i for i in items if i.name.casefold() == name.casefold()]
    return max(matches, key=lambda i: i.confidence) if matches else None


def _normalize(degrees: float) -> float:
    return (degrees + 180.0) % 360.0 - 180.0


def _unique_headings(values: list[float]) -> list[float]:
    result: list[float] = []
    for value in values:
        value = _normalize(value)
        if not any(abs(_normalize(value - old)) < 1.0 for old in result):
            result.append(value)
    return result
