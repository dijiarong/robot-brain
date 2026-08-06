"""Planner-facing native navigation exploration, patrol, and map operations."""
from __future__ import annotations

from typing import Any

from pydantic import BaseModel, Field

from robot_brain.navigation import (
    NavigationPose, PatrolStrategy, create_patrol_route, evaluate_patrol_route,
)
from robot_brain.navigation.exploration import FrontierExplorationController
from robot_brain.navigation.native_go2 import NativeGo2NavigationClient
from robot_brain.navigation.patrol_controller import PatrolController
from robot_brain.navigation.terrain_controller import TerrainPathController
from robot_brain.navigation.terrain_exploration import TerrainFrontierExplorationController
from robot_brain.navigation.visual_navigation import (
    CameraIntrinsics,
    bbox_to_relative_goal,
    detection_bbox_to_pixels,
    detection_label_matches,
)
from robot_brain.navigation.visual_controller import (
    ContinuousVisualServoController,
    VisualTargetObservation,
)
from robot_brain.skills.base import Skill, SkillResult
from robot_brain.skills.builtin.navigation import NavigateRelativeParams, NavigateRelativeSkill
from robot_brain.tools.base import CapabilityMetadata, MotionKind, RiskLevel


class NativeExploreParams(BaseModel):
    max_goals: int = Field(default=10, ge=1, le=30)
    max_no_gain_attempts: int = Field(default=2, ge=1, le=5)


class NativePatrolParams(BaseModel):
    strategy: PatrolStrategy = PatrolStrategy.COVERAGE
    cycles: int = Field(default=1, ge=1, le=10)
    spacing_m: float = Field(default=0.75, ge=0.25, le=2.0)
    max_waypoints: int = Field(default=10, ge=1, le=30)


class BBoxNavigateParams(BaseModel):
    bbox: tuple[float, float, float, float]
    fx: float = Field(gt=0)
    fy: float = Field(gt=0)
    cx: float
    cy: float
    image_width: int = Field(gt=0)
    image_height: int = Field(gt=0)
    goal_distance_m: float = Field(default=1.0, ge=0.2, le=3.0)


class VisualServoNavigateParams(BaseModel):
    object_name: str = Field(min_length=1, max_length=80)
    fx: float = Field(gt=0)
    fy: float = Field(gt=0)
    cx: float
    cy: float
    image_width: int = Field(gt=0)
    image_height: int = Field(gt=0)
    target_distance_m: float = Field(default=1.5, ge=0.8, le=3.0)
    minimum_confidence: float = Field(default=0.6, ge=0.0, le=1.0)
    max_iterations: int = Field(default=30, ge=1, le=100)


class NativeRelocalizeParams(BaseModel):
    initial_x_m: float | None = None
    initial_y_m: float | None = None
    initial_yaw_degrees: float = 0.0
    allow_global_fallback: bool = False


class NativeTerrainNavigateParams(BaseModel):
    forward_m: float = Field(ge=-3.0, le=3.0)
    left_m: float = Field(ge=-3.0, le=3.0)
    up_m: float = Field(default=0.0, ge=-1.5, le=1.5)
    navigation_boundary_xy: tuple[tuple[float, float], ...] = Field(default=(), max_length=100)
    added_obstacles_xyz: tuple[tuple[float, float, float], ...] = Field(default=(), max_length=1000)
    added_obstacle_radius_m: float = Field(default=0.30, ge=0.0, le=2.0)


class NativeTerrainExploreParams(BaseModel):
    max_goals: int = Field(default=5, ge=1, le=20)
    exploration_range_m: float = Field(default=3.0, gt=0.0, le=10.0)
    navigation_boundary_xy: tuple[tuple[float, float], ...] = Field(default=(), max_length=100)
    added_obstacles_xyz: tuple[tuple[float, float, float], ...] = Field(default=(), max_length=1000)
    added_obstacle_radius_m: float = Field(default=0.30, ge=0.0, le=2.0)


class NativeExploreSkill(Skill):
    name = "nav_explore"
    description = "Explore mapped free-space frontiers with bounded goals and information-gain stopping."
    params_model = NativeExploreParams
    capability_metadata = CapabilityMetadata(
        risk_level=RiskLevel.MEDIUM, motion_kind=MotionKind.LINEAR,
        requires_confirmation=True, planner_visible=True,
        tags=frozenset({"navigation", "exploration", "frontier"}),
    )

    def __init__(self, client: NativeGo2NavigationClient) -> None:
        self._client = client

    async def execute(self, params, robot, world) -> SkillResult:
        controller = FrontierExplorationController(
            self._client, self._client.get_costmap,
            max_goals=params.max_goals,
            max_no_gain_attempts=params.max_no_gain_attempts,
            event_sink=getattr(self._client, "record_diagnostic_event", None),
        )
        result = await controller.run()
        success = result.goals_reached > 0 and result.stop_reason.value in {
            "complete", "max_goals", "no_information_gain",
        }
        return SkillResult(
            success=success,
            message=f"native exploration stopped: {result.stop_reason.value}",
            data={**result.__dict__, "stop_reason": result.stop_reason.value},
        )


class NativePatrolSkill(Skill):
    name = "nav_patrol"
    description = "Patrol known free space using coverage, frontier, random, or least-visited routing."
    params_model = NativePatrolParams
    capability_metadata = CapabilityMetadata(
        risk_level=RiskLevel.MEDIUM, motion_kind=MotionKind.LINEAR,
        requires_confirmation=True, planner_visible=True,
        tags=frozenset({"navigation", "patrol", "coverage"}),
    )

    def __init__(self, client: NativeGo2NavigationClient) -> None:
        self._client = client

    async def execute(self, params, robot, world) -> SkillResult:
        grid = await self._client.get_costmap()
        localization = await self._client.get_localization_state()
        if localization.pose is None or localization.pose.frame_id != grid.frame_id:
            return SkillResult(success=False, message="patrol localization/costmap frame mismatch", data={"stop_reason": "frame_mismatch"})
        points = create_patrol_route(
            grid, strategy=params.strategy,
            robot_xy=(localization.pose.x_m, localization.pose.y_m),
            spacing_m=params.spacing_m, max_waypoints=params.max_waypoints,
        )
        route = [
            NavigationPose(
                x_m=x, y_m=y, yaw_degrees=localization.pose.yaw_degrees,
                frame_id=grid.frame_id,
            ) for x, y in points
        ]
        event_sink = getattr(self._client, "record_diagnostic_event", None)
        route_evaluation = evaluate_patrol_route(
            grid, points, strategy=params.strategy,
        )
        if event_sink is not None:
            event_sink("patrol_route", {
                "strategy": params.strategy.value, "cycles": params.cycles,
                "spacing_m": params.spacing_m, "waypoints": len(route),
                "route_xy": [[pose.x_m, pose.y_m] for pose in route],
                "route_evaluation": route_evaluation,
            })
        result = await PatrolController(
            self._client, event_sink=event_sink,
        ).run(route, cycles=params.cycles)
        return SkillResult(
            success=result.completed,
            message=f"native patrol reached {result.reached}/{result.attempted} waypoint(s)",
            data={**result.__dict__, "route_evaluation": route_evaluation,
                  "stop_reason": "complete" if result.completed else "patrol_failed"},
        )


class NativeBBoxNavigateSkill(Skill):
    name = "nav_go_to_bbox"
    description = "Project a camera bounding box to a bounded relative navigation goal."
    params_model = BBoxNavigateParams
    capability_metadata = CapabilityMetadata(
        risk_level=RiskLevel.MEDIUM, motion_kind=MotionKind.LINEAR,
        requires_confirmation=True, planner_visible=True,
        tags=frozenset({"navigation", "vision", "bbox"}),
    )

    def __init__(self, client: NativeGo2NavigationClient) -> None:
        self._relative = NavigateRelativeSkill(client)

    async def execute(self, params, robot, world) -> SkillResult:
        try:
            goal = bbox_to_relative_goal(
                params.bbox,
                CameraIntrinsics(
                    fx=params.fx, fy=params.fy, cx=params.cx, cy=params.cy,
                    width=params.image_width, height=params.image_height,
                ),
                goal_distance_m=params.goal_distance_m,
            )
        except ValueError as exc:
            return SkillResult(success=False, message=f"bbox navigation rejected: {exc}", data={"stop_reason": "invalid_bbox"})
        return await self._relative.execute(
            NavigateRelativeParams.model_validate(goal.model_dump()), robot, world
        )


class NativeVisualServoSkill(Skill):
    name = "nav_visual_servo"
    description = "Continuously reacquire a named camera target and approach through bounded safe navigation segments."
    params_model = VisualServoNavigateParams
    capability_metadata = CapabilityMetadata(
        risk_level=RiskLevel.MEDIUM, motion_kind=MotionKind.LINEAR,
        requires_confirmation=True, planner_visible=True,
        tags=frozenset({"navigation", "vision", "visual_servo"}),
    )

    def __init__(self, client: NativeGo2NavigationClient, frames: Any, recognizer: Any) -> None:
        self._client = client
        self._frames = frames
        self._recognizer = recognizer

    async def execute(self, params, robot, world) -> SkillResult:
        camera = CameraIntrinsics(
            fx=params.fx, fy=params.fy, cx=params.cx, cy=params.cy,
            width=params.image_width, height=params.image_height,
        )

        async def observe() -> VisualTargetObservation | None:
            frame = await self._frames.get_frame()
            captured = getattr(self._frames, "last_frame_monotonic", None)
            if not frame or captured is None:
                return None
            objects = await self._recognizer.recognize(frame, params.object_name)
            matches = [
                item for item in objects
                if detection_label_matches(item.name, params.object_name) and item.bbox is not None
            ]
            if not matches:
                return None
            found = max(matches, key=lambda item: item.confidence)
            try:
                bbox = detection_bbox_to_pixels(found.bbox, camera)
            except ValueError:
                return VisualTargetObservation(
                    observed_monotonic=float(captured), confidence=found.confidence,
                )
            return VisualTargetObservation(
                observed_monotonic=float(captured), bbox=bbox,
                confidence=found.confidence,
            )

        controller = ContinuousVisualServoController(
            self._client, observe, camera,
            # VLM inference can take seconds. The robot is stopped while reacquiring;
            # the source timestamp still prevents motion from an old cached frame.
            max_observation_age_s=5.0,
            minimum_confidence=params.minimum_confidence,
            target_distance_m=params.target_distance_m,
            max_iterations=params.max_iterations,
            event_sink=getattr(self._client, "record_diagnostic_event", None),
        )
        result = await controller.run()
        return SkillResult(
            success=result.completed,
            message=f"visual navigation stopped: {result.stop_reason}",
            data=result.__dict__,
        )


class NativeRelocalizeSkill(Skill):
    name = "nav_relocalize"
    description = "Relocalize the robot against the configured native persistent map."
    params_model = NativeRelocalizeParams
    capability_metadata = CapabilityMetadata(
        risk_level=RiskLevel.LOW, motion_kind=MotionKind.NONE,
        planner_visible=False, tags=frozenset({"navigation", "map", "localization"}),
    )

    def __init__(self, client: NativeGo2NavigationClient) -> None:
        self._client = client

    async def execute(self, params, robot, world) -> SkillResult:
        if (params.initial_x_m is None) != (params.initial_y_m is None):
            return SkillResult(success=False, message="both initial x/y are required", data={"stop_reason": "invalid_initial_pose"})
        initial = None if params.initial_x_m is None else NavigationPose(
            x_m=params.initial_x_m, y_m=params.initial_y_m,
            yaw_degrees=params.initial_yaw_degrees, frame_id="map",
        )
        try:
            result = await self._client.relocalize(
                initial, allow_global_fallback=params.allow_global_fallback
            )
        except Exception as exc:
            return SkillResult(success=False, message=f"relocalization failed: {exc}", data={"stop_reason": "provider_error"})
        return SkillResult(
            success=result.accepted,
            message=f"relocalization {result.reason}",
            data={**result.__dict__, "pose": result.pose.model_dump(mode="json") if result.pose else None},
        )


class NativeTerrainNavigateSkill(Skill):
    name = "nav_go_terrain_relative"
    description = "Plan and follow a bounded multi-level Go2 terrain route in mcf motion mode."
    params_model = NativeTerrainNavigateParams
    capability_metadata = CapabilityMetadata(
        risk_level=RiskLevel.HIGH, motion_kind=MotionKind.LINEAR,
        requires_confirmation=True, planner_visible=True,
        tags=frozenset({"navigation", "terrain", "3d", "mls"}),
    )

    def __init__(self, client: NativeGo2NavigationClient) -> None:
        self._client = client

    async def execute(self, params, robot, world) -> SkillResult:
        try:
            path = await self._client.plan_terrain_relative(
                forward_m=params.forward_m, left_m=params.left_m, up_m=params.up_m,
                navigation_boundary_xy=params.navigation_boundary_xy,
                added_obstacles_xyz=params.added_obstacles_xyz,
                added_obstacle_radius_m=params.added_obstacle_radius_m,
            )
        except Exception as exc:
            return SkillResult(success=False, message=f"terrain planning failed: {exc}",
                               data={"stop_reason": "terrain_plan_failed"})
        result = await TerrainPathController(
            self._client, event_sink=self._client.record_diagnostic_event,
        ).run(path)
        return SkillResult(
            success=result.completed,
            message=f"terrain navigation stopped: {result.stop_reason}",
            data={**result.__dict__, "path_nodes": len(path.nodes),
                  "path_length_m": path.length_m,
                  "elevation_gain_m": path.elevation_gain_m},
        )


class NativeTerrainExploreSkill(Skill):
    name = "nav_terrain_explore"
    description = "Run bounded multi-level frontier exploration on validated Go2 terrain in mcf mode."
    params_model = NativeTerrainExploreParams
    capability_metadata = CapabilityMetadata(
        risk_level=RiskLevel.HIGH, motion_kind=MotionKind.LINEAR,
        requires_confirmation=True, planner_visible=True,
        tags=frozenset({"navigation", "terrain", "3d", "exploration", "tare"}),
    )

    def __init__(self, client: NativeGo2NavigationClient) -> None:
        self._client = client

    async def execute(self, params, robot, world) -> SkillResult:
        controller = TerrainFrontierExplorationController(
            self._client, max_goals=params.max_goals,
            exploration_range_m=params.exploration_range_m,
            navigation_boundary_xy=params.navigation_boundary_xy,
            added_obstacles_xyz=params.added_obstacles_xyz,
            added_obstacle_radius_m=params.added_obstacle_radius_m,
        )
        result = await controller.run()
        return SkillResult(
            success=result.completed,
            message=f"terrain exploration stopped: {result.stop_reason}",
            data=result.__dict__,
        )


def native_navigation_skills(
    client: NativeGo2NavigationClient, *, terrain_motion_enabled: bool = False,
    visual_frames: Any | None = None, visual_recognizer: Any | None = None,
) -> list[Skill]:
    skills: list[Skill] = [
        NativeExploreSkill(client), NativePatrolSkill(client),
        NativeBBoxNavigateSkill(client), NativeRelocalizeSkill(client),
    ]
    if terrain_motion_enabled:
        skills.extend((NativeTerrainNavigateSkill(client), NativeTerrainExploreSkill(client)))
    if visual_frames is not None and visual_recognizer is not None:
        skills.append(NativeVisualServoSkill(client, visual_frames, visual_recognizer))
    return skills
