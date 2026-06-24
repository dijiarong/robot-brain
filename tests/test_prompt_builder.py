"""Unit tests for the LLM PromptBuilder."""
from __future__ import annotations

import pytest

from robot_brain.core.robot_self_state import RobotSelfState, UltrasonicData, Velocity
from robot_brain.core.world_state import WorldState
from robot_brain.llm.prompt_builder import PromptBuilder


@pytest.fixture
def builder() -> PromptBuilder:
    return PromptBuilder()


@pytest.fixture
def normal_world() -> WorldState:
    return WorldState(
        battery_level=80.0,
        robot_self_state=RobotSelfState(
            source="unitree",
            is_standing=True,
            is_moving=False,
            error_code=0,
            state_age_seconds=0.5,
            ultrasonic=UltrasonicData(front_m=1.5, rear_m=1.2),
        ),
    )


@pytest.fixture
def low_battery_world() -> WorldState:
    return WorldState(
        battery_level=20.0,
        robot_self_state=RobotSelfState(
            source="unitree",
            is_standing=True,
            is_moving=False,
            error_code=0,
            state_age_seconds=0.3,
        ),
    )


@pytest.fixture
def critical_world() -> WorldState:
    return WorldState(
        battery_level=5.0,
        estop_active=True,
        robot_self_state=RobotSelfState(
            source="unitree",
            is_standing=False,
            is_moving=False,
            error_code=42,
            state_age_seconds=3.5,
            ultrasonic=UltrasonicData(front_m=0.15),
        ),
    )


class TestPromptBuilderBasic:
    def test_build_produces_nonempty_string(self, builder: PromptBuilder, normal_world: WorldState) -> None:
        result = builder.build_system_prompt(normal_world, backend="unitree")
        assert isinstance(result, str)
        assert len(result) > 100

    def test_role_section_present(self, builder: PromptBuilder, normal_world: WorldState) -> None:
        result = builder.build_system_prompt(normal_world)
        assert "L3 cognitive controller" in result
        assert "safety constraints" in result

    def test_state_section_contains_battery(self, builder: PromptBuilder, normal_world: WorldState) -> None:
        result = builder.build_system_prompt(normal_world)
        assert "Battery: 80%" in result

    def test_unitree_backend_shows_tool_guidance(self, builder: PromptBuilder, normal_world: WorldState) -> None:
        result = builder.build_system_prompt(normal_world, backend="unitree")
        assert "nudge" in result
        assert "scan" in result
        assert "retreat" in result


class TestPromptBuilderPolicies:
    def test_normal_state_shows_nominal(self, builder: PromptBuilder, normal_world: WorldState) -> None:
        result = builder.build_system_prompt(normal_world)
        assert "All systems nominal" in result

    def test_low_battery_policy(self, builder: PromptBuilder, low_battery_world: WorldState) -> None:
        result = builder.build_system_prompt(low_battery_world)
        assert "LOW" in result
        assert "conservative" in result.lower() or "Avoid long-distance" in result

    def test_critical_battery_policy(self, builder: PromptBuilder, critical_world: WorldState) -> None:
        result = builder.build_system_prompt(critical_world)
        assert "CRITICAL" in result
        assert "stop" in result.lower()

    def test_estop_policy(self, builder: PromptBuilder, critical_world: WorldState) -> None:
        result = builder.build_system_prompt(critical_world)
        assert "E-stop ACTIVE" in result

    def test_not_standing_policy(self, builder: PromptBuilder, critical_world: WorldState) -> None:
        result = builder.build_system_prompt(critical_world)
        assert "NOT STANDING" in result

    def test_error_code_policy(self, builder: PromptBuilder, critical_world: WorldState) -> None:
        result = builder.build_system_prompt(critical_world)
        assert "FAULT" in result

    def test_stale_state_policy(self, builder: PromptBuilder, critical_world: WorldState) -> None:
        result = builder.build_system_prompt(critical_world)
        assert "STALE" in result

    def test_obstacle_front_policy(self, builder: PromptBuilder, critical_world: WorldState) -> None:
        result = builder.build_system_prompt(critical_world)
        assert "FRONT" in result or "front" in result


class TestPromptBuilderConversation:
    def test_no_conversation_omits_section(self, builder: PromptBuilder, normal_world: WorldState) -> None:
        result = builder.build_system_prompt(normal_world)
        assert "[Recent Dialogue]" not in result

    def test_conversation_included(self, builder: PromptBuilder, normal_world: WorldState) -> None:
        conversation = [
            {"role": "user", "content": "向前走一步"},
            {"role": "assistant", "content": "已完成 nudge forward 30cm"},
        ]
        result = builder.build_system_prompt(normal_world, conversation=conversation)
        assert "[Recent Dialogue]" in result
        assert "向前走一步" in result
        assert "nudge forward" in result

    def test_conversation_truncated(self, normal_world: WorldState) -> None:
        builder = PromptBuilder(max_conversation_turns=2)
        conversation = [
            {"role": "user", "content": f"msg-{i}"} for i in range(10)
        ]
        result = builder.build_system_prompt(normal_world, conversation=conversation)
        # Only last 2 turns should appear
        assert "msg-8" in result
        assert "msg-9" in result
        assert "msg-0" not in result


class TestPromptBuilderMemories:
    def test_no_memories_omits_section(self, builder: PromptBuilder, normal_world: WorldState) -> None:
        result = builder.build_system_prompt(normal_world)
        assert "[Past Experiences]" not in result

    def test_memories_included(self, builder: PromptBuilder, normal_world: WorldState) -> None:
        memories = ["nudge forward succeeded in 2.1s", "patrol completed 3 waypoints"]
        result = builder.build_system_prompt(normal_world, memories=memories)
        assert "[Past Experiences]" in result
        assert "nudge forward succeeded" in result

    def test_memories_truncated(self, normal_world: WorldState) -> None:
        builder = PromptBuilder(max_memories=2)
        memories = [f"experience-{i}" for i in range(10)]
        result = builder.build_system_prompt(normal_world, memories=memories)
        assert "experience-0" in result
        assert "experience-1" in result
        assert "experience-2" not in result


class TestCognitiveSnapshot:
    def test_cognitive_snapshot_has_state_summary(self, normal_world: WorldState) -> None:
        snap = normal_world.cognitive_snapshot()
        assert "_state_summary" in snap
        assert isinstance(snap["_state_summary"], dict)

    def test_normal_battery_ok(self, normal_world: WorldState) -> None:
        summary = normal_world._build_state_summary()
        assert "OK" in summary["battery"]

    def test_low_battery_marked(self, low_battery_world: WorldState) -> None:
        summary = low_battery_world._build_state_summary()
        assert "LOW" in summary["battery"]

    def test_critical_battery_marked(self, critical_world: WorldState) -> None:
        summary = critical_world._build_state_summary()
        assert "CRITICAL" in summary["battery"]

    def test_estop_marked(self, critical_world: WorldState) -> None:
        summary = critical_world._build_state_summary()
        assert "estop" in summary
        assert "ACTIVE" in summary["estop"]

    def test_not_standing_marked(self, critical_world: WorldState) -> None:
        summary = critical_world._build_state_summary()
        assert "NOT STANDING" in summary["posture"]

    def test_moving_state(self) -> None:
        world = WorldState(
            robot_self_state=RobotSelfState(
                source="unitree",
                is_moving=True,
                velocity=Velocity(vx=0.15, vy=0.0),
            ),
        )
        summary = world._build_state_summary()
        assert "MOVING" in summary["motion"]
        assert "0.15" in summary["motion"]

    def test_obstacle_close(self, critical_world: WorldState) -> None:
        summary = critical_world._build_state_summary()
        assert "proximity" in summary
        assert "OBSTACLE" in summary["proximity"]

    def test_stale_state(self, critical_world: WorldState) -> None:
        summary = critical_world._build_state_summary()
        assert "STALE" in summary["freshness"]

    def test_no_robot_state_minimal_summary(self) -> None:
        world = WorldState(battery_level=50.0)
        summary = world._build_state_summary()
        assert "battery" in summary
        assert "OK" in summary["battery"]
        # No posture/motion/error keys when robot_self_state is None
        assert "posture" not in summary
