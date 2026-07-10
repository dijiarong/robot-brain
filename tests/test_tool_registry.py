"""Tests for the Tool contract and ToolRegistry."""
from __future__ import annotations

import unittest

from pydantic import BaseModel

from robot_brain.actuation.mock import MockRobot
from robot_brain.core.world_state import WorldState
from robot_brain.tools.base import (
    CapabilityMetadata,
    MotionKind,
    RiskLevel,
    Tool,
    ToolContext,
)
from robot_brain.tools.registry import ToolRegistry
from robot_brain.tools.builtin import StopMotionTool, default_tools
from robot_brain.tools.builtin.control import StopMotionParams


# ---------------------------------------------------------------------------
# Test fixtures
# ---------------------------------------------------------------------------

class _AlphaParams(BaseModel):
    n: int = 0


class _AlphaTool(Tool):
    name = "alpha"
    description = "alpha tool"
    params_model = _AlphaParams
    metadata = CapabilityMetadata(
        risk_level=RiskLevel.LOW,
        motion_kind=MotionKind.NONE,
        planner_visible=False,
    )

    async def execute(self, params, context):  # type: ignore[override]
        from robot_brain.tools.base import ToolResult

        return ToolResult(success=True, message="alpha")


class _UnitreeOnlyTool(Tool):
    name = "unitree_only"
    description = "unitree only"
    metadata = CapabilityMetadata(backend_allowlist=("unitree",))

    async def execute(self, params, context):  # type: ignore[override]
        from robot_brain.tools.base import ToolResult

        return ToolResult(success=True, message="ok")


# ---------------------------------------------------------------------------
# Registry
# ---------------------------------------------------------------------------

class ToolRegistryTests(unittest.TestCase):
    def test_register_and_lookup(self):
        reg = ToolRegistry([_AlphaTool()])
        self.assertTrue(reg.has("alpha"))
        self.assertIs(reg.get("alpha"), reg.all()[0])
        self.assertEqual(reg.names(), ("alpha",))

    def test_duplicate_register_raises(self):
        reg = ToolRegistry([_AlphaTool()])
        with self.assertRaises(ValueError):
            reg.register(_AlphaTool())

    def test_get_missing_returns_none(self):
        reg = ToolRegistry()
        self.assertIsNone(reg.get("nope"))
        self.assertFalse(reg.has("nope"))

    def test_default_tools_include_stop_motion(self):
        tools = default_tools()
        names = {t.name for t in tools}
        self.assertIn("stop_motion", names)

    def test_for_backend_filters_allowlist(self):
        reg = ToolRegistry([_AlphaTool(), _UnitreeOnlyTool()])
        # alpha has no allowlist -> available everywhere
        mock_names = {t.name for t in reg.for_backend("mock")}
        self.assertIn("alpha", mock_names)
        self.assertNotIn("unitree_only", mock_names)
        # unitree_only appears only on unitree
        unitree_names = {t.name for t in reg.for_backend("unitree")}
        self.assertIn("alpha", unitree_names)
        self.assertIn("unitree_only", unitree_names)


# ---------------------------------------------------------------------------
# Metadata + schema
# ---------------------------------------------------------------------------

class CapabilityMetadataTests(unittest.TestCase):
    def test_defaults(self):
        md = CapabilityMetadata()
        self.assertEqual(md.risk_level, RiskLevel.LOW)
        self.assertEqual(md.motion_kind, MotionKind.NONE)
        self.assertFalse(md.requires_confirmation)
        self.assertIsNone(md.backend_allowlist)
        self.assertFalse(md.planner_visible)
        self.assertEqual(md.tags, frozenset())

    def test_stop_motion_metadata(self):
        md = StopMotionTool.metadata
        self.assertEqual(md.motion_kind, MotionKind.STOP)
        self.assertNotEqual(md.risk_level, RiskLevel.READ_ONLY)
        self.assertFalse(md.planner_visible)
        self.assertIsNone(md.backend_allowlist)

    def test_metadata_is_frozen(self):
        md = StopMotionTool.metadata
        with self.assertRaises(Exception):
            md.risk_level = RiskLevel.HIGH  # type: ignore[misc]


class ToolSchemaTests(unittest.TestCase):
    def test_params_schema_export(self):
        schema = StopMotionTool().params_schema()
        self.assertIn("reason", schema.get("properties", {}))
        self.assertEqual(
            schema["properties"]["reason"]["default"], "safety stop"
        )

    def test_parse_params(self):
        parsed = StopMotionTool().parse_params({"reason": "estop"})
        self.assertEqual(parsed.reason, "estop")  # type: ignore[attr-defined]


# ---------------------------------------------------------------------------
# StopMotionTool execution
# ---------------------------------------------------------------------------

class StopMotionToolTests(unittest.IsolatedAsyncioTestCase):
    async def test_execute_calls_robot_stop(self):
        robot = MockRobot()
        ctx = ToolContext(settings=None, world=WorldState(), robot=robot)
        result = await StopMotionTool().execute(StopMotionParams(reason="halt"), ctx)
        self.assertTrue(result.success)
        self.assertIn("halt", result.message)
        self.assertEqual(robot.action_history[-1]["action"], "stop")
        self.assertEqual(robot.action_history[-1]["reason"], "halt")
        self.assertTrue(robot.state.stopped)


if __name__ == "__main__":
    unittest.main()
