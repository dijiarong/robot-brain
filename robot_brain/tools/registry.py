"""Runtime-internal registry for atomic tools.

Distinct from :class:`robot_brain.skills.registry.SkillRegistry`: this holds
low-level machine capabilities that are *not* the primary planner-facing unit.
Tools default to ``planner_visible=False`` and reach the LLM only via a
``PlannerCatalog`` (and only when explicitly allowed).
"""
from __future__ import annotations

from collections.abc import Iterable

from robot_brain.tools.base import Tool


class ToolRegistry:
    def __init__(self, tools: Iterable[Tool] | None = None) -> None:
        self._tools: dict[str, Tool] = {}
        for tool in tools or []:
            self.register(tool)

    def register(self, tool: Tool) -> None:
        if tool.name in self._tools:
            raise ValueError(f"tool already registered: {tool.name}")
        self._tools[tool.name] = tool

    def get(self, name: str) -> Tool | None:
        return self._tools.get(name)

    def has(self, name: str) -> bool:
        return name in self._tools

    def names(self) -> tuple[str, ...]:
        return tuple(self._tools)

    def all(self) -> tuple[Tool, ...]:
        return tuple(self._tools.values())

    def for_backend(self, backend: str) -> tuple[Tool, ...]:
        """Tools available on *backend*.

        A tool with ``backend_allowlist=None`` is available everywhere; others
        only on backends in their allowlist.
        """
        return tuple(
            tool
            for tool in self._tools.values()
            if tool.metadata.backend_allowlist is None
            or backend in tool.metadata.backend_allowlist
        )
