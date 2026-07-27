"""Planner-visible capability catalog.

The catalog is the LLM / planner's view of what it can call. It is **not** the
runtime's full capability set: low-level tools default to
``planner_visible=False`` and never appear here, and per-backend filtering
(what the planner may see on ``unitree`` vs ``mock``) lives in this layer
rather than scattered in the registry or validator.

Today the catalog surfaces skills. Future iterations may expose a small set of
safe tools directly (e.g. a read-only ``observe`` tool); the ``planner_visible``
flag on :class:`robot_brain.tools.base.CapabilityMetadata` is the gate.
"""
from __future__ import annotations

from robot_brain.skills.registry import SkillRegistry, UNITREE_LLM_SKILLS


class PlannerCatalog:
    """Filters and describes the capabilities a planner may invoke."""

    def __init__(self, skills: SkillRegistry, backend: str) -> None:
        self.skills = skills
        self.backend = backend

    def visible_skill_names(self) -> set[str]:
        """Skill names the planner may see on the current backend.

        On ``unitree`` only the Go2-capable subset is exposed; generic motion
        skills (navigate/patrol/follow/dock) are hidden. Other backends see
        every registered skill.
        """
        if self.backend == "unitree":
            visible = set(UNITREE_LLM_SKILLS)
            for name in self.skills.names():
                skill = self.skills.get(name)
                metadata = getattr(skill, "capability_metadata", None)
                if metadata is None or not metadata.planner_visible:
                    continue
                if (
                    metadata.backend_allowlist is None
                    or self.backend in metadata.backend_allowlist
                ):
                    visible.add(name)
            return visible
        return set(self.skills.names())

    def planner_tools(self, *, strict: bool = True) -> list[dict[str, object]]:
        """LLM function-tool schemas for the planner-visible capabilities."""
        all_tools = self.skills.tools(strict=strict)
        if self.backend == "unitree":
            visible = self.visible_skill_names()
            return [t for t in all_tools if t["name"] in visible]
        return all_tools
