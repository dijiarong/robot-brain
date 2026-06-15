"""Whitelist registry and function-calling schema export."""
from __future__ import annotations

from collections.abc import Iterable

from robot_brain.skills.base import Skill

# Tools visible to LLM on the Unitree backend.
UNITREE_LLM_SKILLS: frozenset[str] = frozenset({
    "nudge", "scan", "retreat", "recognize", "report", "stop",
})

# Generic motion tools hidden from LLM on Unitree (still registered
# for test compat; rejected by Validator if bypassed).
UNITREE_HIDDEN_SKILLS: frozenset[str] = frozenset({
    "navigate", "patrol", "follow", "dock",
})


class SkillRegistry:
    def __init__(self, skills: Iterable[Skill] | None = None) -> None:
        self._skills: dict[str, Skill] = {}
        for skill in skills or []:
            self.register(skill)

    def register(self, skill: Skill) -> None:
        if skill.name in self._skills:
            raise ValueError(f"skill already registered: {skill.name}")
        self._skills[skill.name] = skill

    def get(self, name: str) -> Skill | None:
        return self._skills.get(name)

    def has(self, name: str) -> bool:
        return name in self._skills

    def names(self) -> tuple[str, ...]:
        return tuple(self._skills)

    def tools(self, *, strict: bool = True) -> list[dict[str, object]]:
        return [
            {
                "type": "function",
                "name": skill.name,
                "description": skill.description,
                "parameters": skill.params_schema(),
                "strict": strict,
            }
            for skill in self._skills.values()
        ]

    def tools_for_backend(
        self, backend: str, *, strict: bool = True,
    ) -> list[dict[str, object]]:
        """Return tools filtered for *backend*.

        On ``unitree``, generic motion skills (navigate / patrol / follow /
        dock) are excluded from the LLM tool list.
        """
        if backend == "unitree":
            return [
                t for t in self.tools(strict=strict)
                if t["name"] in UNITREE_LLM_SKILLS
            ]
        return self.tools(strict=strict)
