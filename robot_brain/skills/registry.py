"""Whitelist registry and function-calling schema export."""
from __future__ import annotations

from collections.abc import Iterable

from robot_brain.skills.base import Skill


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
