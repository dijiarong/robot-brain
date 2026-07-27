"""Built-in cognition skills."""

from .catalog import (
    DockSkill,
    FollowSkill,
    NavigateSkill,
    PatrolSkill,
    RecognizeSkill,
    ReportSkill,
    StopSkill,
    default_skills,
)
from .go2_catalog import (
    NudgeSkill,
    RetreatSkill,
    ScanSkill,
    go2_skills,
)
from .navigation import (
    CancelNavigationSkill,
    NavigateAbsoluteSkill,
    NavigateRelativeSkill,
    navigation_skills,
)
from .spatial_memory import FindObjectSkill, RememberRoomSkill

__all__ = [
    "DockSkill",
    "FollowSkill",
    "NavigateSkill",
    "NudgeSkill",
    "CancelNavigationSkill",
    "NavigateRelativeSkill",
    "NavigateAbsoluteSkill",
    "PatrolSkill",
    "RecognizeSkill",
    "ReportSkill",
    "RetreatSkill",
    "ScanSkill",
    "StopSkill",
    "default_skills",
    "go2_skills",
    "navigation_skills",
    "FindObjectSkill",
    "RememberRoomSkill",
]
