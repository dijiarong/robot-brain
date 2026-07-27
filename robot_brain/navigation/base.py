"""Contracts shared by fake, direct-Go2, DimOS, and Nav2 adapters."""
from __future__ import annotations

from abc import ABC, abstractmethod
from datetime import datetime, timezone
from enum import StrEnum

from pydantic import BaseModel, Field


class NavigationStatus(StrEnum):
    IDLE = "idle"
    ACTIVE = "active"
    SUCCEEDED = "succeeded"
    FAILED = "failed"
    CANCELED = "canceled"
    TIMED_OUT = "timed_out"
    NO_PROGRESS = "no_progress"
    UNAVAILABLE = "unavailable"

    @property
    def terminal(self) -> bool:
        return self not in {NavigationStatus.IDLE, NavigationStatus.ACTIVE}


class NavigationPose(BaseModel):
    x_m: float = 0.0
    y_m: float = 0.0
    yaw_degrees: float = 0.0
    frame_id: str = "odom"


class RelativeNavigationGoal(BaseModel):
    forward_m: float = Field(default=0.0, ge=-1.0, le=1.0)
    left_m: float = Field(default=0.0, ge=-0.5, le=0.5)
    yaw_degrees: float = Field(default=0.0, ge=-90.0, le=90.0)
    max_duration_s: float = Field(default=12.0, ge=0.5, le=20.0)


class NavigationGoalHandle(BaseModel):
    goal_id: str
    accepted: bool = True
    message: str = ""


class NavigationState(BaseModel):
    provider: str
    ready: bool = True
    status: NavigationStatus = NavigationStatus.IDLE
    goal_id: str | None = None
    pose: NavigationPose | None = None
    progress: float | None = Field(default=None, ge=0.0, le=1.0)
    message: str = ""
    error_code: str | None = None
    updated_at: datetime = Field(default_factory=lambda: datetime.now(timezone.utc))


class NavigationError(RuntimeError):
    """Base error raised by a navigation provider adapter."""


class NavigationUnavailableError(NavigationError):
    """Raised when the configured navigation provider is not ready."""


class NavigationClient(ABC):
    """Small adapter surface implemented by every navigation backend."""

    @abstractmethod
    async def get_state(self) -> NavigationState: ...

    @abstractmethod
    async def set_relative_goal(
        self, goal: RelativeNavigationGoal
    ) -> NavigationGoalHandle: ...

    @abstractmethod
    async def cancel(self, goal_id: str | None = None) -> NavigationState: ...
