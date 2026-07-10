"""Capability-driven safety rules.

``SafetyPolicy`` encodes safety bounds that key off
:class:`robot_brain.tools.base.CapabilityMetadata` (risk level, motion kind,
backend allowlist, confirmation requirement) instead of hardcoded skill names.
This is the first step in moving capability knowledge out of
``SafetyValidator``'s per-skill-name branches.

The policy is split into three independent checks so ``SafetyValidator`` can
run them at the right points in its validation order:

- :meth:`check_backend` - backend allowlist (run early, before param parsing)
- :meth:`check_state` - emergency stop + critical battery (run after parsing)
- :meth:`check_confirmation` - operator confirmation (run **last**, after
  preconditions and motion-range checks, so the system never asks an operator
  to confirm an action that is already illegal for other reasons)

:meth:`evaluate` runs all three in order and returns the first denial; it
exists for unit tests and standalone use. ``SafetyValidator`` does **not** use
``evaluate`` because it needs to interleave preconditions / motion checks
between the state and confirmation checks.

``SafetyValidator`` delegates to this policy for capabilities that carry
metadata (migrated tools/skills) and falls back to its legacy name-based
checks otherwise, so migration is gradual.
"""
from __future__ import annotations

from pydantic import BaseModel

from config.settings import Settings
from robot_brain.core.errors import ErrorCode
from robot_brain.core.world_state import WorldState
from robot_brain.tools.base import CapabilityMetadata, MotionKind, RiskLevel


class PolicyDecision(BaseModel):
    allowed: bool
    reason: str = ""
    error_code: ErrorCode | None = None
    requires_confirmation: bool = False


class SafetyPolicy:
    """Generic capability safety bounds."""

    def __init__(self, settings: Settings) -> None:
        self.settings = settings

    # ------------------------------------------------------------------
    # Granular checks (used by SafetyValidator at distinct points)
    # ------------------------------------------------------------------
    def check_backend(
        self, metadata: CapabilityMetadata, backend: str
    ) -> PolicyDecision:
        """Reject if the capability is not allowed on *backend*."""
        if (
            metadata.backend_allowlist is not None
            and backend not in metadata.backend_allowlist
        ):
            return PolicyDecision(
                allowed=False,
                reason=(
                    f"capability unsupported on backend '{backend}' "
                    f"(allowlist: {list(metadata.backend_allowlist)})"
                ),
                error_code=ErrorCode.SAFETY_NOT_WHITELISTED,
            )
        return PolicyDecision(allowed=True)

    def check_state(
        self, metadata: CapabilityMetadata, world: WorldState
    ) -> PolicyDecision:
        """Reject on emergency stop or critical battery.

        Emergency stop permits only ``motion_kind=stop`` or read-only
        capabilities. Critical battery additionally permits non-motion
        (``motion_kind=none``) capabilities.
        """
        if world.estop_active and metadata.motion_kind != MotionKind.STOP and metadata.risk_level != RiskLevel.READ_ONLY:
            return PolicyDecision(
                allowed=False,
                reason="emergency stop is active",
                error_code=ErrorCode.SAFETY_ESTOP_ACTIVE,
            )
        if (
            world.battery_level <= self.settings.critical_battery_threshold
            and metadata.motion_kind not in (MotionKind.STOP, MotionKind.NONE)
            and metadata.risk_level != RiskLevel.READ_ONLY
        ):
            return PolicyDecision(
                allowed=False,
                reason="critical battery only permits stop, dock, or report",
                error_code=ErrorCode.SAFETY_BATTERY_CRITICAL,
            )
        return PolicyDecision(allowed=True)

    def check_confirmation(
        self, metadata: CapabilityMetadata, confirmation_granted: bool
    ) -> PolicyDecision:
        """Require operator confirmation if the capability declares it.

        For migrated capabilities ``metadata.requires_confirmation`` is
        authoritative; ``Settings.require_confirmation_for`` no longer applies
        to them (it still governs non-migrated skills). Confirmation is a
        safety property of the capability, not a per-call toggle.
        """
        if metadata.requires_confirmation and not confirmation_granted:
            return PolicyDecision(
                allowed=False,
                reason="capability requires operator confirmation",
                error_code=ErrorCode.SAFETY_CONFIRMATION_REQUIRED,
                requires_confirmation=True,
            )
        return PolicyDecision(allowed=True)

    # ------------------------------------------------------------------
    # All-in-one (unit tests / standalone use only)
    # ------------------------------------------------------------------
    def evaluate(
        self,
        metadata: CapabilityMetadata,
        world: WorldState,
        *,
        backend: str,
        confirmation_granted: bool = False,
    ) -> PolicyDecision:
        """Run backend -> state -> confirmation; return the first denial."""
        for decision in (
            self.check_backend(metadata, backend),
            self.check_state(metadata, world),
            self.check_confirmation(metadata, confirmation_granted),
        ):
            if not decision.allowed:
                return decision
        return PolicyDecision(allowed=True)
