"""PassabilityAnalyzer - orchestrates frame capture -> VLM -> hint.

Used by ``ExploreSkill`` once per step, before the move decision. Enforces a
minimum interval between VLM calls (the explore loop may poll perception
several times per step; only the first due call actually hits the model).

Failure is always recoverable: a missing frame, HTTP error, or unparseable
JSON returns ``None`` so the caller falls back to the rule-based direction
choice (ultrasonic or fixed +90°).

Staleness policy (review P2/P3):

- **Rate-limited** (within ``vlm_min_interval``): return ``None`` rather than
  reusing the previous hint. Explore scans every step, so a cached hint is for
  a *previous orientation* and reusing its left/right/stop would steer the
  robot on stale framing. Falling back to rules is safer.
- **Failure / no frame**: return ``None`` **and** clear
  ``world.passability_hint`` so the StateInterpreter does not keep displaying a
  hint that no longer reflects the current view.
- **Success**: write the fresh hint onto ``world.passability_hint`` (audit).

Lifecycle: :meth:`aclose` closes the VLM client and stops the frame source;
idempotent. :meth:`diagnostics` exposes the last hint / error / latency / frame
age for the service status API.
"""
from __future__ import annotations

import logging
import time
from typing import TYPE_CHECKING

from robot_brain.core.passability import PassabilityHint
from robot_brain.vlm.frame_source import FrameSource

if TYPE_CHECKING:
    from config.settings import Settings
    from robot_brain.core.world_state import WorldState
    from robot_brain.vlm.client import VLMClient

logger = logging.getLogger(__name__)


class PassabilityAnalyzer:
    def __init__(
        self,
        client: "VLMClient",
        frame_source: FrameSource,
        settings: "Settings",
    ) -> None:
        self._client = client
        self._frame_source = frame_source
        self._settings = settings
        self._last_call_monotonic: float = 0.0
        self._closed = False
        # Diagnostics (for /api/status)
        self._last_hint: PassabilityHint | None = None
        self._last_error: str = ""
        self._last_latency_ms: float | None = None
        self._last_frame_age_ms: float | None = None

    async def analyze_if_due(self, world: "WorldState") -> PassabilityHint | None:
        """Return a fresh hint if due; ``None`` (rule fallback) otherwise.

        On any failure or missing frame also clears ``world.passability_hint``
        so stale hints do not linger in the state summary.
        """
        now = time.monotonic()
        if (now - self._last_call_monotonic) < self._settings.vlm_min_interval:
            # Do NOT reuse the previous hint: it was framed at a different
            # heading (explore scans each step). Fall back to rules instead.
            return None

        self._last_call_monotonic = now
        try:
            frame = await self._frame_source.get_frame()
            if not frame:
                self._record_failure("no frame available", world)
                return None
            hint = await self._client.analyze_passability(frame)
        except Exception as exc:  # noqa: BLE001 - any failure -> rule fallback
            logger.warning("passability VLM call failed, falling back to rules: %s", exc)
            self._record_failure(str(exc), world)
            return None

        self._last_hint = hint
        self._last_error = ""
        self._last_latency_ms = hint.latency_ms
        self._last_frame_age_ms = self._frame_source.frame_age_ms
        world.passability_hint = hint
        return hint

    def _record_failure(self, reason: str, world: "WorldState") -> None:
        self._last_error = reason
        self._last_hint = None
        self._last_latency_ms = None
        self._last_frame_age_ms = self._frame_source.frame_age_ms
        if world.passability_hint is not None:
            world.passability_hint = None

    async def aclose(self) -> None:
        """Close the VLM client and stop the frame source. Idempotent."""
        if self._closed:
            return
        self._closed = True
        try:
            await self._client.aclose()
        except Exception as exc:  # noqa: BLE001 - best-effort cleanup
            logger.warning("VLM client close failed: %s", exc)
        try:
            frame_aclose = getattr(self._frame_source, "aclose", None)
            if callable(frame_aclose):
                await frame_aclose()
            else:
                self._frame_source.stop()
        except Exception as exc:  # noqa: BLE001
            logger.warning("frame source stop failed: %s", exc)

    def diagnostics(self) -> dict[str, object]:
        """Snapshot for the service status API."""
        last_hint = self._last_hint
        return {
            "enabled": True,
            "frame_source": self._frame_source.kind,
            "last_hint": last_hint.model_dump(mode="json") if last_hint else None,
            "last_error": self._last_error,
            "last_latency_ms": self._last_latency_ms,
            "last_frame_age_ms": self._last_frame_age_ms,
        }
