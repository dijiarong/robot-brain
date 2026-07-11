"""Tests for PassabilityAnalyzer (frame -> VLM -> hint, with fallback)."""
from __future__ import annotations

import unittest

from config.settings import Settings
from robot_brain.core.passability import PassabilityHint
from robot_brain.core.world_state import WorldState
from robot_brain.vlm.frame_source import FrameSource, NullFrameSource
from robot_brain.vlm.passability import PassabilityAnalyzer


class _MemoryFrameSource(FrameSource):
    """Returns a scripted list of frames (bytes or None), then None."""

    def __init__(self, frames: list[bytes | None]) -> None:
        self._frames = list(frames)

    async def get_frame(self) -> bytes | None:
        if self._frames:
            return self._frames.pop(0)
        return None


class _FakeClient:
    """Stand-in VLMClient with a scripted per-call outcome (hint or exception)."""

    def __init__(self, outcomes: list[PassabilityHint | None | Exception]) -> None:
        self._outcomes = list(outcomes)
        self.calls = 0

    async def analyze_passability(self, image_bytes: bytes) -> PassabilityHint:
        self.calls += 1
        if not self._outcomes:
            raise RuntimeError("no more scripted outcomes")
        outcome = self._outcomes.pop(0)
        if isinstance(outcome, Exception):
            raise outcome
        assert outcome is not None
        return outcome


def _settings(**kw) -> Settings:
    base = dict(memory_db_path=":memory:", vlm_enabled=True, vlm_min_interval=2.0)
    base.update(kw)
    return Settings(**base)


def _hint(direction: str = "left", confidence: float = 0.8) -> PassabilityHint:
    return PassabilityHint(
        recommended_direction=direction,  # type: ignore[arg-type]
        confidence=confidence,
        reason="test",
    )


class PassabilityAnalyzerTests(unittest.IsolatedAsyncioTestCase):
    async def test_success_writes_hint_to_world(self):
        fs = _MemoryFrameSource([b"frame"])
        client = _FakeClient([_hint("left", 0.82)])
        an = PassabilityAnalyzer(client, fs, _settings())
        world = WorldState()
        hint = await an.analyze_if_due(world)
        self.assertEqual(hint.recommended_direction, "left")
        self.assertEqual(client.calls, 1)
        self.assertEqual(world.passability_hint.recommended_direction, "left")

    async def test_min_interval_skips_within_window(self):
        # P2: within min_interval, do NOT reuse a stale (pre-scan) hint.
        fs = _MemoryFrameSource([b"frame", b"frame"])
        client = _FakeClient([_hint("right", 0.7)])
        an = PassabilityAnalyzer(client, fs, _settings(vlm_min_interval=100.0))
        world = WorldState()
        first = await an.analyze_if_due(world)
        second = await an.analyze_if_due(world)  # rate-limited -> None
        self.assertEqual(first.recommended_direction, "right")
        self.assertIsNone(second)
        self.assertEqual(client.calls, 1)  # no second VLM call

    async def test_no_frame_returns_none(self):
        client = _FakeClient([_hint("left")])
        an = PassabilityAnalyzer(client, NullFrameSource(), _settings())
        hint = await an.analyze_if_due(WorldState())
        self.assertIsNone(hint)
        self.assertEqual(client.calls, 0)

    async def test_client_failure_returns_none(self):
        fs = _MemoryFrameSource([b"frame"])
        client = _FakeClient([RuntimeError("VLM unavailable")])
        an = PassabilityAnalyzer(client, fs, _settings())
        hint = await an.analyze_if_due(WorldState())
        self.assertIsNone(hint)

    async def test_failure_clears_previous_hint(self):
        # P3: a previously-good hint must not linger after a failure.
        fs = _MemoryFrameSource([b"frame", b"frame"])
        client = _FakeClient([_hint("left", 0.9), RuntimeError("flake")])
        an = PassabilityAnalyzer(client, fs, _settings(vlm_min_interval=0.0))
        world = WorldState()
        first = await an.analyze_if_due(world)
        self.assertEqual(first.recommended_direction, "left")
        self.assertIsNotNone(world.passability_hint)
        second = await an.analyze_if_due(world)  # due (interval=0); VLM fails
        self.assertIsNone(second)
        self.assertIsNone(world.passability_hint)  # cleared

    async def test_no_frame_clears_previous_hint(self):
        # P3: losing the frame source also clears the stale hint.
        fs = _MemoryFrameSource([b"frame", None])
        client = _FakeClient([_hint("right", 0.9)])
        an = PassabilityAnalyzer(client, fs, _settings(vlm_min_interval=0.0))
        world = WorldState()
        await an.analyze_if_due(world)
        self.assertIsNotNone(world.passability_hint)
        second = await an.analyze_if_due(world)  # due; no frame this time
        self.assertIsNone(second)
        self.assertIsNone(world.passability_hint)


if __name__ == "__main__":
    unittest.main()
