"""Tests for VLM resource lifecycle (analyzer aclose, runtime aclose/close)."""
from __future__ import annotations

import unittest

from config.settings import Settings
from robot_brain.core.world_state import WorldState
from robot_brain.vlm.frame_source import FrameSource, NullFrameSource
from robot_brain.vlm.passability import PassabilityAnalyzer


class _FakeClient:
    def __init__(self) -> None:
        self.aclose_called = 0

    async def analyze_passability(self, image_bytes: bytes):  # noqa: D401
        raise AssertionError("not used in lifecycle tests")

    async def aclose(self) -> None:
        self.aclose_called += 1


class _FakeFrameSource(FrameSource):
    kind = "fake"

    def __init__(self) -> None:
        self.stop_called = 0

    async def get_frame(self) -> bytes | None:
        return None

    def stop(self) -> None:
        self.stop_called += 1


def _settings(**kw) -> Settings:
    base = dict(memory_db_path=":memory:", vlm_enabled=True)
    base.update(kw)
    return Settings(**base)


class PassabilityAnalyzerLifecycleTests(unittest.IsolatedAsyncioTestCase):
    async def test_aclose_closes_client_and_stops_frame_source(self):
        client = _FakeClient()
        fs = _FakeFrameSource()
        an = PassabilityAnalyzer(client, fs, _settings())
        await an.aclose()
        self.assertEqual(client.aclose_called, 1)
        self.assertEqual(fs.stop_called, 1)

    async def test_aclose_is_idempotent(self):
        client = _FakeClient()
        fs = _FakeFrameSource()
        an = PassabilityAnalyzer(client, fs, _settings())
        await an.aclose()
        await an.aclose()  # second close is a no-op
        self.assertEqual(client.aclose_called, 1)
        self.assertEqual(fs.stop_called, 1)

    async def test_diagnostics_after_failure(self):
        fs = NullFrameSource()  # get_frame returns None -> "no frame" failure
        an = PassabilityAnalyzer(_FakeClient(), fs, _settings(vlm_min_interval=0.0))
        hint = await an.analyze_if_due(WorldState())
        self.assertIsNone(hint)
        diag = an.diagnostics()
        self.assertTrue(diag["enabled"])
        self.assertEqual(diag["frame_source"], "null")
        self.assertEqual(diag["last_error"], "no frame available")
        self.assertIsNone(diag["last_hint"])


class AgentRuntimeLifecycleTests(unittest.IsolatedAsyncioTestCase):
    async def test_aclose_releases_passability(self):
        from robot_brain.runtime.loop import AgentRuntime

        rt = AgentRuntime.create(settings=_settings())  # mock + vlm_enabled -> NullFrameSource
        self.assertIsNotNone(rt.passability)
        await rt.aclose()
        # Idempotent.
        await rt.aclose()
        # VLM client closed exactly once.
        self.assertTrue(rt.passability._closed)

    async def test_vlm_disabled_close_is_noop(self):
        from robot_brain.runtime.loop import AgentRuntime

        rt = AgentRuntime.create(settings=Settings(memory_db_path=":memory:"))
        self.assertIsNone(rt.passability)
        await rt.aclose()  # must not raise
        rt2 = AgentRuntime.create(settings=Settings(memory_db_path=":memory:"))
        rt2.close()  # must not raise

    async def test_service_stop_uses_aclose(self):
        """AgentService.stop() should await runtime.aclose() (releases VLM)."""
        from robot_brain.runtime.loop import AgentRuntime
        from robot_brain.runtime.scheduler import AgentScheduler
        from robot_brain.service.runner import AgentService

        rt = AgentRuntime.create(settings=_settings())
        svc = AgentService(AgentScheduler(rt))
        await svc.start()
        await svc.stop()
        self.assertTrue(rt.passability._closed)


class AgentRuntimeSyncCloseTests(unittest.TestCase):
    """close() is synchronous; without a running loop it runs aclose() fully."""

    def test_close_without_running_loop_releases_vlm(self):
        from robot_brain.runtime.loop import AgentRuntime

        rt = AgentRuntime.create(settings=_settings())
        rt.close()  # no running loop in a plain sync test -> asyncio.run(aclose)
        self.assertTrue(rt.passability._closed)
        rt.close()  # idempotent


if __name__ == "__main__":
    unittest.main()
