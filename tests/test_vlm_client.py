"""Tests for the VLM client (HTTP multimodal -> PassabilityHint)."""
from __future__ import annotations

import unittest
from io import BytesIO

from config.settings import Settings
from robot_brain.vlm.client import VLMClient, _parse_json_object


def _jpeg_bytes() -> bytes:
    from PIL import Image

    img = Image.new("RGB", (8, 8), color=(255, 0, 0))
    buf = BytesIO()
    img.save(buf, format="JPEG")
    return buf.getvalue()


class _FakeResponse:
    def __init__(self, payload: dict, status: int = 200) -> None:
        self._payload = payload
        self.status_code = status

    def raise_for_status(self) -> None:
        if self.status_code >= 400:
            raise RuntimeError(f"HTTP {self.status_code}")

    def json(self) -> dict:
        return self._payload


class _FakeHttp:
    """Minimal async stand-in for httpx.AsyncClient."""

    def __init__(self, payload: dict, status: int = 200) -> None:
        self._payload = payload
        self._status = status
        self.last_payload: dict | None = None

    async def post(self, url: str, *, json: dict, headers: dict) -> _FakeResponse:
        self.last_payload = json
        return _FakeResponse(self._payload, status=self._status)


def _content_message(content: str) -> dict:
    return {"choices": [{"message": {"content": content}}]}


def _settings(**kw) -> Settings:
    base = dict(memory_db_path=":memory:", vlm_enabled=True)
    base.update(kw)
    return Settings(**base)


class VLMClientParseTests(unittest.IsolatedAsyncioTestCase):
    async def test_parses_left_json(self):
        http = _FakeHttp(_content_message(
            '{"recommended_direction":"left","confidence":0.82,"reason":"left open"}'
        ))
        client = VLMClient(_settings(), http_client=http)
        hint = await client.analyze_passability(_jpeg_bytes())
        self.assertEqual(hint.recommended_direction, "left")
        self.assertAlmostEqual(hint.confidence, 0.82, places=2)
        self.assertIn("left open", hint.reason)
        self.assertIsNotNone(hint.latency_ms)
        self.assertIsNotNone(hint.frame_timestamp)  # audit field populated
        # Payload shape: multimodal messages with image_url.
        msg = http.last_payload["messages"][0]
        self.assertEqual(msg["content"][0]["type"], "text")
        self.assertEqual(msg["content"][1]["type"], "image_url")
        self.assertTrue(msg["content"][1]["image_url"]["url"].startswith("data:image/jpeg;base64,"))
        self.assertEqual(http.last_payload["temperature"], 0)

    async def test_parses_markdown_fenced_json(self):
        http = _FakeHttp(_content_message(
            '```json\n{"recommended_direction":"right","confidence":0.7}\n```'
        ))
        client = VLMClient(_settings(), http_client=http)
        hint = await client.analyze_passability(_jpeg_bytes())
        self.assertEqual(hint.recommended_direction, "right")

    async def test_confidence_clamped(self):
        http = _FakeHttp(_content_message(
            '{"recommended_direction":"forward","confidence":1.5}'
        ))
        client = VLMClient(_settings(), http_client=http)
        hint = await client.analyze_passability(_jpeg_bytes())
        self.assertEqual(hint.confidence, 1.0)

    async def test_list_content_parts(self):
        http = _FakeHttp({"choices": [{"message": {"content": [
            {"type": "text", "text": '{"recommended_direction":"stop","confidence":0.9}'},
        ]}}]})
        client = VLMClient(_settings(), http_client=http)
        hint = await client.analyze_passability(_jpeg_bytes())
        self.assertEqual(hint.recommended_direction, "stop")

    async def test_invalid_direction_raises(self):
        http = _FakeHttp(_content_message(
            '{"recommended_direction":"up","confidence":0.9}'
        ))
        client = VLMClient(_settings(), http_client=http)
        with self.assertRaises(ValueError):
            await client.analyze_passability(_jpeg_bytes())

    async def test_non_json_raises(self):
        http = _FakeHttp(_content_message("I think you should go forward."))
        client = VLMClient(_settings(), http_client=http)
        with self.assertRaises(ValueError):
            await client.analyze_passability(_jpeg_bytes())

    async def test_http_error_raises(self):
        http = _FakeHttp({}, status=500)
        client = VLMClient(_settings(), http_client=http)
        with self.assertRaises(Exception):
            await client.analyze_passability(_jpeg_bytes())


class JsonObjectParseTests(unittest.TestCase):
    def test_extracts_from_prose(self):
        obj = _parse_json_object('Here is the answer: {"recommended_direction":"left"} done.')
        self.assertEqual(obj["recommended_direction"], "left")

    def test_no_object_raises(self):
        with self.assertRaises(ValueError):
            _parse_json_object("no json here")


if __name__ == "__main__":
    unittest.main()
