"""VLM client for passability hints (local Qwen3-VL, OpenAI-compatible).

Distinct from :class:`robot_brain.llm.compatible_client.CompatibleLLMClient`:
this only does vision -> JSON hint, with no tool-calling and no text planning.
Keeping them separate avoids coupling multimodal vision with the planner prompt
and keeps a single frame out of the planner context.
"""
from __future__ import annotations

import json
import logging
import re
import time
from datetime import datetime, timezone
from typing import TYPE_CHECKING

import httpx

from robot_brain.core.passability import PassabilityHint
from robot_brain.vlm.encoding import data_url

if TYPE_CHECKING:
    from config.settings import Settings

logger = logging.getLogger(__name__)

#: Prompt forces JSON-only output at temperature=0 for deterministic parsing.
PASSABILITY_PROMPT = (
    "你是四足机器狗的前视相机助手。根据这张前视图像，判断机器狗下一步更适合朝哪个方向移动。"
    "只能选一个：forward（正前方可通行）、left（左转更可通行）、"
    "right（右转更可通行）、stop（不宜移动，如楼梯/人/玻璃/危险）。"
    "只输出 JSON：{\"recommended_direction\":\"...\",\"confidence\":0.0-1.0,\"reason\":\"...\"}"
    "不要输出其它文字。"
)

_JSON_FENCE_RE = re.compile(r"```(?:json)?\s*(\{.*?\})\s*```", re.DOTALL)
_BARE_OBJECT_RE = re.compile(r"\{.*\}", re.DOTALL)
_VALID_DIRECTIONS = {"forward", "left", "right", "stop"}


class VLMClient:
    """Calls a local OpenAI-compatible multimodal endpoint for passability."""

    def __init__(self, settings: "Settings", *, http_client: httpx.AsyncClient | None = None) -> None:
        self._settings = settings
        self._http = http_client or httpx.AsyncClient(timeout=settings.vlm_timeout)

    async def analyze_passability(self, image_bytes: bytes) -> PassabilityHint:
        """Encode *image_bytes* and return a validated :class:`PassabilityHint`.

        Raises on HTTP error, missing content, or unparseable JSON; the caller
        (:class:`PassabilityAnalyzer`) catches and falls back to rules.
        """
        url = f"{self._settings.vlm_base_url.rstrip('/')}/v1/chat/completions"
        payload = {
            "model": self._settings.vlm_model,
            "messages": [
                {
                    "role": "user",
                    "content": [
                        {"type": "text", "text": PASSABILITY_PROMPT},
                        {"type": "image_url", "image_url": {"url": data_url(image_bytes, self._settings.vlm_max_edge)}},
                    ],
                }
            ],
            "max_tokens": 128,
            "temperature": 0,
        }
        headers = {"Authorization": f"Bearer {self._settings.vlm_api_key}"}

        start = time.monotonic()
        resp = await self._http.post(url, json=payload, headers=headers)
        resp.raise_for_status()
        latency_ms = (time.monotonic() - start) * 1000.0

        data = resp.json()
        content = _extract_content(data)
        return self._parse_hint(content, latency_ms=latency_ms)

    def _parse_hint(self, content: str, *, latency_ms: float) -> PassabilityHint:
        obj = _parse_json_object(content)
        if not isinstance(obj, dict):
            raise ValueError(f"VLM output is not a JSON object: {content!r}")

        direction = obj.get("recommended_direction")
        if direction not in _VALID_DIRECTIONS:
            raise ValueError(f"invalid recommended_direction: {direction!r}")

        confidence = obj.get("confidence", 0.0)
        try:
            confidence = float(confidence)
        except (TypeError, ValueError):
            confidence = 0.0
        confidence = max(0.0, min(1.0, confidence))

        reason = str(obj.get("reason", ""))[:200]
        return PassabilityHint(
            recommended_direction=direction,  # type: ignore[arg-type]
            confidence=confidence,
            reason=reason,
            source="qwen3-vl",
            frame_timestamp=datetime.now(timezone.utc),
            latency_ms=round(latency_ms, 1),
            raw_model=self._settings.vlm_model,
        )

    async def aclose(self) -> None:
        await self._http.aclose()


def _extract_content(data: dict) -> str:
    """Pull the assistant message content out of a Chat Completions response."""
    choices = data.get("choices") or []
    if not choices:
        raise ValueError("VLM response has no choices")
    message = choices[0].get("message") or {}
    content = message.get("content")
    if not content:
        raise ValueError("VLM response message has no content")
    if isinstance(content, list):
        # Some providers return content as a list of parts.
        parts = [p.get("text", "") for p in content if isinstance(p, dict)]
        content = "".join(parts)
    return str(content)


def _parse_json_object(content: str) -> object:
    """Extract the first JSON object from *content* (tolerates fences/prose)."""
    text = content.strip()
    m = _JSON_FENCE_RE.search(text)
    if m:
        return json.loads(m.group(1))
    try:
        return json.loads(text)
    except (json.JSONDecodeError, TypeError):
        pass
    m = _BARE_OBJECT_RE.search(text)
    if m:
        return json.loads(m.group(0))
    raise ValueError(f"no JSON object found in VLM output: {content!r}")
