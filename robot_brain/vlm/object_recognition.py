"""VLM object inventory and target recognition."""
from __future__ import annotations

from pydantic import BaseModel, Field

from robot_brain.vlm.client import VLMClient, _extract_content, _parse_json_object
from robot_brain.vlm.encoding import data_url


class VisualObject(BaseModel):
    name: str
    confidence: float = Field(default=0.0, ge=0.0, le=1.0)
    bbox: tuple[float, float, float, float] | None = None


class ObjectRecognizer:
    def __init__(self, client: VLMClient) -> None:
        self._client = client

    async def recognize(self, image_bytes: bytes, target: str | None = None) -> list[VisualObject]:
        prompt = (
            "识别图中可见物品。bbox 使用归一化坐标[x1,y1,x2,y2]。"
            + (f"重点确认目标物品：{target}。" if target else "列出清晰可见的主要物品。")
            + '只输出JSON：{"objects":[{"name":"物品名","confidence":0.0,"bbox":[0,0,1,1]}]}'
        )
        settings = self._client._settings
        response = await self._client._http.post(
            f"{settings.vlm_base_url.rstrip('/')}/v1/chat/completions",
            json={"model": settings.vlm_model, "messages": [{"role": "user", "content": [
                {"type": "text", "text": prompt},
                {"type": "image_url", "image_url": {"url": data_url(image_bytes, settings.vlm_max_edge)}},
            ]}], "max_tokens": 512, "temperature": 0},
            headers={"Authorization": f"Bearer {settings.vlm_api_key}"},
        )
        response.raise_for_status()
        obj = _parse_json_object(_extract_content(response.json()))
        raw = obj.get("objects", []) if isinstance(obj, dict) else []
        results: list[VisualObject] = []
        for item in raw:
            try:
                results.append(VisualObject.model_validate(item))
            except (ValueError, TypeError):
                continue
        return results
