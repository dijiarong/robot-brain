"""JPEG base64 encoding for VLM image payloads.

Mirrors the data-URL form expected by OpenAI-compatible multimodal endpoints:
``data:image/jpeg;base64,...``. Images are normalized to RGB and downscaled so
the longest edge does not exceed ``max_edge`` before JPEG re-encoding, keeping
request size bounded for a low-bandwidth LAN VLM service.
"""
from __future__ import annotations

import base64
from io import BytesIO


def encode_jpeg_b64(image_bytes: bytes, max_edge: int = 768, *, quality: int = 85) -> str:
    """Return base64-encoded JPEG for *image_bytes*, longest edge <= *max_edge*.

    The input may be any format Pillow can decode (JPEG/PNG/...). It is
    re-encoded as JPEG so the data URL is always image/jpeg. Returns the
    base64 string without the ``data:`` prefix (the caller wraps it).
    """
    if not image_bytes:
        raise ValueError("image_bytes is empty")

    from PIL import Image  # imported lazily so the module imports without PIL

    img = Image.open(BytesIO(image_bytes))
    img = img.convert("RGB")

    w, h = img.size
    longest = max(w, h)
    if max_edge > 0 and longest > max_edge:
        scale = max_edge / longest
        img = img.resize((max(1, int(w * scale)), max(1, int(h * scale))))

    buf = BytesIO()
    img.save(buf, format="JPEG", quality=quality)
    return base64.b64encode(buf.getvalue()).decode("ascii")


def data_url(image_bytes: bytes, max_edge: int = 768, *, quality: int = 85) -> str:
    """Return a full ``data:image/jpeg;base64,...`` URL."""
    return f"data:image/jpeg;base64,{encode_jpeg_b64(image_bytes, max_edge, quality=quality)}"
