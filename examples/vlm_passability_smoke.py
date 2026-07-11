"""VLM passability smoke test - probe a LAN Qwen3-VL with one image.

Verifies connectivity and the JSON contract without involving the robot or
the explore loop. Configure the service via env::

    export RDB_VLM_ENABLED=true
    export RDB_VLM_BASE_URL=http://10.10.197.175:8080
    export RDB_VLM_MODEL=/Users/dijia/models/Qwen3-VL-8B-4bit
    python -m examples.vlm_passability_smoke --image path/to/front.jpg
"""
from __future__ import annotations

import argparse
import asyncio
import sys
from pathlib import Path


async def main(image: str) -> int:
    from config.settings import SETTINGS
    from robot_brain.vlm.client import VLMClient

    if not Path(image).exists():
        print(f"image not found: {image}", file=sys.stderr)
        return 2

    image_bytes = Path(image).read_bytes()
    print(f"VLM base_url={SETTINGS.vlm_base_url} model={SETTINGS.vlm_model}")
    print(f"image={image} ({len(image_bytes)} bytes)")

    client = VLMClient(SETTINGS)
    try:
        hint = await client.analyze_passability(image_bytes)
    except Exception as exc:
        print(f"VLM call failed: {exc}", file=sys.stderr)
        return 1
    finally:
        await client.aclose()

    print(
        f"hint: direction={hint.recommended_direction} "
        f"confidence={hint.confidence:.2f} latency={hint.latency_ms}ms"
    )
    print(f"reason: {hint.reason}")
    return 0


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="VLM passability smoke test")
    parser.add_argument("--image", required=True, help="Path to a front-camera image")
    args = parser.parse_args()
    sys.exit(asyncio.run(main(args.image)))
