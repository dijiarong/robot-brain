"""Local VLM passability hint subsystem.

Read-only vision sensor for the explore skill: captures a frame, asks a local
Qwen3-VL (OpenAI-compatible multimodal) which direction is passable, and
returns a validated :class:`~robot_brain.core.passability.PassabilityHint`.

Ultrasonic proximity remains the hard safety gate; the VLM hint is a soft
suggestion only. See docs/plans/2026-06-24-170000-vlm-passability-hint.md.
"""
from robot_brain.vlm.client import PASSABILITY_PROMPT, VLMClient
from robot_brain.vlm.encoding import data_url, encode_jpeg_b64
from robot_brain.vlm.frame_source import (
    FileFrameSource,
    FrameSource,
    Go2VideoFrameSource,
    MockFrameSource,
    NullFrameSource,
)
from robot_brain.vlm.passability import PassabilityAnalyzer

__all__ = [
    "FileFrameSource",
    "FrameSource",
    "Go2VideoFrameSource",
    "MockFrameSource",
    "NullFrameSource",
    "PASSABILITY_PROMPT",
    "PassabilityAnalyzer",
    "VLMClient",
    "data_url",
    "encode_jpeg_b64",
]
