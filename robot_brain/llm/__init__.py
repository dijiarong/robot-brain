"""LLM adapter interfaces and built-in implementations."""

from .base import LLMClient, ToolCall
from .mock import MockLLM

__all__ = ["LLMClient", "MockLLM", "ToolCall"]
