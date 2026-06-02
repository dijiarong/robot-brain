"""Command-line entry point for the robot-brain HTTP service."""
from __future__ import annotations

import uvicorn


def main() -> None:
    uvicorn.run(
        "robot_brain.service.app:create_default_app",
        factory=True,
        host="127.0.0.1",
        port=8000,
    )


if __name__ == "__main__":
    main()
