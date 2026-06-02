"""Run the local FastAPI dashboard at http://127.0.0.1:8000."""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from robot_brain.service.main import main


if __name__ == "__main__":
    main()
