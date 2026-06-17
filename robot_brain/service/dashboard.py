"""Dashboard HTML loader — reads from static/index.html."""

from pathlib import Path


_STATIC_DIR = Path(__file__).resolve().parent / "static"
_INDEX_PATH = _STATIC_DIR / "index.html"


def load_dashboard_html() -> str:
    """Return the full dashboard HTML string, with a fallback for development."""
    if _INDEX_PATH.exists():
        return _INDEX_PATH.read_text(encoding="utf-8")
    # Minimal fallback if the static file is missing
    return """<!doctype html><html lang="en"><head><meta charset="utf-8">
<title>Robot Brain</title></head><body>
<h1>Robot Brain Console</h1><p>Dashboard static file not found at {path}.</p>
</body></html>""".format(path=str(_INDEX_PATH))


DASHBOARD_HTML = load_dashboard_html()
