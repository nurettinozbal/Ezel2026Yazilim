import json
from pathlib import Path
from typing import Any, Dict


def load_scenario(path: str) -> Dict[str, Any]:
    """Load a scenario file.

    Files in this repo are JSON-compatible YAML, so json works without extra
    dependencies. If a team later writes richer YAML, PyYAML is used when present.
    """
    text = Path(path).read_text(encoding="utf-8")
    try:
        return json.loads(text)
    except json.JSONDecodeError:
        try:
            import yaml  # type: ignore

            return yaml.safe_load(text)
        except Exception as exc:
            raise RuntimeError(f"Unable to parse scenario file: {path}") from exc
