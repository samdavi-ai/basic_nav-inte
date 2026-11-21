"""
Authoritative Places Loader for GRaCEmo ViRa.
Reads directly from config/places.json (single source of truth).
Provides canonical place coordinates, names, and orientation targets across all nodes.
"""

import json
import os
from pathlib import Path
from typing import Dict, Any, Optional


def _default_places_path() -> Path:
    configured = os.environ.get("GRACEMO_PLACES_FILE")
    if configured:
        path = Path(configured).expanduser()
        if path.is_file():
            return path
        raise FileNotFoundError(f"GRACEMO_PLACES_FILE does not point to a file: {path}")

    # Source checkout: find the repository config without assuming its install path.
    for parent in Path(__file__).resolve().parents:
        candidate = parent / "config" / "places.json"
        if candidate.is_file():
            return candidate

    # Installed ROS package: setup.py installs the canonical file to package share.
    try:
        from ament_index_python.packages import get_package_share_directory

        candidate = Path(get_package_share_directory("gracemo_rl")) / "config" / "places.json"
        if candidate.is_file():
            return candidate
    except (ImportError, LookupError):
        pass

    raise FileNotFoundError(
        "Could not locate canonical config/places.json in the source tree or gracemo_rl package share"
    )


def load_authoritative_places(json_path: Optional[str] = None) -> Dict[str, Dict[str, Any]]:
    path = Path(json_path).expanduser() if json_path else _default_places_path()
    if not path.is_file():
        raise FileNotFoundError(f"Places file does not exist: {path}")

    places = {}
    with path.open("r", encoding="utf-8") as f:
        data = json.load(f)

    for key, val in data.items():
        if key.startswith("_"):
            continue
        if isinstance(val, (list, tuple)) and len(val) >= 3:
            x = float(val[0])
            y = float(val[1])
            name = str(val[2])
            yaw = float(val[3]) if len(val) > 3 else 0.0
            place_entry = {
                "name": name,
                "x": x,
                "y": y,
                "yaw": yaw,
                "coords": {"x": x, "y": y, "yaw": yaw, "frame_id": "map"}
            }
            # Register direct key, lower, and space/underscore variations
            k_low = key.lower().strip()
            places[k_low] = place_entry
            places[k_low.replace(" ", "_")] = place_entry
            places[k_low.replace("_", " ")] = place_entry

    return places

