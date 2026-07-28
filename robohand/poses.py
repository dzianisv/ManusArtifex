"""Named-pose storage: home/ready/grip presets plus user-taught/calibrated poses.

Poses are stored as JSON: {name: {"positions": {joint: pos, ...}, "duration_ms": int,
"comment": str}}. The default set ships in ``robohand/data/poses.json``; saved/taught
poses are merged into the same file so they persist across CLI invocations.
"""

from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Dict, Optional

from . import config

DEFAULT_POSES_PATH = config.PACKAGE_DIR / "data" / "poses.json"


def _poses_path() -> Path:
    return Path(os.environ.get("ROBOHAND_POSES", DEFAULT_POSES_PATH))


def load_poses(path: Optional[Path] = None) -> Dict[str, dict]:
    path = path or _poses_path()
    if not path.exists():
        return {}
    with open(path) as f:
        return json.load(f)


def save_poses(poses: Dict[str, dict], path: Optional[Path] = None) -> None:
    path = path or _poses_path()
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w") as f:
        json.dump(poses, f, indent=2, sort_keys=True)
        f.write("\n")


def get_pose(name: str, path: Optional[Path] = None) -> dict:
    poses = load_poses(path)
    if name not in poses:
        raise KeyError(f"Unknown pose {name!r}. Known: {sorted(poses)}")
    return poses[name]


def list_poses(path: Optional[Path] = None) -> Dict[str, dict]:
    return load_poses(path)


def save_pose(
    name: str,
    positions: Dict[str, int],
    duration_ms: int = config.DEFAULT_DURATION_MS,
    comment: str = "",
    path: Optional[Path] = None,
) -> None:
    poses = load_poses(path)
    poses[name] = {"positions": positions, "duration_ms": duration_ms, "comment": comment}
    save_poses(poses, path)


def delete_pose(name: str, path: Optional[Path] = None) -> None:
    poses = load_poses(path)
    poses.pop(name, None)
    save_poses(poses, path)
