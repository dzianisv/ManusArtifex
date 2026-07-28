"""Named-pose storage: home/ready/grip presets plus user-taught/calibrated poses.

Poses are stored as JSON: {name: {"positions": {joint: pos, ...}, "duration_ms": int,
"comment": str}}. The default set ships in ``robohand/data/poses.json``; saved/taught
poses are merged into the same file so they persist across CLI invocations.

Every pose is validated -- both when loaded from disk and immediately before being
written -- against ``config.validate_positions``/``config.validate_duration_ms``.
This means a hand-corrupted poses.json is rejected at load time (never silently
clamped later, at move time), and a bad pose can never enter the file in the first
place via ``save_pose``/``save_poses``, regardless of caller (``pose --save``,
``teach``, or any future writer).
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


def _validate_pose(name: str, pose: dict) -> None:
    if not isinstance(pose, dict) or "positions" not in pose:
        raise config.InvalidPosition(f"pose {name!r} must be an object with a 'positions' key.")
    try:
        config.validate_positions(pose["positions"])
    except config.InvalidPosition as e:
        raise config.InvalidPosition(f"pose {name!r}: {e}") from e
    if "duration_ms" in pose:
        try:
            config.validate_duration_ms(pose["duration_ms"], allow_none=True)
        except config.InvalidPosition as e:
            raise config.InvalidPosition(f"pose {name!r}: {e}") from e


def _validate_poses(poses: Dict[str, dict]) -> None:
    if not isinstance(poses, dict):
        raise config.InvalidPosition("poses file must contain a JSON object of {name: pose}.")
    for name, pose in poses.items():
        _validate_pose(name, pose)


def load_poses(path: Optional[Path] = None) -> Dict[str, dict]:
    path = path or _poses_path()
    if not path.exists():
        return {}
    with open(path) as f:
        poses = json.load(f)
    _validate_poses(poses)
    return poses


def save_poses(poses: Dict[str, dict], path: Optional[Path] = None) -> None:
    _validate_poses(poses)
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
    duration_ms: Optional[int] = None,
    comment: str = "",
    path: Optional[Path] = None,
) -> None:
    positions = config.validate_positions(positions)
    if duration_ms is None:
        duration_ms = config.DEFAULT_DURATION_MS
    duration_ms = config.validate_duration_ms(duration_ms, allow_none=False)
    poses = load_poses(path)
    poses[name] = {"positions": positions, "duration_ms": duration_ms, "comment": comment}
    save_poses(poses, path)


def delete_pose(name: str, path: Optional[Path] = None) -> None:
    poses = load_poses(path)
    poses.pop(name, None)
    save_poses(poses, path)
