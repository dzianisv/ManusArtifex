"""Configuration for the robohand package: USB identity, servo map, and safety limits.

Defaults live in ``robohand/data/config.json`` so they can be tuned without touching
code. Set the ``ROBOHAND_CONFIG`` environment variable to point at an alternate file
(e.g. a per-arm calibration override).
"""

from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Dict, NamedTuple

PACKAGE_DIR = Path(__file__).resolve().parent
DEFAULT_CONFIG_PATH = PACKAGE_DIR / "data" / "config.json"
DEFAULT_POSES_PATH = PACKAGE_DIR / "data" / "poses.json"

# Absolute hardware safety rail. No command may ever be sent outside this range,
# regardless of what a config file or CLI argument requests.
HARD_MIN = 500
HARD_MAX = 2500


class JointLimit(NamedTuple):
    servo_id: int
    min: int
    max: int
    center: int


def _config_path() -> Path:
    return Path(os.environ.get("ROBOHAND_CONFIG", DEFAULT_CONFIG_PATH))


def load_config(path: Path | None = None) -> dict:
    path = path or _config_path()
    with open(path) as f:
        return json.load(f)


_CONFIG = load_config()

VENDOR_ID = int(_CONFIG["usb"]["vendor_id"], 16)
PRODUCT_ID = int(_CONFIG["usb"]["product_id"], 16)

_hw = _CONFIG.get("hardware_limits", {})
CONFIG_HARD_MIN = max(HARD_MIN, int(_hw.get("min", HARD_MIN)))
CONFIG_HARD_MAX = min(HARD_MAX, int(_hw.get("max", HARD_MAX)))

JOINT_LIMITS: Dict[str, JointLimit] = {}
SERVO_IDS: Dict[str, int] = {}
ID_TO_NAME: Dict[int, str] = {}

for name, spec in _CONFIG["servos"].items():
    lo = max(CONFIG_HARD_MIN, int(spec["min"]))
    hi = min(CONFIG_HARD_MAX, int(spec["max"]))
    center = int(spec.get("center", (lo + hi) // 2))
    JOINT_LIMITS[name] = JointLimit(servo_id=int(spec["id"]), min=lo, max=hi, center=center)
    SERVO_IDS[name] = int(spec["id"])
    ID_TO_NAME[int(spec["id"])] = name

JOINT_NAMES = list(JOINT_LIMITS.keys())

_motion = _CONFIG.get("motion", {})
DEFAULT_DURATION_MS = int(_motion.get("default_duration_ms", 1000))
MIN_DURATION_MS = int(_motion.get("min_duration_ms", 400))
MAX_STEP_PER_MOVE = int(_motion.get("max_step_per_move", 400))
SETTLE_MARGIN_MS = int(_motion.get("settle_margin_ms", 100))


def clamp(name: str, position: int) -> int:
    """Clamp a requested position to this joint's configured safe range."""
    limit = JOINT_LIMITS[name]
    return max(limit.min, min(limit.max, int(position)))


def resolve_servo(name_or_id) -> int:
    """Accept either a joint name (e.g. 'GRIP') or a raw numeric servo id."""
    if isinstance(name_or_id, str):
        key = name_or_id.strip().upper()
        if key not in SERVO_IDS:
            raise KeyError(f"Unknown joint name {name_or_id!r}. Known: {JOINT_NAMES}")
        return SERVO_IDS[key]
    servo_id = int(name_or_id)
    if servo_id not in ID_TO_NAME:
        raise KeyError(f"Unknown servo id {servo_id}. Known ids: {sorted(ID_TO_NAME)}")
    return servo_id
