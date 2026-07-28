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

# Absolute *electrical* hardware safety rail (the physical PWM range the servos
# accept at all). No command may ever be sent outside this range, regardless of
# what a config file or CLI argument requests.
HARD_MIN = 500
HARD_MAX = 2500

# Global absolute guard (Layer 1, outermost ring). We stay 200 units clear of both
# electrical ends unconditionally -- this is NOT configurable via config.json on
# purpose: a bad edit to the JSON file must not be able to widen this rail. Every
# other bound (per-joint config min/max, provisional band, grip backstops) is
# intersected with this one before anything is ever sent to a transport.
GLOBAL_GUARD_MIN = 700
GLOBAL_GUARD_MAX = 2300

# Width of the provisional (uncalibrated-joint) safety band, in raw units either
# side of a joint's configured center. Configurable via config.json
# safety.provisional_band because it is a calibration *policy* knob, not a hard
# electrical/mechanical rail like GLOBAL_GUARD_*.
DEFAULT_PROVISIONAL_BAND = 300


class JointLimit(NamedTuple):
    servo_id: int
    min: int
    max: int
    center: int
    calibrated: bool


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
    calibrated = bool(spec.get("calibrated", False))
    JOINT_LIMITS[name] = JointLimit(servo_id=int(spec["id"]), min=lo, max=hi, center=center, calibrated=calibrated)
    SERVO_IDS[name] = int(spec["id"])
    ID_TO_NAME[int(spec["id"])] = name

JOINT_NAMES = list(JOINT_LIMITS.keys())

_motion = _CONFIG.get("motion", {})
DEFAULT_DURATION_MS = int(_motion.get("default_duration_ms", 1000))
MIN_DURATION_MS = int(_motion.get("min_duration_ms", 400))
MAX_STEP_PER_MOVE = int(_motion.get("max_step_per_move", 400))
SETTLE_MARGIN_MS = int(_motion.get("settle_margin_ms", 100))

_safety = _CONFIG.get("safety", {})
PROVISIONAL_BAND = int(_safety.get("provisional_band", DEFAULT_PROVISIONAL_BAND))
HOLD_MS = int(_safety.get("hold_ms", 2000))
IDLE_UNLOAD_MS = int(_safety.get("idle_unload_ms", 5000))
MAX_CONTINUOUS_TORQUE_MS = int(_safety.get("max_continuous_torque_ms", 8000))

_vision = _CONFIG.get("vision", {})
VISION_ROI = tuple(_vision.get("roi", [0.15, 0.1, 0.7, 0.8]))
VISION_DIFF_THRESHOLD = int(_vision.get("diff_threshold", 25))
VISION_MIN_CHANGED_PIXELS = int(_vision.get("min_changed_pixels", 150))
VISION_BLUR_KSIZE = int(_vision.get("blur_ksize", 5))

_gripper = _CONFIG.get("gripper", {})
GRIP_STEP = int(_gripper.get("step", 40))
GRIP_LADDER_DURATION_MS = int(_gripper.get("ladder_duration_ms", 250))
GRIP_MIN_SAFE = int(_gripper.get("grip_min_safe", JOINT_LIMITS["GRIP"].min if "GRIP" in JOINT_LIMITS else GLOBAL_GUARD_MIN))
GRIP_MAX_SAFE = int(_gripper.get("grip_max_safe", JOINT_LIMITS["GRIP"].max if "GRIP" in JOINT_LIMITS else GLOBAL_GUARD_MAX))


def provisional_bounds(name: str) -> tuple:
    """The conservative center +/- PROVISIONAL_BAND band for an uncalibrated joint."""
    limit = JOINT_LIMITS[name]
    lo = max(GLOBAL_GUARD_MIN, limit.center - PROVISIONAL_BAND)
    hi = min(GLOBAL_GUARD_MAX, limit.center + PROVISIONAL_BAND)
    return lo, hi


def effective_bounds(name: str, allow_uncalibrated_override: bool = False) -> tuple:
    """The actual min/max a move to this joint may use right now.

    Calibrated joints: configured [min, max], intersected with the global guard.
    Uncalibrated joints: the provisional band, intersected with configured [min, max]
    and the global guard -- UNLESS ``allow_uncalibrated_override`` is set, in which
    case the configured [min, max] (still intersected with the global guard) applies.
    This function never returns bounds wider than GLOBAL_GUARD_MIN/MAX.
    """
    limit = JOINT_LIMITS[name]
    lo = max(GLOBAL_GUARD_MIN, limit.min)
    hi = min(GLOBAL_GUARD_MAX, limit.max)
    if limit.calibrated or allow_uncalibrated_override:
        return lo, hi
    plo, phi = provisional_bounds(name)
    return max(lo, plo), min(hi, phi)


def clamp(name: str, position: int, allow_uncalibrated_override: bool = False) -> int:
    """Clamp a requested position to this joint's *effective* safe range.

    This is the last-resort backstop (defense in depth), not the primary safety
    check -- callers that want "refuse instead of silently clamp" behaviour for an
    out-of-band request on an uncalibrated joint should use
    ``safety.SafetyLayer.move`` / check ``effective_bounds`` themselves first.
    """
    lo, hi = effective_bounds(name, allow_uncalibrated_override=allow_uncalibrated_override)
    return max(lo, min(hi, int(position)))


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
