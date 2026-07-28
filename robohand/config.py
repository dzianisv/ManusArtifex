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
#
# IMPORTANT: this pair is a reasonable default for ordinary hobby servos across
# the full 500..2500 electrical range, but it is NOT sufficient on its own for
# every joint. The 2026-07-28 incident (root-caused after the fact, see
# docs/research-hardware.md) showed that GRIP (servo id 1) is a LeArm-family
# gripper servo whose *own* valid electrical range is 1500..2500, not 500..2500
# -- 700 is comfortably inside this global pair yet is 800 units past GRIP's real
# floor. A single global pair cannot express that. See ABSOLUTE_MIN_OVERRIDE /
# ABSOLUTE_MAX_OVERRIDE below, which layer stricter, per-servo rails on top of
# this one wherever a servo's own hardware disagrees with the generic default.
GLOBAL_GUARD_MIN = 700
GLOBAL_GUARD_MAX = 2300

# Per-servo absolute rails that are STRICTER than GLOBAL_GUARD_MIN/MAX for that
# specific servo model. Like GLOBAL_GUARD_MIN/MAX, these are hardcoded in Python
# -- not read from config.json -- for the same reason: editing the JSON config
# must never be able to widen a true electrical/mechanical limit. `absolute_bounds`
# below intersects these with GLOBAL_GUARD_MIN/MAX, so the effective floor/ceiling
# for a joint is always the *tighter* of the two.
#
# GRIP (servo id 1): confirmed by the reference xArmServoController firmware
# (`case LeArm: if (servo == 1) return clamp(value, 1500, 2500);`) and
# independently by a LeArm owner (ccourson/Hiwonder-xArm1S#5). Lower raw = MORE
# OPEN; 1500 is the true open mechanical/electrical stop. The 2026-07-28 incident
# command (close_grip=700) was 800 units below this floor -- this override is
# what makes that specific failure mode structurally impossible even if every
# other layer (config.json bounds, calibration flag, provisional band) were
# somehow misconfigured back to the old, wrong 500..2500 assumption.
ABSOLUTE_MIN_OVERRIDE: Dict[str, int] = {
    "GRIP": 1500,
}
ABSOLUTE_MAX_OVERRIDE: Dict[str, int] = {}


def absolute_bounds(name: str) -> tuple:
    """The strictest hardcoded floor/ceiling for this joint: GLOBAL_GUARD_MIN/MAX
    intersected with any per-servo override. Nothing in config.json can widen this.
    """
    lo = max(GLOBAL_GUARD_MIN, ABSOLUTE_MIN_OVERRIDE.get(name, GLOBAL_GUARD_MIN))
    hi = min(GLOBAL_GUARD_MAX, ABSOLUTE_MAX_OVERRIDE.get(name, GLOBAL_GUARD_MAX))
    return lo, hi


# Width of the provisional (uncalibrated-joint) safety band, in raw units either
# side of a joint's configured center. Configurable via config.json
# safety.provisional_band because it is a calibration *policy* knob, not a hard
# electrical/mechanical rail like GLOBAL_GUARD_*/ABSOLUTE_*_OVERRIDE.
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

_safety_raw = _CONFIG.get("safety", {})
_json_gripper_floor = _safety_raw.get("gripper_absolute_floor")
if _json_gripper_floor is not None and int(_json_gripper_floor) != ABSOLUTE_MIN_OVERRIDE.get("GRIP"):
    # This is a deliberate tripwire, not defensive paranoia: safety.gripper_absolute_floor
    # in config.json is documentation for humans reading the file, NOT the enforcement
    # mechanism (ABSOLUTE_MIN_OVERRIDE above is). If someone edits one without the other,
    # that is exactly the kind of silent drift that must fail loudly instead of quietly
    # using whichever value happened to win.
    raise ValueError(
        "config.json safety.gripper_absolute_floor "
        f"({_json_gripper_floor}) disagrees with the hardcoded "
        f"ABSOLUTE_MIN_OVERRIDE['GRIP'] ({ABSOLUTE_MIN_OVERRIDE.get('GRIP')}) in "
        "robohand/config.py. These must always agree -- the JSON value is "
        "documentation only; the Python constant is what is actually enforced. "
        "Fix config.json or config.py so they match."
    )

JOINT_LIMITS: Dict[str, JointLimit] = {}
SERVO_IDS: Dict[str, int] = {}
ID_TO_NAME: Dict[int, str] = {}

for name, spec in _CONFIG["servos"].items():
    abs_lo, abs_hi = absolute_bounds(name)
    lo = max(CONFIG_HARD_MIN, abs_lo, int(spec["min"]))
    hi = min(CONFIG_HARD_MAX, abs_hi, int(spec["max"]))
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

_safety = _safety_raw
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
_grip_abs_lo, _grip_abs_hi = absolute_bounds("GRIP") if "GRIP" in JOINT_LIMITS else (GLOBAL_GUARD_MIN, GLOBAL_GUARD_MAX)
GRIP_STEP = int(_gripper.get("step", 40))
GRIP_LADDER_DURATION_MS = int(_gripper.get("ladder_duration_ms", 250))
GRIP_MIN_SAFE = int(_gripper.get("grip_min_safe", JOINT_LIMITS["GRIP"].min if "GRIP" in JOINT_LIMITS else _grip_abs_lo))
GRIP_MAX_SAFE = int(_gripper.get("grip_max_safe", JOINT_LIMITS["GRIP"].max if "GRIP" in JOINT_LIMITS else _grip_abs_hi))
if GRIP_MIN_SAFE < _grip_abs_lo:
    raise ValueError(
        f"config.json gripper.grip_min_safe ({GRIP_MIN_SAFE}) is below the absolute "
        f"GRIP floor ({_grip_abs_lo}). This backstop must never be looser than the "
        "hardcoded absolute rail -- fix gripper.grip_min_safe in config.json."
    )
if GRIP_MAX_SAFE > _grip_abs_hi:
    raise ValueError(
        f"config.json gripper.grip_max_safe ({GRIP_MAX_SAFE}) is above the absolute "
        f"GRIP ceiling ({_grip_abs_hi}). This backstop must never be looser than the "
        "hardcoded absolute rail -- fix gripper.grip_max_safe in config.json."
    )


def provisional_bounds(name: str) -> tuple:
    """The conservative center +/- PROVISIONAL_BAND band for an uncalibrated joint."""
    limit = JOINT_LIMITS[name]
    guard_lo, guard_hi = absolute_bounds(name)
    lo = max(guard_lo, limit.center - PROVISIONAL_BAND)
    hi = min(guard_hi, limit.center + PROVISIONAL_BAND)
    return lo, hi


def effective_bounds(name: str, allow_uncalibrated_override: bool = False) -> tuple:
    """The actual min/max a move to this joint may use right now.

    Calibrated joints: configured [min, max], intersected with the per-servo
    absolute guard (``absolute_bounds`` -- global guard tightened by any per-servo
    override, e.g. GRIP's 1500 floor). Uncalibrated joints: the provisional band,
    intersected with configured [min, max] and the absolute guard -- UNLESS
    ``allow_uncalibrated_override`` is set, in which case the configured [min, max]
    (still intersected with the absolute guard) applies. This function never
    returns bounds wider than ``absolute_bounds(name)``, regardless of override.
    """
    limit = JOINT_LIMITS[name]
    guard_lo, guard_hi = absolute_bounds(name)
    lo = max(guard_lo, limit.min)
    hi = min(guard_hi, limit.max)
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
