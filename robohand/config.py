"""Configuration for the robohand package: USB identity, servo map, and safety limits.

Defaults live in ``robohand/data/config.json`` so they can be tuned without touching
code. Set the ``ROBOHAND_CONFIG`` environment variable to point at an alternate file
(e.g. a per-arm calibration override).
"""

from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Dict, NamedTuple, Tuple

PACKAGE_DIR = Path(__file__).resolve().parent
DEFAULT_CONFIG_PATH = PACKAGE_DIR / "data" / "config.json"
DEFAULT_POSES_PATH = PACKAGE_DIR / "data" / "poses.json"


class ConfigError(ValueError):
    """Raised when config.json (or an equivalent override file) is structurally
    invalid: an inverted min/max, a range that widens past a servo's electrical
    limit, a center outside its own band, an unknown joint name, a non-integer
    value, or a missing required key. Always names the offending key and value.
    """


class InvalidPosition(ValueError):
    """Raised whenever a requested joint position, joint name, or duration is not
    a legal value -- used uniformly by CLI parsing, pose/sequence file loading,
    pose --save/teach write paths, and the direct API. This is the "reject, don't
    silently clamp" half of the safety design: anything a human or a file supplied
    that is out of range is refused here, before it goes anywhere near a transport.
    """


# ---------------------------------------------------------------------------
# Single source of truth: hardware wiring and electrical limits.
#
# CANONICAL_JOINTS fixes which joint name is wired to which numeric servo id --
# this is a physical fact about the arm, not a policy choice, so it is hardcoded
# here and NOT settable via config.json. A config.json that tries to reassign a
# joint's id (e.g. claim GRIP is id 2) is rejected at load time (see
# ``_validate_servo_spec``) rather than silently trusted.
#
# ELECTRICAL_LIMITS is the *only* place the servos' raw hardware-legal PWM ranges
# are allowed to be defined, keyed by canonical servo id. Every other bound in
# this module (GLOBAL_GUARD_MIN/MAX, per-joint configured min/max, the
# provisional band, the grip ladder backstops) is derived from this table and
# may only ever be equal to or narrower than it -- nothing in this package may
# widen past these numbers, from any entry point, ever.
# ---------------------------------------------------------------------------
CANONICAL_JOINTS: Dict[str, int] = {
    "GRIP": 1,
    "WRIST_ROLL": 2,
    "WRIST_FLEX": 3,
    "ELBOW": 4,
    "SHOULDER_LIFT": 5,
    "SHOULDER_PAN": 6,
}

# GRIP (servo id 1): confirmed by the reference xArmServoController firmware
# (`case LeArm: if (servo == 1) return clamp(value, 1500, 2500);`) and
# independently by a LeArm owner (ccourson/Hiwonder-xArm1S#5). Lower raw = MORE
# OPEN; 1500 is the true open mechanical/electrical stop. The 2026-07-28 incident
# command (close_grip=700) was 800 units below this floor. Every other joint uses
# the generic 500..2500 electrical range for this controller/servo family.
ELECTRICAL_LIMITS: Dict[int, Tuple[int, int]] = {
    1: (1500, 2500),  # GRIP
    2: (500, 2500),  # WRIST_ROLL
    3: (500, 2500),  # WRIST_FLEX
    4: (500, 2500),  # ELBOW
    5: (500, 2500),  # SHOULDER_LIFT
    6: (500, 2500),  # SHOULDER_PAN
}

# Backward-compatible aliases: the flattest possible electrical envelope across
# every servo. Only used for the config.json `hardware_limits` documentation
# tripwire below -- never used to compute a per-joint bound (ELECTRICAL_LIMITS is
# always used for that, per-servo, so GRIP's tighter 1500 floor can never be lost
# by accidentally falling back to these flat numbers).
HARD_MIN = min(lo for lo, _ in ELECTRICAL_LIMITS.values())
HARD_MAX = max(hi for _, hi in ELECTRICAL_LIMITS.values())

# Global conservative *policy* ring (Layer 1, outermost operational ring). We stay
# clear of both electrical ends unconditionally while joints are uncalibrated --
# this is NOT configurable via config.json on purpose: a bad edit to the JSON file
# must not be able to widen this rail. It is intersected with each servo's own
# ELECTRICAL_LIMITS (see ``absolute_bounds``) so a joint whose electrical floor is
# already stricter than this ring (GRIP) keeps its own, tighter floor.
GLOBAL_GUARD_MIN = 700
GLOBAL_GUARD_MAX = 2300


def electrical_bounds(name: str) -> Tuple[int, int]:
    """The hardware-legal electrical range for this joint -- the outermost limit
    that can never be widened, configured, or overridden past, from any entry
    point in this package (config.json, poses, sequences, the CLI, the direct
    API, or the transport layer itself). Looked up from ``ELECTRICAL_LIMITS`` by
    this joint's fixed ``CANONICAL_JOINTS`` servo id.
    """
    if name not in CANONICAL_JOINTS:
        raise InvalidPosition(f"unknown joint {name!r}. Known joints: {sorted(CANONICAL_JOINTS)}")
    return ELECTRICAL_LIMITS[CANONICAL_JOINTS[name]]


def absolute_bounds(name: str) -> tuple:
    """The strictest hardcoded floor/ceiling for this joint: ``GLOBAL_GUARD_MIN/MAX``
    intersected with this joint's own ``ELECTRICAL_LIMITS``. Nothing in config.json
    can widen this -- it is the ceiling every configured min/max, provisional band,
    and override is measured against.
    """
    elec_lo, elec_hi = electrical_bounds(name)
    lo = max(GLOBAL_GUARD_MIN, elec_lo)
    hi = min(GLOBAL_GUARD_MAX, elec_hi)
    return lo, hi


def validate_joint_name(name) -> str:
    """Normalize and validate a joint name. Raises ``InvalidPosition`` if ``name``
    is not a string, or does not match one of the six known canonical joints.
    """
    if not isinstance(name, str):
        raise InvalidPosition(f"joint name must be a string, got {name!r} ({type(name).__name__}).")
    key = name.strip().upper()
    if key not in CANONICAL_JOINTS:
        raise InvalidPosition(f"unknown joint {name!r}. Known joints: {sorted(CANONICAL_JOINTS)}.")
    return key


def validate_position_value(name: str, value) -> int:
    """Validate a single requested position is a real ``int`` inside ``name``'s
    hardware-legal electrical range. This is the absolute floor every entry point
    (CLI, pose file, sequence file, ``pose --save``/``teach``, direct API) must
    apply before a position is considered further -- it does NOT apply the
    calibration/provisional-band *policy* (that is ``safety.SafetyLayer``'s job);
    it only rejects values that are illegal on this hardware no matter what.
    """
    key = validate_joint_name(name)
    if isinstance(value, bool) or not isinstance(value, int):
        raise InvalidPosition(
            f"{key}: position must be an integer (raw PWM microseconds), got "
            f"{value!r} ({type(value).__name__}). Non-integer/float/string positions "
            "are never accepted."
        )
    lo, hi = electrical_bounds(key)
    if not (lo <= value <= hi):
        raise InvalidPosition(
            f"{key}: position {value} is outside its hardware-legal electrical range "
            f"[{lo}, {hi}]. This is {key}'s own physical servo limit and can never be "
            f"widened by any config, override, or flag -- choose a value inside "
            f"[{lo}, {hi}]."
        )
    return value


def validate_positions(positions) -> Dict[str, int]:
    """Validate a ``{joint_name: position}`` mapping. Returns a normalized dict
    (uppercased keys, plain ``int`` values). Raises ``InvalidPosition`` naming the
    first offending entry. This is the single choke point used by CLI parsing,
    pose file loading, sequence file loading, and ``pose --save``/``teach`` --
    every entry point that accepts raw positions from outside the safety layer's
    own calibration policy.
    """
    if not isinstance(positions, dict) or not positions:
        raise InvalidPosition(
            f"positions must be a non-empty object of {{JOINT: position}}, got {positions!r}."
        )
    validated: Dict[str, int] = {}
    for name, value in positions.items():
        key = validate_joint_name(name)
        validated[key] = validate_position_value(key, value)
    return validated


# Sanity cap on any single move/step duration. Not a hardware limit -- guards
# against a pathological input (e.g. a stray ``1e9``, or a negative value) causing
# a divide-by-zero downstream, an effectively-instant slam (duration <= 0), or a
# multi-hour sleep that looks like a hang.
MAX_REASONABLE_DURATION_MS = 60_000


def validate_duration_ms(value, *, allow_none: bool = True):
    """Validate a duration in milliseconds. ``None`` is allowed (caller substitutes
    a default) unless ``allow_none=False``. Otherwise ``value`` must be a positive
    ``int`` no larger than ``MAX_REASONABLE_DURATION_MS``.
    """
    if value is None:
        if allow_none:
            return None
        raise InvalidPosition("duration_ms is required.")
    if isinstance(value, bool) or not isinstance(value, int):
        raise InvalidPosition(
            f"duration_ms must be an integer, got {value!r} ({type(value).__name__})."
        )
    if value <= 0:
        raise InvalidPosition(
            f"duration_ms must be a positive integer, got {value} (non-positive durations "
            "would be an instant slam or a divide-by-zero)."
        )
    if value > MAX_REASONABLE_DURATION_MS:
        raise InvalidPosition(
            f"duration_ms {value} exceeds the sanity cap of {MAX_REASONABLE_DURATION_MS}ms."
        )
    return value


def _require_int(value, key_path: str):
    if isinstance(value, bool) or not isinstance(value, int):
        raise ConfigError(
            f"config.json {key_path} must be an integer, got {value!r} ({type(value).__name__})."
        )
    return value


def _require_positive_int(value, key_path: str, *, allow_zero: bool = False) -> int:
    value = _require_int(value, key_path)
    if allow_zero and value < 0:
        raise ConfigError(f"config.json {key_path} must be >= 0, got {value}.")
    if not allow_zero and value <= 0:
        raise ConfigError(f"config.json {key_path} must be a positive integer, got {value}.")
    return value


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

_safety_raw = _CONFIG.get("safety", {})
_json_gripper_floor = _safety_raw.get("gripper_absolute_floor")
_grip_electrical_floor = ELECTRICAL_LIMITS[CANONICAL_JOINTS["GRIP"]][0]
if _json_gripper_floor is not None and _require_int(_json_gripper_floor, "safety.gripper_absolute_floor") != _grip_electrical_floor:
    # This is a deliberate tripwire, not defensive paranoia: safety.gripper_absolute_floor
    # in config.json is documentation for humans reading the file, NOT the enforcement
    # mechanism (ELECTRICAL_LIMITS above is). If someone edits one without the other,
    # that is exactly the kind of silent drift that must fail loudly instead of quietly
    # using whichever value happened to win.
    raise ConfigError(
        "config.json safety.gripper_absolute_floor "
        f"({_json_gripper_floor}) disagrees with the hardcoded "
        f"ELECTRICAL_LIMITS floor for GRIP ({_grip_electrical_floor}) in "
        "robohand/config.py. These must always agree -- the JSON value is "
        "documentation only; the Python constant is what is actually enforced. "
        "Fix config.json or config.py so they match."
    )

_hw = _CONFIG.get("hardware_limits", {})
if "min" in _hw and _require_int(_hw["min"], "hardware_limits.min") != HARD_MIN:
    raise ConfigError(
        f"config.json hardware_limits.min ({_hw['min']}) disagrees with the hardcoded "
        f"floor across ELECTRICAL_LIMITS ({HARD_MIN}). This field is documentation only "
        "-- fix config.json or robohand/config.py so they match."
    )
if "max" in _hw and _require_int(_hw["max"], "hardware_limits.max") != HARD_MAX:
    raise ConfigError(
        f"config.json hardware_limits.max ({_hw['max']}) disagrees with the hardcoded "
        f"ceiling across ELECTRICAL_LIMITS ({HARD_MAX}). This field is documentation "
        "only -- fix config.json or robohand/config.py so they match."
    )


def _validate_servo_spec(name: str, spec: dict) -> JointLimit:
    """Validate one ``servos.<name>`` entry from config.json. Raises ``ConfigError``
    naming the exact offending key on any of: an unknown joint name, a servo id
    that disagrees with the fixed hardware wiring, a missing/non-integer min or
    max, an inverted min/max, a min/max that widens past this joint's electrical
    range, a non-integer or out-of-band center, or a non-bool ``calibrated`` flag.
    """
    if name not in CANONICAL_JOINTS:
        raise ConfigError(
            f"config.json servos: unknown joint name {name!r}. Known joints: "
            f"{sorted(CANONICAL_JOINTS)}. Remove or rename this entry in config.json."
        )
    servo_id = CANONICAL_JOINTS[name]
    if "id" in spec and _require_int(spec["id"], f"servos.{name}.id") != servo_id:
        raise ConfigError(
            f"config.json servos.{name}.id ({spec['id']}) disagrees with the fixed "
            f"hardware wiring id ({servo_id}) for {name}. The joint-name-to-servo-id "
            "mapping is fixed in robohand/config.py (CANONICAL_JOINTS) and physically "
            f"wired -- it cannot be changed via config.json. Fix or remove servos.{name}.id."
        )
    if "min" not in spec or "max" not in spec:
        raise ConfigError(f"config.json servos.{name} is missing required key 'min' and/or 'max'.")
    lo_raw = _require_int(spec["min"], f"servos.{name}.min")
    hi_raw = _require_int(spec["max"], f"servos.{name}.max")
    if lo_raw > hi_raw:
        raise ConfigError(
            f"config.json servos.{name}: min ({lo_raw}) > max ({hi_raw}) -- inverted "
            f"range. Fix servos.{name}.min/max in config.json."
        )
    elec_lo, elec_hi = ELECTRICAL_LIMITS[servo_id]
    if lo_raw < elec_lo or hi_raw > elec_hi:
        raise ConfigError(
            f"config.json servos.{name} range [{lo_raw}, {hi_raw}] exceeds {name}'s "
            f"hardware-legal electrical range [{elec_lo}, {elec_hi}]. Narrow "
            f"servos.{name}.min/max in config.json to stay inside the electrical range "
            "-- a config file must never be able to widen a joint past its hardware limit."
        )
    guard_lo, guard_hi = absolute_bounds(name)
    lo = max(guard_lo, lo_raw)
    hi = min(guard_hi, hi_raw)
    if "center" in spec:
        center = _require_int(spec["center"], f"servos.{name}.center")
        if not (lo_raw <= center <= hi_raw):
            raise ConfigError(
                f"config.json servos.{name}.center ({center}) is outside its own "
                f"configured band [{lo_raw}, {hi_raw}]. Fix servos.{name}.center."
            )
    else:
        center = (lo + hi) // 2
    calibrated = spec.get("calibrated", False)
    if not isinstance(calibrated, bool):
        raise ConfigError(
            f"config.json servos.{name}.calibrated must be true or false, got {calibrated!r}."
        )
    return JointLimit(servo_id=servo_id, min=lo, max=hi, center=center, calibrated=calibrated)


_servos_raw = _CONFIG.get("servos", {})
_missing_joints = sorted(set(CANONICAL_JOINTS) - set(_servos_raw))
if _missing_joints:
    raise ConfigError(f"config.json servos is missing required joint(s): {_missing_joints}.")

JOINT_LIMITS: Dict[str, JointLimit] = {}
SERVO_IDS: Dict[str, int] = dict(CANONICAL_JOINTS)
ID_TO_NAME: Dict[int, str] = {v: k for k, v in CANONICAL_JOINTS.items()}

for _name, _spec in _servos_raw.items():
    JOINT_LIMITS[_name] = _validate_servo_spec(_name, _spec)

JOINT_NAMES = list(CANONICAL_JOINTS)

_motion = _CONFIG.get("motion", {})
DEFAULT_DURATION_MS = _require_positive_int(_motion.get("default_duration_ms", 1000), "motion.default_duration_ms")
MIN_DURATION_MS = _require_positive_int(_motion.get("min_duration_ms", 400), "motion.min_duration_ms")
MAX_STEP_PER_MOVE = _require_positive_int(_motion.get("max_step_per_move", 400), "motion.max_step_per_move")
SETTLE_MARGIN_MS = _require_positive_int(_motion.get("settle_margin_ms", 100), "motion.settle_margin_ms", allow_zero=True)

_safety = _safety_raw
PROVISIONAL_BAND = _require_positive_int(_safety.get("provisional_band", DEFAULT_PROVISIONAL_BAND), "safety.provisional_band")
HOLD_MS = _require_positive_int(_safety.get("hold_ms", 2000), "safety.hold_ms", allow_zero=True)
IDLE_UNLOAD_MS = _require_positive_int(_safety.get("idle_unload_ms", 5000), "safety.idle_unload_ms")
MAX_CONTINUOUS_TORQUE_MS = _require_positive_int(_safety.get("max_continuous_torque_ms", 8000), "safety.max_continuous_torque_ms")

_vision = _CONFIG.get("vision", {})
VISION_ROI = tuple(_vision.get("roi", [0.15, 0.1, 0.7, 0.8]))
VISION_DIFF_THRESHOLD = int(_vision.get("diff_threshold", 25))
VISION_MIN_CHANGED_PIXELS = int(_vision.get("min_changed_pixels", 150))
VISION_BLUR_KSIZE = int(_vision.get("blur_ksize", 5))

# --- Opt-in brightness-normalised metric (see docs/verification-camera.md) ---
# The default raw-absdiff path (VISION_NORMALIZE="none") is unchanged and is what
# every existing move/grip-ladder/calibration test exercises. A deployment on a
# camera that cannot lock exposure (macOS AVFoundation in a dark scene) can opt
# into "zscore" -- per-frame mean/std normalisation, which is invariant to the
# affine (gain+offset) brightness swings auto-exposure produces, so the changed-
# pixel count reflects real local motion instead of global exposure hunting.
# When "zscore" is used the changed-pixel threshold is VISION_NORMALIZED_THRESHOLD
# (in per-frame standard-deviation units), NOT VISION_DIFF_THRESHOLD (0..255
# grey levels), and VISION_MIN_CHANGED_PIXELS must be re-tuned for the new metric
# via `robohand camera-check` before it is trusted.
VISION_NORMALIZE = str(_vision.get("normalize", "none")).lower()
if VISION_NORMALIZE not in ("none", "mean", "zscore"):
    raise ConfigError(
        f"config.json vision.normalize must be one of 'none', 'mean', 'zscore', "
        f"got {VISION_NORMALIZE!r}."
    )
VISION_NORMALIZED_THRESHOLD = float(_vision.get("normalized_threshold", 0.5))

# --- Camera-health gate (robohand/camera_health.py) --------------------------
# Passive no-motion frames sampled to measure the observer's own noise floor, the
# spacing between them (long enough to span an auto-exposure hunting cycle), the
# safety factor the detection threshold must clear the measured p95 noise by, and
# the factor a known-motion sample must clear the noise floor by (visibility).
# Defaults: 15 frames x 0.4 s ~= 5.6 s -- measured empirically to reliably span an
# auto-exposure hunting burst; a shorter window can land entirely inside a stable
# stretch and miss the hunting (a false PASS). See docs/verification-camera.md.
VISION_HEALTH_FRAMES = int(_vision.get("health_frames", 15))
VISION_HEALTH_INTERVAL_S = float(_vision.get("health_interval_s", 0.4))
VISION_HEALTH_MARGIN = float(_vision.get("health_margin", 1.5))
VISION_HEALTH_SEPARATION = float(_vision.get("health_separation", 3.0))

_gripper = _CONFIG.get("gripper", {})
_grip_abs_lo, _grip_abs_hi = absolute_bounds("GRIP")
GRIP_STEP = _require_positive_int(_gripper.get("step", 40), "gripper.step")
GRIP_LADDER_DURATION_MS = _require_positive_int(_gripper.get("ladder_duration_ms", 250), "gripper.ladder_duration_ms")
GRIP_MIN_SAFE = _require_int(_gripper.get("grip_min_safe", JOINT_LIMITS["GRIP"].min), "gripper.grip_min_safe")
GRIP_MAX_SAFE = _require_int(_gripper.get("grip_max_safe", JOINT_LIMITS["GRIP"].max), "gripper.grip_max_safe")
if GRIP_MIN_SAFE < _grip_abs_lo:
    raise ConfigError(
        f"config.json gripper.grip_min_safe ({GRIP_MIN_SAFE}) is below the absolute "
        f"GRIP floor ({_grip_abs_lo}). This backstop must never be looser than the "
        "hardcoded absolute rail -- fix gripper.grip_min_safe in config.json."
    )
if GRIP_MAX_SAFE > _grip_abs_hi:
    raise ConfigError(
        f"config.json gripper.grip_max_safe ({GRIP_MAX_SAFE}) is above the absolute "
        f"GRIP ceiling ({_grip_abs_hi}). This backstop must never be looser than the "
        "hardcoded absolute rail -- fix gripper.grip_max_safe in config.json."
    )
if GRIP_MIN_SAFE > GRIP_MAX_SAFE:
    raise ConfigError(
        f"config.json gripper.grip_min_safe ({GRIP_MIN_SAFE}) > gripper.grip_max_safe "
        f"({GRIP_MAX_SAFE}) -- inverted range. Fix config.json."
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
