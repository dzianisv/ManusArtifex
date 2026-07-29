"""Vision-guided auto-calibration: find each joint's real mechanical limits.

Why this exists
---------------
Every joint in ``data/config.json`` ships ``calibrated: false`` and is therefore
restricted to a narrow provisional band (``config.PROVISIONAL_BAND`` either side of
its configured center), because the true mechanical end stops are unknown and this
controller has no position feedback (``protocol.NoResponse``). The documented way to
find them is a human moving each joint by hand with torque off. This module adds a
*semi-automated* alternative: command a small increment, ask a ``MotionObserver``
whether the arm visibly moved, and treat repeated no-motion as a candidate stop.

Where it is allowed to look
---------------------------
Out to ``config.absolute_bounds(name)`` -- 700..2300 for most joints, 1500..2300
for GRIP -- and no further. NOT the provisional/configured band: on a genuinely
uncalibrated arm the true stops are, by definition, somewhere in the wide unknown
territory the provisional band was built to stay clear of, so a search confined to
that band cannot find them and would only ever rediscover its own ceiling. See
``search_bounds``. Reaching that range needs ``SafetyLayer``'s audited
``probe_absolute_bounds`` capability; ``SafetyLayer``'s unconditional
absolute-rail backstop still applies to every single packet.

What makes this different from the 2026-07-28 incident
------------------------------------------------------
The incident was a single absolute command into a mechanical stop with torque left
on, held until a human cut power. Every step here is a *torque burst*: one small
increment, commanded for ``step_duration_ms`` (default ``config.MIN_DURATION_MS``,
400 ms -- see ``CalibrationSettings``), then an immediate unload through
``SafetyLayer``, and only then the observation. Worst case, a single step puts
torque into a stop for one burst, not indefinitely. The whole run additionally
carries a cumulative torque budget, a wall-clock cap, and a step cap; blowing any
of them aborts, and an aborted run writes nothing.

What this does NOT solve
------------------------
Vision cannot distinguish "the jaws closed on an object" from "the servo reached
its rotational limit" -- both are simply "no motion". Calibrate GRIP with the jaws
empty and unobstructed, or the discovered band will be the object's width, not the
gripper's range. See docs/calibration.md for the full, blunt failure-mode list.

Everything here goes through ``arm.Arm`` -> ``safety.SafetyLayer.move`` -> the
motion-token choke point. This module never touches a transport directly.
"""

from __future__ import annotations

import dataclasses
import datetime as _dt
import json
import logging
import math
import os
import time
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple

from . import config
from . import camera_health
from .camera_health import CameraHealthReport, CameraUnusableError
from .observer import MotionObserver, ObserverError, observer_for_arm
from .safety import SafetyViolation

logger = logging.getLogger("robohand.calibrate")

SCHEMA = "robohand-calibration-v2"

#: How the search ended on one side of one joint. These are deliberately distinct,
#: and the second one is deliberately NOT called an "electrical" limit:
#:
#: ``config.absolute_bounds()`` is ``GLOBAL_GUARD_MIN/MAX`` (700/2300) -- a
#: conservative *policy* ring chosen by this package -- intersected with the
#: servo's own ``ELECTRICAL_LIMITS`` (500..2500 for most joints, 1500..2500 for
#: GRIP), which is wider on every end except one. So reaching that bound without
#: ever observing a stop means "we ran out of range we are willing to command",
#: NOT "we hit the servo's electrical or mechanical rating". Those are different
#: claims and this finding must never be read as the latter.
#:
#: The one exception is GRIP's floor, where the policy number and the confirmed
#: true stop coincide at exactly 1500 (see docs/research-hardware.md). The label
#: stays generic anyway: the algorithm has no independent way to know which case
#: it is in, and a finding that quietly means two different things is worse than
#: one that means less.
FINDING_MECHANICAL_STOP = "bounded_by_mechanical_stop"
FINDING_POLICY_LIMIT = "bounded_by_policy_limit"

#: Hard ceiling on a single step's torque burst. The design requirement is "well
#: under half a second of torque per step"; anything at or above this is refused
#: rather than quietly accepted, because a longer burst is a longer stall.
MAX_STEP_DURATION_MS = 500

#: Hard ceiling on the step increment. This routine is only safe because each step
#: is small enough that overshooting into a stop costs one short burst at low
#: travel. A big step is a small slam.
MAX_STEP = 100


class CalibrationError(RuntimeError):
    """Base class for every auto-calibration failure."""


class CalibrationRefused(CalibrationError):
    """Raised *before* any motion when the requested calibration is not safe to run.

    Bad settings, a joint whose searchable band is too narrow for the requested
    margin, no usable observer, a calibration file that claims a range wider than
    the joint's absolute bound. Fail closed: refuse, do not adjust the request into
    something legal and proceed as if that is what was asked for.
    """


class CalibrationAborted(CalibrationError):
    """Raised when a run that had already started must stop.

    By the time this is raised every joint touched by the run has been unloaded and
    **no result file has been written**. A partial calibration is worse than none:
    it looks like a measurement and is not one.
    """


class JointUnresponsive(CalibrationError):
    """Raised by the per-joint liveness check when one joint's actuator is dead.

    Subclasses ``CalibrationError`` and NOT ``ObserverError`` on purpose: this is a
    fact about a single joint's *hardware*, not about the observer's ability to see.
    A dead joint (disconnected / failed-open, e.g. the real SHOULDER_LIFT / servo-5
    fault: commanded across its band it moved less than the camera noise floor while
    the pack stayed flat -- no current drawn -- see docs/hardware-fault-servo5.md)
    produces the exact same "commanded motion, saw nothing" signal a mechanical stop
    does, so without this stage the limit search would converge on a degenerate band
    and mark a dead joint calibrated: confidently wrong.

    Because it does not invalidate the *measurement process* -- the camera is fine,
    the other joints are fine -- ``calibrate`` treats it differently from every
    other failure: it isolates the one dead joint (records it, skips it, and keeps
    going) instead of aborting the whole run. Every other exception, including the
    ``ObserverError`` family and ``CalibrationAborted``, still aborts everything,
    because those mean the process itself can no longer be trusted for ANY joint.
    """


@dataclasses.dataclass(frozen=True)
class CalibrationSettings:
    """Tunables for one calibration run. Validated in ``__post_init__``; every
    out-of-policy value is refused rather than clamped.
    """

    #: Raw units per probe step. 30 sits between the grip ladder's 40
    #: (``config.GRIP_STEP``) and something finer: it resolves a limit to ~1 step
    #: while keeping the longest possible sweep -- centre (1500) to a rail (700 or
    #: 2300) for a non-GRIP joint, i.e. 800 units -- to 27 steps per direction.
    step: int = 30

    #: Torque per step. ``SafetyLayer.move`` floors every duration at
    #: ``config.MIN_DURATION_MS`` (400 ms in the shipped config), so asking for less
    #: would be silently bumped -- we default to exactly that floor instead, and
    #: refuse anything below it so nobody thinks they configured a 50 ms burst.
    #: Still under ``MAX_STEP_DURATION_MS``.
    step_duration_ms: int = config.MIN_DURATION_MS

    #: Back-off from the last position that verifiably moved. 60 = two steps, so a
    #: recorded limit is two increments clear of anything we ever saw move.
    margin: int = 60

    #: Consecutive no-motion observations required before believing a stop. One miss
    #: is indistinguishable from a shadow, a dropped frame, or a joint leaving the
    #: ROI, so a value below 2 is refused.
    confirmations: int = 2

    #: Size of the reliability burst run before any stop is recorded: an oscillation
    #: between two positions the joint has already been watched to reach, so every
    #: move in the burst is known-real motion and every miss is provably the
    #: observer's error. Six moves make an observer that misses half of all real
    #: motion fail ~89% of the time, while an observer missing ~1 in 7 passes.
    verification_moves: int = 6

    #: Misses tolerated within that burst. One absorbs a single unlucky frame; more
    #: than one means the observer is not good enough to infer a stop from silence.
    max_verification_misses: int = 1

    #: Defensive backstop; the search normally terminates at the joint's own bound.
    max_steps_per_direction: int = 64

    #: Cumulative commanded torque time over the whole run, all joints.
    #:
    #: Derived, not guessed. Per joint and direction the search commands at most:
    #:   * ``ceil(rail_span_from_centre / step)`` probe packets. The widest rail
    #:     half-span is ``1500 -> 700`` = 800 units, so ``ceil(800/30)`` = 27.
    #:   * ``verification_moves`` (6) reliability-verification packets.
    #:   * 1 retreat/settle packet.
    #: = 34 packets per direction, 68 for both. Re-centring between and after the
    #: directions crosses up to the full 800-unit half-span, which
    #: ``SafetyLayer`` ramps into ``ceil(800/300)`` = 3 packets, twice = 6.
    #: => 74 packets/joint x 6 joints x 400ms = 177,600ms. Rounded to 180,000 to
    #: absorb an odd extra settle packet. Measured worst case (a real stop placed
    #: as deep inside the rail as it can be on every joint and direction, so every
    #: probe sweep AND every verification burst runs in full): 151,600ms. A
    #: realistic ``--all`` against ``sim.DEFAULT_MECHANICAL_LIMITS``: 137,600ms --
    #: which already blew the pre-fix 120,000ms default, because before the
    #: ``search_bounds`` fix the search could only ever crawl over the 300-unit
    #: provisional band and never had to sweep out to the rail.
    #:
    #: **This default is a sentinel.** Leave it as ``None`` and it is DERIVED from
    #: ``step``, ``step_duration_ms`` and ``verification_moves`` in ``__post_init__``
    #: (see ``derive_torque_budget``). The arithmetic above is the *default-settings*
    #: instance of that formula, not a constant that survives changing the knobs:
    #: halving ``step`` doubles the probe packets, and a fixed 180,000 would then
    #: abort a perfectly legitimate run. Pass an explicit int to override.
    max_torque_ms: Optional[int] = None

    #: Cumulative simulated stall time. Only enforceable against ``SimTransport``
    #: (real hardware cannot report stalls -- that is the entire problem this
    #: package exists to work around), so on hardware this is unenforced by
    #: construction, not merely unset.
    #:
    #: Derived: stall accrues *only* while the commanded target is beyond the true
    #: mechanical stop, and once a step overshoots the stop the joint is pinned, so
    #: every later step in that direction is a no-motion miss and the direction
    #: terminates after ``confirmations`` of them. Worst case per direction is
    #: therefore ``1`` overshoot-that-still-moved ``+ confirmations`` (2) = 3
    #: stalling packets -- a number set by ``confirmations``, NOT by how far the
    #: search is allowed to travel. Widening the search to the absolute rail
    #: multiplies the *number of steps* but not the *number of stalling steps*.
    #: => 6 joints x 2 directions x 3 packets x 400ms = 14,400ms.
    #: Measured worst case: 14,000ms. Realistic ``--all``: 10,400ms.
    #:
    #: **Sentinel, as above.** Left ``None`` it is derived from ``confirmations``
    #: and ``step_duration_ms`` (see ``derive_stall_budget``). The 14,400 figure is
    #: the default-settings instance: raising ``confirmations`` to 3 would make the
    #: true worst case 19,200ms and a fixed 14,400 cap would abort every full run.
    max_stall_ms: Optional[int] = None

    #: Wall-clock cap for the whole run. Each packet costs
    #: ``step_duration_ms + config.SETTLE_MARGIN_MS`` (500ms) plus observation
    #: time, so the 444-packet ceiling above is ~222s of commanding. Measured
    #: worst case end to end under ``--sim``: 191.9s; realistic ``--all``: 174.2s.
    #: 900s leaves room for a real camera observer, which is far slower per
    #: observation than ``SimMotionObserver``, without being so loose that a hung
    #: run sits on the arm indefinitely.
    #:
    #: **Sentinel, as above** -- derived from the packet count and per-packet cost,
    #: with generous headroom for a real camera observer. Pass a number to override.
    max_wall_clock_s: Optional[float] = None

    #: A no-motion report that a later observation contradicts (the joint moved
    #: again afterwards) is an *ambiguous* observation. A handful means a twitchy
    #: camera; a lot means the observer is not trustworthy enough to calibrate with.
    #:
    #: This is a run-wide count, and it had to be re-derived when the search was
    #: widened to the absolute rail: a single joint now makes roughly twice as many
    #: observations as it did over the old 300-unit provisional band (~64 vs ~28),
    #: so leaving the cap at 8 would have silently halved the observer error rate
    #: the design intended to tolerate. 16 restores it. Measured: a camera that
    #: drops one genuine motion report in seven produces 6 ambiguous observations
    #: over one joint.
    #:
    #: Note this is a backstop, not the primary defence. The observer-reliability
    #: burst (``verification_moves`` / ``max_verification_misses``) is far stricter
    #: and fires first: a 1-in-5 dropper is rejected by that gate long before this
    #: counter matters.
    max_ambiguous_observations: int = 16

    #: Run the per-joint liveness excursion (stage 2) before the limit search.
    #:
    #: **Default OFF, deliberately, and this is the honest part.** The check itself is
    #: implemented and correct (see ``_liveness_targets`` / ``_verify_joint_liveness``),
    #: but it cannot be shown *safe* to run by default without hardware measurements
    #: this project does not currently have. The three constraints do not intersect:
    #:
    #:   * **Visibility** needs delta >= ~41 units, for the weakest *measured* joint
    #:     (GRIP, 42.9 px/unit) at ``VISION_HEALTH_SEPARATION`` 3.0 over a 580 px noise
    #:     floor -- with zero margin left over for WRIST_ROLL / WRIST_FLEX / ELBOW,
    #:     which have never been measured at all.
    #:   * **Battery corroboration** -- the strongest dead-servo signal this project
    #:     has, and how servo 5 was actually diagnosed -- needs >= 125 units of travel
    #:     to clear the 0.05 V detection epsilon.
    #:   * **Stall-freedom** caps delta at 45 units on the narrowest joint the suite
    #:     models (GRIP with stops at 1855/2045: only 45 units of headroom below its
    #:     1900 centre).
    #:
    #: 125 > 45, so no fixed delta satisfies both battery corroboration and
    #: stall-freedom. Even ignoring the battery, the vision-only window is [41, 45] --
    #: four units wide, with no allowance for the three unmeasured joints. Picking a
    #: number inside that window would be fitting a test fixture, not engineering, and
    #: would silently mis-declare weak joints dead: the same class of bug that produced
    #: the original bogus measurements.
    #:
    #: The root cause is structural, not a bad constant: with no position feedback the
    #: centre is the *only* safe point, and ``centre +/- delta`` is not safe by
    #: construction for ANY delta > 0, because a mechanical stop may sit anywhere.
    #:
    #: TODO(hardware): measure px-per-unit for WRIST_ROLL, WRIST_FLEX and ELBOW, and
    #: the true mechanical range of every joint, then re-derive delta and flip this
    #: default. Blocked: SHOULDER_LIFT (servo 5) is dead and the arm is powered down.
    #: Until then a dead joint is caught by the search's own observer-reliability
    #: burst, which aborts the run rather than silently calibrating it.
    liveness_check: bool = False

    def __post_init__(self) -> None:
        def _int(name: str, value: Any, lo: int, hi: int) -> None:
            if isinstance(value, bool) or not isinstance(value, int):
                raise CalibrationRefused(
                    f"{name} must be an integer, got {value!r} ({type(value).__name__})."
                )
            if not (lo <= value <= hi):
                raise CalibrationRefused(f"{name} must be in [{lo}, {hi}], got {value}.")

        _int("step", self.step, 1, MAX_STEP)
        _int("margin", self.margin, 0, 400)
        _int("confirmations", self.confirmations, 2, 20)
        _int("verification_moves", self.verification_moves, 2, 50)
        _int("max_verification_misses", self.max_verification_misses, 0, 10)
        if self.max_verification_misses >= self.verification_moves:
            raise CalibrationRefused(
                f"max_verification_misses ({self.max_verification_misses}) must be smaller "
                f"than verification_moves ({self.verification_moves}) -- otherwise the "
                "observer-reliability check can never fail and is not a check."
            )
        _int("max_steps_per_direction", self.max_steps_per_direction, 1, 1000)
        # Frozen dataclass: object.__setattr__ is the sanctioned way to fill in a
        # derived default, and it keeps the instance immutable to everyone else.
        if self.max_torque_ms is None:
            object.__setattr__(self, "max_torque_ms", derive_torque_budget(
                self.step, self.step_duration_ms, self.verification_moves))
        if self.max_stall_ms is None:
            object.__setattr__(self, "max_stall_ms", derive_stall_budget(
                self.step_duration_ms, self.confirmations))
        if self.max_wall_clock_s is None:
            object.__setattr__(self, "max_wall_clock_s", derive_wall_clock_budget(
                self.step, self.step_duration_ms, self.verification_moves))
        _int("max_torque_ms", self.max_torque_ms, 1, 3_600_000)
        _int("max_stall_ms", self.max_stall_ms, 0, 3_600_000)
        _int("max_ambiguous_observations", self.max_ambiguous_observations, 0, 1000)
        if not isinstance(self.liveness_check, bool):
            raise CalibrationRefused(
                f"liveness_check must be a bool, got {self.liveness_check!r} "
                f"({type(self.liveness_check).__name__})."
            )
        config.validate_duration_ms(self.step_duration_ms, allow_none=False)
        if self.step_duration_ms >= MAX_STEP_DURATION_MS:
            raise CalibrationRefused(
                f"step_duration_ms {self.step_duration_ms} is at or above the "
                f"{MAX_STEP_DURATION_MS}ms torque-burst ceiling. Each calibration step "
                "must apply torque only briefly before unloading; a longer burst is a "
                "longer stall if the step happens to be into a mechanical stop."
            )
        if self.step_duration_ms < config.MIN_DURATION_MS:
            raise CalibrationRefused(
                f"step_duration_ms {self.step_duration_ms} is below "
                f"config.MIN_DURATION_MS ({config.MIN_DURATION_MS}). SafetyLayer.move "
                "would silently raise it to that floor, so the configured value would "
                "be a lie. Ask for at least the floor."
            )
        if not isinstance(self.max_wall_clock_s, (int, float)) or self.max_wall_clock_s <= 0:
            raise CalibrationRefused(
                f"max_wall_clock_s must be a positive number, got {self.max_wall_clock_s!r}."
            )

    def as_dict(self) -> Dict[str, Any]:
        return dataclasses.asdict(self)


@dataclasses.dataclass
class DirectionResult:
    """Outcome of searching one joint in one direction."""

    direction: str  # "min" or "max"
    limit: int
    finding: str  # FINDING_MECHANICAL_STOP | FINDING_POLICY_LIMIT
    margin: int
    steps: int
    ambiguous_observations: int
    last_confirmed_moving: int
    search_bound: int

    def as_dict(self) -> Dict[str, Any]:
        return dataclasses.asdict(self)


@dataclasses.dataclass
class JointCalibration:
    joint: str
    servo_id: int
    min: int
    max: int
    margin: int
    min_finding: str
    max_finding: str
    steps_min: int
    steps_max: int
    ambiguous_observations: int
    torque_ms: int
    stall_ms: Optional[float]
    search_bounds: Tuple[int, int]

    def as_dict(self) -> Dict[str, Any]:
        payload = dataclasses.asdict(self)
        payload["search_bounds"] = list(self.search_bounds)
        return payload


@dataclasses.dataclass
class CalibrationRun:
    joints: Dict[str, JointCalibration]
    settings: CalibrationSettings
    observer: str
    transport: str
    elapsed_s: float
    torque_ms: int
    stall_ms: Optional[float]
    output_path: Optional[Path] = None
    #: Joints that failed the per-joint liveness stage (name -> diagnostic message).
    #: A joint here is a proven-dead actuator: it is deliberately kept OUT of
    #: ``joints`` so the written file never records a band for it and its config.json
    #: ``calibrated`` flag stays false. Present so a run can report "5 calibrated, 1
    #: dead" instead of silently dropping the dead one.
    failed: Dict[str, str] = dataclasses.field(default_factory=dict)

    def as_dict(self) -> Dict[str, Any]:
        return {
            "schema": SCHEMA,
            "generated_utc": _dt.datetime.now(_dt.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
            "transport": self.transport,
            "observer": self.observer,
            "elapsed_s": round(self.elapsed_s, 3),
            "torque_ms": self.torque_ms,
            "stall_ms": self.stall_ms,
            "settings": self.settings.as_dict(),
            "joints": {name: jc.as_dict() for name, jc in self.joints.items()},
            "failed_joints": dict(self.failed),
        }


class _Budget:
    """Tracks the run-wide caps and raises ``CalibrationAborted`` when one blows."""

    def __init__(self, settings: CalibrationSettings, transport) -> None:
        self._settings = settings
        self._transport = transport
        self.started = time.monotonic()
        self.torque_ms = 0
        self.ambiguous = 0
        self._stall_at_start = self._read_stall()

    def _read_stall(self) -> Optional[float]:
        reader = getattr(self._transport, "stall_ms_total", None)
        if reader is None:
            return None
        try:
            return float(reader())
        except Exception:  # pragma: no cover - a transport that lies about stalls
            return None

    @property
    def stall_ms(self) -> Optional[float]:
        now = self._read_stall()
        if now is None or self._stall_at_start is None:
            return None
        return now - self._stall_at_start

    @property
    def elapsed_s(self) -> float:
        return time.monotonic() - self.started

    def charge_torque(self, duration_ms: int) -> None:
        self.torque_ms += int(duration_ms)
        self.check()

    def note_ambiguous(self, count: int = 1) -> None:
        self.ambiguous += count
        self.check()

    def check(self) -> None:
        if self.torque_ms > self._settings.max_torque_ms:
            raise CalibrationAborted(
                f"cumulative commanded torque {self.torque_ms}ms exceeded the run budget "
                f"of {self._settings.max_torque_ms}ms -- aborting and unloading."
            )
        stall = self.stall_ms
        if stall is not None and stall > self._settings.max_stall_ms:
            raise CalibrationAborted(
                f"accumulated stall time {stall:.0f}ms exceeded the run budget of "
                f"{self._settings.max_stall_ms}ms -- the arm is spending too long pushing "
                "against something. Aborting and unloading."
            )
        if self.elapsed_s > self._settings.max_wall_clock_s:
            raise CalibrationAborted(
                f"wall-clock budget of {self._settings.max_wall_clock_s}s exceeded "
                f"({self.elapsed_s:.1f}s) -- aborting and unloading."
            )
        if self.ambiguous > self._settings.max_ambiguous_observations:
            raise CalibrationAborted(
                f"{self.ambiguous} ambiguous observations (a no-motion report later "
                f"contradicted by real motion) exceeded the cap of "
                f"{self._settings.max_ambiguous_observations}. The observer is not "
                "reliable enough to trust a 'no motion means mechanical stop' inference "
                "-- aborting without writing a result."
            )


def derive_stall_budget(step_duration_ms: int, confirmations: int, joints: int = 6) -> int:
    """Worst-case cumulative stall time for a run, derived from the settings.

    Stall accrues only while the commanded target is beyond the true mechanical
    stop. Once a step overshoots the stop the joint is pinned, so every subsequent
    step in that direction is a no-motion miss and the direction terminates after
    ``confirmations`` of them. The worst case per direction is therefore
    ``1`` overshoot-that-still-moved ``+ confirmations`` pinned packets -- a count
    set by ``confirmations``, NOT by how far the search is allowed to travel.
    Widening the search multiplies the number of steps but not the number of
    *stalling* steps.

    This must be derived rather than hardcoded: at the shipped defaults it is
    ``6 x 2 x 3 x 400 = 14,400ms``, but ``confirmations=3`` makes the true worst
    case 19,200ms, and a fixed 14,400 cap would then abort every legitimate run.
    """
    return joints * 2 * (1 + confirmations) * step_duration_ms


def derive_packet_budget(step: int, verification_moves: int, joints: int = 6) -> int:
    """Worst-case number of commanded packets for a whole run.

    Per joint and direction: ``ceil(widest_half_span / step)`` probe packets,
    ``verification_moves`` reliability packets, and 1 retreat/settle packet.
    Re-centring between and after the two directions crosses up to the full
    half-span, which ``SafetyLayer`` ramps into ``ceil(half_span / MAX_STEP_PER_MOVE)``
    packets, twice.
    """
    half_span = max(
        max(center - lo, hi - center)
        for name in config.JOINT_NAMES
        for (lo, hi) in [search_bounds(name)]
        for center in [(lo + hi) // 2]
    )
    probes = -(-half_span // step)
    per_direction = probes + verification_moves + 1
    recentre = 2 * max(1, -(-half_span // config.MAX_STEP_PER_MOVE))
    return joints * (2 * per_direction + recentre)


def derive_torque_budget(step: int, step_duration_ms: int, verification_moves: int, joints: int = 6) -> int:
    """Worst-case cumulative commanded-torque time, derived from the settings.

    Hardcoding this is a trap: halving ``step`` doubles the probe packets, so a
    constant sized for ``step=30`` aborts a legitimate ``step=15`` run. The 1.05
    factor absorbs an odd extra settle packet without being loose enough to stop
    functioning as a safeguard.
    """
    return int(derive_packet_budget(step, verification_moves, joints) * step_duration_ms * 1.05)


def derive_wall_clock_budget(step: int, step_duration_ms: int, verification_moves: int, joints: int = 6) -> float:
    """Worst-case wall-clock seconds, derived from the settings.

    Each packet costs ``step_duration_ms + config.SETTLE_MARGIN_MS`` plus
    observation time. The 4x factor is headroom for a real camera observer, which
    is far slower per observation than ``SimMotionObserver`` -- generous, but still
    bounded, so a hung run cannot sit on the arm indefinitely.
    """
    packets = derive_packet_budget(step, verification_moves, joints)
    return round(packets * (step_duration_ms + config.SETTLE_MARGIN_MS) / 1000.0 * 4.0, 1)


def search_bounds(name: str) -> Tuple[int, int]:
    """The widest range this routine is ever allowed to probe for ``name``.

    ``config.absolute_bounds(name)`` and nothing narrower: the hardcoded
    ``GLOBAL_GUARD_MIN/MAX`` ring intersected with that servo's own
    ``ELECTRICAL_LIMITS``. Nothing in ``config.json`` can widen it, which is
    exactly why it is the right ceiling for a supervised, torque-burst-limited,
    vision-confirmed search.

    It is deliberately NOT ``config.effective_bounds(..., override=True)``. That
    function is always intersected with ``config.JOINT_LIMITS[name].min/max`` --
    the configured band -- regardless of the override flag, and for every
    uncalibrated joint in the shipped config that band is numerically identical to
    the provisional band (both ``center +/- 300``). Searching inside it can never
    find a real stop on an uncalibrated arm, because the provisional band exists
    *precisely because* the true limits are assumed to lie outside it. A search
    that can only ever hit its own ceiling produces a placeholder laundered into a
    file that looks like a measurement.

    Reaching these bounds requires ``SafetyLayer.move(probe_absolute_bounds=True)``
    (see ``_torque_burst``), which is audited and still sits behind
    ``SafetyLayer``'s unconditional absolute-rail backstop.
    """
    key = config.validate_joint_name(name)
    lo, hi = config.absolute_bounds(key)
    # Belt and braces: a no-op by construction today, and an assertion that stays
    # true if absolute_bounds ever changes shape.
    abs_lo, abs_hi = config.absolute_bounds(key)
    lo, hi = max(lo, abs_lo), min(hi, abs_hi)
    assert (abs_lo, abs_hi) == (lo, hi), (
        f"{key}: search bounds [{lo}, {hi}] must never exceed the absolute rail "
        f"[{abs_lo}, {abs_hi}]"
    )
    return lo, hi


def _start_position(name: str, lo: int, hi: int) -> int:
    center = config.JOINT_LIMITS[name].center
    return max(lo, min(hi, center))


def _torque_burst(
    arm, name: str, target: int, settings: CalibrationSettings, budget: _Budget, *, probe: bool = True
) -> None:
    """One bounded step: command it, wait for it, then unload before observing.

    ``vision_guard=None`` disables ``SafetyLayer``'s own automatic hard-fail vision
    check for this call, for the same reason the grip ladder does it: here "no
    motion" is the *expected* successful outcome of the last step in a direction,
    not an anomaly to raise ``StallDetected`` for. This routine does its own,
    graceful no-motion handling immediately below the call site.

    ``probe`` (default ``True``) selects the *limit-search* posture: the widened
    ``search_bounds`` are only reachable with ``probe_absolute_bounds=True`` (and
    the ``allow_uncalibrated_override`` + reason that gates it), because otherwise
    ``SafetyLayer`` would refuse anything outside ``config.effective_bounds``. It is
    still audited, still requires the override + reason, and is still subject to
    ``SafetyLayer``'s unconditional absolute-rail backstop.

    ``probe=False`` is the *liveness* posture: the target is inside the joint's
    provisional band (== its uncalibrated ``effective_bounds``), so it needs
    NEITHER capability -- no ``probe_absolute_bounds``, no override. The liveness
    check must not borrow elevated privilege to prove a joint moves; a move it can
    only make with the rails unlocked is not the move a normal command would make.

    The budget is charged the *actual* commanded torque, not one nominal burst.
    A probe step is 30 units and is always a single packet, but the re-centering
    move between the two search directions can span the full rail (800 units) and
    ``SafetyLayer`` ramps anything over ``max_step_per_move`` into several packets,
    each of at least ``config.MIN_DURATION_MS``. Charging one burst for those would
    under-count by up to 2x and make ``max_torque_ms`` a budget for something other
    than what the servos were actually told to do.
    """
    start = arm._shadow.get(name)
    delta = 0 if start is None else abs(int(target) - int(start))
    packets, packet_ms = arm._safety.plan_ramp(delta, settings.step_duration_ms)
    arm.set_position(
        name,
        target,
        duration_ms=settings.step_duration_ms,
        wait=True,
        vision_guard=None,
        allow_uncalibrated_override=probe,
        probe_absolute_bounds=probe,
        override_reason=(
            f"auto-calibration probe for {name} (bounded {settings.step_duration_ms}ms burst)"
            if probe else None
        ),
    )
    # Unload through SafetyLayer *before* observing -- bounded exposure per step is
    # the entire safety argument for this routine. Deliberately not arm.relax(),
    # which additionally erases the shadow position estimate that SafetyLayer needs
    # to decide ramping for the next step.
    arm._safety.unload([name])
    budget.charge_torque(packets * packet_ms)


def _observe(observer: MotionObserver, before, after, context: str) -> bool:
    try:
        return bool(observer.observe(before, after))
    except ObserverError:
        raise
    except Exception as exc:  # an observer that blows up is an unusable observer
        raise ObserverError(f"observer failed while judging {context}: {exc}") from exc


def _capture(observer: MotionObserver, context: str):
    try:
        return observer.capture()
    except ObserverError:
        raise
    except Exception as exc:
        raise ObserverError(f"observer failed to capture ({context}): {exc}") from exc


#: Battery-drop threshold, in volts, that separates "the servo drew current" from
#: "the pack stayed flat" during the liveness legs. The real servo-5 fault showed a
#: pack that "stayed flat" across a full-band command (no current = disconnected /
#: failed-open); a live servo turning a load sags the pack measurably. 0.05 V sits
#: at the low end of the task's 0.05-0.10 V window: small enough to catch the light
#: load of a short liveness excursion, large enough to ignore reading jitter. It is
#: only ever used to *soften* a dead verdict (see ``_verify_joint_liveness``), never
#: to manufacture one, so erring small is the safe direction.
_LIVENESS_BATTERY_EPSILON_V = 0.05


def _read_battery(arm) -> Optional[float]:
    """Read pack voltage, or ``None`` if the read fails. Never raises.

    A battery read is a diagnostic nicety, not a precondition: a controller that
    cannot answer ``get_battery_voltage`` must not crash the liveness check, it must
    just leave the voltage out of the diagnosis.
    """
    try:
        return float(arm.battery_voltage())
    except Exception:  # a transport that cannot report voltage is not a liveness failure
        return None


#: Half-amplitude of the liveness excursion, in raw units, measured from the joint's
#: safe centre. **Reasoned from hardware data, not measured at this delta** -- see the
#: honest-limits note in ``CalibrationSettings.liveness_check``.
#:
#: Derivation from the 2026-07-28 hardware session (docs/verification-camera.md):
#:   * camera noise floor, warmed up and flushed: 450-580 changed px (use 580, worst)
#:   * SHOULDER_PAN across its ~600-unit band: 78,116 px -> 130.2 px per unit
#:   * GRIP across its ~600-unit band:          25,746 px ->  42.9 px per unit
#:   * dead SHOULDER_LIFT across its full band: 230-330 px, i.e. BELOW the floor
#: GRIP is the weakest *measured* joint. ``config.VISION_HEALTH_SEPARATION`` (3.0)
#: demands 580 * 3 = 1,740 px, so GRIP alone needs only ~41 units. 150 units gives
#: GRIP 6,436 px = 11.1x the floor -- 3.7x more margin than the separation demands.
#: Stated as a *coverage* bound: 150 units still clears 3.0x for any joint down to
#: 11.6 px/unit, i.e. one 3.7x weaker than the weakest joint ever measured. That
#: margin is what absorbs WRIST_ROLL / WRIST_FLEX / ELBOW, for which NO px-per-unit
#: figure exists -- an unmeasured allowance, not a measurement.
#: It also clears the simulator's battery-sag epsilon: 150 * 0.0004 V = 0.060 V > 0.05 V.
LIVENESS_DELTA = 150


def _liveness_center(name: str) -> int:
    """The one position for this joint that is safe to command *without* knowing where
    its mechanical stops are: the midpoint of its ABSOLUTE (electrical) range.

    With no position feedback the only position we can reason about is the one we
    most recently commanded, and the only one that is safe by construction is the
    electrical midpoint -- it cannot be past a mechanical stop on either side,
    because the stops necessarily lie inside the electrical range.

    Note this is deliberately ``absolute_bounds``, NOT a flat 1500. For five joints
    those agree (700..2300 -> 1500), but **GRIP does not**: its electrical floor is
    1500, so its midpoint is 1900, and 1500 is GRIP's *open mechanical stop* rather
    than a safe centre. Commanding GRIP to a hardcoded 1500 to "home" it would drive
    it onto that stop -- the same class of mistake as the GRIP=700 command that
    destroyed the gripper on 2026-07-28.
    """
    lo, hi = config.absolute_bounds(name)
    return (lo + hi) // 2


def _liveness_targets(name: str, delta: int = LIVENESS_DELTA) -> Tuple[int, int, int]:
    """``(centre, centre - d, centre + d)`` -- a symmetric excursion about the safe centre.

    This deliberately inverts the original derivation, which took targets from the
    *provisional band*. That band is by definition WIDER than the true mechanical
    range (its whole reason to exist is that the real limits are unknown and assumed
    to lie outside it), so a target drawn from it can never be known-reachable: the
    old code sent GRIP to 1800/2100 against true stops at 1855/2045 and stalled the
    servo on both legs. Deriving from the centre instead means the target is at most
    ``delta`` from a point that is safe by construction.

    ``d`` is additionally clamped into the provisional band so the excursion needs no
    elevated privilege (``probe=False``: no ``probe_absolute_bounds``, no uncalibrated
    override). A move that can only be made with the rails unlocked is not the move a
    normal command would make, so it would not prove what liveness needs to prove.
    """
    centre = _liveness_center(name)
    lo, hi = config.provisional_bounds(name)
    d = max(1, min(int(delta), centre - lo, hi - centre))
    return centre, centre - d, centre + d


def _verify_joint_liveness(
    arm,
    name: str,
    settings: CalibrationSettings,
    observer: MotionObserver,
    budget: _Budget,
    health: CameraHealthReport,
) -> None:
    """Stage 2 of the pipeline: prove this joint's actuator is actually alive.

    Runs AFTER the camera-health gate (stage 1: is the sensor trustworthy?) and
    BEFORE the limit search (stage 3: where are the stops?). Each stage is
    meaningless unless the previous one passed: "no motion" is only evidence of a
    mechanical stop once we know the camera can see and the servo can move. A
    completely dead joint (disconnected / failed-open -- the servo-5 fault) emits the
    exact "commanded, saw nothing" signal a stop does, so without this the search
    would converge on a degenerate band and mark it calibrated.

    Two legs, opposite directions, symmetric about the joint's safe centre (see
    ``_liveness_targets``) and clamped into the provisional band, so ``probe=False``:
    no elevated privilege. The joint is first homed to that centre, because an
    excursion is only bounded if we know where it started from.
    Each leg's changed-pixel score must clear the *already-measured* noise floor in
    ``health`` via the shared ``camera_health.motion_clears_floor`` primitive (no
    per-joint re-sampling). If either leg fails to be seen, the joint is *provisionally*
    dead -- but a blind camera on a live joint produces the same silence, so before
    raising we consult an independent channel: the battery. A joint that really
    moved drew current and sagged the pack; a joint that failed *open* drew nothing
    and left it flat. Only "not seen AND no current drawn (or voltage unreadable)"
    is reported as a dead actuator; "not seen BUT current WAS drawn" means the servo
    is electrically alive and the camera simply missed it, which is the limit
    search's observer-reliability burst's job to adjudicate (it aborts the whole run
    as ``CalibrationAborted`` if the observer truly cannot see this joint) -- not a
    hardware verdict liveness can support. The joint is unloaded before any raise so
    it is never left under torque.
    """
    centre, down, up = _liveness_targets(name)
    legs = [("-", down), ("+", up)]

    v_before = _read_battery(arm)
    results: List[Tuple[str, int, float, float, bool]] = []
    try:
        # Home first: the excursion is only bounded relative to a known start, and
        # the centre is the sole position that is safe without knowing the stops.
        _torque_burst(arm, name, centre, settings, budget, probe=False)
        for tag, target in legs:
            before = _capture(observer, f"{name} liveness leg {tag} before")
            _torque_burst(arm, name, target, settings, budget, probe=False)
            after = _capture(observer, f"{name} liveness leg {tag} after")
            score, floor, ok = camera_health.motion_clears_floor(
                observer, before, after, health, separation=config.VISION_HEALTH_SEPARATION
            )
            results.append((tag, target, score, floor, ok))
    finally:
        # Never leave the joint under torque, whatever happened above.
        arm._safety.unload([name])
    v_after = _read_battery(arm)

    if all(ok for _, _, _, _, ok in results):
        return  # alive and plainly visible: stages 2 done, hand off to the search

    drew_current = (
        v_before is not None
        and v_after is not None
        and (v_before - v_after) > _LIVENESS_BATTERY_EPSILON_V
    )
    if drew_current:
        # The actuator IS electrically alive (it pulled the pack down turning a
        # load); the camera just could not confirm the motion. That is a *sight*
        # problem, not a dead joint. Defer: the search's observer-reliability burst
        # owns "cannot reliably see this joint" and will abort the whole run there
        # if so. Liveness must not pre-empt it with a hardware verdict it cannot
        # support from a single unseen move.
        logger.warning(
            "%s: liveness legs were not clearly seen, but the pack sagged "
            "%.2fV -> %.2fV (current was drawn), so the actuator is alive and the "
            "camera missed the motion. Deferring to the search's observer-reliability "
            "check rather than declaring the joint dead.",
            name, v_before, v_after,
        )
        return

    raise JointUnresponsive(_liveness_failure_message(name, results, v_before, v_after))


def _liveness_failure_message(
    name: str,
    results: List[Tuple[str, int, float, float, bool]],
    v_before: Optional[float],
    v_after: Optional[float],
) -> str:
    lo, hi = config.provisional_bounds(name)
    legs_txt = "; ".join(
        f"leg {tag} -> {target}: scored {score:.0f} changed px vs required floor "
        f"{floor:.0f} ({'seen' if ok else 'NOT seen'})"
        for tag, target, score, floor, ok in results
    )
    parts = [
        f"{name}: unresponsive. Two probe excursions inside the provisional band "
        f"[{lo}, {hi}] produced no clearly-visible motion -- {legs_txt}.",
    ]
    if v_before is not None and v_after is not None:
        drop = v_before - v_after
        if abs(drop) <= _LIVENESS_BATTERY_EPSILON_V:
            parts.append(
                f"Battery stayed flat ({v_before:.2f}V -> {v_after:.2f}V, "
                f"delta {drop:+.2f}V): consistent with a disconnected / failed-open "
                "servo drawing no current, not a mechanical jam."
            )
        else:
            parts.append(
                f"Battery sagged ({v_before:.2f}V -> {v_after:.2f}V, "
                f"delta {drop:+.2f}V): consistent with the servo drawing current "
                "against something -- check for a mechanical obstruction / jam."
            )
    else:
        parts.append(
            "Battery voltage was unavailable, so current draw could not be checked; "
            "the per-leg scores above are the only evidence."
        )
    parts.append(
        "See docs/hardware-fault-servo5.md for how to tell a dead servo from a jam "
        "and what to inspect (cable seating, connector, servo swap)."
    )
    return " ".join(parts)


def _search_direction(
    arm,
    name: str,
    sign: int,
    settings: CalibrationSettings,
    observer: MotionObserver,
    budget: _Budget,
    health: Optional[CameraHealthReport] = None,
) -> DirectionResult:
    """Step ``name`` in one direction until a stop is confirmed or the rail is hit.

    ``sign`` is -1 (searching for the minimum) or +1 (the maximum). The two possible
    endings are reported differently on purpose: ``FINDING_MECHANICAL_STOP`` is a
    measurement ("it stopped moving here"), ``FINDING_POLICY_LIMIT`` is an
    admission ("we ran out of range we are willing to command and never found a
    stop"). The latter is emphatically not a claim about the servo's electrical or
    mechanical rating -- see the constants' comment at the top of this module.
    """
    lo, hi = search_bounds(name)
    bound = lo if sign < 0 else hi
    start = _start_position(name, lo, hi)
    direction = "min" if sign < 0 else "max"

    current = start
    last_confirmed = start
    consecutive_misses = 0
    pending_misses = 0  # misses not yet resolved into either a stop or an ambiguity
    ambiguous = 0
    steps = 0

    span = hi - lo
    step_cap = min(
        settings.max_steps_per_direction,
        math.ceil(span / settings.step) + settings.confirmations + 2,
    )

    def _result(limit: int, finding: str) -> DirectionResult:
        _retreat(arm, name, limit, settings, budget)
        return DirectionResult(
            direction=direction,
            limit=limit,
            finding=finding,
            margin=settings.margin,
            steps=steps,
            ambiguous_observations=ambiguous,
            last_confirmed_moving=last_confirmed,
            search_bound=bound,
        )

    def _verify_observer_reliability() -> None:
        """Measure the observer against known-real motion before believing a stop.

        "No motion" from an unreliable observer is worthless evidence, and N
        consecutive misses is only significant if the observer's own miss rate is
        low. So before recording a limit, retreat to two positions that BOTH
        verifiably moved earlier -- one and two steps back from ``last_confirmed``,
        strictly on the safe side of the suspected stop -- and oscillate between
        them ``verification_moves`` times. Every one of those moves genuinely
        changes the joint's position, so every "no motion" report during the burst
        is demonstrably the observer's error, not the joint's.

        More than ``max_verification_misses`` errors and the run aborts: an observer
        that misses real motion this often cannot distinguish a mechanical stop from
        its own blindness, and neither can we. A totally blind observer (lens cap,
        dead camera, joint outside the ROI) fails this immediately, which is the
        point -- "I cannot see" must never be recorded as "I measured a limit".

        The oscillation positions are deliberately inside the travel we already
        confirmed, so this burst costs torque time but accrues no stall time.
        """
        nonlocal current
        near = _apply_margin(name, last_confirmed, -sign * settings.step, lo, hi)
        far = _apply_margin(name, last_confirmed, -sign * 2 * settings.step, lo, hi)
        if near == far or near == current:
            # Not enough confirmed travel behind us to build a known-motion probe.
            raise CalibrationAborted(
                f"{name}: no room to verify the observer against known motion near "
                f"{current} -- refusing to record a limit on unverified evidence."
            )
        sequence = [near if i % 2 == 0 else far for i in range(settings.verification_moves)]
        misses = 0
        for target in sequence:
            probe_before = _capture(observer, f"{name} reliability probe before")
            _torque_burst(arm, name, target, settings, budget)
            probe_after = _capture(observer, f"{name} reliability probe after")
            moved_back = _observe(observer, probe_before, probe_after, f"{name} reliability probe")
            current = target
            if not moved_back:
                misses += 1
        if misses > settings.max_verification_misses:
            raise CalibrationAborted(
                f"{name}: the observer missed {misses} of {len(sequence)} moves that "
                "definitely happened (an oscillation between two positions it had already "
                f"watched the joint reach), more than the {settings.max_verification_misses} "
                "allowed. It cannot reliably see this joint -- occlusion, a bad ROI, a dead "
                "camera, lighting, or a viewing angle that makes this joint's motion nearly "
                "invisible. Its 'no motion' reports are therefore not evidence of a "
                "mechanical stop. Aborting without writing a result."
            )

    # Always start each direction from the same known-safe place so the two
    # searches cannot contaminate each other. This move is *already happening* for
    # that reason -- we piggy-back the visibility spot-check on it (item 3) so it
    # costs zero extra torque/packets: capture a frame either side and confirm the
    # observer can actually SEE this known-real motion clearly above its measured
    # noise floor. When no start move happens (the joint is already there) or the
    # noise floor was unmeasurable (a synthetic/noise-free observer), the check
    # degrades to a no-op rather than fabricating a pass -- see
    # camera_health.verify_motion_visibility.
    if arm._shadow.get(name) != start:
        vis_before = _capture(observer, f"{name} {direction} visibility spot-check before start move")
        _torque_burst(arm, name, start, settings, budget)
        vis_after = _capture(observer, f"{name} {direction} visibility spot-check after start move")
        if health is not None:
            camera_health.verify_motion_visibility(
                observer, vis_before, vis_after, health,
                separation=config.VISION_HEALTH_SEPARATION,
                context=f"{name} {direction} start move",
            )

    while True:
        if steps >= step_cap:
            raise CalibrationAborted(
                f"{name} {direction} search hit the step cap ({step_cap}) without "
                "resolving a limit -- aborting rather than guessing where the stop is."
            )
        next_pos = current + sign * settings.step
        reached_bound = False
        if (sign < 0 and next_pos <= bound) or (sign > 0 and next_pos >= bound):
            next_pos = bound
            reached_bound = True

        if next_pos == current:
            # Sitting on the rail: there is no further legal position to probe in
            # this direction, so the search ends here whatever happens.
            if consecutive_misses:
                # We have unconfirmed evidence of a stop right at the rail. Verify
                # the observer is not blind, then record conservatively from the
                # last position that verifiably moved -- it is a real stop, just one
                # we could not confirm with a full run of extra steps.
                _verify_observer_reliability()
                return _result(
                    _apply_margin(name, last_confirmed, -sign * settings.margin, lo, hi),
                    FINDING_MECHANICAL_STOP,
                )
            return _result(
                _apply_margin(name, bound, -sign * settings.margin, lo, hi),
                FINDING_POLICY_LIMIT,
            )

        before = _capture(observer, f"{name} before step -> {next_pos}")
        _torque_burst(arm, name, next_pos, settings, budget)
        after = _capture(observer, f"{name} after step -> {next_pos}")
        moved = _observe(observer, before, after, f"{name} step -> {next_pos}")
        steps += 1
        current = next_pos

        if moved:
            if pending_misses:
                # Earlier "no motion" reports were contradicted by real motion --
                # exactly the flakiness this routine must not mistake for a stop.
                # Count them and let the run-wide cap decide when the observer has
                # become too unreliable to calibrate with.
                ambiguous += pending_misses
                budget.note_ambiguous(pending_misses)
                pending_misses = 0
            last_confirmed = next_pos
            consecutive_misses = 0
            if reached_bound:
                # Still moving when we ran out of legal room: no stop was found.
                return _result(
                    _apply_margin(name, bound, -sign * settings.margin, lo, hi),
                    FINDING_POLICY_LIMIT,
                )
            continue

        consecutive_misses += 1
        pending_misses += 1
        if consecutive_misses < settings.confirmations:
            # Not enough evidence yet. If we are already sitting on the rail there
            # is no further forward probe to take, and the next loop iteration falls
            # into the "sitting on the rail" branch above, which resolves it.
            continue

        _verify_observer_reliability()
        return _result(
            _apply_margin(name, last_confirmed, -sign * settings.margin, lo, hi),
            FINDING_MECHANICAL_STOP,
        )


def _apply_margin(name: str, base: int, delta: int, lo: int, hi: int) -> int:
    """``base + delta``, hard-clamped into the searchable band and the absolute rail.

    The clamp here is on an internally-computed value (a back-off from a position
    we already commanded), never on a user-supplied target -- the same carve-out
    ``SafetyLayer`` makes for its own ramp intermediates.
    """
    abs_lo, abs_hi = config.absolute_bounds(name)
    value = int(base + delta)
    value = max(lo, min(hi, value))
    return max(abs_lo, min(abs_hi, value))


def _retreat(arm, name: str, target: int, settings: CalibrationSettings, budget: _Budget) -> None:
    """Move back off the suspected stop to the recorded limit, then unload."""
    if arm._shadow.get(name) == target:
        return
    _torque_burst(arm, name, target, settings, budget)


def _calibration_health_kwargs(settings: CalibrationSettings) -> Dict[str, Any]:
    """Health-gate parameters for a calibration run.

    The inter-frame interval is derived from ``step_duration_ms`` rather than
    exposed as a new ``CalibrationSettings`` field on purpose: it keeps the gate's
    sampling cadence tied to the same torque-burst timing the run itself uses (so a
    fast test fixture samples fast and a real 400 ms deployment samples across a
    realistic exposure-hunting window), and avoids perturbing ``as_dict``
    serialisation that several tests pin exactly.
    """
    interval = min(settings.step_duration_ms / 1000.0, config.VISION_HEALTH_INTERVAL_S)
    return dict(
        frames=config.VISION_HEALTH_FRAMES,
        interval_s=interval,
        margin=config.VISION_HEALTH_MARGIN,
    )


def calibrate_joint(
    arm,
    name: str,
    *,
    settings: Optional[CalibrationSettings] = None,
    observer: Optional[MotionObserver] = None,
    budget: Optional[_Budget] = None,
    health: Optional[CameraHealthReport] = None,
) -> JointCalibration:
    """Discover one joint's usable band. Never writes a file; see ``calibrate``."""
    settings = settings or CalibrationSettings()
    key = config.validate_joint_name(name)
    observer = observer or observer_for_arm(arm)
    if not observer.available():
        raise CalibrationRefused(
            f"observer {observer.describe()} reports it is unavailable -- cannot "
            "calibrate without a way to tell whether the joint moved."
        )
    # The camera-health gate must run before ANY motion. ``calibrate`` runs it once
    # and threads the report through so it is not re-measured per joint; a direct
    # ``calibrate_joint`` call (the bypass path) has ``health is None`` and so runs
    # the gate itself -- there is no way to reach the search loop without it.
    if health is None:
        health = camera_health.require_camera_health(
            observer, **_calibration_health_kwargs(settings)
        )
    budget = budget if budget is not None else _Budget(settings, arm._transport)

    lo, hi = search_bounds(key)
    span = hi - lo
    if span <= 0:
        raise CalibrationRefused(f"{key}: searchable band [{lo}, {hi}] is empty.")
    if 2 * settings.margin >= span:
        raise CalibrationRefused(
            f"{key}: a margin of {settings.margin} on each end consumes {2 * settings.margin} "
            f"of a {span}-unit searchable band [{lo}, {hi}] -- the result would be empty or "
            "inverted. Refusing to run with a margin this large."
        )
    if 3 * settings.step >= span:
        raise CalibrationRefused(
            f"{key}: step {settings.step} is too coarse for a {span}-unit searchable band "
            f"[{lo}, {hi}] -- the search needs room for at least a couple of probe steps "
            "plus the two-step retreat the observer-reliability burst oscillates over."
        )

    # Stage 2 of the pipeline: prove the actuator is alive before believing any
    # "no motion" the search reports. OPT-IN (``settings.liveness_check``, default
    # False) -- not because the check is unfinished, but because no excursion size
    # is simultaneously visible enough to trust and small enough to be provably
    # stall-free without knowing each joint's true mechanical range. The full
    # numeric argument is on ``CalibrationSettings.liveness_check``. When enabled,
    # no path (via ``calibrate`` or a direct ``calibrate_joint`` call) can reach
    # ``_search_direction`` without it: a dead joint raises ``JointUnresponsive``
    # here and never reaches the search.
    if settings.liveness_check:
        _verify_joint_liveness(arm, key, settings, observer, budget, health)

    torque_before = budget.torque_ms
    stall_before = budget.stall_ms

    try:
        low = _search_direction(arm, key, -1, settings, observer, budget, health)
        high = _search_direction(arm, key, +1, settings, observer, budget, health)
    except (SafetyViolation, config.InvalidPosition) as exc:
        # A step this routine generated was refused by the safety layer. That means
        # this module's own bounds logic disagrees with config.py's -- never swallow
        # it, never retry a "nearby" value.
        raise CalibrationAborted(
            f"{key}: a calibration step was refused by the safety layer ({type(exc).__name__}: "
            f"{exc}). This should be impossible -- calibration only ever probes inside "
            "config.absolute_bounds -- so it is a bug, not a condition to work around. "
            "Aborting without writing a result."
        ) from exc

    if low.limit >= high.limit:
        raise CalibrationAborted(
            f"{key}: discovered min {low.limit} is not below discovered max {high.limit} "
            "-- the search produced a degenerate band (usually an observer that cannot "
            "see this joint). Aborting without writing a result."
        )

    abs_lo, abs_hi = config.absolute_bounds(key)
    if not (abs_lo <= low.limit <= high.limit <= abs_hi):
        raise CalibrationAborted(
            f"{key}: discovered band [{low.limit}, {high.limit}] escaped the absolute rail "
            f"[{abs_lo}, {abs_hi}]. Aborting."
        )

    stall_after = budget.stall_ms
    stall_delta = None if (stall_after is None or stall_before is None) else stall_after - stall_before

    return JointCalibration(
        joint=key,
        servo_id=config.resolve_servo(key),
        min=low.limit,
        max=high.limit,
        margin=settings.margin,
        min_finding=low.finding,
        max_finding=high.finding,
        steps_min=low.steps,
        steps_max=high.steps,
        ambiguous_observations=low.ambiguous_observations + high.ambiguous_observations,
        torque_ms=budget.torque_ms - torque_before,
        stall_ms=stall_delta,
        search_bounds=(lo, hi),
    )


def calibrate(
    arm,
    joints: Optional[Iterable[str]] = None,
    *,
    settings: Optional[CalibrationSettings] = None,
    observer: Optional[MotionObserver] = None,
    output_dir: Optional[Path] = None,
    write: bool = True,
) -> CalibrationRun:
    """Calibrate one or more joints and, on success only, write a result file.

    Failure handling is deliberately asymmetric, and the asymmetry is the whole
    point of the three-stage pipeline (camera health -> per-joint liveness -> limit
    search):

    * Almost every failure aborts the ENTIRE run and writes NOTHING --
      ``CalibrationRefused`` (before any motion), ``CameraUnusableError`` and the
      rest of the ``ObserverError`` family, ``CalibrationAborted`` (a blown budget,
      a degenerate band, a safety-layer refusal), ``SafetyViolation``, an interrupt.
      They all share one property: they mean the *measurement process itself* can no
      longer be trusted for ANY joint (the camera is lying, the observer is blind,
      the run is out of budget). A partial calibration written under those
      conditions would look like a measurement and not be one, so we unload
      everything and write nothing. This invariant is NOT weakened below.

    * ``JointUnresponsive`` is the sole exception, because it is the sole failure
      that is a fact about ONE joint's hardware rather than the process: a dead
      (disconnected / failed-open) actuator. The camera is fine, the other joints
      are fine. Aborting the whole run would let one dead servo hide every healthy
      one. So each ``calibrate_joint`` call is wrapped in its own
      ``try/except JointUnresponsive``: on catch we unload just that joint, record
      it in ``run.failed`` (never in ``run.joints``, so the written file omits it
      and its config.json ``calibrated`` flag stays false), warn, and continue. The
      run still succeeds for the healthy joints and still writes their result.

    Raises ``CalibrationRefused`` before any motion for an unsafe request,
    ``CameraUnusableError`` before any motion if the observer's no-motion noise
    floor swamps its motion threshold, or ``CalibrationAborted`` mid-run for any
    other abort condition. A run in which every requested joint turned out to be
    dead completes normally with an empty ``joints`` and a populated ``failed``.
    """
    settings = settings or CalibrationSettings()
    names = [config.validate_joint_name(n) for n in (joints or config.JOINT_NAMES)]
    if not names:
        raise CalibrationRefused("no joints requested.")
    observer = observer or observer_for_arm(arm)
    budget = _Budget(settings, arm._transport)

    results: Dict[str, JointCalibration] = {}
    failed: Dict[str, str] = {}
    try:
        # Blocking camera-health gate (item 2): measure the observer's OWN no-motion
        # noise floor before any joint is touched and refuse to proceed if it swamps
        # the motion-detection threshold. This is the mirror image of the blindness
        # net (_verify_observer_reliability): that catches an observer that misses
        # real motion; this catches one that "sees" motion in pure sensor/exposure
        # noise -- the fault that otherwise walks every joint to its absolute rail.
        # It commands zero motion, so on failure nothing has moved and nothing is
        # written. Measured once here and threaded into every joint.
        health = camera_health.require_camera_health(
            observer, **_calibration_health_kwargs(settings)
        )
        for name in names:
            logger.info("auto-calibrating %s using %s", name, observer.describe())
            try:
                results[name] = calibrate_joint(
                    arm, name, settings=settings, observer=observer, budget=budget, health=health
                )
            except JointUnresponsive as exc:
                # ONE joint's actuator is dead. Isolate it: this does NOT invalidate
                # the camera or the other joints, so we do not abort the run. Unload
                # just this joint, record the diagnosis, and move on. Crucially, do
                # NOT re-raise into the outer handlers below -- those are for
                # process-wide failures and would abort everything.
                arm.relax([name])
                failed[name] = str(exc)
                logger.warning("skipping %s: %s", name, exc)
                continue
    except CameraUnusableError:
        # Propagate unwrapped so callers (and tests) can catch the specific fault.
        # It is an ObserverError subclass, so the generic branch below would
        # otherwise rewrap it into a CalibrationAborted and hide the cause.
        arm.relax(names)
        raise
    except ObserverError as exc:
        arm.relax(names)
        raise CalibrationAborted(
            f"motion observer failed mid-run ({exc}). Without a trustworthy answer to "
            "'did it move', every further step would be a guess. Aborting without "
            "writing a result."
        ) from exc
    except BaseException:
        # Includes CalibrationAborted, KeyboardInterrupt, and anything unforeseen:
        # unload first, re-raise second, write nothing either way.
        arm.relax(names)
        raise
    else:
        arm.relax(names)

    run = CalibrationRun(
        joints=results,
        settings=settings,
        observer=observer.describe(),
        transport="sim" if getattr(arm, "sim", False) else "hardware",
        elapsed_s=budget.elapsed_s,
        torque_ms=budget.torque_ms,
        stall_ms=budget.stall_ms,
        failed=failed,
    )
    if write:
        run.output_path = write_calibration(run, output_dir=output_dir)
    return run


def default_output_dir() -> Path:
    return config.PACKAGE_DIR / "data"


def write_calibration(run: CalibrationRun, output_dir: Optional[Path] = None) -> Path:
    """Write a timestamped calibration result. Never touches ``config.json``.

    Promotion into the live config is a separate, explicit act -- see
    ``apply_calibration``.
    """
    directory = Path(output_dir) if output_dir is not None else default_output_dir()
    directory.mkdir(parents=True, exist_ok=True)
    stamp = _dt.datetime.now(_dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    path = directory / f"calibration-{stamp}.json"
    suffix = 1
    while path.exists():
        path = directory / f"calibration-{stamp}-{suffix}.json"
        suffix += 1
    payload = run.as_dict()
    with open(path, "w") as f:
        json.dump(payload, f, indent=2)
        f.write("\n")
    logger.info("wrote calibration result to %s", path)
    return path


# ---------------------------------------------------------------------------
# Promotion: a separate, explicit step. Discovering limits and trusting them are
# two different decisions and are kept two different actions on purpose.
# ---------------------------------------------------------------------------


def load_calibration(path: Path) -> dict:
    with open(path) as f:
        payload = json.load(f)
    if not isinstance(payload, dict):
        raise CalibrationRefused(f"{path}: calibration file must be a JSON object.")
    if payload.get("schema") != SCHEMA:
        raise CalibrationRefused(
            f"{path}: unknown calibration schema {payload.get('schema')!r} (expected {SCHEMA!r})."
        )
    if not isinstance(payload.get("joints"), dict) or not payload["joints"]:
        raise CalibrationRefused(f"{path}: calibration file has no 'joints' object.")
    return payload


def apply_calibration(
    path: Path,
    *,
    joints: Optional[Iterable[str]] = None,
    config_path: Optional[Path] = None,
    allow_simulated: bool = False,
) -> Dict[str, dict]:
    """Merge a calibration result into ``config.json`` and mark those joints calibrated.

    Every value is re-validated against the hardcoded rails before it is written --
    a hand-edited calibration file claiming a wider range than
    ``config.absolute_bounds`` is refused, not honoured. The updated file is then
    re-parsed through ``config._validate_servo_spec`` before it replaces the live
    one, so a promotion can never leave ``config.json`` in a state that would fail
    at import.

    A result produced against the simulator is refused unless ``allow_simulated``
    is set: simulated mechanical limits are *assumptions* about this hardware, not
    measurements of it, and promoting them would launder one into the other.
    """
    path = Path(path)
    payload = load_calibration(path)
    if payload.get("transport") != "hardware" and not allow_simulated:
        raise CalibrationRefused(
            f"{path} was produced against the {payload.get('transport')!r} transport, not real "
            "hardware. Its limits are the simulator's assumptions, not measurements of this "
            "arm. Re-run against hardware, or pass allow_simulated=True / --allow-simulated "
            "if you are deliberately promoting simulated numbers (e.g. for a test)."
        )

    target = Path(config_path) if config_path is not None else Path(
        os.environ.get("ROBOHAND_CONFIG", config.DEFAULT_CONFIG_PATH)
    )
    raw = config.load_config(target)
    servos = raw.get("servos")
    if not isinstance(servos, dict):
        raise CalibrationRefused(f"{target}: config.json has no 'servos' object to update.")

    wanted = [config.validate_joint_name(n) for n in joints] if joints else None
    applied: Dict[str, dict] = {}

    for name, entry in payload["joints"].items():
        key = config.validate_joint_name(name)
        if wanted is not None and key not in wanted:
            continue
        if key not in servos:
            raise CalibrationRefused(f"{target}: config.json has no servos.{key} entry to update.")
        lo = entry.get("min")
        hi = entry.get("max")
        for label, value in (("min", lo), ("max", hi)):
            if isinstance(value, bool) or not isinstance(value, int):
                raise CalibrationRefused(
                    f"{path}: joints.{key}.{label} must be an integer, got {value!r}."
                )
        if lo >= hi:
            raise CalibrationRefused(f"{path}: joints.{key} range [{lo}, {hi}] is empty or inverted.")
        abs_lo, abs_hi = config.absolute_bounds(key)
        if lo < abs_lo or hi > abs_hi:
            raise CalibrationRefused(
                f"{path}: joints.{key} range [{lo}, {hi}] is outside {key}'s absolute rail "
                f"[{abs_lo}, {abs_hi}]. A calibration file can never widen a joint past the "
                "hardcoded bound -- refusing to promote it."
            )

        spec = dict(servos[key])
        spec["min"] = lo
        spec["max"] = hi
        spec["calibrated"] = True
        center = spec.get("center")
        if not isinstance(center, int) or isinstance(center, bool) or not (lo <= center <= hi):
            spec["center"] = (lo + hi) // 2
        spec["calibration_source"] = str(path.name)
        # The shipped comments say "PROVISIONAL ... NOT calibrated". Leaving them in
        # place next to calibrated:true would be actively misleading to the next
        # human who opens this file, so replace them with honest provenance.
        spec["comment"] = (
            f"Auto-calibrated from {path.name} "
            f"(min: {entry.get('min_finding', 'unknown')}, max: {entry.get('max_finding', 'unknown')}, "
            f"margin {entry.get('margin', '?')} raw units per end, observed via "
            f"{payload.get('observer', 'unknown observer')}). Vision-derived, open loop, "
            "not a hand-measured value -- cross-check by hand before trusting it with a payload."
        )
        # Fail before writing, not after: this is the same validation config.py runs
        # at import, so a spec that would break the next import is rejected here.
        config._validate_servo_spec(key, spec)
        servos[key] = spec
        applied[key] = spec

    if not applied:
        raise CalibrationRefused(f"{path}: no matching joints to apply.")

    _write_config_atomically(target, raw)

    # Round-trip: re-read what we just wrote and re-validate every joint, not only
    # the ones we touched, so a promotion can never leave the file unloadable.
    reloaded = config.load_config(target)
    for joint_name, joint_spec in reloaded.get("servos", {}).items():
        config._validate_servo_spec(joint_name, joint_spec)

    logger.warning(
        "promoted calibration for %s into %s -- these joints are now marked calibrated "
        "and are no longer limited to the provisional band.",
        sorted(applied), target,
    )
    return applied


def _write_config_atomically(target: Path, payload: dict) -> None:
    """Write ``payload`` to ``target`` via a same-directory temp file + os.replace.

    Same directory so the replace is atomic (a cross-filesystem move is not), and
    so a half-written config can never be the file the next process imports.
    """
    target = Path(target)
    tmp = target.with_name(target.name + ".calibration-tmp")
    with open(tmp, "w") as f:
        json.dump(payload, f, indent=2)
        f.write("\n")
    os.replace(tmp, target)


def summarize(run: CalibrationRun) -> List[str]:
    """Human-readable one line per joint, for the CLI."""
    lines = []
    for name, jc in run.joints.items():
        lines.append(
            f"  {name:<14} [{jc.min}, {jc.max}]  min={jc.min_finding} max={jc.max_finding} "
            f"steps={jc.steps_min}/{jc.steps_max} ambiguous={jc.ambiguous_observations} "
            f"torque={jc.torque_ms}ms"
        )
    for name, reason in run.failed.items():
        lines.append(f"  {name:<14} UNRESPONSIVE -- {reason}")
    return lines
