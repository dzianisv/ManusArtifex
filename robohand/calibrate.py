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
    max_torque_ms: int = 120_000

    #: Cumulative simulated stall time. Only enforceable against ``SimTransport``
    #: (real hardware cannot report stalls -- that is the entire problem this
    #: package exists to work around), so on hardware this is unenforced by
    #: construction, not merely unset.
    max_stall_ms: int = 6_000

    #: Wall-clock cap for the whole run.
    max_wall_clock_s: float = 900.0

    #: A no-motion report that a later observation contradicts (the joint moved
    #: again afterwards) is an *ambiguous* observation. A handful means a twitchy
    #: camera; a lot means the observer is not trustworthy enough to calibrate with.
    max_ambiguous_observations: int = 8

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
        _int("max_torque_ms", self.max_torque_ms, 1, 3_600_000)
        _int("max_stall_ms", self.max_stall_ms, 0, 3_600_000)
        _int("max_ambiguous_observations", self.max_ambiguous_observations, 0, 1000)
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


def _torque_burst(arm, name: str, target: int, settings: CalibrationSettings, budget: _Budget) -> None:
    """One bounded step: command it, wait for it, then unload before observing.

    ``vision_guard=None`` disables ``SafetyLayer``'s own automatic hard-fail vision
    check for this call, for the same reason the grip ladder does it: here "no
    motion" is the *expected* successful outcome of the last step in a direction,
    not an anomaly to raise ``StallDetected`` for. This routine does its own,
    graceful no-motion handling immediately below the call site.

    ``probe_absolute_bounds=True`` is what makes the widened ``search_bounds``
    actually reachable: without it ``SafetyLayer`` would check this target against
    ``config.effective_bounds`` (the configured band) and refuse anything outside
    it, even with the override applied. It is still audited, still requires the
    override + reason below, and is still subject to ``SafetyLayer``'s
    unconditional absolute-rail backstop.
    """
    arm.set_position(
        name,
        target,
        duration_ms=settings.step_duration_ms,
        wait=True,
        vision_guard=None,
        allow_uncalibrated_override=True,
        probe_absolute_bounds=True,
        override_reason=f"auto-calibration probe for {name} (bounded {settings.step_duration_ms}ms burst)",
    )
    # Unload through SafetyLayer *before* observing -- bounded exposure per step is
    # the entire safety argument for this routine. Deliberately not arm.relax(),
    # which additionally erases the shadow position estimate that SafetyLayer needs
    # to decide ramping for the next step.
    arm._safety.unload([name])
    budget.charge_torque(settings.step_duration_ms)


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


def _search_direction(
    arm,
    name: str,
    sign: int,
    settings: CalibrationSettings,
    observer: MotionObserver,
    budget: _Budget,
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
    # searches cannot contaminate each other.
    if arm._shadow.get(name) != start:
        _torque_burst(arm, name, start, settings, budget)

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


def calibrate_joint(
    arm,
    name: str,
    *,
    settings: Optional[CalibrationSettings] = None,
    observer: Optional[MotionObserver] = None,
    budget: Optional[_Budget] = None,
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

    torque_before = budget.torque_ms
    stall_before = budget.stall_ms

    try:
        low = _search_direction(arm, key, -1, settings, observer, budget)
        high = _search_direction(arm, key, +1, settings, observer, budget)
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

    Raises ``CalibrationRefused`` before any motion for an unsafe request, or
    ``CalibrationAborted`` mid-run for any abort condition. In both cases every
    joint involved is unloaded and **nothing is written** -- there is no partial
    result, on purpose.
    """
    settings = settings or CalibrationSettings()
    names = [config.validate_joint_name(n) for n in (joints or config.JOINT_NAMES)]
    if not names:
        raise CalibrationRefused("no joints requested.")
    observer = observer or observer_for_arm(arm)
    budget = _Budget(settings, arm._transport)

    results: Dict[str, JointCalibration] = {}
    try:
        for name in names:
            logger.info("auto-calibrating %s using %s", name, observer.describe())
            results[name] = calibrate_joint(
                arm, name, settings=settings, observer=observer, budget=budget
            )
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
    return lines
