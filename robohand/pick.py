"""Vision-guided pick: an explicit state machine, not an open-loop lunge.

Why this exists in this shape
-----------------------------
This arm has **no position feedback** (``protocol.NoResponse``; see
docs/research-hardware.md). Nothing can tell you where a joint actually is, only
where you last asked it to go. Its kinematics are equally unknown: the raw PWM
value -> geometry relationship has never been measured on this unit and there is
no encoder to measure it with. So the two obvious designs are both unavailable:

* inverse kinematics -- needs a verified geometric model this arm does not have;
* a learned VLA policy -- needs joint state in the observation, which this
  hardware physically cannot produce (docs/cocktail-robot.md).

What is left is the thing ``calibrate.py`` already does for limits: **command a
small increment, look, decide.** This module does that for position. It measures
its own local sensitivity by nudging each servo joint a known amount and watching
how far the gripper moves *in the image*, then uses that 2x2 image Jacobian to
turn "reduce this pixel error" into "move these joints this much" -- re-observing
after every single step. No trajectory is ever computed in advance, and no
single perception result is ever trusted enough to authorise a lunge.

State machine
-------------
::

    LOCATE ---> APPROACH ---> DESCEND ---> GRASP ---> VERIFY ---> LIFT ---> VERIFY_HELD ---> DONE
       |           |             |           |          |          |            |
       |           |             |           |          |          |            |
       +-----------+-------------+-----------+----------+----------+------------+
                                     |
                          FAILED (clean) / ABORTED (injected or Ctrl-C)
                                     |
                            release grip + unload every servo

Closed loop where it can be, open loop where it structurally cannot
-------------------------------------------------------------------
APPROACH and the post-descend refinement are genuinely closed loop: locate, move
a little, locate again, recompute. **DESCEND and LIFT are open loop and cannot be
anything else.** A single camera on a known table plane gives X and Y only
(``vision.Calibration`` is a plane-to-plane homography); there is no depth
measurement anywhere in this system, and nothing in this file pretends otherwise.
The descend/lift amounts are operator-supplied raw-unit offsets whose *sign* must
be established by hand on real hardware -- this module cannot discover which
direction is "down" from a top-down view.

Every motion goes through ``arm.set_positions`` -> ``safety.SafetyLayer.move``.
There is no other path here: no ``transport.move_servos`` call, no
``MotionToken`` construction, no direct transport access for motion. The gripper
is only ever driven by ``arm.close_grip`` / ``arm.open_grip`` (the incremental
ladder), never by an absolute "closed" target.
"""

from __future__ import annotations

import dataclasses
import enum
import logging
import math
import time
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

from . import config
from .guard import StallDetected
from .locate import GripperLocation, Locator, LocatorError, ObjectLocation
from .safety import SafetyViolation

logger = logging.getLogger("robohand.pick")


class PickError(RuntimeError):
    """Base class for pick refusals."""


class PickRefused(PickError):
    """Raised *before any motion* when the routine will not run at all.

    Currently: an uncalibrated joint with no explicit override, or no way to tell
    whether the gripper actually moved (no motion observer, which makes
    "grasped something" indistinguishable from "closed on air").
    """


class PickState(enum.Enum):
    LOCATE = "LOCATE"
    APPROACH = "APPROACH"
    DESCEND = "DESCEND"
    GRASP = "GRASP"
    VERIFY = "VERIFY"
    LIFT = "LIFT"
    VERIFY_HELD = "VERIFY_HELD"
    DONE = "DONE"
    FAILED = "FAILED"
    ABORTED = "ABORTED"


#: States the routine can be told to abort from, for fault-injection tests.
ABORTABLE_STATES = (
    PickState.LOCATE,
    PickState.APPROACH,
    PickState.DESCEND,
    PickState.GRASP,
    PickState.VERIFY,
    PickState.LIFT,
    PickState.VERIFY_HELD,
)


@dataclasses.dataclass(frozen=True)
class PickSettings:
    """Tunables. Every default here is a *starting point*, not a measurement.

    The pixel tolerances are in source-frame pixels, which means they are only
    meaningful for a given camera distance and resolution -- re-derive them for
    your own setup rather than trusting these.
    """

    #: The two joints the visual servo loop is allowed to move. Two, because the
    #: image error it is correcting is two-dimensional; a third would make the
    #: 2x2 Jacobian underdetermined without buying any observable improvement.
    servo_joints: Tuple[str, str] = ("SHOULDER_PAN", "ELBOW")
    #: The joint used for the open-loop height changes.
    height_joint: str = "SHOULDER_LIFT"

    #: Converged when the object<->gripper pixel error drops below this.
    approach_tolerance_px: float = 12.0
    #: Tighter tolerance for the refinement pass done at grasp height.
    grasp_tolerance_px: float = 6.0
    max_approach_steps: int = 14
    max_refine_steps: int = 8
    #: Largest single joint increment the servo loop may command, raw units.
    max_joint_step: int = 120
    #: Increment used to measure the image Jacobian, raw units.
    probe_delta: int = 60
    #: Fraction of the computed correction actually applied, per step. < 1 so an
    #: over-estimated sensitivity damps out instead of oscillating.
    gain: float = 0.85
    #: Consecutive non-improving iterations tolerated before declaring the loop
    #: divergent. This is the difference between "slow but working" and "stuck".
    convergence_patience: int = 3
    #: An iteration counts as improvement only if it beats the best error so far
    #: by at least this much.
    min_improvement_px: float = 1.0
    #: Below this commanded delta, absence of visible motion is uninformative, so
    #: the automatic vision stall check is skipped for that step (it would fire
    #: on sub-pixel motion that a real camera simply cannot resolve).
    min_guarded_delta_units: int = 20

    #: OPEN LOOP. Raw-unit offset applied to ``height_joint`` to reach grasp
    #: height, and back off again to lift. The sign is hardware-specific and MUST
    #: be verified by hand -- see docs/pick.md.
    descend_units: int = -200
    lift_units: int = 300
    #: Height-joint position treated as "travel height". None = the joint's centre.
    travel_position: Optional[int] = None

    #: Max pixel distance between object and gripper for the grasp to be believed.
    verify_max_distance_px: float = 25.0
    #: Same, after the lift.
    held_max_distance_px: float = 18.0
    #: The object must visibly move by at least this much during the lift, or it
    #: is still on the table and we are holding nothing.
    min_lift_displacement_px: float = 10.0

    move_duration_ms: Optional[int] = None
    grip_step: Optional[int] = None
    grip_duration_ms: Optional[int] = None

    #: On success, leave GRIP (and the arm joints holding the load) under torque
    #: rather than unloading. Required for the pick to mean anything -- an
    #: immediate unload drops the object -- but understand what it costs: the
    #: safety watchdog's ``max_continuous_torque_ms`` cap (8 s by default) WILL
    #: force an unload shortly afterwards regardless. This arm cannot hold
    #: anything indefinitely; see docs/pick.md.
    hold_on_success: bool = True

    #: Fault injection: abort cleanly on entry to any of these states.
    abort_states: Tuple[str, ...] = ()

    def abortable(self) -> Tuple[str, ...]:
        return tuple(s.strip().upper() for s in self.abort_states)


@dataclasses.dataclass
class PickLogEntry:
    state: str
    detail: str
    timestamp: float
    data: Dict[str, Any] = dataclasses.field(default_factory=dict)

    def as_dict(self) -> Dict[str, Any]:
        return {
            "state": self.state,
            "detail": self.detail,
            "timestamp": round(self.timestamp, 3),
            "data": self.data,
        }


@dataclasses.dataclass
class PickResult:
    """Everything needed to diagnose a failed pick after the fact.

    Deliberately self-contained: ``log`` carries one entry per decision the state
    machine made, with the numbers it made them from, so a failure can be
    understood from this object alone without re-running anything.
    """

    success: bool
    state: PickState
    reason: str
    locator: str
    log: List[PickLogEntry] = dataclasses.field(default_factory=list)
    grip_result: Optional[Dict[str, Any]] = None
    approach_error_px: Optional[float] = None
    elapsed_s: float = 0.0
    unloaded: bool = False
    #: True when the routine deliberately left servos under torque (a successful
    #: pick still holding the object). Mutually exclusive with ``unloaded``.
    holding: bool = False

    def as_dict(self) -> Dict[str, Any]:
        return {
            "success": self.success,
            "state": self.state.value,
            "reason": self.reason,
            "locator": self.locator,
            "approach_error_px": None if self.approach_error_px is None else round(self.approach_error_px, 2),
            "elapsed_s": round(self.elapsed_s, 3),
            "unloaded": self.unloaded,
            "holding": self.holding,
            "grip_result": self.grip_result,
            "log": [e.as_dict() for e in self.log],
        }

    def states_visited(self) -> List[str]:
        out: List[str] = []
        for entry in self.log:
            if not out or out[-1] != entry.state:
                out.append(entry.state)
        return out


@dataclasses.dataclass
class _ServoOutcome:
    converged: bool
    reason: str
    error_px: Optional[float]
    steps: int


def required_joints(settings: PickSettings) -> List[str]:
    """Every joint this routine will command, in the order it first touches them."""
    names = [config.validate_joint_name(n) for n in settings.servo_joints]
    names.append(config.validate_joint_name(settings.height_joint))
    names.append("GRIP")
    seen: List[str] = []
    for name in names:
        if name not in seen:
            seen.append(name)
    return seen


def uncalibrated_joints(settings: PickSettings) -> List[str]:
    return [n for n in required_joints(settings) if not config.JOINT_LIMITS[n].calibrated]


def ensure_motion_observer(arm) -> bool:
    """Make sure the grip ladder has *some* way to see whether a step moved.

    Without one, ``arm.close_grip()`` can never report ``no_motion_detected`` and
    "I grasped something" becomes indistinguishable from "I closed on empty air"
    -- which is precisely the distinction this routine's GRASP state depends on.

    On a simulated arm with no guard attached we install
    ``observer.SimMotionObserver``'s guard, which reads the simulator's *achieved*
    joint positions (ground truth, never intent). On real hardware we never
    fabricate one: no camera means no pick.
    """
    guard = getattr(arm, "vision_guard", None)
    if guard is not None and guard.available():
        return True
    if getattr(arm, "sim", False):
        from .observer import SimMotionObserver

        arm.vision_guard = SimMotionObserver(arm._transport).guard
        return True
    return False


class PickRoutine:
    """The state machine. Construct, then call :meth:`run` once."""

    def __init__(
        self,
        arm,
        locator: Locator,
        settings: Optional[PickSettings] = None,
        *,
        allow_uncalibrated_override: bool = False,
        override_reason: Optional[str] = None,
    ) -> None:
        self.arm = arm
        self.locator = locator
        self.settings = settings or PickSettings()
        self.allow_uncalibrated_override = bool(allow_uncalibrated_override)
        self.override_reason = override_reason
        self.log: List[PickLogEntry] = []
        self._jacobian: Optional[Tuple[Tuple[float, float], Tuple[float, float]]] = None
        self._started = 0.0

    # -- logging -----------------------------------------------------------

    def _note(self, state: PickState, detail: str, **data) -> None:
        entry = PickLogEntry(state=state.value, detail=detail, timestamp=time.time(), data=data)
        self.log.append(entry)
        logger.debug("pick[%s] %s %s", state.value, detail, data)

    # -- motion, always through SafetyLayer via Arm -------------------------

    def _move(self, positions: Dict[str, int], *, hold=None, guarded: bool = True, duration_ms=None):
        """The only motion call in this module. Forwards straight to ``Arm``.

        ``guarded=False`` disables ``SafetyLayer``'s automatic vision stall check
        for this one call. It is used exactly twice, both times because *absence
        of motion is the expected outcome*, not an anomaly: moving to a start
        pose we may already be in, and pressing the jaws into an object we have
        deliberately grasped. Everything else keeps the check.
        """
        kwargs: Dict[str, Any] = {
            "duration_ms": duration_ms if duration_ms is not None else self.settings.move_duration_ms,
            "hold": hold,
        }
        if self.allow_uncalibrated_override:
            kwargs["allow_uncalibrated_override"] = True
            kwargs["override_reason"] = self.override_reason
        if not guarded:
            kwargs["vision_guard"] = None
        return self.arm.set_positions(positions, **kwargs)

    def _current(self, name: str) -> int:
        value = self.arm._shadow.get(name)
        return int(value) if value is not None else int(config.JOINT_LIMITS[name].center)

    def _bounds(self, name: str) -> Tuple[int, int]:
        return config.effective_bounds(
            name, allow_uncalibrated_override=self.allow_uncalibrated_override
        )

    def _clamp_internal(self, name: str, value: float) -> int:
        """Clamp an internally-computed servo increment to the joint's own band.

        This is the one place clamping is legitimate, for the same reason
        ``SafetyLayer`` allows it on ramp intermediates: the value is generated
        by this loop, not supplied by an operator, so there is no user intent to
        silently corrupt. A *user*-supplied target is never clamped anywhere.
        """
        lo, hi = self._bounds(name)
        return int(max(lo, min(hi, round(value))))

    # -- perception, always validated --------------------------------------

    def _validated_object(self) -> Optional[ObjectLocation]:
        found = self.locator.locate_object()
        if found is None:
            return None
        if not self._accept(found.px, found.py):
            self._note(
                PickState.LOCATE,
                "rejected object location outside the workspace",
                px=found.px, py=found.py, source=found.source, label=found.label,
            )
            return None
        return found

    def _validated_gripper(self) -> Optional[GripperLocation]:
        found = self.locator.locate_gripper()
        if found is None:
            return None
        if not self._accept(found.px, found.py):
            self._note(
                PickState.LOCATE,
                "rejected gripper location outside the workspace",
                px=found.px, py=found.py, source=found.source,
            )
            return None
        return found

    def _accept(self, px: float, py: float) -> bool:
        """Independent re-validation of every location, whatever produced it.

        The locators validate their own output too; this is the second,
        non-negotiable check that lives on the motion side of the boundary. It
        applies identically to a 20-line HSV threshold and to a large multimodal
        model -- there is no privileged backend, and a Gemini response has
        exactly the same standing as any other signal. That is a direct
        consequence of watching ER confidently point at a background object when
        asked to find something that was not in the frame
        (docs/cocktail-robot.md).
        """
        from .locate import point_in_workspace

        return point_in_workspace(
            px,
            py,
            frame_shape=getattr(self.locator, "frame_shape", None),
            calibration=getattr(self.locator, "calibration", None),
            envelope=getattr(self.locator, "envelope", None),
        )

    # -- empirically measured image Jacobian --------------------------------

    def _probe_jacobian(self) -> Optional[str]:
        """Measure d(gripper pixel)/d(joint unit) by nudging each servo joint.

        Exactly the ``calibrate.py`` idiom: command a small increment, look,
        decide. No kinematic model is assumed, imported, or hardcoded -- if the
        arm's geometry, the camera pose, or the joint directions change, this
        measurement changes with them. Its validity is strictly local, which is
        why the loop re-observes after every step instead of trusting it to plan
        a whole trajectory.
        """
        columns: List[Tuple[float, float]] = []
        for name in self.settings.servo_joints:
            name = config.validate_joint_name(name)
            before = self._validated_gripper()
            if before is None:
                return "gripper_not_found"
            start = self._current(name)
            lo, hi = self._bounds(name)
            delta = int(self.settings.probe_delta)
            if start + delta > hi:
                delta = -delta
            target = start + delta
            if not (lo <= target <= hi):
                self._note(PickState.LOCATE, "no room to probe joint sensitivity", joint=name, start=start, bounds=(lo, hi))
                return "no_room_to_probe"
            self._move({name: target}, guarded=abs(delta) >= self.settings.min_guarded_delta_units)
            after = self._validated_gripper()
            self._move({name: start}, guarded=abs(delta) >= self.settings.min_guarded_delta_units)
            if after is None:
                return "gripper_not_found"
            columns.append(((after.px - before.px) / delta, (after.py - before.py) / delta))
            self._note(
                PickState.LOCATE,
                "measured joint sensitivity",
                joint=name, delta=delta,
                dpx_per_unit=round(columns[-1][0], 5), dpy_per_unit=round(columns[-1][1], 5),
            )

        (ax, ay), (bx, by) = columns
        det = ax * by - bx * ay
        if abs(det) < 1e-9:
            self._note(PickState.LOCATE, "image Jacobian is singular", determinant=det)
            return "degenerate_jacobian"
        self._jacobian = ((ax, ay), (bx, by))
        return None

    def _solve(self, ex: float, ey: float) -> Tuple[float, float]:
        (ax, ay), (bx, by) = self._jacobian  # type: ignore[misc]
        det = ax * by - bx * ay
        da = (by * ex - bx * ey) / det
        db = (-ay * ex + ax * ey) / det
        return da, db

    # -- the look-move-look-again loop --------------------------------------

    def _servo_loop(self, state: PickState, tolerance_px: float, max_steps: int) -> _ServoOutcome:
        best = math.inf
        stagnant = 0
        last_error: Optional[float] = None
        for step in range(1, max_steps + 1):
            obj = self._validated_object()
            if obj is None:
                return _ServoOutcome(False, "object_not_found", last_error, step - 1)
            grip = self._validated_gripper()
            if grip is None:
                return _ServoOutcome(False, "gripper_not_found", last_error, step - 1)

            ex, ey = obj.px - grip.px, obj.py - grip.py
            error = math.hypot(ex, ey)
            last_error = error
            self._note(
                state, "observed error",
                step=step, error_px=round(error, 2),
                object_px=[round(obj.px, 1), round(obj.py, 1)],
                gripper_px=[round(grip.px, 1), round(grip.py, 1)],
                object_source=obj.source, gripper_source=grip.source,
            )
            if error <= tolerance_px:
                return _ServoOutcome(True, "converged", error, step)

            # Convergence guard: "still trying" and "stuck" must not look the
            # same. An iteration only counts if it beats the best error so far.
            if error < best - self.settings.min_improvement_px:
                best = error
                stagnant = 0
            else:
                stagnant += 1
                if stagnant >= self.settings.convergence_patience:
                    self._note(
                        state, "error stopped decreasing -- aborting rather than "
                        "burning the step budget",
                        step=step, error_px=round(error, 2), best_px=round(best, 2),
                        patience=self.settings.convergence_patience,
                    )
                    return _ServoOutcome(False, "no_convergence", error, step)

            da, db = self._solve(ex * self.settings.gain, ey * self.settings.gain)
            targets: Dict[str, int] = {}
            pinned = 0
            max_delta = 0
            for name, delta in zip(self.settings.servo_joints, (da, db)):
                name = config.validate_joint_name(name)
                delta = max(-self.settings.max_joint_step, min(self.settings.max_joint_step, delta))
                start = self._current(name)
                target = self._clamp_internal(name, start + delta)
                if target != start:
                    targets[name] = target
                    max_delta = max(max_delta, abs(target - start))
                elif abs(delta) >= 1.0:
                    pinned += 1
            if not targets:
                reason = "joint_limits_reached" if pinned else "correction_below_resolution"
                self._note(
                    state, "cannot reduce the error any further",
                    step=step, error_px=round(error, 2), reason=reason,
                )
                return _ServoOutcome(False, reason, error, step)
            self._move(targets, guarded=max_delta >= self.settings.min_guarded_delta_units)
        return _ServoOutcome(False, "step_budget_exhausted", last_error, max_steps)

    # -- failure handling ---------------------------------------------------

    def _safe_stop(self, release: bool) -> bool:
        """Leave the arm in a defined state: jaws open (if asked), torque off.

        Called on *every* exit path that is not DONE. ``release`` runs the open
        ladder first so a part-grasped object is let go before torque disappears;
        if that itself fails, the unload still happens -- an unloaded servo is
        always safe, which is why ``SafetyLayer.unload`` needs no token.
        """
        if release:
            try:
                self.arm.open_grip(
                    step=self.settings.grip_step, duration_ms=self.settings.grip_duration_ms
                )
            except Exception as e:  # noqa: BLE001 - never let cleanup mask the failure
                self._note(PickState.FAILED, "could not open the gripper during cleanup", error=str(e))
        try:
            self.arm.relax()
            return True
        except Exception as e:  # noqa: BLE001 - pragma: no cover
            self._note(PickState.FAILED, "unload failed during cleanup", error=str(e))
            return False

    def _finish(
        self,
        state: PickState,
        reason: str,
        *,
        success: bool = False,
        release: bool = False,
        grip_result=None,
        approach_error_px=None,
    ) -> PickResult:
        self._note(state, reason)
        unloaded = self._safe_stop(release=release)
        return PickResult(
            success=success,
            state=state,
            reason=reason,
            locator=self.locator.describe(),
            log=list(self.log),
            grip_result=grip_result,
            approach_error_px=approach_error_px,
            elapsed_s=time.time() - self._started,
            unloaded=unloaded,
        )

    def _aborted(self, state: PickState) -> bool:
        return state.value in self.settings.abortable()

    # -- the routine --------------------------------------------------------

    def run(self) -> PickResult:
        self._started = time.time()
        settings = self.settings

        # --- gates, before any motion whatsoever --------------------------
        uncal = uncalibrated_joints(settings)
        if uncal and not self.allow_uncalibrated_override:
            raise PickRefused(
                f"refusing to run a pick: {', '.join(uncal)} "
                f"{'is' if len(uncal) == 1 else 'are'} not calibrated "
                "(calibrated: false in config.json). A pick drives these joints to "
                "positions derived from a camera, so an unknown joint range means "
                "an unknown reach. Calibrate first (robohand calibrate --all, then "
                "--apply; see docs/calibration.md), or pass an explicit, logged "
                "override with a reason."
            )
        if self.allow_uncalibrated_override and not self.override_reason:
            raise ValueError("allow_uncalibrated_override requires a non-empty override_reason")
        if not self.locator.available():
            raise PickRefused(
                f"locator {self.locator.describe()} is not available -- a pick with no "
                "perception is an open-loop lunge, which is exactly what this package "
                "exists to prevent."
            )
        if not ensure_motion_observer(self.arm):
            raise PickRefused(
                "no motion observer (no camera) -- without one, the grip ladder cannot "
                "tell 'closed onto an object' from 'closed onto empty air', so a grasp "
                "could never be verified. Attach a camera, or run with --sim."
            )

        try:
            return self._run_states()
        except (StallDetected, SafetyViolation, LocatorError) as e:
            return self._finish(PickState.FAILED, f"{type(e).__name__}: {e}", release=True)
        except KeyboardInterrupt:
            return self._finish(PickState.ABORTED, "interrupted by operator", release=True)

    def _run_states(self) -> PickResult:
        settings = self.settings
        height = config.validate_joint_name(settings.height_joint)

        # ---------------- LOCATE ------------------------------------------
        if self._aborted(PickState.LOCATE):
            return self._finish(PickState.ABORTED, "aborted at LOCATE (injected)")
        travel = settings.travel_position
        if travel is None:
            travel = config.JOINT_LIMITS[height].center
        travel = self._clamp_internal(height, travel)
        start_pose = {config.validate_joint_name(n): self._current(config.validate_joint_name(n))
                      for n in settings.servo_joints}
        start_pose[height] = travel
        # guarded=False: we have no position feedback, so this move may
        # legitimately be a no-op (we may already be at the travel pose). A no-op
        # here is not evidence of a stall.
        self._move(start_pose, guarded=False)
        self._note(PickState.LOCATE, "at travel pose", pose=start_pose)

        try:
            self.arm.open_grip(step=settings.grip_step, duration_ms=settings.grip_duration_ms)
        except StallDetected as e:
            return self._finish(PickState.FAILED, f"could not open the gripper: {e}", release=False)

        obj = self._validated_object()
        if obj is None:
            return self._finish(PickState.FAILED, "object_not_found")
        self._note(PickState.LOCATE, "object located", px=round(obj.px, 1), py=round(obj.py, 1),
                   confidence=obj.confidence, source=obj.source, label=obj.label)

        probe_failure = self._probe_jacobian()
        if probe_failure is not None:
            return self._finish(PickState.FAILED, probe_failure)

        # ---------------- APPROACH (closed loop) --------------------------
        if self._aborted(PickState.APPROACH):
            return self._finish(PickState.ABORTED, "aborted at APPROACH (injected)")
        approach = self._servo_loop(
            PickState.APPROACH, settings.approach_tolerance_px, settings.max_approach_steps
        )
        if not approach.converged:
            return self._finish(
                PickState.FAILED, f"approach_{approach.reason}", approach_error_px=approach.error_px
            )

        # ---------------- DESCEND (OPEN LOOP -- see module docstring) ------
        if self._aborted(PickState.DESCEND):
            return self._finish(PickState.ABORTED, "aborted at DESCEND (injected)",
                                approach_error_px=approach.error_px)
        grasp_height = self._clamp_internal(height, self._current(height) + settings.descend_units)
        if grasp_height == self._current(height):
            return self._finish(PickState.FAILED, "descend_blocked_by_joint_limits",
                                approach_error_px=approach.error_px)
        self._move({height: grasp_height})
        self._note(PickState.DESCEND, "descended (open loop -- no depth sensing exists)",
                   joint=height, target=grasp_height, units=settings.descend_units)

        # Re-align at grasp height: the apparent position of the gripper shifts
        # when its height changes (any real camera has some parallax), so the
        # alignment achieved at travel height is not the alignment that matters.
        refine = self._servo_loop(PickState.DESCEND, settings.grasp_tolerance_px, settings.max_refine_steps)
        if not refine.converged:
            return self._finish(PickState.FAILED, f"refine_{refine.reason}",
                                approach_error_px=refine.error_px)

        # ---------------- GRASP (incremental ladder only) ------------------
        if self._aborted(PickState.GRASP):
            return self._finish(PickState.ABORTED, "aborted at GRASP (injected)", release=True,
                                approach_error_px=refine.error_px)
        grip_result = self.arm.close_grip(step=settings.grip_step, duration_ms=settings.grip_duration_ms)
        self._note(PickState.GRASP, "grip ladder finished", **{k: v for k, v in grip_result.items()})
        if grip_result.get("stopped_reason") != "no_motion_detected" or grip_result.get("contact_position") is None:
            # Reached the ladder's own backstop without ever touching anything:
            # the jaws closed on air. Not a grasp, and never treated as one.
            return self._finish(PickState.FAILED, "grasp_closed_on_nothing", release=True,
                                grip_result=grip_result, approach_error_px=refine.error_px)

        # ``Arm._grip_ladder`` unloads GRIP even on a successful contact, so the
        # jaws are OPEN-loop free by the time it returns -- close_grip() alone
        # does not hold anything. Re-apply torque at the contact position, with
        # hold=["GRIP"] so the safety watchdog's hold_ms timer does not drop it
        # mid-lift. guarded=False because pressing into a grasped object is
        # *supposed* to produce no visible motion; that is the definition of a
        # successful grasp, not a stall.
        contact = int(grip_result["contact_position"])
        self._move({"GRIP": contact}, hold=["GRIP"], guarded=False,
                   duration_ms=settings.grip_duration_ms)
        self._note(PickState.GRASP, "re-applied grip torque at the contact position",
                   position=contact)

        # ---------------- VERIFY -------------------------------------------
        if self._aborted(PickState.VERIFY):
            return self._finish(PickState.ABORTED, "aborted at VERIFY (injected)", release=True,
                                grip_result=grip_result)
        obj_before = self._validated_object()
        grip_px = self._validated_gripper()
        if obj_before is None:
            return self._finish(PickState.FAILED, "verify_object_not_found", release=True,
                                grip_result=grip_result)
        if grip_px is None:
            return self._finish(PickState.FAILED, "verify_gripper_not_found", release=True,
                                grip_result=grip_result)
        distance = math.hypot(obj_before.px - grip_px.px, obj_before.py - grip_px.py)
        self._note(PickState.VERIFY, "object/gripper separation after grasp",
                   distance_px=round(distance, 2), limit_px=settings.verify_max_distance_px)
        if distance > settings.verify_max_distance_px:
            # The ladder believed it touched something; the camera disagrees about
            # what. Never retry blindly -- release and stop.
            return self._finish(PickState.FAILED, "verify_object_not_at_gripper", release=True,
                                grip_result=grip_result)

        # ---------------- LIFT (OPEN LOOP) ---------------------------------
        if self._aborted(PickState.LIFT):
            return self._finish(PickState.ABORTED, "aborted at LIFT (injected)", release=True,
                                grip_result=grip_result)
        lift_target = self._clamp_internal(height, self._current(height) + settings.lift_units)
        if lift_target == self._current(height):
            return self._finish(PickState.FAILED, "lift_blocked_by_joint_limits", release=True,
                                grip_result=grip_result)
        # Command the servo joints to where they already are, purely so they can
        # be named in ``hold`` -- otherwise their hold_ms timers expire mid-lift
        # and the arm sags under the load it just picked up.
        held_names = [config.validate_joint_name(n) for n in settings.servo_joints] + [height, "GRIP"]
        lift_pose = {config.validate_joint_name(n): self._current(config.validate_joint_name(n))
                     for n in settings.servo_joints}
        lift_pose[height] = lift_target
        self._move(lift_pose, hold=held_names)
        self._note(PickState.LIFT, "lifted (open loop -- no depth sensing exists)",
                   joint=height, target=lift_target, units=settings.lift_units)

        # ---------------- VERIFY_HELD --------------------------------------
        if self._aborted(PickState.VERIFY_HELD):
            return self._finish(PickState.ABORTED, "aborted at VERIFY_HELD (injected)", release=True,
                                grip_result=grip_result)
        obj_after = self._validated_object()
        grip_after = self._validated_gripper()
        if obj_after is None:
            return self._finish(PickState.FAILED, "verify_held_object_not_found", release=True,
                                grip_result=grip_result)
        if grip_after is None:
            return self._finish(PickState.FAILED, "verify_held_gripper_not_found", release=True,
                                grip_result=grip_result)
        displacement = math.hypot(obj_after.px - obj_before.px, obj_after.py - obj_before.py)
        separation = math.hypot(obj_after.px - grip_after.px, obj_after.py - grip_after.py)
        self._note(PickState.VERIFY_HELD, "object movement during the lift",
                   displacement_px=round(displacement, 2), separation_px=round(separation, 2),
                   min_displacement_px=settings.min_lift_displacement_px,
                   max_separation_px=settings.held_max_distance_px)
        if displacement < settings.min_lift_displacement_px:
            return self._finish(PickState.FAILED, "verify_held_object_did_not_move", release=True,
                                grip_result=grip_result)
        if separation > settings.held_max_distance_px:
            return self._finish(PickState.FAILED, "verify_held_object_left_behind", release=True,
                                grip_result=grip_result)

        # ---------------- DONE ---------------------------------------------
        # A "successful" pick that immediately unloads has dropped the object, so
        # by default we leave torque on. That is not a safe steady state and is
        # not presented as one: the safety watchdog's max_continuous_torque_ms cap
        # (8 s by default) will force an unload shortly, and this arm therefore
        # cannot hold anything indefinitely. See docs/pick.md.
        self._note(PickState.DONE, "object held after lift")
        if settings.hold_on_success:
            self._note(PickState.DONE,
                       "leaving GRIP and the load-bearing joints under torque -- the "
                       "watchdog will force-unload them shortly",
                       max_continuous_torque_ms=self.arm._safety.max_continuous_torque_ms)
            unloaded, holding = False, True
        else:
            unloaded, holding = self._safe_stop(release=False), False
        return PickResult(
            success=True,
            state=PickState.DONE,
            reason="picked",
            locator=self.locator.describe(),
            log=list(self.log),
            grip_result=grip_result,
            approach_error_px=refine.error_px,
            elapsed_s=time.time() - self._started,
            unloaded=unloaded,
            holding=holding,
        )


def pick(
    arm,
    locator: Locator,
    settings: Optional[PickSettings] = None,
    *,
    allow_uncalibrated_override: bool = False,
    override_reason: Optional[str] = None,
) -> PickResult:
    """Run one pick attempt. See :class:`PickRoutine`."""
    return PickRoutine(
        arm,
        locator,
        settings,
        allow_uncalibrated_override=allow_uncalibrated_override,
        override_reason=override_reason,
    ).run()
