"""``MotionObserver``: "did the joint actually move?", as a swappable component.

There is no position feedback on this hardware (``protocol.NoResponse``), so the
only way to answer that question is to *watch* the arm. ``guard.VisionGuard``
already implements the frame-differencing that answers it for a real camera; this
module wraps that behind a narrow interface so the auto-calibration routine
(``calibrate.py``) can be exercised end-to-end against ``sim.SimTransport`` with
no camera, no ``cv2`` window, and no hardware.

The interface is deliberately two calls, not one:

* ``capture()`` -- take an opaque snapshot of the world *now*.
* ``observe(before, after)`` -- decide whether motion happened between two
  snapshots.

That split matters because the calibration routine must capture "before", apply a
bounded torque burst, unload, and only then capture "after". A single
``did_it_move()`` call would hide the unload step, which is the whole point of the
torque-burst discipline.

Implementations here:

* ``VisionMotionObserver`` -- the real one. Thin adapter over ``guard.VisionGuard``;
  it does not reimplement frame differencing, it delegates to
  ``VisionGuard.detect_motion``.
* ``SimMotionObserver`` -- renders a synthetic frame in which each joint's
  *actually achieved* ``sim.SimJoint.position`` is drawn as a bar, then feeds those
  frames through a real ``VisionGuard``. It is a **stand-in for a camera**, not a
  simulation of one: it has none of a real camera's noise, latency, motion blur,
  auto-exposure, or occlusion. It reads ground truth (where the simulated joint
  ended up), never intent (where we asked it to go), so a commanded-but-blocked
  move correctly renders as "no motion".
* ``FlakyMotionObserver`` / ``NullMotionObserver`` -- deliberate fault injectors for
  tests: an observer that occasionally misses real motion, and one that never sees
  anything at all (a lens cap, a dead camera, a joint outside the ROI).
* ``NoisyMotionObserver`` -- the mirror-image fault injector: an observer whose
  no-motion *noise floor* sits at or above its own "motion" threshold, so it reports
  motion on every frame pair regardless of whether the joint moved. This is the
  auto-exposure-hunting failure the existing consecutive-miss / reliability-burst
  net is structurally blind to (there are no misses to count), and the one
  ``camera_health`` exists to catch. See docs/verification-camera.md.

Two extra, defaulted methods sit alongside ``observe`` so a health gate can reason
about *how much* an observer sees, not only the yes/no answer:

* ``score(before, after) -> float`` -- a continuous motion magnitude. The default
  is a 0/1 projection of ``observe`` so every existing subclass keeps working
  untouched; the vision path overrides it with the real changed-pixel count.
* ``motion_threshold() -> float`` -- the score at/above which this observer calls a
  pair "motion". Default 1.0 (matching the 0/1 ``score``); the vision path returns
  its guard's ``min_changed_pixels``.
"""

from __future__ import annotations

import abc
import random
from typing import Any, Iterable, Optional, Sequence

import numpy as np

from . import config
from .guard import VisionGuard


class ObserverError(RuntimeError):
    """Raised when a motion observer cannot produce a usable answer.

    Callers must treat this as "I do not know whether the joint moved", which for
    the calibration routine means *abort*, never "assume it stopped" (that would
    silently record a mechanical limit that was never actually found).
    """


class MotionObserver(abc.ABC):
    """Answers "did the arm visibly move between these two snapshots?"."""

    @abc.abstractmethod
    def available(self) -> bool:
        """False when this observer cannot see anything at all (no camera)."""

    @abc.abstractmethod
    def capture(self) -> Any:
        """Snapshot the world now. Return value is opaque to callers."""

    @abc.abstractmethod
    def observe(self, before: Any, after: Any) -> bool:
        """True if motion is detected between two snapshots from ``capture()``."""

    def score(self, before: Any, after: Any) -> float:
        """Continuous motion magnitude between two snapshots.

        Default: a coarse 0/1 projection of ``observe`` so every existing observer
        keeps working with no change. The real-camera path overrides this with the
        guard's changed-pixel count, which is what ``camera_health`` needs to
        measure a no-motion *noise floor* (a boolean cannot express "reported motion
        but only barely" vs "reported motion by a mile").
        """
        return 1.0 if self.observe(before, after) else 0.0

    def motion_threshold(self) -> float:
        """The ``score`` at or above which this observer calls a pair "motion".

        Default 1.0, consistent with the 0/1 default ``score``. The vision path
        overrides it with the guard's ``min_changed_pixels`` so the health gate can
        compare a measured noise floor against the *actual* decision boundary.
        """
        return 1.0

    def describe(self) -> str:
        return type(self).__name__


class VisionMotionObserver(MotionObserver):
    """Real-camera observer: a thin adapter over ``guard.VisionGuard``.

    All the actual detection (ROI crop, absdiff, blur, threshold, changed-pixel
    count) stays in ``VisionGuard`` -- this class only reshapes its API into the
    capture/observe pair and converts "camera unavailable / capture failed" into a
    loud ``ObserverError`` instead of ``VisionGuard.check``'s deliberate no-op.
    A no-op is the right degrade-safely behaviour for a one-shot move guarded by
    layers 1+2; it is the *wrong* behaviour for calibration, where a missing
    answer must abort the run.
    """

    def __init__(self, guard: VisionGuard) -> None:
        self._guard = guard

    @property
    def guard(self) -> VisionGuard:
        return self._guard

    def available(self) -> bool:
        return self._guard.available()

    def capture(self) -> np.ndarray:
        if not self._guard.available():
            raise ObserverError(
                "vision observer has no frame source (no camera opened) -- refusing "
                "to guess whether the joint moved."
            )
        frame = self._guard.capture()
        if frame is None:
            raise ObserverError("camera returned no frame")
        return frame

    def observe(self, before: np.ndarray, after: np.ndarray) -> bool:
        if before is None or after is None:
            raise ObserverError("cannot compare a missing frame")
        return bool(self._guard.detect_motion(before, after))

    def score(self, before: np.ndarray, after: np.ndarray) -> float:
        """Real continuous magnitude: the guard's changed-pixel count for this pair.

        This is what makes a no-motion noise floor measurable -- ``camera_health``
        samples this across passive frame pairs and compares its p95 against
        ``motion_threshold`` to decide whether the observer can tell a real move
        from its own noise.
        """
        if before is None or after is None:
            raise ObserverError("cannot compare a missing frame")
        return float(self._guard.motion_score(before, after))

    def motion_threshold(self) -> float:
        return float(self._guard.min_changed_pixels)

    def describe(self) -> str:
        return (
            f"VisionMotionObserver(roi={self._guard.roi}, "
            f"min_changed_pixels={self._guard.min_changed_pixels})"
        )


# Synthetic-frame geometry. One horizontal band per rendered servo; each band
# contains a single bright bar whose x position is that joint's achieved position.
# 1 px per raw unit keeps even a 1-unit residual travel visible (a 1 px shift of an
# 8 px bar changes 2 * BAND_HEIGHT pixels, comfortably above MIN_CHANGED_PIXELS),
# which matters because a joint creeping the last few units into its end stop must
# still read as "moved".
_BAND_HEIGHT = 24
_BAR_WIDTH = 8
_PIXELS_PER_UNIT = 1.0
_MIN_CHANGED_PIXELS = 12


class SimMotionObserver(VisionMotionObserver):
    """Camera stand-in for ``sim.SimTransport``. **Not** a camera simulation.

    Renders each simulated joint's achieved ``SimJoint.position`` as a bar and runs
    the result through a genuine ``VisionGuard``, so the decision logic under test
    is the same frame-differencing code the real camera path uses. What it does
    *not* model: sensor noise, lighting change, motion blur, occlusion, ROI
    mis-framing, rolling shutter, or a joint whose motion is nearly invisible from
    the camera's angle. Every one of those is a real way the real observer can be
    wrong (see docs/calibration.md); none of them are reproduced here.

    Ground truth only: it reads where the simulated servo *ended up*, never the
    target it was asked to reach. A command that the simulated mechanical stop
    blocked therefore renders as an unchanged frame, exactly as a camera would see.
    """

    def __init__(
        self,
        transport,
        servo_ids: Optional[Iterable[int]] = None,
        *,
        pixels_per_unit: float = _PIXELS_PER_UNIT,
        min_changed_pixels: int = _MIN_CHANGED_PIXELS,
    ) -> None:
        self._transport = transport
        ids = sorted(servo_ids) if servo_ids is not None else sorted(transport.joints)
        self._servo_ids: Sequence[int] = list(ids)
        if not self._servo_ids:
            raise ObserverError("SimMotionObserver needs at least one servo to watch")
        self._ppu = float(pixels_per_unit)
        spans = []
        for servo_id in self._servo_ids:
            lo, hi = config.ELECTRICAL_LIMITS[servo_id]
            spans.append(hi - lo)
        self._width = int(max(spans) * self._ppu) + _BAR_WIDTH + 2
        self._height = _BAND_HEIGHT * len(self._servo_ids)
        super().__init__(
            VisionGuard(
                frame_source=self._render,
                roi=None,
                diff_threshold=25,
                min_changed_pixels=min_changed_pixels,
                # The synthetic frame is noise-free by construction, so blurring
                # would only smear away small-but-real single-pixel motion.
                blur_ksize=1,
            )
        )

    def _render(self) -> np.ndarray:
        frame = np.zeros((self._height, self._width, 3), dtype=np.uint8)
        for row, servo_id in enumerate(self._servo_ids):
            joint = self._transport.joints[servo_id]
            lo, _hi = config.ELECTRICAL_LIMITS[servo_id]
            x = int(round((joint.position - lo) * self._ppu)) + 1
            x = max(0, min(x, self._width - _BAR_WIDTH))
            y = row * _BAND_HEIGHT
            frame[y : y + _BAND_HEIGHT, x : x + _BAR_WIDTH] = 255
        return frame

    def describe(self) -> str:
        return f"SimMotionObserver(servos={list(self._servo_ids)}, SYNTHETIC -- not a real camera)"


class FlakyMotionObserver(MotionObserver):
    """Wraps another observer and drops real motion reports on purpose.

    Fault injector for tests and for reasoning about the real failure mode: a
    camera that intermittently misses genuine motion (a passing shadow, a frame
    drop, a joint briefly leaving the ROI) looks *identical* to a mechanical stop.
    Reports are only ever flipped True -> False; it never invents motion that did
    not happen, because the dangerous direction for calibration is a false "it
    stopped", not a false "it moved".

    ``miss_every=N`` forces every Nth observation to report no motion
    (deterministic, for reproducible tests). ``miss_probability`` does the same
    randomly from a seeded RNG.
    """

    def __init__(
        self,
        inner: MotionObserver,
        *,
        miss_every: Optional[int] = None,
        miss_probability: float = 0.0,
        seed: int = 0,
    ) -> None:
        if miss_every is not None and (not isinstance(miss_every, int) or miss_every < 1):
            raise ValueError(f"miss_every must be a positive int or None, got {miss_every!r}")
        if not 0.0 <= miss_probability <= 1.0:
            raise ValueError(f"miss_probability must be in [0, 1], got {miss_probability!r}")
        self._inner = inner
        self._miss_every = miss_every
        self._miss_probability = miss_probability
        self._rng = random.Random(seed)
        self.calls = 0
        self.injected_misses = 0

    def available(self) -> bool:
        return self._inner.available()

    def capture(self) -> Any:
        return self._inner.capture()

    def observe(self, before: Any, after: Any) -> bool:
        moved = self._inner.observe(before, after)
        self.calls += 1
        if not moved:
            return False
        if self._miss_every is not None and self.calls % self._miss_every == 0:
            self.injected_misses += 1
            return False
        if self._miss_probability and self._rng.random() < self._miss_probability:
            self.injected_misses += 1
            return False
        return True

    def describe(self) -> str:
        return (
            f"FlakyMotionObserver(inner={self._inner.describe()}, "
            f"miss_every={self._miss_every}, miss_probability={self._miss_probability})"
        )


class NullMotionObserver(MotionObserver):
    """An observer that is technically "available" but never sees anything.

    Models a lens cap, a dead sensor that still returns frames, a completely
    mis-aimed ROI, or a joint that simply is not in shot. This must never be
    mistaken for "every joint is at its mechanical stop", which is why the
    calibration routine cross-checks its observer before believing a stop.
    """

    def __init__(self) -> None:
        self.calls = 0

    def available(self) -> bool:
        return True

    def capture(self) -> Any:
        return None

    def observe(self, before: Any, after: Any) -> bool:
        self.calls += 1
        return False

    def describe(self) -> str:
        return "NullMotionObserver(sees nothing, ever)"


class NoisyMotionObserver(MotionObserver):
    """The mirror image of ``NullMotionObserver``: it sees "motion" *everywhere*.

    Fault injector for the exact defect ``camera_health`` exists to catch. On a real
    camera that cannot lock exposure (macOS AVFoundation, dark scene), auto-exposure
    hunts continuously, so a raw changed-pixel count on two frames taken with **zero
    commanded motion** can exceed the count from a genuine joint move, pushing the
    observer's noise floor *above* its own ``min_changed_pixels`` threshold so it
    answers "moved" to every pair.

    (The originally-reported "6-12x / 268k-550k px vs ~46k" magnitudes were inflated
    by a since-fixed capture-staleness bug in ``camera.Camera.grab`` -- see
    docs/verification-camera.md; with capture fixed the raw+ROI no-motion floor is
    ~30 px. This injector is deliberately magnitude-agnostic: it reproduces the
    *shape* of the fault -- noise floor >= threshold, independent of real motion --
    which is what makes it invisible to a boolean-only sensitivity check, not any
    particular pixel count.)

    Why this is dangerous and why the existing net misses it: ``calibrate.py`` only
    runs its observer-reliability burst after ``confirmations`` *consecutive
    no-motion* reports. An observer that always reports motion never produces a
    single miss, so ``consecutive_misses`` never accumulates, the reliability burst
    is never triggered, and every search step reads ``moved=True`` -- the routine
    walks straight to the absolute rail on every joint, reproducing the incident.

    This class reports ``score`` from a synthetic noise-floor distribution that is
    *independent of whether the arm actually moved* -- exactly the property that
    makes the real fault invisible to a boolean-only sensitivity check. With
    ``noise_floor`` at or above ``threshold`` (the default), ``observe`` is True on
    every pair. Set ``noise_floor`` below ``threshold`` to model a merely-degraded
    (but still passing) camera.
    """

    def __init__(
        self,
        *,
        noise_floor: float,
        threshold: float = float(config.VISION_MIN_CHANGED_PIXELS),
        jitter: float = 0.0,
        seed: int = 0,
    ) -> None:
        if noise_floor < 0 or threshold < 0 or jitter < 0:
            raise ValueError("noise_floor, threshold and jitter must be non-negative")
        self._noise_floor = float(noise_floor)
        self._threshold = float(threshold)
        self._jitter = float(jitter)
        self._rng = random.Random(seed)
        self.calls = 0

    def available(self) -> bool:
        return True

    def capture(self) -> Any:
        # Opaque, monotonic token: the content is irrelevant because score/observe
        # deliberately ignore it -- that independence from real motion IS the fault.
        self.calls += 1
        return self.calls

    def score(self, before: Any, after: Any) -> float:
        noise = self._noise_floor
        if self._jitter:
            noise += self._rng.uniform(-self._jitter, self._jitter)
        return max(0.0, noise)

    def observe(self, before: Any, after: Any) -> bool:
        return self.score(before, after) >= self._threshold

    def motion_threshold(self) -> float:
        return self._threshold

    def describe(self) -> str:
        return (
            f"NoisyMotionObserver(noise_floor={self._noise_floor}, "
            f"threshold={self._threshold}, jitter={self._jitter})"
        )


def observer_for_arm(arm, *, servo_ids: Optional[Iterable[int]] = None) -> MotionObserver:
    """Best available observer for ``arm``: the sim stand-in, or its camera guard.

    Raises ``ObserverError`` if neither is usable -- calibration without any way to
    tell whether a joint moved is not a degraded mode, it is a nonsense operation.
    """
    if getattr(arm, "sim", False):
        return SimMotionObserver(arm._transport, servo_ids=servo_ids)
    guard = getattr(arm, "vision_guard", None)
    if guard is None or not guard.available():
        raise ObserverError(
            "no camera available -- auto-calibration needs a motion observer to tell "
            "whether a commanded step actually moved the joint. Layers 1+2 alone "
            "cannot answer that (there is no position feedback on this controller). "
            "Attach a camera, or calibrate by hand (docs/safety.md)."
        )
    return VisionMotionObserver(guard)
