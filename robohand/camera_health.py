"""Layer-3 pre-flight: is the camera actually able to tell motion from its own noise?

Why this module exists
-----------------------
``guard.VisionGuard`` answers "did the joint move between these two frames?" by
counting changed pixels and comparing to ``min_changed_pixels``. That answer is only
meaningful if the camera's *no-motion* noise floor sits **below** that threshold. On
hardware that cannot lock exposure (macOS AVFoundation, dark scene) the assumption
fails silently and catastrophically: auto-exposure hunts continuously, so two frames
captured with **zero commanded motion** can differ by more changed pixels than a
genuine joint move. The observer then reports "motion" on *every* pair.

(The originally-reported magnitudes -- 268k-550k no-motion px vs ~46k for a real
move -- were additionally *inflated* by a since-fixed capture-staleness bug in
``camera.Camera.grab`` and should not be read as the stable noise floor; with
capture fixed the raw+ROI no-motion floor is ~30 changed px. The *failure mode*
here -- noise floor at or above the threshold in adverse lighting -- is real
regardless, and is exactly what this gate measures. See docs/verification-camera.md.)

This is the mirror image of a blind camera, and the existing safety net is
structurally blind to it. ``calibrate.py`` only runs its observer-reliability burst
after ``confirmations`` *consecutive no-motion* reports; an observer that always
reports motion never produces a miss, so that net never even fires, and the search
walks straight to the absolute rail on every joint -- exactly the 2026-07-28
incident, once per joint, on every run.

What the gate does
------------------
1. Noise floor (``assess_camera_health``): capture N frames from the observer with
   **no motion commanded**, score every consecutive pair, and take the p95. Require
   the observer's own motion threshold to sit above that p95 by a safety ``margin``.
   If it does not, the observer cannot distinguish a real move from its own noise:
   raise ``CameraUnusableError`` and refuse to proceed.

2. Visibility (``verify_motion_visibility``, optional): a pure noise-floor check
   cannot catch a camera pointed at a wall (near-zero noise *and* near-zero signal
   trivially "passes"). Given a *known-real* motion sample, require its score to
   clear the measured noise floor by a ``separation`` factor. This is a spot-check
   tied to whatever motion happens to occur, not an exhaustive per-joint visibility
   certification -- see the honest limits in docs/verification-camera.md.

Fail-closed vs degrade-safely
-----------------------------
Calibration calls ``require_*`` and **fails closed** (refuses, writes nothing): a
supervised search that infers mechanical stops from "no motion" is worthless if the
observer's "no motion" is a coin flip. A regular one-shot ``SafetyLayer.move`` treats
a *proven-unhealthy* camera the same as *no* camera (warn, keep layers 1+2) -- the
same degrade-safely convention ``guard.VisionGuard`` already uses for a missing
camera. See docs/safety.md.
"""

from __future__ import annotations

import dataclasses
import time
from typing import Any, Callable, List, Optional, Tuple

import numpy as np

from . import config
from .observer import MotionObserver, ObserverError, VisionMotionObserver

Frame = Any
MotionProbe = Callable[[], Tuple[Frame, Frame]]


class CameraUnusableError(ObserverError):
    """Raised when the camera/observer cannot reliably tell motion from its own noise.

    A subclass of ``ObserverError`` on purpose: every place that already treats "I
    cannot answer 'did it move'" as fatal for calibration (``calibrate.py``'s
    ``except ObserverError`` branches, ``cli.main``'s handler) propagates this the
    same way, so no call site has to learn a new exception to stay fail-closed.
    """


@dataclasses.dataclass
class CameraHealthReport:
    """The measured verdict. Serialisable for ``--json`` output."""

    available: bool
    healthy: bool
    frames: int
    interval_s: float
    pairs: int
    noise_p50: float
    noise_p95: float
    noise_max: float
    threshold: float
    margin: float
    required_threshold: float  # threshold must be >= this (== noise_p95 * margin)
    normalize: str = "none"
    # threshold / max(noise_p95, epsilon): how much of the required margin we
    # actually have against the *live* no-motion floor. Needs no motion sample.
    headroom: Optional[float] = None
    separation: Optional[float] = None
    motion_score: Optional[float] = None
    # motion_score / max(noise_p95, epsilon): the *measured* separation of a known
    # real move from the live noise floor. Only set when a motion_probe is supplied,
    # and deliberately distinct from ``separation`` (the required factor) and from
    # ``headroom`` (which needs no motion sample) -- they answer different questions.
    motion_separation: Optional[float] = None
    separation_floor: Optional[float] = None  # motion_score must be >= this
    separation_ok: Optional[bool] = None
    skipped: bool = False
    reason: str = ""

    def as_dict(self) -> dict:
        return dataclasses.asdict(self)


# Floor for the noise p95 when computing ratios, so a genuinely noise-free observer
# (p95 == 0, e.g. the synthetic sim camera) yields a large-but-finite headroom
# instead of a division by zero. 1.0 == "at least one changed pixel of noise".
_NOISE_EPSILON = 1.0


def _percentile(values: List[float], pct: float) -> float:
    if not values:
        return 0.0
    return float(np.percentile(np.asarray(values, dtype=float), pct))


def _observer_normalize(observer: MotionObserver) -> str:
    guard = getattr(observer, "guard", None)
    return getattr(guard, "normalize", "none") if guard is not None else "none"


def sample_noise_floor(
    observer: MotionObserver,
    *,
    frames: int,
    interval_s: float,
    percentile: float = 95.0,
    sleep: Callable[[float], None] = time.sleep,
) -> Tuple[List[float], float, float, float, int]:
    """Capture ``frames`` passive snapshots and score every consecutive pair.

    **No motion is commanded** -- this measures the observer's own noise. Consecutive
    (not all-)pairs are used deliberately: a real move is judged from one frame and
    the very next one, so consecutive pairs spaced by ``interval_s`` sample exactly
    the noise a real before/after check would see, including a full auto-exposure
    hunting cycle when ``interval_s`` spans one.

    Returns ``(scores, p50, p95, max, pair_count)``.
    """
    if frames < 2:
        raise ValueError("need at least 2 frames to form a no-motion pair")
    snaps: List[Frame] = []
    for i in range(frames):
        snaps.append(observer.capture())
        if interval_s > 0 and i < frames - 1:
            sleep(interval_s)
    scores = [float(observer.score(snaps[i], snaps[i + 1])) for i in range(len(snaps) - 1)]
    p50 = _percentile(scores, 50.0)
    p95 = _percentile(scores, percentile)
    mx = float(max(scores)) if scores else 0.0
    return scores, p50, p95, mx, len(scores)


def assess_camera_health(
    observer: MotionObserver,
    *,
    frames: int = config.VISION_HEALTH_FRAMES,
    interval_s: float = config.VISION_HEALTH_INTERVAL_S,
    margin: float = config.VISION_HEALTH_MARGIN,
    separation: float = config.VISION_HEALTH_SEPARATION,
    percentile: float = 95.0,
    motion_probe: Optional[MotionProbe] = None,
    sleep: Callable[[float], None] = time.sleep,
) -> CameraHealthReport:
    """Measure the observer's no-motion noise floor (and optionally visibility).

    ``margin`` is the safety factor the detection threshold must clear the p95 noise
    by. ``separation`` (only used when ``motion_probe`` is supplied) is the factor a
    known-motion sample must clear the noise floor by. Never commands motion of its
    own; if ``motion_probe`` is given it is expected to return a before/after pair
    around a move the caller is already performing for another reason.
    """
    if not observer.available():
        return CameraHealthReport(
            available=False, healthy=False, frames=0, interval_s=interval_s, pairs=0,
            noise_p50=0.0, noise_p95=0.0, noise_max=0.0,
            threshold=float(observer.motion_threshold()), margin=margin,
            required_threshold=0.0, normalize=_observer_normalize(observer),
            skipped=True, reason="observer reports it is unavailable (no camera).",
        )

    scores, p50, p95, mx, pairs = sample_noise_floor(
        observer, frames=frames, interval_s=interval_s, percentile=percentile, sleep=sleep
    )
    threshold = float(observer.motion_threshold())
    required = p95 * margin
    healthy = threshold >= required

    report = CameraHealthReport(
        available=True, healthy=healthy, frames=frames, interval_s=interval_s, pairs=pairs,
        noise_p50=p50, noise_p95=p95, noise_max=mx, threshold=threshold, margin=margin,
        required_threshold=required, normalize=_observer_normalize(observer),
        headroom=threshold / max(p95, _NOISE_EPSILON),
    )
    if not healthy:
        report.reason = (
            f"no-motion noise floor p95={p95:.0f} changed px is too close to the "
            f"motion threshold {threshold:.0f} (need threshold >= {required:.0f}, i.e. "
            f"{margin:g}x the p95 noise). The camera cannot tell a real move from its "
            "own noise -- most likely auto-exposure hunting in a dark/low-contrast "
            "scene. Improve lighting, lock exposure, re-aim the ROI, or enable the "
            "brightness-normalised metric (config vision.normalize)."
        )

    if motion_probe is not None:
        report.separation = separation
        before, after = motion_probe()
        motion_score, floor, ok = motion_clears_floor(
            observer, before, after, report, separation=separation
        )
        report.motion_score = motion_score
        report.motion_separation = motion_score / max(p95, _NOISE_EPSILON)
        report.separation_floor = floor
        report.separation_ok = ok
        if not report.separation_ok:
            report.healthy = False
            report.reason = (
                f"a known-real move scored only {motion_score:.0f} changed px, which "
                f"does not clearly exceed the noise floor (need >= {floor:.0f} = "
                f"max(threshold {threshold:.0f}, {separation:g}x p95 noise {p95:.0f})). "
                "The camera cannot see this motion -- it may be pointed away from the "
                "arm, occluded, or framing a region the joint does not move within."
            )

    return report


def require_camera_health(observer: MotionObserver, **kwargs: Any) -> CameraHealthReport:
    """Assess and raise ``CameraUnusableError`` if the observer is not healthy.

    Returns the report on success so the caller can reuse the measured noise floor
    (e.g. as the baseline for a later visibility spot-check) without re-sampling.
    """
    report = assess_camera_health(observer, **kwargs)
    if report.skipped:
        # "Unavailable" is a different failure (no camera) handled by the caller's
        # own availability check; do not conflate it with "present but unusable".
        return report
    if not report.healthy:
        raise CameraUnusableError(report.reason)
    return report


def motion_clears_floor(
    observer: MotionObserver,
    before: Frame,
    after: Frame,
    report: CameraHealthReport,
    *,
    separation: float = config.VISION_HEALTH_SEPARATION,
) -> Tuple[float, float, bool]:
    """Score a known-real move and test it against the visibility floor.

    The single, exception-free primitive behind BOTH the camera-health gate's
    visibility spot-check (``verify_motion_visibility`` -> ``CameraUnusableError``)
    and ``calibrate.py``'s per-joint liveness check (-> ``JointUnresponsive``).
    Returns ``(score, required_floor, ok)`` where

        required_floor = max(observer.motion_threshold(), report.noise_p95 * separation)

    -- the *same* floor ``assess_camera_health`` applies to its own motion probe,
    so all three sites agree on what "clearly seen" means -- and ``ok`` is
    ``score >= required_floor``. Raises nothing: an unmet floor means different
    things to different callers (an unusable camera vs a dead actuator), and each
    decides for itself. ``separation`` defaults to ``config.VISION_HEALTH_SEPARATION``
    (3.0): a real move must clear the live noise floor by 3x to count as seen.
    """
    score = float(observer.score(before, after))
    floor = max(float(observer.motion_threshold()), report.noise_p95 * separation)
    return score, floor, score >= floor


def verify_motion_visibility(
    observer: MotionObserver,
    before: Frame,
    after: Frame,
    report: CameraHealthReport,
    *,
    separation: float = config.VISION_HEALTH_SEPARATION,
    context: str = "",
) -> Optional[float]:
    """Spot-check that a *known-real* move is clearly visible above the noise floor.

    Uses the noise floor already measured in ``report`` (no re-sampling). Returns the
    move's score, or ``None`` if the check was skipped.

    Deliberately a no-op when ``report.noise_p95 <= 0``: with no measurable noise
    baseline there is nothing to separate a real move *from*, and the fully-blind /
    zero-noise degenerate case (lens cap, a wall with no sensor noise, a joint
    outside the ROI) is already covered -- and covered with a more specific
    diagnosis -- by ``calibrate.py``'s consecutive-miss + observer-reliability burst.
    Firing here on p95==0 would only duplicate that protection with a worse message,
    and would misclassify a genuinely noise-free synthetic observer as unusable.
    """
    if report.noise_p95 <= 0:
        return None
    score, floor, ok = motion_clears_floor(observer, before, after, report, separation=separation)
    if not ok:
        raise CameraUnusableError(
            f"{context + ': ' if context else ''}a known-real move scored only "
            f"{score:.0f} changed px, below the visibility floor of {floor:.0f} "
            f"(max(threshold {observer.motion_threshold():.0f}, {separation:g}x p95 "
            f"noise {report.noise_p95:.0f})). The camera cannot see this joint move "
            "-- refusing to infer a mechanical stop from 'no motion' reports it "
            "clearly cannot produce reliably."
        )
    return score


# -- convenience wrappers ----------------------------------------------------


def assess_guard_health(guard, **kwargs: Any) -> CameraHealthReport:
    """Assess a bare ``guard.VisionGuard`` by wrapping it in a ``VisionMotionObserver``.

    Used by ``safety.SafetyLayer`` (which holds a guard, not an observer) for its
    lazy, cached, degrade-safely health screen.
    """
    return assess_camera_health(VisionMotionObserver(guard), **kwargs)
