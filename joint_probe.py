#!/usr/bin/env python3
"""Per-joint responsiveness probe -- does this servo actually move?

Answers one question per joint, using the only two signals this arm has:

* **vision** -- ``VisionGuard.motion_score`` between a before and after frame;
* **battery sag** -- a servo under load pulls the pack down; a failed-open
  servo (no coil current) does not.

The motion score is deliberately computed with the *project's own* guard,
configured exactly as ``robohand`` configures it. That matters: the noise floor
you compare against (``robohand camera-check`` reports a p95) is in these
units. A hand-rolled ``absdiff`` would produce a number on a different scale and
quietly invite a wrong conclusion.

Neither signal alone is conclusive. A blind camera reports "no motion" for a
perfectly good joint, and a joint that is barely loaded hardly sags the pack.
Read them together, and always against a joint you already trust.

Safety, in order of how much it matters:

1. Every target is clamped into the joint's *effective* band and then into its
   absolute rail. Nothing here can command a joint past its configured limit.
2. Moves are slow (``--duration-ms``, default 2000) so a joint that starts far
   from its target eases into it instead of snapping.
3. Servos are **relaxed at the end of every run**, in a ``finally``, including
   on Ctrl-C. Releasing torque is also what restarts the torque watchdog clock.
4. The pack voltage is checked before every leg and the run aborts if it has
   collapsed -- a sagging pack under a stalled servo is the exact signature
   that preceded this arm's gripper failure.

This probe deliberately does NOT print a "dead"/"alive" verdict. The two
failure modes it cannot separate -- a dead joint, and a camera that simply
cannot see that joint -- look identical in the pixel column. The sag column is
what separates them, and a human should make that call.

Usage:
    python3 joint_probe.py --joints SHOULDER_PAN SHOULDER_LIFT
"""
from __future__ import annotations

import argparse
import sys
import time
from typing import Dict, List, Optional

from robohand import config
from robohand.arm import Arm
from robohand.camera import Camera, CameraConfig
from robohand.guard import VisionGuard

#: Half-amplitude of the probe excursion, raw servo units. Small on purpose:
#: this is a "did it twitch" test, not a range-finding sweep. It sits well
#: inside every provisional band, so the clamp below is a backstop rather than
#: the thing actually choosing the target.
DEFAULT_DELTA = 120

#: Below this the pack is too flat to trust any reading, and commanding more
#: torque into it risks the brownout-then-stall pattern that has already
#: destroyed one servo on this arm. ESTIMATE: 2S LiPo nominal 7.4 V; this arm
#: idles around 7.44 V.
MIN_SAFE_VOLTS = 6.6


def _build_guard(cam: Camera) -> VisionGuard:
    """Exactly the guard ``robohand`` builds itself -- same ROI, same threshold,
    same normalisation -- so scores are comparable to ``camera-check`` output."""
    return VisionGuard(
        frame_source=cam.grab,
        roi=config.VISION_ROI,
        diff_threshold=config.VISION_DIFF_THRESHOLD,
        min_changed_pixels=config.VISION_MIN_CHANGED_PIXELS,
        blur_ksize=config.VISION_BLUR_KSIZE,
        normalize=config.VISION_NORMALIZE,
        normalized_threshold=config.VISION_NORMALIZED_THRESHOLD,
    )


def _clamp(joint: str, target: int) -> int:
    """Clamp into the effective band, then the absolute rail. Never widens."""
    lo, hi = config.effective_bounds(joint)
    abs_lo, abs_hi = config.absolute_bounds(joint)
    return max(max(lo, abs_lo), min(min(hi, abs_hi), target))


def _probe_joint(
    arm: Arm,
    guard: VisionGuard,
    joint: str,
    delta: int,
    duration_ms: int,
    settle_s: float,
) -> Dict[str, object]:
    lo, hi = config.effective_bounds(joint)
    centre = (lo + hi) // 2
    down = _clamp(joint, centre - delta)
    up = _clamp(joint, centre + delta)

    result: Dict[str, object] = {
        "joint": joint,
        "band": [lo, hi],
        "centre": centre,
        "targets": [down, up],
        "legs": [],
        "error": None,
    }

    if down == up:
        result["error"] = (
            f"band [{lo}, {hi}] is too narrow for a +/-{delta} probe; both legs "
            f"clamp to {down}. Not probing -- it would prove nothing."
        )
        return result

    # Start from centre so the excursion is symmetric and known-reachable.
    arm.set_positions({joint: centre}, duration_ms=duration_ms, wait=True)
    time.sleep(settle_s)

    for tag, target in (("down", down), ("up", up)):
        v_before = arm.battery_voltage()
        if v_before is not None and v_before < MIN_SAFE_VOLTS:
            result["error"] = (
                f"pack at {v_before:.3f} V is below the {MIN_SAFE_VOLTS} V floor "
                f"before the {tag} leg. Refusing to command more torque."
            )
            return result

        before = guard.capture()
        arm.set_positions({joint: target}, duration_ms=duration_ms, wait=True)
        time.sleep(settle_s)
        after = guard.capture()
        v_after = arm.battery_voltage()

        score: Optional[int] = None
        if before is not None and after is not None:
            score = guard.motion_score(before, after)

        sag = None
        if v_before is not None and v_after is not None:
            sag = round(v_before - v_after, 4)

        result["legs"].append(
            {
                "leg": tag,
                "target": target,
                "score": score,
                "v_before": v_before,
                "v_after": v_after,
                "sag_v": sag,
            }
        )
        # Back to centre between legs so each leg starts from the same place.
        arm.set_positions({joint: centre}, duration_ms=duration_ms, wait=True)
        time.sleep(settle_s)

    return result


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--joints", nargs="+", required=True, help="Joint names to probe, in order.")
    ap.add_argument("--delta", type=int, default=DEFAULT_DELTA, help=f"Half-amplitude, raw units (default {DEFAULT_DELTA}).")
    ap.add_argument("--duration-ms", type=int, default=2000, help="Move duration; longer is gentler (default 2000).")
    ap.add_argument("--settle-s", type=float, default=0.6, help="Pause after each move before capturing (default 0.6).")
    args = ap.parse_args()

    for joint in args.joints:
        if joint not in config.JOINT_NAMES:
            print(f"unknown joint {joint!r}; known: {sorted(config.JOINT_NAMES)}", file=sys.stderr)
            return 2

    results: List[Dict[str, object]] = []
    cam = Camera(CameraConfig(index=0))
    cam.open()
    # use_vision=False: the arm's own guard would abort a move it judges as
    # "no motion", which is precisely the condition being diagnosed. This probe
    # measures instead of aborting; the voltage floor and the small delta are
    # what keep it safe.
    arm = Arm(use_vision=False)
    try:
        guard = _build_guard(cam)
        v0 = arm.battery_voltage()
        print(f"pack before any motion: {v0:.3f} V" if v0 is not None else "pack voltage unreadable")
        if v0 is not None and v0 < MIN_SAFE_VOLTS:
            print(f"ABORT: {v0:.3f} V is below the {MIN_SAFE_VOLTS} V floor.", file=sys.stderr)
            return 1

        for joint in args.joints:
            print(f"\n--- probing {joint} ---", flush=True)
            res = _probe_joint(arm, guard, joint, args.delta, args.duration_ms, args.settle_s)
            results.append(res)
            if res["error"]:
                print(f"  {res['error']}")
                continue
            print(f"  band {res['band']}  centre {res['centre']}  targets {res['targets']}")
            for leg in res["legs"]:  # type: ignore[union-attr]
                sag = leg["sag_v"]
                sag_s = f"{sag:+.4f} V" if sag is not None else "n/a"
                score = leg["score"]
                score_s = "n/a" if score is None else f"{score:>8}"
                print(f"  {leg['leg']:>4} -> {leg['target']:>5}   {score_s} px   sag {sag_s}")
    finally:
        # Relax whatever happened, including on Ctrl-C. Also restarts the
        # torque watchdog clock.
        try:
            arm.relax()
            print("\nall servos relaxed (torque OFF)")
        finally:
            arm.close()
            cam.release()

    print("\n=== summary ===")
    print(f"{'joint':<15}{'weakest leg':>14}{'largest sag':>14}")
    for res in results:
        if res["error"]:
            print(f"{res['joint']:<15} NOT PROBED -- {res['error']}")
            continue
        legs = res["legs"]  # type: ignore[assignment]
        scores = [leg["score"] for leg in legs if leg["score"] is not None]  # type: ignore[index]
        sags = [leg["sag_v"] for leg in legs if leg["sag_v"] is not None]  # type: ignore[index]
        worst = min(scores) if scores else None
        max_sag = max(sags) if sags else None
        worst_s = "n/a" if worst is None else f"{worst} px"
        sag_s = "n/a" if max_sag is None else f"{max_sag:+.4f} V"
        print(f"{res['joint']:<15}{worst_s:>14}{sag_s:>14}")
    print(
        "\nNo verdict is printed on purpose. A dead joint and a camera that cannot\n"
        "see that joint both produce a low pixel count; the sag column is what\n"
        "separates them. Compare every joint against a known-good one, and compare\n"
        "the pixel counts against the noise floor from 'robohand camera-check'."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
