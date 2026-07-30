#!/usr/bin/env python3
"""Find an arm pose that puts the gripper inside the camera's field of view.

Why this exists: vision is this arm's only sensor, and several things we need
(calibrating WRIST_ROLL, verifying a grasp, checking a pour) are judged by
looking at the *gripper*. At the home pose every joint sits at 1500 and the arm
points straight up -- the wrist and gripper are above the top edge of the frame.
Any vision check aimed at them then reports "no motion" on hardware that is
working perfectly, which is indistinguishable from a dead joint.

So: sweep a few candidate shoulder/elbow poses, snapshot each, and report how
much of the *upper* frame each one fills, as a cheap proxy for "did the arm come
down into view". Then look at the contact sheet and pick one.

Safety: clamped targets, slow moves, pack voltage floor, unconditional relax.
Poses are kept well inside the provisional bands -- this sweep is not trying to
find limits, only to find a viewpoint.

Usage:
    python3 pose_finder.py
"""
from __future__ import annotations

import argparse
import sys
import time
from typing import Dict, List, Tuple

import cv2
import numpy as np

from robohand import config
from robohand.arm import Arm
from robohand.camera import Camera, CameraConfig

MIN_SAFE_VOLTS = 6.6

#: Candidate (SHOULDER_LIFT, ELBOW) pairs. Kept inside the provisional band and
#: spread across it, so at least one should bring the gripper down into frame.
DEFAULT_POSES: List[Tuple[int, int]] = [
    (1500, 1500),
    (1350, 1350),
    (1350, 1650),
    (1650, 1350),
    (1250, 1500),
    (1500, 1250),
    (1250, 1250),
]


def _clamp(joint: str, target: int) -> int:
    lo, hi = config.effective_bounds(joint)
    alo, ahi = config.absolute_bounds(joint)
    return max(max(lo, alo), min(min(hi, ahi), target))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--duration-ms", type=int, default=1800)
    ap.add_argument("--settle-s", type=float, default=0.8)
    ap.add_argument("--out", default="/tmp/pose_finder")
    args = ap.parse_args()

    cam = Camera(CameraConfig(index=0))
    cam.open()
    arm = Arm(use_vision=False)
    shots: List[Tuple[str, np.ndarray]] = []
    try:
        v0 = arm.battery_voltage()
        print(f"pack: {v0:.3f} V")
        if v0 is not None and v0 < MIN_SAFE_VOLTS:
            print("ABORT: pack below floor", file=sys.stderr)
            return 1

        # A reference frame with the arm parked, to diff each pose against.
        ref = cam.grab()
        h, w = ref.shape[:2]

        print(f"\n{'pose (lift, elbow)':<22}{'upper-frame change':>20}")
        for lift, elbow in DEFAULT_POSES:
            tgt: Dict[str, int] = {
                "SHOULDER_LIFT": _clamp("SHOULDER_LIFT", lift),
                "ELBOW": _clamp("ELBOW", elbow),
            }
            arm.set_positions(tgt, duration_ms=args.duration_ms, wait=True)
            time.sleep(args.settle_s)
            frame = cam.grab()

            # Proxy for "the arm moved into the top of the frame": changed pixels
            # in the upper half, relative to the parked reference.
            gb = cv2.cvtColor(ref[: h // 2], cv2.COLOR_BGR2GRAY)
            ga = cv2.cvtColor(frame[: h // 2], cv2.COLOR_BGR2GRAY)
            changed = int((cv2.absdiff(gb, ga) > 25).sum())

            label = f"{tgt['SHOULDER_LIFT']},{tgt['ELBOW']}"
            print(f"{label:<22}{changed:>20}")
            tagged = frame.copy()
            cv2.putText(tagged, label, (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 1.6, (0, 0, 255), 4)
            shots.append((label, tagged))
            cv2.imwrite(f"{args.out}_{label}.png", frame)
    finally:
        try:
            arm.relax()
            print("\nrelaxed (torque OFF)")
        finally:
            arm.close()
            cam.release()

    if shots:
        # Contact sheet, quarter scale, 3 per row.
        small = [cv2.resize(f, (0, 0), fx=0.35, fy=0.35) for _, f in shots]
        per_row = 3
        rows = []
        for i in range(0, len(small), per_row):
            chunk = small[i : i + per_row]
            while len(chunk) < per_row:
                chunk.append(np.zeros_like(small[0]))
            rows.append(np.hstack(chunk))
        cv2.imwrite(f"{args.out}_sheet.png", np.vstack(rows))
        print(f"contact sheet: {args.out}_sheet.png")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
