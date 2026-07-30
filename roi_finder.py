#!/usr/bin/env python3
"""Find the image region where one joint's motion actually shows up.

Why this exists: ``robohand calibrate`` infers a mechanical stop from *repeated
no-motion*, and it judges motion with the global ``vision.roi``. If a joint's
movement is small in the frame -- WRIST_ROLL spins the gripper about its own
axis and barely changes any pixels -- the calibrator sees "no motion" on a
perfectly free joint and writes a confidently wrong, too-narrow band.

So before calibrating such a joint, find out *where* in the frame it moves and
whether a tighter ROI lifts its signal above the camera's noise floor.

The method: command a real excursion, diff before/after, and reduce the diff
over a coarse grid of tiles. The tiles that light up are where that joint lives.
Then score a few candidate ROIs at the small step size the calibrator will
actually use -- because a signal that survives a big move may still vanish at
``--step 30``, and the small step is the one that matters.

Safety: same rules as joint_probe.py -- clamped targets, slow moves, a pack
voltage floor, and an unconditional relax in a ``finally``.

Usage:
    python3 roi_finder.py --joint WRIST_ROLL
"""
from __future__ import annotations

import argparse
import sys
import time
from typing import List, Optional, Tuple

import cv2
import numpy as np

from robohand import config
from robohand.arm import Arm
from robohand.camera import Camera, CameraConfig
from robohand.guard import VisionGuard

MIN_SAFE_VOLTS = 6.6


def _guard(cam: Camera, roi) -> VisionGuard:
    return VisionGuard(
        frame_source=cam.grab,
        roi=roi,
        diff_threshold=config.VISION_DIFF_THRESHOLD,
        min_changed_pixels=config.VISION_MIN_CHANGED_PIXELS,
        blur_ksize=config.VISION_BLUR_KSIZE,
        normalize=config.VISION_NORMALIZE,
        normalized_threshold=config.VISION_NORMALIZED_THRESHOLD,
    )


def _clamp(joint: str, target: int) -> int:
    lo, hi = config.effective_bounds(joint)
    alo, ahi = config.absolute_bounds(joint)
    return max(max(lo, alo), min(min(hi, ahi), target))


def _tile_map(before: np.ndarray, after: np.ndarray, rows: int, cols: int) -> np.ndarray:
    """Changed-pixel count per tile, as a rows x cols grid."""
    gb = cv2.cvtColor(before, cv2.COLOR_BGR2GRAY)
    ga = cv2.cvtColor(after, cv2.COLOR_BGR2GRAY)
    diff = cv2.GaussianBlur(cv2.absdiff(gb, ga), (5, 5), 0)
    mask = (diff > config.VISION_DIFF_THRESHOLD).astype(np.uint32)
    h, w = mask.shape
    out = np.zeros((rows, cols), dtype=np.int64)
    for r in range(rows):
        for c in range(cols):
            y0, y1 = r * h // rows, (r + 1) * h // rows
            x0, x1 = c * w // cols, (c + 1) * w // cols
            out[r, c] = int(mask[y0:y1, x0:x1].sum())
    return out


def _bbox_from_tiles(tiles: np.ndarray, shape, rows: int, cols: int, keep_frac: float) -> Tuple[int, int, int, int]:
    """Bounding box (x, y, w, h) over tiles holding the strongest motion."""
    thresh = tiles.max() * keep_frac
    hot = np.argwhere(tiles >= thresh)
    h, w = shape[:2]
    r0, c0 = hot.min(axis=0)
    r1, c1 = hot.max(axis=0)
    x = int(c0 * w // cols)
    y = int(r0 * h // rows)
    x2 = int((c1 + 1) * w // cols)
    y2 = int((r1 + 1) * h // rows)
    return x, y, x2 - x, y2 - y


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--joint", required=True)
    ap.add_argument("--delta", type=int, default=120, help="Excursion for the mapping move (default 120).")
    ap.add_argument("--calib-step", type=int, default=30, help="Step size the calibrator will use (default 30).")
    ap.add_argument("--duration-ms", type=int, default=1500)
    ap.add_argument("--settle-s", type=float, default=0.6)
    ap.add_argument("--rows", type=int, default=6)
    ap.add_argument("--cols", type=int, default=8)
    ap.add_argument("--save-prefix", default="/tmp/roi_finder")
    args = ap.parse_args()

    joint = args.joint
    if joint not in config.JOINT_NAMES:
        print(f"unknown joint {joint!r}", file=sys.stderr)
        return 2

    lo, hi = config.effective_bounds(joint)
    centre = (lo + hi) // 2
    big_a, big_b = _clamp(joint, centre - args.delta), _clamp(joint, centre + args.delta)
    small_a, small_b = _clamp(joint, centre - args.calib_step), _clamp(joint, centre + args.calib_step)

    cam = Camera(CameraConfig(index=0))
    cam.open()
    arm = Arm(use_vision=False)
    full = _guard(cam, config.VISION_ROI)
    try:
        v0 = arm.battery_voltage()
        print(f"pack: {v0:.3f} V")
        if v0 is not None and v0 < MIN_SAFE_VOLTS:
            print("ABORT: pack below floor", file=sys.stderr)
            return 1

        # --- 1. Map where this joint's motion appears -------------------
        arm.set_positions({joint: big_a}, duration_ms=args.duration_ms, wait=True)
        time.sleep(args.settle_s)
        before = cam.grab()
        arm.set_positions({joint: big_b}, duration_ms=args.duration_ms, wait=True)
        time.sleep(args.settle_s)
        after = cam.grab()

        cv2.imwrite(f"{args.save_prefix}_before.png", before)
        cv2.imwrite(f"{args.save_prefix}_after.png", after)

        tiles = _tile_map(before, after, args.rows, args.cols)
        print(f"\nmotion map for {joint}, {big_a} -> {big_b}  (frame {before.shape[1]}x{before.shape[0]})")
        print("changed px per tile:")
        for r in range(args.rows):
            print("  " + " ".join(f"{int(v):>7}" for v in tiles[r]))

        candidates: List[Tuple[str, Optional[Tuple[int, int, int, int]]]] = [
            ("full frame (current config)", config.VISION_ROI)
        ]
        for frac in (0.5, 0.25):
            box = _bbox_from_tiles(tiles, before.shape, args.rows, args.cols, frac)
            candidates.append((f"hot tiles >= {frac:.0%} of peak", box))

        print("\ncandidate ROIs scored on the SAME big-move frame pair:")
        for label, box in candidates:
            score = _guard(cam, box).motion_score(before, after)
            print(f"  {label:<32} roi={box}  score={score}")

        # --- 2. The decisive test: does it survive the calibrator's step? ---
        print(f"\nnow the test that actually matters: a {args.calib_step}-unit step,")
        print("the increment 'robohand calibrate' probes with.\n")
        arm.set_positions({joint: small_a}, duration_ms=args.duration_ms, wait=True)
        time.sleep(args.settle_s)
        sb = cam.grab()
        arm.set_positions({joint: small_b}, duration_ms=args.duration_ms, wait=True)
        time.sleep(args.settle_s)
        sa = cam.grab()

        floor = config.VISION_MIN_CHANGED_PIXELS
        print(f"  {'roi':<32} {'score':>8}  vs min_changed_pixels={floor}")
        for label, box in candidates:
            score = _guard(cam, box).motion_score(sb, sa)
            verdict = "DETECTED" if score >= floor else "MISSED -> calibrate would call this a stop"
            print(f"  {label:<32} {score:>8}  {verdict}")
    finally:
        try:
            arm.relax()
            print("\nrelaxed (torque OFF)")
        finally:
            arm.close()
            cam.release()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
