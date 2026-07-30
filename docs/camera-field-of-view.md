# Blocker: the gripper is outside the camera's field of view

Status: **PARTLY FIXED 2026-07-30 — the camera was moved and `WRIST_ROLL` is now
visible. Two new constraints took its place: the framing is pose-dependent, and a
person in the room blocks calibration.**
Found: 2026-07-29, during live bring-up of `WRIST_ROLL`.

## What the camera move fixed (2026-07-30)

The operator moved the camera back. Re-probed with `joint_probe.py --delta 120
--roi 0 0 820 720`, against a no-motion floor of p95=1,243 changed px:

| Joint | 2026-07-29 (gripper out of frame) | 2026-07-30 (after the move) |
|---|---:|---:|
| **WRIST_ROLL** | **337 px** | **10,770 px** |
| SHOULDER_PAN | 4,867 | 13,341 |
| ELBOW | 13,105 | 56,860 |
| WRIST_FLEX | 93,510 | 17,969 |
| GRIP | 23,412 | 986 |

`WRIST_ROLL` went from invisible (337, under the noise floor) to 8.7x the noise
floor. That was the blocker, and it is gone.

## Constraint 1: the framing is pose-dependent, and the home pose is the bad one

The gripper is in frame when the arm is **folded**, and out of frame when it is
**straight up** — which is exactly what every joint sitting at its provisional
centre (1500) produces. `joint_probe.py` centres each joint before probing it, so
running the probe *puts the arm into the one pose where the hand is invisible*.
GRIP's collapse from 23,412 px to 986 px in the table above is that, not a camera
regression: the jaws were above the top edge while they were being measured.

Consequences:

* GRIP's 986 px reading is **not** a measurement of the gripper's visibility. Do
  not use it to re-tune `vision.min_changed_pixels`.
* The `min_changed_pixels = 3000` in `robohand/data/config.json` is still justified
  by the 25,746 px GRIP figure from the earlier full-arm-in-frame setup, but that
  was a different camera position. **It needs re-measuring in a folded pose** before
  anything trusts a GRIP vision check again.
* Folding the arm down far enough to see the hand needs more travel than the
  provisional +/-300 band allows on `ELBOW` alone — and `SHOULDER_LIFT`, which is
  the joint that would do it, is dead again (docs/hardware-fault-servo5.md).

## Constraint 2: a person in the room fails the camera-health gate

Measured 2026-07-30, same camera, minutes apart, no arm motion:

| Scene | ROI | no-motion p95 | gate |
|---|---|---:|---|
| person out of shot | full frame | 1,243 | PASS, 2.4x headroom |
| person seated in frame | full frame | 43,335 | FAIL |
| person seated in frame | cropped to `0 0 820 720` | 13,676 | FAIL |
| person leaning in, close | cropped to the arm only | 6,375 | FAIL |

A human moving anywhere in shot swamps the motion threshold by 4-14x. Cropping
helps but does not save it once they are close to the camera, because auto-exposure
reacts to them and shifts the brightness of the *whole* frame, including the crop.
The brightness-normalised metric (`vision.normalize=zscore`) was tried on the same
frames and did not help (10,000 p95 vs 6,375 for raw absdiff on the tight crop).

The gate is behaving correctly — it refuses to calibrate off a camera that cannot
tell a joint moving from a person shifting in a chair. **The fix is staging, not
software: nobody in frame while a calibration runs.**

`robohand calibrate --roi X Y W H` now exists (it was listed as gap #2 below) and
aims the observer *and* the health gate at the same crop for one run without
touching config.json. It buys margin; it does not make a person in frame
acceptable.

## How it was found, and the near-miss worth recording

`WRIST_ROLL` looked like a second dead servo. The per-joint probe scored it at 337
against a 1,317 noise floor, and **0 changed pixels** at the 30-unit step
`robohand calibrate` actually probes with (motion threshold 3,000). On camera
evidence alone the verdict would have been "this joint does not move."

The second signal disagreed: the pack sagged **0.22 V** during the same moves —
comparable to `ELBOW` (0.29 V) and `WRIST_FLEX` (0.28 V), and 55x `SHOULDER_PAN`'s
0.004 V. A servo that turns a load draws current; a failed-open one does not. The
joint was moving.

Looking at the captured frames settled it:

```
  ┌─────────────────────┐
  │   ▲ wrist + gripper │  ← above the top edge, never in shot
  ├─────────────────────┤
  │  ║ arm links        │
  │ ███ base   bottle   │  ← all the camera actually sees
  └─────────────────────┘
```

`pose_finder.py` then swept 7 shoulder/elbow poses across the provisional bands. The
gripper **never** enters frame in any of them: the ±300 provisional band cannot fold
the arm far enough down to bring the hand into view.

Had vision been trusted alone, a working servo would have been condemned — and worse,
a calibration run would have written a confidently wrong band into `config.json`.

## Consequences, in order of importance

1. **Do not run `robohand calibrate --joint WRIST_ROLL` until the camera is moved.**
   The limit search infers a mechanical stop from repeated no-motion, and an invisible
   joint reports no-motion from the first step. A wrong calibration is worse than no
   calibration, because everything downstream then trusts it.

   The code already refuses rather than guessing: before recording any stop,
   `calibrate` runs an observer-reliability burst, and a joint it cannot see raises
   `CalibrationAborted` — "it cannot reliably see this joint … aborting without
   writing a result". Reproduced in the simulator with a blind observer:
   `robohand/data/config.json` is byte-identical after the aborted run. The rule
   above is about not wasting a hardware session, not about avoiding corruption.

2. **`calibrate` has no `--roi` flag.** It uses the global `vision.roi`, so there is
   no per-run way to aim the observer at the gripper even once the gripper is
   visible. Worth adding when the camera moves.

   **Added 2026-07-30**: `robohand calibrate --roi X Y W H`, same semantics as
   `camera-check --roi`. It overrides the observer for one run only — the arm's own
   guard and `config.json` are untouched — and because the camera-health gate
   measures that same observer, the crop applies to the gate too. Pass the same
   values to `camera-check` first so the numbers you are trusting are the numbers
   the run will be judged by.

3. **Every vision-judged operation aimed at the gripper is blind for the same
   reason** — grasp verification in `pick.py`, and any future pour check. They are
   not wrong, they are unsupported by evidence, which is the same thing here.

## What the operator has to do

**Done (2026-07-30):** the camera was moved and `WRIST_ROLL` is visible. What is
left:

1. **Nobody in frame during a calibration run.** ~2 minutes per joint. This is the
   one blocking the next step right now.
2. Fix `SHOULDER_LIFT` (docs/hardware-fault-servo5.md) — without it the arm cannot
   fold down into the pose where the gripper is visible, so GRIP vision stays
   unmeasurable.
3. Then, in a folded pose: `robohand camera-check --roi ...` to re-measure the
   noise floor, re-probe GRIP, and re-derive `vision.min_changed_pixels` from a
   real gripper-visible reading.

### The original instructions, for the next time the camera moves

Move the camera back, up, or tilt it up so the **whole arm workspace, including the
gripper at the home pose, is inside the frame**. Then:

1. `robohand camera-check` — confirm the noise floor is still well under the motion
   threshold from the new position (it is position-dependent: a new background and a
   new working distance change it).
2. Re-run `joint_probe.py --joints WRIST_ROLL` — the motion score should rise from
   337 to something comparable with the other joints (10,000+). That is the check
   that the fix worked; the pack sag was never the problem.
3. Only then `robohand calibrate --joint WRIST_ROLL`.

Until step 2 passes, `WRIST_ROLL`'s band in `robohand/data/config.json` stays
`"calibrated": false` — a provisional centre ±300, i.e. 54° of travel where a pour
needs 90–120°. `../bender` models the consequences of that band and refuses every
pour because of it.
