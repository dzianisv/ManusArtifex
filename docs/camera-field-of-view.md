# Blocker: the gripper is outside the camera's field of view

Status: **OPEN — needs the operator to physically move the camera. No software fix.**
Found: 2026-07-29, during live bring-up of `WRIST_ROLL`.

## Summary

The camera cannot see the end of the arm. At the home pose the arm points straight
up and the wrist and gripper sit **above the top edge of the frame**; only the base
and the lower links are in shot. Nothing is broken — the hand is simply not in the
picture.

This blocks `WRIST_ROLL` calibration, which blocks pouring, which is the whole
bartender goal (`docs/cocktail-robot.md`, and `../bender`).

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

3. **Every vision-judged operation aimed at the gripper is blind for the same
   reason** — grasp verification in `pick.py`, and any future pour check. They are
   not wrong, they are unsupported by evidence, which is the same thing here.

## What the operator has to do

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
