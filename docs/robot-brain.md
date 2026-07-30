# The robot's brain: what should be AI, and what should not

Date: 2026-07-30. Supersedes the perception half of `docs/verification-camera.md`
as *policy*; the measurements there stay valid.

## The mistake this document corrects

Every vision-based decision in this project ran through one question: **how many
pixels changed between two frames?** (`robohand/guard.py`). Calibration used it to
decide whether a joint moved. `pick.py` used it to check a grasp. The camera-health
gate existed to check whether the question was even answerable.

That question is not about the arm. Measured on 2026-07-30, arm perfectly still:

| scene | no-motion floor (p95) | verdict at threshold 3,000 |
|---|---:|---|
| nobody in the room | 1,243 changed px | PASS |
| one person seated in frame | 43,335 | FAIL |
| same person, cropped out of the ROI | 5,869–13,676 | FAIL |
| same person, tight crop on the arm | 6,375 | FAIL |

A person shifting in a chair produces a bigger signal than the joint we are trying
to measure. Three fixes were tried and all three lose: a tighter ROI (the person's
reflection is in the TV, and auto-exposure reacts to them and shifts the *whole*
frame), a brightness-normalised metric (`vision.normalize=zscore` — worse, 10,000
vs 6,375), and a higher threshold (it would then sit above real motion too).

They lose because **signal and noise are the same kind of thing.** No threshold
separates "the arm moved" from "a human moved" when the only measurement is "some
pixels are different now". The fix is not a better threshold. It is a different
question.

## What everyone else does

**1. Joint encoders — the normal answer, closed to us.** Every arm LeRobot supports
(SO-100, Koch, OMX, Reachy2, …) reports its own joint positions, and the standard
loop is `obs = robot.get_observation(); action = model.select_action(obs);
robot.send_action(action)`. Our LSC-6 emits PWM and reads nothing back
(`docs/research-hardware.md`). This also closes the door on imitation learning —
ACT and diffusion policies need readable state on both the teleop-recording side
and the execution side, so "just train a policy like LeRobot does" is not available
until the servos are replaced with bus servos (costed in the Notion servo page:
~6 × $16.99 + a controller, a multi-week project).

**2. Fiducial markers — the standard substitute on cheap hardware.** A printed
pattern with a known id, found by a detector looking for *that pattern*. Ships
inside OpenCV as `cv2.aruco` (present here, OpenCV 5.0). This is what people put on
a gripper when they have no encoders, and it is what `robohand/markers.py` now
implements.

**3. A VLM — for meaning, not for geometry.** Gemini Robotics-ER 1.6 is on our key
and works, and Google's own guidance is to pair it with a low-level controller you
supply. This project already tested it on a real photo of this arm
(`docs/cocktail-robot.md`, 2026-07-28): it pointed at the gripper **20–30 px off**
and **hallucinated a graspable object in a frame that contained none**. So: a VLM
is the right tool for "which bottle is the gin" and the wrong tool for "is the claw
2 mm left of the bottle".

## The architecture

Three layers, split by what each one is actually good at. Nothing crosses.

```
  ┌─ SEMANTICS ──────────────────────────────────────────────────────┐
  │  LLM agent (pi) + VLM                                            │
  │  "which object do I want", "is this a bottle or a glass",        │
  │  "the plan is: approach, grasp, lift, verify"                    │
  │  Slow (1-3 s), costs money, occasionally invents. Never trusted  │
  │  for a number that a servo will act on.                          │
  └────────────────────────┬─────────────────────────────────────────┘
                           │ symbolic goals: "grasp marker 1"
  ┌─ GEOMETRY ─────────────▼─────────────────────────────────────────┐
  │  Fiducial markers (robohand/markers.py)                          │
  │  "marker 0 is at (412, 233), marker 1 at (455, 240)"             │
  │  ~10 ms, free, deterministic, and a person is not a marker.      │
  │  This is the error signal the control loop closes on.            │
  └────────────────────────┬─────────────────────────────────────────┘
                           │ pixel error
  ┌─ SAFETY + ACTUATION ───▼─────────────────────────────────────────┐
  │  robohand safety layer, unchanged                                │
  │  bands, absolute rails, single-use move tokens, torque budget    │
  │  Refuses anything the layers above ask for that is out of range. │
  └──────────────────────────────────────────────────────────────────┘
```

The layering is the point. The LLM decides *what*; markers measure *where*;
deterministic code decides *whether it is allowed*. A hallucinating model cannot
produce a bad servo command, because it never emits one — it names a marker id.

## Why markers, concretely

Same two frames — arm still, large high-contrast clutter moving elsewhere
(`tests/test_markers.py`, run head to head):

```
absdiff  -> 106,750 changed px   (threshold 3,000  -> "MOTION", wrong)
markers  ->       0.00 px moved  (threshold 3.0    -> "no motion", right)
```

And on a real photograph of this room, with a person in it: **zero markers
detected**. Not "below threshold" — absent from the result. There is nothing to
tune, because the room is not in the measurement.

Three properties follow, and they are why this replaces rather than supplements the
absdiff path for anything the arm must trust:

- **No noise floor, no ROI, no health gate.** `camera-check` exists to decide
  whether absdiff is answerable today. That whole question disappears.
- **The answer is a distance, not a statistic.** "The gripper moved 40 px" composes
  into a control loop. "17,000 pixels changed" does not.
- **Absence is distinguishable from stillness.** A marker that is not detected
  raises, it does not return "no motion" — because `calibrate` infers a mechanical
  stop from repeated no-motion, and a marker falling off would otherwise be written
  into `config.json` as a joint limit.

## What is built (2026-07-30)

- `robohand/markers.py` — detection, a printable sheet generator, and
  `ArucoMotionObserver`, which implements the existing `MotionObserver` interface
  so `calibrate` cannot tell the difference.
- `robohand markers --generate sheet.png` — print it, tape marker 0 to the gripper,
  marker 1 to the object.
- `robohand markers` — what the camera can see right now. Exit 0 if any marker is
  found, 1 if none, so it works as a pre-flight.
- `robohand calibrate --observer aruco --marker-id 0` — calibrate with a person in
  the room.
- 15 tests, including the head-to-head above and a real room photograph.

## What is deliberately NOT built

- **No pose estimation (`solvePnP`).** That needs camera intrinsics from a
  checkerboard calibration nobody has run on this webcam. A confidently wrong 3D
  pose is worse than an honest 2D one. The seam is marked in `MarkerView`.
- **No hand-eye calibration, no IK.** The pick loop servos in image space
  (`docs/pick.md`) and does not need either.
- **No VLA.** Every VLA surveyed needs readable joint state (`docs/cocktail-robot.md`).
  Unchanged by any of this.
- **The absdiff path is not deleted.** It is still the right cheap check for "did
  *anything* move" as a stall guard during a single commanded move, where a false
  positive is harmless and a marker may be occluded by the arm itself.

## The honest limitation

Markers must be physically stuck to the arm and stay stuck, and they must be big
enough to resolve and unoccluded at the poses that matter. That is a real,
recurring maintenance cost, and it is a *physical* dependency exactly like the
camera aim. It buys determinism, not freedom from the workshop.

## Next

1. Print `robohand markers --generate`, tape marker 0 to the gripper.
2. `robohand markers` to confirm it is seen at the poses that matter.
3. `robohand calibrate --observer aruco --joint WRIST_ROLL --i-am-present-and-watching`
   — with the room occupied, which is the whole point.
4. Then the pick loop closes on marker 0 vs marker 1, with the LLM choosing the
   target and calling abort.
