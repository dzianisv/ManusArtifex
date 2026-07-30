# Hardware fault: servo 5 (SHOULDER_LIFT) non-responsive

Status: **RESOLVED 2026-07-29 — the joint responds. No replacement is needed.**
Found: 2026-07-28, first full hardware session after the gripper stall incident.
Cleared: 2026-07-29, on the next live session, after the operator worked on the arm.

## Resolution (2026-07-29)

`SHOULDER_LIFT` moves. It was re-probed with `joint_probe.py`, which reads **two**
independent signals per joint rather than the one the original report relied on:
the project's own `VisionGuard` motion score (so the numbers are comparable with
`camera-check`) and pack sag during the move.

`joint_probe.py --delta 120`, both legs, camera noise floor 1,317:

| Joint          | best motion score | largest pack sag | Verdict |
|----------------|------------------:|-----------------:|---------|
| SHOULDER_PAN   |            42,671 |          0.004 V | alive; gravity-neutral, so near-zero sag is correct |
| **SHOULDER_LIFT** |      **134,641** |      **0.60 V** | **alive** |
| ELBOW          |            13,105 |           0.29 V | alive, loaded |
| WRIST_FLEX     |            93,510 |           0.28 V | alive, loaded |
| WRIST_ROLL     |            19,400 |           0.22 V | alive; sag proves it, the camera barely sees it |
| GRIP           |            23,412 |           0.02 V | alive; low sag correct, the jaws close on air |

`SHOULDER_LIFT` scored **100x the noise floor** and loaded the pack **150x harder
than `SHOULDER_PAN`**. A failed-open servo does neither. Both signals agree, which
is the standard this project holds itself to after nearly condemning `WRIST_ROLL`
on a camera reading alone (see docs/camera-field-of-view.md).

**This is not a retraction of the original diagnosis.** On 2026-07-28 the joint
really did not move: zero motion over the full 600-unit band, four consecutive
attempts, flat pack. Between the two sessions the operator handled the arm. Which
of the two failure families below was actually at fault, and what fixed it, was
not recorded — so the runbook below stays, unchanged, for the next time.

**What this cost, and the lesson kept.** The dead-servo claim propagated into
planning documents in two repositories (`research/04-architecture.md`'s M4 gate,
`research/07-mini-bottles.md`'s "hard, total blocker"), where it blocked work for a
day after the hardware was already fine. A fault that gates a plan needs its status
re-checked before the plan is, not after.

**One thing the re-probe changed in code.** The measured per-joint sag column above
is now the evidence behind `_MEASURED_LIVE_SAG_V` in `robohand/calibrate.py`. A live
`SHOULDER_PAN` sags 0.004 V and a live `GRIP` 0.02 V — at or under the 0.02 V noise
floor — so on those two joints a flat pack is what *healthy* looks like, and the
failed-open discriminator is no longer allowed to convict them on it. Before that
change, any burst over 100 raw units was trusted on travel alone, which would have
diagnosed a live-but-unseen `SHOULDER_PAN` exactly the way servo 5 was diagnosed
here.

---

# Original report (2026-07-28), unchanged

Status at the time: OPEN — requires physical inspection by the operator.

## Summary

`SHOULDER_LIFT` (servo ID 5, see `robohand/config.py:58`) does not move. Every other
joint on the arm responds normally. The fault is almost certainly a disconnected or
failed servo, **not** a software limit and **not** a mechanical jam.

## Evidence

Method: command one joint at a time between two positions inside its provisional band,
capture a camera frame after each move, and count changed pixels (absdiff > 25) over an
ROI covering the arm only (columns 0..700, which excludes a person standing on the right
of the frame). Camera warmed up with 20 reads after open and 8 flush reads before every
captured frame. Battery voltage sampled while each move was in flight.

Per-joint responsiveness, 1400 -> 1700 (300 raw units):

| Joint          | Servo ID | changed px | Verdict     |
|----------------|----------|-----------:|-------------|
| ELBOW          | 4        |     73,003 | responds    |
| WRIST_FLEX     | 3        |     49,425 | responds    |
| SHOULDER_PAN   | 6        |     35,604 | responds    |
| WRIST_ROLL     | 2        |     15,326 | responds    |
| GRIP           | 1        |     25,746 | responds    |
| **SHOULDER_LIFT** | **5** |    **294** | **NO MOTION** |
| (baseline noise, zero commanded motion) | — | 335 | — |

`SHOULDER_LIFT` changed fewer pixels than the no-motion noise floor.

Confirmation over the **full** provisional band (1200 <-> 1800, 600 raw units), four
consecutive attempts, with battery sampled under load:

| Command   | changed px | battery during move        |
|-----------|-----------:|----------------------------|
| SL -> 1200 |        306 | 7.528 V (flat)             |
| SL -> 1800 |        330 | 7.530 V (flat)             |
| SL -> 1200 |        274 | 7.562 V (flat)             |
| SL -> 1800 |        230 | 7.530 V (flat)             |
| ELBOW -> 1250 (control) | 63,971 | 7.561 V (flat) |
| ELBOW -> 1750 (control) | 69,623 | 7.524 V (flat) |

## Diagnosis

Two facts together narrow this down:

1. **Zero motion across the full 600-unit band, repeatably.** Not a limit, not a small
   range, not a marginal signal — the joint is completely static.
2. **No battery sag while commanded.** A servo fighting a mechanical obstruction draws
   locked-rotor current and visibly sags this 7.5 V pack. The voltage does not move at
   all. The servo is therefore drawing no meaningful current.

No current draw + no motion. Two distinct failure families fit this equally well, and the
operator should check both — they are told apart by whether the *servo itself* moves:

**A. The servo is not being driven** (no motion at the servo output shaft)

1. **Servo 5 signal/power lead unseated at the controller board.** Most likely, and
   cheapest to check. The arm was power-cycled and handled after the stall incident.
2. **Servo 5 has failed open** (burnt winding or dead driver IC in the servo).
3. Broken conductor in the servo cable.

**B. The servo is driving, but the joint is not following** (output shaft turns, link does not)

4. **Servo horn screw backed out, or the horn/linkage has detached from the link.** This
   fits the evidence just as well: an unloaded servo spinning freely draws very little
   current, so the battery would stay flat exactly as observed. This is a realistic
   outcome after a violent stall event and subsequent handling of the arm.
5. Stripped output spline / stripped internal gear train. A servo with stripped gears
   also turns freely and draws almost nothing.

Distinguishing A from B takes two seconds by eye or by touch: command SHOULDER_LIFT
between 1200 and 1800 and **watch the servo's own output horn**, not the arm link.
If the horn turns and the link does not, it is family B (mechanical). If nothing moves
at all, it is family A (electrical).

A further quick check for family B: with the arm powered and SHOULDER_LIFT commanded to
hold, try to back-drive the shoulder joint gently by hand. A healthy, driven joint
resists. A joint whose linkage has come adrift swings freely.

Ruled out: software limits (other joints move over the same band through the same code
path), wrong servo ID (IDs 1,2,3,4,6 all respond, so the ID map in `config.py` is
correct), and a *loaded* mechanical jam (a jam would draw locked-rotor current and sag
the pack; the voltage is perfectly flat).

## Operator action required

1. Power the arm off.
2. **Look at the servo 5 horn while the joint is commanded** (see the A-vs-B test above).
   That one observation halves the search space before you touch anything.
3. If nothing moves at the servo: reseat the servo 5 lead at the controller board and at
   the servo itself. Servo 5 is the shoulder-lift joint — the one that raises the upper
   arm link.
4. If the servo horn turns but the link does not: check the horn screw and the linkage.
5. Power on and re-run the per-joint check (`robohand selftest`, or the procedure above).
6. If it is still dead, swap the servo 5 lead with a known-good channel to determine
   whether the fault is in the cable/board or in the servo.

## Impact (as assessed on 2026-07-28; the fault is now cleared)

Without shoulder lift the arm loses its main vertical-reach degree of freedom. Base
rotation, elbow, wrist flex, wrist roll and the gripper still work, so a constrained
pick may still be possible, but the reachable workspace is badly reduced and any
calibration of `SHOULDER_LIFT` is meaningless until the joint responds.

Auto-calibration must not be run on `SHOULDER_LIFT` while this fault is open: the limit
search detects a mechanical stop by observing that motion has ceased, and a joint that
never moves at all looks exactly like a joint that is already at its stop. It would
"find" a bogus limit immediately in both directions.

That last hazard is no longer left to a document. `calibrate` now runs an
observer-reliability burst before it will record any stop, and a joint whose motion
is never seen raises instead of writing a band — `JointUnresponsive` when the pack
stays flat on a joint whose live sag is known to be measurable, `CalibrationAborted`
otherwise. Verified in the simulator against a blind observer: the run aborts and
`robohand/data/config.json` is byte-identical afterwards.
