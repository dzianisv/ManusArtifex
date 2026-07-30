# Hardware fault: servo 5 (SHOULDER_LIFT) non-responsive

Status: **OPEN AGAIN, 2026-07-30 — and now known to be INTERMITTENT.**
Found: 2026-07-28. Cleared: 2026-07-29. Failed again: 2026-07-30, right after the
arm was physically moved.

## The intermittency is the finding (2026-07-30)

| Date | What happened to the arm just before | Motion | Pack |
|------|--------------------------------------|-------:|------|
| 07-28 | power-cycled and handled after the stall incident | 230-330 px (noise floor 335) | flat |
| 07-29 | operator handled the arm between sessions | 134,641 px | 0.60 V sag |
| 07-30 | operator moved the camera and the robot | 0-2 px | 0.008 V |

The joint works after the arm is handled and stops working after the arm is
handled. That pattern is mechanical, not electrical: a winding or a driver IC does
not heal itself and then fail again on being carried. It fits **family B** below —
a horn screw or linkage that is not holding — far better than family A.

Measured today, with `SHOULDER_LIFT` commanded across its provisional band and the
camera cropped to exclude a person in the room:

| Command | changed px | pack |
|---------|-----------:|------|
| 1500 -> 1800 | 0 | 7.398 -> 7.550 V (rose) |
| 1500 -> 1200 | 0 | 7.565 -> 7.557 V (flat) |
| 1200 -> 1500 | 2 | 7.557 -> 7.384 V (sagged 0.17 V) |

Every other joint moved strongly through the same code path in the same session
(ELBOW 56,860 px, WRIST_FLEX 17,969, SHOULDER_PAN 13,341, WRIST_ROLL 10,770).

The pack behaviour is *asymmetric*, which is new information the 07-28 report did
not have: commanding the joint one way draws real current while nothing moves, and
commanding it the other way draws nothing. Current drawn with no motion is a servo
pushing against something it cannot turn — or turning a horn that is no longer
coupled to the link. Both are family B.

**Torque into this joint was stopped as soon as the asymmetry showed up.** Three
commanded moves total. This project has already destroyed one servo by leaving
torque in a joint that would not move.

### The two-second check that settles it, before any part is ordered

From the Notion runbook, and unchanged since 07-28: **command the joint and watch
the servo's own output horn, not the arm link.**

* Horn turns, link does not -> the horn screw or linkage has come adrift. Tighten
  it. **No new servo needed** — and this now looks like the most probable outcome,
  because a loose fastener is exactly what "works after handling, fails after
  handling" looks like.
* Nothing moves at the horn either -> electrical, order the LDX-218.

A second check with the arm powered and the joint commanded to hold: try to
back-drive the shoulder gently by hand. A driven joint resists; a joint whose
linkage is adrift swings freely.

---

## The 2026-07-29 clear, kept for the record
`SHOULDER_LIFT` moved. It was re-probed with `joint_probe.py`, which reads **two**
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

**What that clear cost, and the lesson kept.** The dead-servo claim propagated into
planning documents in two repositories (`research/04-architecture.md`'s M4 gate,
`research/07-mini-bottles.md`'s "hard, total blocker"), where it blocked work for a
day. Then the opposite happened: this file was marked RESOLVED on the strength of
one good session, and the joint failed again the next day. **Neither "dead" nor
"fixed" was ever a durable fact about this joint.** An intermittent fault needs a
re-check at the start of every session that depends on it, which is what
`joint_probe.py` is for.

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

## Impact (as assessed on 2026-07-28; applies again while the fault is open)

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
