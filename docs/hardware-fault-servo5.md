# Hardware fault: servo 5 (SHOULDER_LIFT) non-responsive

Status: **OPEN — requires physical inspection by the operator.**
Found: 2026-07-28, first full hardware session after the gripper stall incident.

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

## Impact

Without shoulder lift the arm loses its main vertical-reach degree of freedom. Base
rotation, elbow, wrist flex, wrist roll and the gripper still work, so a constrained
pick may still be possible, but the reachable workspace is badly reduced and any
calibration of `SHOULDER_LIFT` is meaningless until the joint responds.

Auto-calibration must not be run on `SHOULDER_LIFT` while this fault is open: the limit
search detects a mechanical stop by observing that motion has ceased, and a joint that
never moves at all looks exactly like a joint that is already at its stop. It would
"find" a bogus limit immediately in both directions.
