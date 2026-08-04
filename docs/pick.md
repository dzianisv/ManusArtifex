# Vision-guided pick

**Status: works in simulation, never run on hardware.** Every number in this
document comes from `SimTransport` + `SimWorld`. The physical arm has been
unpowered and disconnected throughout; nothing here has touched a servo.

Read [safety.md](safety.md) first, then [calibration.md](calibration.md). This
document assumes you know why the 2026-07-28 gripper incident happened, what the
three defence rings do, and why every joint still ships `"calibrated": false`.

---

## Why visual servoing and not inverse kinematics

The obvious way to pick something up is: find it, solve IK for that pose, drive
the joints there. That needs two things this arm does not have.

**A verified kinematic model.** Nobody has measured the link lengths, the joint
axes, or the raw-PWM-value → angle relationship on this unit. There is no
datasheet number for "1500 units = N degrees" that has been checked against
reality here. An IK solution built on unverified geometry is a confident number
with nothing behind it.

**Joint state.** There is no encoder and no feedback wire.
`CMD_MULT_SERVO_POS_READ` (cmd 21) never replies on this controller — verified
against the real hardware, and `SimTransport.get_positions()` raises
`NoResponse` for exactly that reason. So even if you had a model, you could not
close the loop on it, and you could not tell whether the arm arrived.

```
  what IK needs                      what this arm provides
  ---------------------------        ------------------------------------
  link geometry, verified            never measured
  joint angles, continuously         cmd 21 -> silence, always
  a way to detect arrival            nothing, except a camera
```

That second row is also why no learned VLA policy can drive this arm — every
such policy takes joint state in its observation, and this hardware physically
cannot produce it. [cocktail-robot.md](cocktail-robot.md) works through that
argument in full and lands on the Code-as-Policies split: a vision model decides
*where things are*, ordinary bounded code decides *how to move*. This module is
the "ordinary bounded code" half.

So the routine does what `calibrate.py` already does for limits — **command a
small increment, look, decide** — except for position instead of travel. It
never computes a trajectory. It measures how the image responds to each joint,
moves a little, looks again, and recomputes. If the geometry, the camera pose,
or a joint's direction changes, the measurement changes with it and nothing has
to be updated by hand.

### The image Jacobian, in full

Before approaching, the routine nudges each of the two servo joints by
`probe_delta` raw units and watches how far the gripper moves *in the image*:

```
  probe SHOULDER_PAN +60 units  ->  gripper moved (+45, +0) px  ->  (0.75, 0.00) px/unit
  probe ELBOW        +60 units  ->  gripper moved (+0, -36) px  ->  (0.00, -0.60) px/unit

                      [ 0.75   0.00 ]
                  J = [            ]      det = -0.45, invertible
                      [ 0.00  -0.60 ]
```

Then each step solves `J · Δjoints = gain · error_px` and applies the result,
clamped to `max_joint_step`. A singular `J` (two joints that move the gripper
the same way, or a detector that reports a frozen point) is reported as
`degenerate_jacobian` rather than inverted.

This is a *local* linearisation and is treated as one: it is re-applied only for
small steps, and every step is followed by a fresh observation. It is not a
Jacobian in the robotics sense — there is no configuration-space model behind
it — and it is deliberately not extended into one.

---

## The state machine

```
                      +--------------------------------------------------+
                      |                                                  |
   START              v                                                  |
     |          +----------+                                             |
     +--------> |  LOCATE  |  travel pose, open jaws, find object,        |
                +----------+  probe the image Jacobian                    |
                      |                                                  |
                      v                                                  |
                +----------+                                             |
                | APPROACH |  CLOSED LOOP: look -> move a little -> look  |
                +----------+  until error <= approach_tolerance_px        |
                      |                                                  |
                      v                                                  |
                +----------+                                             |
                | DESCEND  |  OPEN LOOP height change, then a second      |
                +----------+  closed-loop pass at grasp_tolerance_px      |
                      |                                                  |
                      v                                                  |
                +----------+                                             |
                |  GRASP   |  arm.close_grip() ladder ONLY, then          |
                +----------+  re-apply torque at the contact position     |
                      |                                                  |
                      v                                                  |
                +----------+                                             |
                |  VERIFY  |  is the object actually at the gripper?      |
                +----------+                                             |
                      |                                                  |
                      v                                                  |
                +----------+                                             |
                |   LIFT   |  OPEN LOOP height change, hold=[...]         |
                +----------+                                             |
                      |                                                  |
                      v                                                  |
                +----------+                                             |
                |VERIFY_HE-|  did the object move WITH the gripper?       |
                |    LD    |                                             |
                +----------+                                             |
                      |                                                  |
                      v                                                  |
                +----------+          any failure, from any state        |
                |   DONE   |               (or Ctrl-C, or --abort)       |
                +----------+                                             |
                                                                          |
                +--------------------+  <-------------------------------+
                | FAILED / ABORTED   |
                +--------------------+
                          |
                          v
                open the jaws, then unload every servo
```

Every state is individually abortable (`PickSettings.abort_states`, used by the
tests to inject a failure at each one in turn). Every non-`DONE` exit runs the
same cleanup: open the gripper if anything might be in it, then
`arm.relax()` — an unloaded servo is always safe, which is why
`SafetyLayer.unload` needs no motion token.

### Closed loop where it can be, open loop where it structurally cannot

`APPROACH` and the post-descend refinement are genuinely closed loop.

**`DESCEND` and `LIFT` are open loop and cannot be anything else.** One camera
looking at a known table plane gives X and Y. `vision.Calibration` is a
plane-to-plane homography; it has no depth term and cannot acquire one. There is
no rangefinder, no stereo pair, no structured light, and no encoder. So the
descend and lift amounts are operator-supplied raw-unit offsets
(`--descend-units`, `--lift-units`), and **their sign has to be established by
hand on real hardware** — the routine cannot discover which direction is "down"
from a top-down view. Nothing in the code pretends otherwise; the log entries
say `(open loop -- no depth sensing exists)` in as many words.

### What "verified" means here

`VERIFY` and `VERIFY_HELD` do not ask the grip ladder whether it thinks it
grasped something. They re-locate the object and the gripper independently:

| check | question | failure reason |
|---|---|---|
| `VERIFY` | is the object within `verify_max_distance_px` of the jaws? | `verify_object_not_at_gripper` |
| `VERIFY_HELD` | did the object move at least `min_lift_displacement_px` during the lift? | `verify_held_object_did_not_move` |
| `VERIFY_HELD` | is it still within `held_max_distance_px` of the jaws? | `verify_held_object_left_behind` |

A failure at either releases, unloads, and stops. It never retries.

---

## Running it in simulation

```bash
# The default scene: an object at (10, 140) mm, ground-truth locator.
python3 -m robohand.cli --sim pick --locator sim

# Somewhere else in the workspace, with machine-readable output.
python3 -m robohand.cli --sim pick --locator sim --object-x -40 --object-y 95 --json

# Out of reach: a clean FAILED with a named reason, exit code 1.
python3 -m robohand.cli --sim pick --locator sim --object-x 100
```

The shipped `config.json` has every joint `"calibrated": false`, so all three of
those refuse to run (exit 2) until you either calibrate or say why you are
overriding:

```bash
python3 -m robohand.cli --sim pick --locator sim \
    --allow-uncalibrated --override-reason "sim rehearsal, no hardware attached"
```

The override is logged per move through `SafetyLayer.override_log`, the same
path every other override in this package uses.

`--locator sim` reads simulator ground truth and is refused outright on real
hardware. `--locator classical` (HSV blob detection) needs a camera and is the
**only sanctioned locator** (Policy 2026-08-04: no Gemini/`GEMINI_API_KEY`
anywhere in this project). `--locator gemini` (Robotics-ER 1.6) still exists in
the code as of this writing but must not be used — see below.

### About `--locator gemini` (RETIRED — Policy 2026-08-04)

**Do not use.** `--locator gemini` still exists in the code as of this writing, but Gemini and
`GEMINI_API_KEY` are retired project-wide — not as an LLM provider, not as a perception
fallback. Use `--locator classical` (HSV blob detection against a colored band/marker, see step
7 above) for every real-hardware pick.

Kept for historical record — this is *why* perception here is classical-CV-only, not "VLM as a
checked hint": the model was verified against a real photo of this exact setup on 2026-07-28 and
**it hallucinated**. Asked for "a graspable object on the table" in a frame containing none, it
confidently pointed at a background box instead of returning `[]`
([cocktail-robot.md](cocktail-robot.md)). At the time, the mitigation was to treat its output as
one signal among others and never as authorisation (explicit `[]`-when-absent prompting,
`point_in_workspace` re-validation with no privileged bypass, independent re-validation in
`pick.py`, a required converged look-move-look-again loop). A model that invents objects when
asked to say "none" cannot be trusted even as a secondary signal, which is the reasoning behind
retiring it rather than continuing to gate it.

---

## Measured success rate in simulation

From `tests/test_pick.py`, seeded and reproducible:

| sample | region | rate |
|---|---|---|
| 12 positions, seed 20260728 | `x ∈ [-60, 60]`, `y ∈ [80, 165]` mm | **12/12 = 100%** |
| 16 positions, seed 4711 | full travel: `x ∈ [-75, 75]`, `y ∈ [60, 180]` mm | **15/16 = 94%** |

A wider unseeded sweep during development (100 in-band positions across four
seeds, 50 full-travel positions across two more) gave 100/100 in band and 44/50
at full travel, i.e. roughly 88% at the edges.

**Why the edge cases fail, honestly.** The routine converges an *image* error,
and the toy camera — like any real one — sees the raised gripper through some
parallax. Aligning the two pixels therefore puts the jaws about 6 mm short of
the object in the table plane. Away from the limits that is absorbed by the
grasp radius. Within about 6 mm of a joint's end stop there is no travel left to
absorb it, and the routine correctly reports `approach_joint_limits_reached`
rather than grinding into the stop. The outermost few millimetres of nominal
reach are simply not pickable, and the fix is a calibrated hand-eye transform,
not a bigger step budget.

The two numbers are reported separately on purpose. A single averaged figure
would hide which half of the workspace you are in.

---

## First attempt on real hardware

Do not skip steps. Do not run this unattended.

1. **Clear the workspace.** Nothing fragile, nothing in the arm's swept volume,
   nothing behind it. One light object with a distinct colour, on the table,
   well inside the reachable area — not at the edge.
2. **Power on.** Keep your hand on the switch. It stays within reach for the
   entire session.
3. **`python3 reset_arm.py`.** Gets the arm to a known pose before anything else
   commands it.
4. **Calibrate.** `robohand calibrate --all --i-am-present-and-watching`, per
   [calibration.md](calibration.md). It writes a timestamped file and never
   touches `config.json`.
5. **Read the calibration file by hand.** Cross-check the numbers against the
   manual procedure in [safety.md](safety.md#calibration-procedure). Do not
   promote numbers you have not looked at.
6. **Apply it.** `robohand calibrate --apply <file>`.
7. **Check the camera.** `robohand doctor` should report it available. Put a
   coloured marker on the gripper — `ClassicalLocator` has no appearance model
   of the arm and finds the gripper the same way it finds anything else: as a
   distinctly coloured blob.
8. **Establish the sign of "down".** `--descend-units` defaults to `-200`, which
   is a guess about `SHOULDER_LIFT`'s direction on this arm. Verify it with a
   single small `robohand move` first. Getting this backwards drives the gripper
   *up* on descend and into the table on lift.
9. **Then, and only then:**
   ```bash
   robohand pick --locator classical --target red \
       --i-am-present-and-watching --json
   ```
   Watch it. Cut power the moment anything looks wrong.

The `--i-am-present-and-watching` gate is checked *before* the HID device is
opened, so a pick you did not consent to cannot even reach the hardware.

---

## What will probably go wrong on real hardware

The simulator cannot predict any of this. It is a control-flow test rig, not a
physics engine, and `SimLocator` is explicitly a ground-truth stand-in rather
than a camera model.

**Perception.**

- `SimLocator` has no sensor noise, no lighting variation, no exposure or white
  balance drift, no motion blur, no occlusion, and it never returns a false
  positive. A real HSV threshold does all of those, and the arm's own shadow
  moves across the object as the arm moves.
- The gripper marker will be occluded by the arm itself at some poses. The sim's
  gripper is always visible.
- Frame latency is zero in the sim. A real camera delivers a frame describing
  where the arm *was* 30-100 ms ago, which turns a stable proportional loop into
  a possibly-oscillating one. The convergence guard will catch that as
  `approach_no_convergence` — it will not fix it. Expect to lower `gain`.

**Geometry and mechanics.**

- The sim's forward kinematics are linear: X is proportional to `SHOULDER_PAN`,
  Y to `ELBOW`, Z to `SHOULDER_LIFT`, with fixed coefficients. The real arm is
  none of those things. Joints couple, the measured Jacobian will be
  configuration-dependent in a way the sim never shows, and it will go stale
  faster the further a single step travels.
- Backlash is not modelled at all. A real servo commanded +10 units after moving
  the other way may not move measurably, which the guard may read as a stall.
- Gravity sag is not modelled. Unloading a loaded joint drops the arm; the sim's
  joints hold position when torque is removed.
- The true relationship between a commanded PWM value and actual arm geometry
  has never been measured on this unit. Every raw-unit offset in `PickSettings`
  is a guess with no physical calibration behind it.

**Gripping.**

- The sim models contact as a rigid obstruction at a single GRIP value. A real
  gripper has closing force, jaw compliance, and friction, none of which exist
  here. A soft object will deform; a hard smooth one will squirt out sideways.
- The sim never lets a grasped object rotate, topple, or roll. Real ones do.
- The ladder overshoots contact by one step: the partial motion *into* an
  obstruction still registers as motion, so `contact_position` reads a step past
  where contact actually occurred. Harmless in sim; on real hardware it means
  the re-hold squeezes slightly harder than the detected contact point.

**The two gaps you should know about before trusting any of this.**

1. **`Arm._grip_ladder` unloads GRIP even on a *successful* contact.** As
   shipped, `close_grip()` cannot hold anything through a lift — it detects the
   grasp and then de-energises the jaws. `pick.py` works around it by
   re-commanding GRIP to the reported contact position with `hold=["GRIP"]`
   immediately afterwards, rather than modifying the safety-critical ladder. If
   you write another routine that grasps, you must do the same.
2. **This arm cannot hold anything indefinitely.** A successful pick
   deliberately ends with torque still on, because unloading drops the object.
   But the safety watchdog's `max_continuous_torque_ms` (8 s by default) will
   force an unload regardless of `hold`. A pick is therefore a *momentary*
   grasp, and any place-somewhere-else sequence has to complete inside that
   window or re-grasp. `PickResult.holding` says which state you were left in;
   it is not a claim that the state is stable.

---

## See also

- [safety.md](safety.md) — the three defence rings, the gripper incident,
  and why `SafetyLayer` is the only path to a `SERVO_MOVE`.
- [calibration.md](calibration.md) — measure the joint limits this routine
  refuses to run without.
- [cocktail-robot.md](cocktail-robot.md) — why no VLA can drive this arm, and
  the Gemini Robotics-ER verification (including the hallucination).
- [research-hardware.md](research-hardware.md) — the no-position-feedback
  finding, verified against the real controller.
