# Vision-assisted calibration

**Status: semi-automated, supervised, approximate.** This routine does not solve
calibration. It automates the tedious part of it, conservatively, and it fails
closed. It still needs a human physically present at the arm, and its output is
still worth cross-checking against the manual procedure in
[safety.md](safety.md#calibration-procedure).

Read [safety.md](safety.md) first. This document assumes you know why the
gripper incident happened and what the three defence layers do.

---

## Why this exists

Every joint in `robohand/data/config.json` is `"calibrated": false` and clamped
to a narrow provisional band, because the **true mechanical limits are
unknown**. They are narrower than the electrical range, and nobody has measured
them. Widening a band without measuring it is exactly how a servo ends up
stalled against a stop.

Measuring them the documented way needs a human to move each joint by hand and
write down numbers. That works, and remains the reference method, but it is
slow and it is the reason nothing has actually been calibrated yet.

The blocker was never the arm. It was the missing sensor:

```
  what an encoder would give you            what we have
  ----------------------------------        ------------------------------
  command 1470  ->  read back 1470          command 1470 -> silence
  command 1440  ->  read back 1452  <-stop  command 1440 -> silence
                    (position != target
                     => you are at a stop)
```

`CMD_MULT_SERVO_POS_READ` (cmd 21) never replies on this controller — there is
no feedback wire. `SimTransport.get_positions()` raises `NoResponse` for the
same reason, deliberately. So the controller cannot tell you where a joint is,
and therefore cannot tell you when a joint stopped moving.

A camera can, crudely:

```
  before burst        after burst         verdict
  -----------         -----------         -------
  frame A             frame B             pixels changed -> it moved
  frame A             frame A'            nothing changed -> maybe a stop
```

That is the entire idea. Command a small increment, look, and if nothing
visibly changed, treat it as a *candidate* stop. It substitutes an external,
noisy, indirect observation for a missing internal, precise, direct one, and
everything awkward about this design follows from that substitution.

---

## What the routine actually does

`robohand/calibrate.py`, per joint, once downward and once upward:

1. Move to the joint's configured centre, clamped into its searchable band.
   Both directions start from the same known-safe place, so they are
   independent measurements rather than one continuous sweep.
2. Step by `step` raw units (default 30) toward the bound.
3. **Unload the servo**, then observe. Motion since the previous frame means
   the joint reached the commanded position; record it as confirmed-moving and
   keep stepping.
4. No motion is *not* immediately a limit. Require `confirmations` consecutive
   no-motion observations (default 2, minimum 2 — a value of 1 is refused).
5. Before recording anything, run the **observer-reliability burst** (below).
6. Record `limit = last_confirmed_moving_position -/+ margin` (default margin
   60 = two steps), retreat there, and unload.

The searchable band comes from `config.effective_bounds(name,
allow_uncalibrated_override=True)`, intersected with
`config.absolute_bounds(name)`. The search **cannot** leave `absolute_bounds`;
`SafetyLayer` would refuse the move regardless, and the routine clamps before
it asks.

### Two findings, not one

The report distinguishes them and you must not conflate them:

| finding | meaning |
|---|---|
| `bounded_by_mechanical_stop` | a stop was observed, and the recorded limit sits `margin` short of it. This is a **measurement**. |
| `bounded_by_electrical_limit` | the search ran out of legal room before finding a stop. This is **"we did not find out"** — the joint may travel further. |

An `bounded_by_electrical_limit` result is not a calibration of that end. It is
a statement that the joint's real limit is somewhere beyond the band the search
was allowed to explore, and that a wider search (or the manual procedure) would
be needed to find it.

### Default constants and why

| setting | default | reasoning |
|---|---|---|
| `step` | 30 | Between `config.GRIP_STEP` (40, the increment the grip ladder already uses in anger) and something finer. Ten steps crosses a 300-unit provisional band, so a full direction is a bounded, inspectable number of bursts, and the worst case overshoot into a stop is one step. |
| `margin` | 60 | Two steps of clearance below the last position that verifiably moved. The stop itself is somewhere in the one-step gap past that position, so the recorded limit sits roughly 2–3 steps short of the truth. Deliberately pessimistic: under-claiming travel is safe, over-claiming is not. |
| `confirmations` | 2 | One miss is indistinguishable from a dropped frame or a shadow. Minimum enforced at 2. |
| `step_duration_ms` | `config.MIN_DURATION_MS` (400) | See torque discipline below. |
| `verification_moves` | 6 | Size of the reliability burst. |
| `max_verification_misses` | 1 | Absorbs one unlucky frame; more means the observer is unusable. |

### Precision you should actually expect

Against the simulator, with a perfect observer, the recorded limit lands
**within roughly one step plus the margin** of the true mechanical stop — about
30–90 raw units short, on the safe side, always inside the true range.
Representative sim runs:

| joint | true stop | recorded | short by |
|---|---|---|---|
| GRIP | 1723 / 2131 | 1770 / 2100 | 47 / 31 |
| ELBOW | 1287 / 1712 | 1320 / 1680 | 33 / 32 |
| SHOULDER_PAN | 1341 / 1663 | 1380 / 1620 | 39 / 43 |
| WRIST_FLEX | 1252 / 1749 | 1290 / 1710 | 38 / 39 |

Those numbers are the *simulator's* accuracy, against a simulator whose stops
are exact and whose observer is noiseless. **On real hardware they are an upper
bound on how good this can be, not a prediction.** A real camera adds
detection latency, servo settle time adds more, and gravity adds sag (below).
Do not quote the sim numbers as hardware accuracy.

---

## Torque-burst discipline

This is the part that matters, and it is the direct answer to the 2026-07-28
incident.

```
  the incident                        this routine, per step
  ------------------------------      ------------------------------------
  one command, wrong target           small increment, always in-bounds
  torque ON                           torque ON  ......... <= 400 ms
  |                                   torque OFF (unload)   <-- always
  |  stalled, locked rotor            |
  |  drawing stall current            |  observe here, at rest, no torque
  |                                   |
  V  until the operator cut power     next step, or stop
```

Three properties, each independently enforced:

- **Bounded per step.** `step_duration_ms` must be under `MAX_STEP_DURATION_MS`
  (500). If a servo is already against a stop, that is the *entire* exposure
  for that step.
- **Unload before observe.** The servo is unloaded via
  `SafetyLayer.unload([name])` **before** the after-frame is captured. Torque is
  never on while the routine is thinking. There is no code path where a "hold
  still and look" turns into an indefinite hold.
- **Bounded in aggregate.** The run tracks cumulative commanded torque time and,
  where the transport exposes it, cumulative stall time. Exceeding either cap,
  or the wall-clock cap, aborts.

A note on honesty about `step_duration_ms`: the spec asked for 150–250 ms, but
`config.MIN_DURATION_MS` is **400** and `SafetyLayer.move()` silently raises any
shorter duration to it. Shipping a 200 ms default would have been a lie in the
config file — the hardware would still see 400. So the default *is* 400, values
below `MIN_DURATION_MS` are **refused** rather than silently bumped, and if you
want genuinely shorter bursts you must lower `MIN_DURATION_MS` (a safety-policy
change, made deliberately, not a side effect of a calibration flag).

Measured: a full two-direction single-joint run against the simulator accrues
**at most 2400 ms** of stall time, and the test suite asserts that number
directly. Derivation: per direction at most one partial step lands on the stop,
then exactly `confirmations` (2) more push into it — 3 stalling commands, ×2
directions, ×400 ms. Every other move in the run (the initial move to centre,
the reliability burst, the retreat) targets a reachable position and accrues
none. The reliability burst deliberately oscillates *inside* already-confirmed
travel precisely so that verification costs time but not stall.

Compare with the incident: unbounded, until a human intervened.

---

## The observer-reliability burst

`N` consecutive no-motion reports only mean "mechanical stop" if the observer's
own miss rate is low. Silence from a blind camera looks exactly like silence
from a stopped joint. Early versions of this routine checked that with a single
retreat probe; against an observer dropping half its reports that is just one
more coin flip, and it duly produced a fabricated "measurement" (recorded 1410
for a true stop at 1287).

So before any limit is recorded, the routine **measures the observer**:

```
  suspected stop somewhere here
                              |
    ... ---[far]---[near]---[?]|
            ^       ^
            |       |  both positions already observed to move
            +-------+  oscillate between them 6 times
                       every move is known-real motion
                       => every "no motion" is the observer's error
```

More than `max_verification_misses` (1) errors out of `verification_moves` (6)
and the run aborts without writing anything. Empirically, over 60 seeds with an
observer that drops 50% of real motion: **60 aborts, 0 completed runs, 0
falsely-narrow results.** An observer that misses about 1 in 7 rides through and
still finds the correct band.

"I cannot see" must never be recorded as "I measured a limit." That is the
whole point of this mechanism.

---

## Abort conditions

Any of these aborts the run:

- cumulative torque-time budget exceeded (`max_torque_ms`);
- cumulative stall-time budget exceeded (`max_stall_ms`, where the transport
  reports stall);
- wall-clock cap exceeded (`max_wall_clock_s`);
- the observer raises, or has no frame available;
- the observer fails the reliability burst;
- too many ambiguous observations across the run (`max_ambiguous_observations`);
- `SafetyLayer` refuses a step (`SafetyViolation`, `PositionOutOfRange`,
  `CalibrationRequired`, ...) — surfaced, never swallowed;
- no room to build a known-motion probe.

**On any abort: every involved joint is unloaded and NO result file is
written.** A partial calibration is worse than none, because a partial
calibration looks like a measurement.

---

## Output, and promotion

A successful run writes a timestamped file — `robohand/data/calibration-<UTC
timestamp>.json` — and **does not touch `config.json`**. Per joint it records
the discovered min and max, the margin applied, how each end was determined
(`bounded_by_mechanical_stop` vs `bounded_by_electrical_limit`), steps taken per
direction, ambiguous-observation count, and torque time spent.

Promotion is a separate, explicit action:

```sh
robohand --sim calibrate --joint GRIP              # measure, writes a file
robohand calibrate --apply robohand/data/calibration-....json   # promote
```

`apply_calibration()`:

- re-validates every min/max against `config.absolute_bounds` — a hand-edited
  file claiming a wider range than the servo's absolute bound is **refused**,
  not clamped;
- runs the same per-servo validation `config.py` applies at load time;
- writes atomically, then re-reads and re-validates the whole file;
- sets `calibrated: true` only for the joints it promoted, and rewrites their
  stale "PROVISIONAL — NOT calibrated" comments;
- **refuses to promote a simulator-derived result** unless you pass
  `--allow-simulated`. Simulated limits are assumptions about this arm, not
  measurements of it. Promoting them would put fiction into the file that Layer
  1 trusts.

---

## Running it on real hardware

Non-negotiable preconditions:

- **A human is physically present, watching the arm, with the power switch in
  reach.** This is automation that deliberately drives joints toward their
  stops. It is bounded, but bounded is not the same as safe.
- The workspace is clear and **the gripper jaws are empty and unobstructed** —
  see the ambiguity below.
- A working camera with the arm in frame and an ROI that actually contains the
  joint being calibrated.

The CLI enforces the first one structurally: without `--sim`, `robohand
calibrate` refuses to run unless `--i-am-present-and-watching` is passed, and
exits non-zero with an explanation. That flag is a statement of fact by the
operator, not a formality.

---

## Failure modes — where this gets fooled

Read this section before trusting any number this routine produces.

### Gripper closed on an object vs gripper at its mechanical stop

**This is not resolved by this design, and cannot be resolved by vision alone.**

Both look identical: the jaws stop moving, the frame stops changing. There is no
current sensor, no force sensor, and no position feedback to distinguish "the
servo reached its rotational limit" from "the jaws met something rigid". The
simulator models these as different things (`mech_min`/`mech_max` accrue stall,
`obstruction_min`/`obstruction_max` register a contact event), but that
distinction exists only because the simulator knows the ground truth. The
routine, looking through a camera, does not and cannot.

The consequence: **calibrating GRIP with anything in the jaws will record the
object's width as the gripper's mechanical limit.** The mitigation is entirely
operational — run GRIP calibration with the jaws empty — and it is not enforced
by anything in software. Solving it properly needs a different sensor (stall
current, a force-sensitive jaw, a limit switch), not better vision.

GRIP is also where the original incident happened, so: the downward search on
GRIP can never go below `config.absolute_bounds("GRIP")[0]` = **1500**. That is
enforced by `config.py`, by `SafetyLayer`, and again by the routine's own
clamping, and asserted in the test suite (no MOVE packet in any calibration test
ever carries a GRIP target below 1500). The incident's value, 700, is not
expressible.

### Occlusion

Anything between the camera and the joint — the arm's own links at certain
poses, a cable, a hand reaching in, the workpiece — makes a moving joint look
stopped. The reliability burst catches *persistent* occlusion. It does not catch
occlusion that happens to start exactly when the search nears the stop, which is
the case that produces a plausible-looking wrong answer.

### A joint that is nearly invisible from this angle

A joint rotating about an axis pointing at the camera moves very few pixels.
WRIST_ROLL viewed end-on is the obvious case. Frame differencing on a small
number of changed pixels is noise-sensitive, and `min_changed_pixels` cannot be
tuned high enough to reject noise and low enough to see the motion at the same
time. The reliability burst will usually abort here rather than produce a wrong
answer — that is the intended outcome, and it means **some joints are simply not
calibratable from a given camera position.** Move the camera, or calibrate that
joint by hand.

### Lighting

Auto-exposure, auto-white-balance, a cloud, a monitor changing, someone walking
past — any of these change enough pixels to read as motion. This produces
false *positives*: the search believes the joint moved when it did not, keeps
stepping, and can walk into a stop for more steps than intended. The
per-step torque bound and the aggregate stall budget cap the damage; they do
not prevent the wrong reading. Fixed lighting and a locked-exposure camera are
a real requirement, not a nicety.

### Gravity sag after unload

This one is inherent to the design, not a bug in it. The routine unloads before
observing, on purpose. A joint carrying weight will sag the moment torque
drops. That sag is real motion, in the frame, at exactly the moment the routine
is asking "did it move?" — so a joint at a hard stop can still read as moving.
On a loaded SHOULDER_LIFT or ELBOW this is likely, and it would cause the search
to keep stepping into a stop rather than stopping at it. The torque and stall
budgets bound the consequence. Calibrate with the arm unloaded and in a pose
where the joint under test is not fighting gravity.

### What none of this covers

Self-collision between links, the arm falling under gravity when a joint
unloads, and the difference between the simulator's model of a servo and the
actual servo. See safety.md's "What is proven, and what is not".

---

## Simulator vs hardware

Everything above has been exercised **only against `SimTransport`**. The
simulator has exact mechanical stops, a noiseless observer, and no gravity. The
accuracy table is a statement about the algorithm's logic, not about this arm.

The honest summary: this routine is a plausible, conservative, well-bounded way
to *propose* calibration numbers, which a human should then sanity-check by hand
before promoting. Treat its output as a first draft with error bars of unknown
width, not as a measurement.
