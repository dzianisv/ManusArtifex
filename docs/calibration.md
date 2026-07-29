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

### Where the search is allowed to look

`calibrate.search_bounds(name)` returns **`config.absolute_bounds(name)`** —
`(700, 2300)` for every joint except GRIP, which is `(1500, 2300)`. That is the
hardcoded `GLOBAL_GUARD_MIN`/`GLOBAL_GUARD_MAX` ring intersected with each
servo's `ELECTRICAL_LIMITS`, and **`config.json` cannot widen it**.

It deliberately does *not* use the configured band or
`config.effective_bounds(..., allow_uncalibrated_override=True)`. Both of those
are anchored to `config.JOINT_LIMITS[name].min/max`, and in the shipped
`config.json` every uncalibrated joint's configured band is numerically
identical to its provisional band (`centre ± 300` = `[1200, 1800]`). A search
confined there **cannot reach a real mechanical stop on real hardware**, because
the provisional band exists precisely to stay clear of limits nobody has
measured. A run so confined would stop at its own fence, report that it ran out
of room, and write `[1260, 1740]`-ish numbers that look like a measurement and
are not one. (That was a real bug; see the `bounded_by_policy_limit` note below
and the schema bump to `robohand-calibration-v2`.)

Reaching the rail needs an explicit, narrowly-scoped capability:
`SafetyLayer.move(..., probe_absolute_bounds=True)`. It:

- still requires `allow_uncalibrated_override=True` **and** a non-empty
  `override_reason`, and lands in the same `override_log` and the same WARNING
  line as every other override (annotated `ABSOLUTE-RAIL PROBE`);
- only replaces the *initial range check's* ceiling. The final, unconditional
  `config.absolute_bounds` backstop that raises `OutOfGlobalGuard` is **not**
  gated on it and is not weakened in any way. That backstop is what guarantees
  GRIP can never be commanded below 1500 no matter what `calibrate.py` computes;
- defaults to `False`, is not exposed on `robohand move` or any other CLI
  subcommand, and `robohand/calibrate.py` is the only file in the package that
  ever passes it. A test asserts that last property structurally.

### Two findings, not one

The report distinguishes them and you must not conflate them:

| finding | meaning |
|---|---|
| `bounded_by_mechanical_stop` | a stop was observed, and the recorded limit sits `margin` short of it. This is a **measurement**. |
| `bounded_by_policy_limit` | the search ran out of room **we are willing to command** before finding a stop. This is **"we did not find out"** — the joint may travel further. |

`bounded_by_policy_limit` is named for what it is. The bound it hit is
`config.absolute_bounds()`, which for every joint except GRIP's floor is the
`GLOBAL_GUARD_MIN`/`MAX` **policy** ring of 700/2300 — deliberately conservative,
and *narrower* than the servo's own `ELECTRICAL_LIMITS` (500/2500 for most,
1500/2500 for GRIP). So this finding means:

> we ran out of range we are willing to command

and it does **not** mean:

> we hit the servo's electrical or mechanical rating

Those are different claims. The finding was previously called
`bounded_by_electrical_limit`, which invited exactly the second reading. It was
renamed, and the calibration file schema was bumped to
`robohand-calibration-v2` so that no file written by the older, wrong version
can be loaded or promoted into `config.json`.

(One special case: GRIP's floor of 1500 *is* simultaneously the policy floor and
a real, confirmed mechanical/electrical stop — see `docs/research-hardware.md`.
The label stays generic anyway, because the algorithm has no independent way to
tell which case it is in.)

### Default constants and why

| setting | default | reasoning |
|---|---|---|
| `step` | 30 | Between `config.GRIP_STEP` (40, the increment the grip ladder already uses in anger) and something finer. The worst case overshoot into a stop is one step, i.e. 30 raw units. A full direction now sweeps up to `ceil(800/30)` = 27 steps to cross the widest centre-to-rail span — still a bounded, inspectable number of bursts. |
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

A full `robohand --sim calibrate --all` against `sim.DEFAULT_MECHANICAL_LIMITS`,
at the shipped production defaults:

| joint | true stop | `absolute_bounds` | recorded | short by | finding (min / max) |
|---|---|---|---|---|---|
| GRIP | 1500 / 2500 | 1500 / 2300 | 1560 / 2240 | 60 / **260** | policy / policy |
| WRIST_ROLL | 900 / 2100 | 700 / 2300 | 960 / 2040 | 60 / 60 | mechanical / mechanical |
| WRIST_FLEX | 900 / 2100 | 700 / 2300 | 960 / 2040 | 60 / 60 | mechanical / mechanical |
| ELBOW | 850 / 2150 | 700 / 2300 | 900 / 2100 | 50 / 50 | mechanical / mechanical |
| SHOULDER_LIFT | 850 / 2150 | 700 / 2300 | 900 / 2100 | 50 / 50 | mechanical / mechanical |
| SHOULDER_PAN | 800 / 2200 | 700 / 2300 | 840 / 2160 | 40 / 40 | mechanical / mechanical |

GRIP's max row is the odd one out and is explained under **The GRIP asymmetry**
below. It is not a precision failure — the routine is forbidden from looking
that far.

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

### Stall time does not grow with the search span

This is the load-bearing safety claim, and it is worth stating precisely
because it is counter-intuitive.

Widening the search from the 300-unit provisional band out to the rail
multiplies the number of *steps* by roughly four. It does **not** multiply the
number of *stalling* steps at all.

Stall only accrues while the commanded target is past the true mechanical stop.
The first command that overshoots pins the joint at the stop, so every later
step in that direction is a no-motion miss, and the direction terminates after
`confirmations` of them. Per direction the bound is therefore:

```
  1 overshoot-that-still-moved  +  confirmations (2) misses  =  3 packets
```

— a number set by `confirmations`, not by how far the search may travel. Every
other move in the run (the initial move to centre, the re-centring between
directions, the reliability burst, the retreat) targets a reachable position and
accrues none. The reliability burst deliberately oscillates *inside*
already-confirmed travel precisely so that verification costs time but not stall.

A test asserts this directly: a stop 650 units from centre accrues exactly as
much stall as one 170 units away, despite sweeping three times as many steps.

```
  per direction   3 x 400 ms  =  1,200 ms
  per joint       x2          =  2,400 ms   <- test asserts this exactly
  --all           x6 joints   = 14,400 ms   <- max_stall_ms default
```

Compare with the incident: unbounded, until a human intervened.

### Measured run budgets

All measured under `--sim` at the shipped production defaults (`step=30`,
`step_duration_ms=400`, `margin=60`, `confirmations=2`, real
`config.SETTLE_MARGIN_MS`) — **not** the accelerated fixture the tests use.

| scenario | torque | stall | wall clock |
|---|---|---|---|
| one joint (ELBOW, true stops 850/2150) | 25,200 ms | 2,400 ms | 31.9 s |
| `--all` vs `sim.DEFAULT_MECHANICAL_LIMITS` | 137,600 ms | 10,400 ms | 174.2 s |
| `--all`, worst case¹ | 151,600 ms | 14,000 ms | 191.9 s |

¹ a real stop placed as deep inside the rail as it can be on every joint and
direction, so every probe sweep *and* every verification burst runs in full.

Both the pre-fix defaults were below even a *realistic* run, because they were
sized for a search that could only crawl over the provisional band. Current
defaults, with their arithmetic:

| budget | default | derivation |
|---|---|---|
| `max_stall_ms` | 14,400 | 6 joints × 2 directions × (1 + `confirmations`) × 400 ms |
| `max_torque_ms` | 180,000 | 6 joints × (2 × (27 probe + 6 verification + 1 retreat) + 6 re-centring ramp packets) × 400 ms = 177,600, rounded |
| `max_wall_clock_s` | 900 | the 444-packet ceiling is ~222 s of commanding at 400+100 ms/packet; ×2 leaves room for a real (slow) camera observer |
| `max_ambiguous_observations` | 16 | a joint now makes ~64 observations rather than ~28, so the old cap of 8 would have silently halved the tolerated observer error rate |

**Expected run time: about 30 s per joint, about 3 minutes for `--all`**, under
`--sim` with the instant `SimMotionObserver`. On hardware, add the real camera's
per-observation latency (~440 observations for `--all`) — budget 5–10 minutes and
watch the whole thing.

### A note on the re-centring move

Between the two search directions the routine returns the joint to centre. That
move can now span up to 800 units, and `SafetyLayer` ramps anything over
`max_step_per_move` (300) into several back-to-back packets with no unload
between them. This is the one place in the run where torque stays on across more
than one packet.

It is bounded and it is not a probe: the path is exclusively through territory
the joint has *already been watched* to traverse, moving *away* from the stop,
and `config.MAX_CONTINUOUS_TORQUE_MS` (8,000) bounds it independently. The
torque budget accounts for it exactly — `_torque_burst` charges
`packets × per_packet_duration` as computed by `SafetyLayer.plan_ramp`, not one
nominal step duration, and a test asserts `budget.torque_ms` equals the summed
duration of every MOVE packet in the transport log.

### Is 400 ms per burst actually safe? An honest answer

**We are keeping `step_duration_ms` at 400 ms**, and here is the unvarnished
reasoning.

Before the `search_bounds` fix, this routine could never actually reach a real
mechanical stop — so the torque-burst-then-unload discipline, although correctly
implemented, was never once exercised against a genuine stall. After the fix it
will be, on every joint and every direction that has a stop inside the rail, by
design. That is the entire point of the change, and it means the burst duration
now matters in a way it previously did not.

What the hardware data supports (`docs/research-hardware.md`): the LDX-218 is
rated 17 kg·cm at 7.4 V with a minimum working current of 1 A; the LFD-06's
closest documented relative, the LFD-01M, quotes a locked-rotor current of
≤1.1 A and claims a "built-in anti-blocking algorithm". That is thin. It gives
no stall-duration derating curve, no thermal time constant, and no independent
confirmation that the anti-blocking claim is real or that it applies to the
LFD-06 we actually have.

So: **we cannot fully validate burst-duration safety without hardware.** What we
can do, and have done, is bound the worst case — 400 ms per stalling packet,
at most 3 stalling packets per joint per direction (a bound set by
`confirmations`, not by the search span), 2,400 ms per joint and 14,400 ms
across a full `--all`, with an unload between every single one. Anyone who tells
you more than that from the datasheets we have is guessing.

Why not shorten it? `config.MIN_DURATION_MS` is 400 and `SafetyLayer.move()`
silently raises anything shorter to it, so 400 ms is the practical floor without
changing `MIN_DURATION_MS` — a global safety-policy change affecting every
caller in the package, far outside the blast radius of this fix. Shipping a
smaller number here would be a lie in the config file: the hardware would still
see 400.

Why not lengthen it, or reduce `confirmations` to save steps? Neither helps.
`confirmations` stays at its enforced minimum of **2** — one miss is
indistinguishable from a dropped frame, and it is also the multiplier on total
stall exposure, so lowering it is not even available and raising it would be
paid for directly in stall time.

**The mitigation that actually carries the risk here is operator supervision.**
`robohand calibrate` requires `--i-am-present-and-watching` on real hardware for
exactly this reason. Run it with a hand on the power switch. The first hardware
run of this routine is the first time these bursts have ever hit a real stop.

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
(`bounded_by_mechanical_stop` vs `bounded_by_policy_limit`), steps taken per
direction, ambiguous-observation count, and torque time spent.

The file carries `"schema": "robohand-calibration-v2"`. `load_calibration()`
refuses anything else, which is deliberate: `v1` files were produced by the
version whose `search_bounds` could not reach past the provisional band, so
their numbers are placeholders wearing a measurement's clothes. They must never
be promotable, even by accident. Re-run the calibration instead.

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

## The GRIP asymmetry — a known, permanent limitation

GRIP is the one joint where the two ends behave differently, and neither
behaviour is a bug to be fixed later. Both are consequences of where our own
policy ring sits relative to GRIP's real travel.

```
        1500        1560              2240   2300      2500
         |           |                  |     |          |
   ------+===========+==================+=====+~~~~~~~~~~+------
         ^           ^                  ^     ^          ^
   policy floor  recorded min     recorded max |    TRUE open stop
   == TRUE                                policy ceiling
   closed stop                            (stricter than the hardware)

   ===  searchable          ~~~  real travel we refuse to command
```

**The low end (1500) is discoverable, and coincides with a real stop.**
`absolute_bounds("GRIP")[0]` is 1500 because `ELECTRICAL_LIMITS[1]` is
`(1500, 2500)` — and per `docs/research-hardware.md` 1500 is also the confirmed
true mechanical closed stop. So the search sweeps down, commands 1500, the joint
visibly moves to it, and the direction terminates on "reached the bound while
still moving". It reports `bounded_by_policy_limit` at `1500 + margin` = **1560**.

Note what *does not* happen: the routine never observes a no-motion event at
this end, because it runs out of legal room on the very command that lands on
the stop. So the finding is `bounded_by_policy_limit`, not
`bounded_by_mechanical_stop` — which is the honest answer. The algorithm has no
way to know that the policy floor and the mechanical floor happen to be the same
number here; it only knows it ran out of room. Zero stall time accrues.

If a particular arm's real closed stop sits *above* 1500 — say at 1610 — that
one **is** discoverable and is correctly reported as
`bounded_by_mechanical_stop`. (Before the `search_bounds` fix it was not: the
old search floor was 1600.)

**The high end (2500) is NOT discoverable, permanently.** GRIP's true open stop
is at 2500, but `GLOBAL_GUARD_MAX` is 2300, so `absolute_bounds("GRIP")[1]` is
2300 — **our policy ceiling is stricter than the hardware**. The search reaches
2300, reports `bounded_by_policy_limit`, and records `2300 - margin` = **2240**.
It cannot go further and it should not: we refuse to command GRIP that high
regardless of what the servo could physically do.

This is intentional and permanent. It means the recorded GRIP maximum is
~260 units short of the truth, and that shortfall is a *policy* decision, not a
measurement error. Do not "fix" it by widening `GLOBAL_GUARD_MAX`; that guard is
the outermost ring in the whole safety design and the 2026-07-28 gripper
incident is the reason it exists.

The practical consequence: **GRIP's open end will never be calibrated by this
routine.** If you need the true 2500 figure, it must come from the manual
procedure, recorded deliberately, with the guard consequences understood.

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
