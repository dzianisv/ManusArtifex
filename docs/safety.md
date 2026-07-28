# Robo hand safety

## Incident 2026-07-28 — gripper servo stall

**Symptom.** The gripper finger emitted a loud grinding/straining noise. The
operator cut power.

**Offending command.** `robohand pose close_grip`, which drove servo 1 (GRIP) to
raw position **700**. The absolute servo minimum is 500, and the gripper jaws
reach their *mechanical* end stop well before that when closing on empty air.
The servo could not reach the commanded position, so it held locked-rotor
torque — stripping gears and overheating the motor.

**Why nothing caught it.** The LSC-6 controller on this arm does not answer
`CMD_MULT_SERVO_POS_READ` (cmd 21). There is **no position feedback**. Motion is
fully open loop, so a stall is undetectable in software. This is the single most
important fact about this hardware.

**Contributing factors.**
- Poses were guessed, not measured (`pick_approach`, `pick_descend` etc.).
- Several motion commands were chained in one shell run with no confirmation
  between them.
- Limits were set to the full 500..2500 electrical range, which is *not* the
  mechanical range of any joint.

## Rules

1. **Never command a joint near its electrical limit.** Stay at least 200 units
   clear of 500 and 2500. Real mechanical limits are narrower still.
2. **Never close the gripper on empty air.** `close_grip` is a *partial* close
   (1300). Below ~1250 the jaws stall.
3. **Never hold torque on a servo that did not reach its target.** Unload it.
4. **Unload (torque off) when idle.** `reset_arm.py` does this.
5. **No motion without calibration.** Every joint's band must be hand-taught
   with torque off before it is widened past the provisional band.
6. **One motion command at a time.** No chained motion in a single shell run.

## Emergency reset

    python3 reset_arm.py

Sends action-group-stop then multi-servo unload (cmd 20). Commands **no motion**,
so it is safe to run at any time, including mid-stall. Afterwards the joints are
limp and back-drivable by hand.

## Provisional limits

Until calibration, every joint is clamped to **center ±300** (1200..1800). This
band is far from every mechanical end stop on every joint. `calibrated: false`
is recorded per joint in `robohand/data/config.json`.

## Calibration procedure (requires a human)

1. Power the arm on.
2. `python3 reset_arm.py` — torque off, joints limp.
3. For each joint, move it **by hand** to its comfortable minimum, then its
   comfortable maximum, stopping well before anything binds.
4. Record those readings and write them into `config.json`, setting
   `calibrated: true` for that joint.
5. Only then widen the band and re-enable motion.

# Robo hand safety

## Incident 2026-07-28 — gripper servo stall

**Symptom.** The gripper finger emitted a loud grinding/straining noise. The
operator cut power.

**Offending command.** `robohand pose close_grip`, which drove servo 1 (GRIP) to
raw position **700**. The absolute servo minimum is 500, and the gripper jaws
reach their *mechanical* end stop well before that when closing on empty air.
The servo could not reach the commanded position, so it held locked-rotor
torque — stripping gears and overheating the motor.

**Why nothing caught it.** The LSC-6 controller on this arm does not answer
`CMD_MULT_SERVO_POS_READ` (cmd 21). There is **no position feedback**. Motion is
fully open loop, so a stall is undetectable in software. This is the single most
important fact about this hardware.

**Contributing factors.**
- Poses were guessed, not measured (`pick_approach`, `pick_descend` etc.).
- Several motion commands were chained in one shell run with no confirmation
  between them.
- Limits were set to the full 500..2500 electrical range, which is *not* the
  mechanical range of any joint.

## Rules

1. **Never command a joint near its electrical limit.** Stay at least 200 units
   clear of 500 and 2500. Real mechanical limits are narrower still.
2. **Never close the gripper on empty air.** `close_grip` is a *partial* close
   (1300). Below ~1250 the jaws stall.
3. **Never hold torque on a servo that did not reach its target.** Unload it.
4. **Unload (torque off) when idle.** `reset_arm.py` does this.
5. **No motion without calibration.** Every joint's band must be hand-taught
   with torque off before it is widened past the provisional band.
6. **One motion command at a time.** No chained motion in a single shell run.

## Emergency reset

    python3 reset_arm.py

Sends action-group-stop then multi-servo unload (cmd 20). Commands **no motion**,
so it is safe to run at any time, including mid-stall. Afterwards the joints are
limp and back-drivable by hand.

## Provisional limits

Until calibration, every joint is clamped to **center ±300** (1200..1800). This
band is far from every mechanical end stop on every joint. `calibrated: false`
is recorded per joint in `robohand/data/config.json`.

## Calibration procedure (requires a human)

1. Power the arm on.
2. `python3 reset_arm.py` — torque off, joints limp.
3. For each joint, move it **by hand** to its comfortable minimum, then its
   comfortable maximum, stopping well before anything binds.
4. Record those readings and write them into `config.json`, setting
   `calibrated: true` for that joint.
5. Only then widen the band and re-enable motion.

Note: because position read-back does not work on this controller, step 3
currently cannot be automated. Fixing the read path (or adding an external
camera-based joint estimate) is a prerequisite for autonomous calibration.

---

## The redesign: three independent layers of defence

The rules above were policy. They relied on a human remembering them every
time. The incident happened because a human (or a script acting on a human's
behalf) forgot rule 1 and rule 2 in the same command. Policy is not a control.
This section describes the actual **structural** redesign that makes the
incident's specific failure mode — commanding the gripper to a raw extreme and
having it lock rotor against the mechanical stop — impossible to express in
code, not just discouraged in prose.

Nothing here changes the fundamental hardware constraint: **there is still no
position feedback.** `cmd 21` still never replies. Every layer below is built
around that fact, not in denial of it.

```
                          ┌─────────────────────────────────────────┐
                          │              robohand.cli                │
                          │   (doctor / move / grip / pose / pick)   │
                          └───────────────────┬───────────────────────┘
                                              │
                          ┌───────────────────▼───────────────────────┐
                          │                robohand.arm.Arm            │
                          │  shadow position · grip ladder · doctor    │
                          └───────────────────┬───────────────────────┘
                                              │  (never touches transport directly)
     ┌────────────────────────────────────────▼────────────────────────────────────────┐
     │                         robohand.safety.SafetyLayer.move()                       │
     │                          *** THE ONLY CHOKE POINT ***                            │
     │                                                                                   │
     │  LAYER 1 — limits that cannot be bypassed                                        │
     │    • per-joint min/max/center + calibrated:bool  (config.json)                   │
     │    • uncalibrated  → clamp to provisional band (center ±300), REFUSE if outside  │
     │    • global guard 700..2300, hardcoded, not JSON-configurable, never widened      │
     │    • max_step_per_move → large moves ramped into steps                           │
     │    • mints a single-use MotionToken, bound to this transport instance             │
     │                                                                                   │
     │  LAYER 2 — time-bounded torque (a stall that cannot persist cannot destroy)      │
     │    • per-move hold_ms timer → auto-unload unless explicitly told to hold          │
     │    • idle_unload_ms watchdog thread → unload ALL if nothing commanded in time    │
     │    • max_continuous_torque_ms hard cap → forced unload + warning, no exceptions  │
     │    • unload-all on atexit / SIGINT / SIGTERM / unhandled exception / __exit__     │
     │                                                                                   │
     │  LAYER 3 — vision stall detection (guard.VisionGuard, the only real feedback)    │
     │    • frame before / frame after, diff → blur → threshold → count changed px      │
     │    • no motion where motion was commanded → unload immediately, raise            │
     │      StallDetected                                                               │
     │    • degrades safely: no camera → skipped, layers 1+2 still enforced, CLI warns  │
     └───────────────────────────────────────────┬────────────────────────────────────┘
                                                  │  (SERVO_MOVE, valid token only)
                          ┌───────────────────────▼────────────────────────┐
                          │   protocol.HidTransport   /   sim.SimTransport  │
                          │   require_motion_token() — refuses SERVO_MOVE   │
                          │   without a token minted by SafetyLayer above   │
                          └──────────────────────────────────────────────────┘
```

### Layer 1 — limits that cannot be bypassed

Every joint has `min`, `max`, `center`, and a **`calibrated: bool`** in
`robohand/data/config.json`. `robohand/config.py` computes, for every request,
the *effective* bounds as the intersection of three nested rings:

1. **Global guard — 700..2300.** Hardcoded in `config.py` as
   `GLOBAL_GUARD_MIN`/`GLOBAL_GUARD_MAX`. This is the one thing in the whole
   safety stack that is deliberately **not** read from JSON: a careless config
   edit can widen a joint's configured band, but it can never widen this
   outer rail. The electrical range is 500..2500; we stay 200 clear of both
   ends unconditionally.
2. **Configured min/max** from `config.json` (currently 1200..1800 for every
   joint).
3. **Provisional band — center ±300** (`safety.provisional_band` in
   `config.json`), applied only when a joint's `calibrated` flag is `false`.

If a joint is uncalibrated and the requested position falls outside its
provisional band, `SafetyLayer.move()` raises `CalibrationRequired` — it
**refuses**, it does not silently clamp and move. Clamping still happens one
layer deeper (`config.clamp`) as a pure defense-in-depth backstop for internal
ramp-step rounding, but the primary, user-visible behaviour for an explicit
out-of-band request is a refusal with a clear message, so an operator never
gets false confidence that a command succeeded when it was actually
redirected. `robohand move`/`grip`/`pose` on the CLI surface this as a non-zero
exit code.

Large moves are never sent as one packet: `max_step_per_move` (default 400)
splits any move whose delta exceeds it into a ramp of smaller steps, each
independently re-clamped through the same three rings.

**The choke point.** `SafetyLayer.move()` is the only method in the codebase
allowed to produce a `MotionToken` (`protocol.MotionToken`, minted by
`SafetyLayer._mint_token()`). Both `protocol.HidTransport.move_servos()` and
`sim.SimTransport.move_servos()` call the same shared function,
`protocol.require_motion_token()`, before they will send a SERVO_MOVE packet —
if the token is missing, was minted for a different transport instance, or has
already been spent (tokens are single-use), they raise
`protocol.SafetyBypassError`, a `PermissionError` subclass. This is enforced by
the transport itself, at runtime, not by code review or convention:
`tests/test_no_bypass.py` calls `SimTransport().move_servos(...)` directly,
with no token, and asserts it raises. There is no code path — `Arm`,
`sequences.py`, the CLI, a REPL, a future script — that can reach a real
SERVO_MOVE without first passing through every Layer-1 check inside
`SafetyLayer.move()`, because there is no other way to obtain a valid token.

### Layer 2 — time-bounded torque

The insight this layer is built on: **a stall that cannot persist cannot
destroy the servo.** We cannot detect a stall from position feedback (it does
not exist), so instead of trying to detect it, we bound how long any servo is
ever allowed to hold torque without an explicit, ongoing reason to.

- **Per-move hold timer.** Every completed move schedules an automatic unload
  `hold_ms` (default 2000 ms) later, unless the caller explicitly listed that
  joint in `hold=[...]` (e.g. mid-ladder, or a pose that must actively resist
  gravity for a moment).
- **Idle watchdog.** A background thread unloads *every* servo if no new
  command has arrived within `idle_unload_ms` (default 5000 ms) — this is the
  backstop for "the process is stuck, crashed into an infinite loop, or the
  operator walked away."
- **Hard continuous-torque cap.** `max_continuous_torque_ms` (default 8000 ms)
  forces an unload and logs a warning regardless of any hold request — no
  `hold=[...]` list can keep a servo energized forever.
- **Every exit path unloads.** `atexit`, `SIGINT`, `SIGTERM`, an unhandled
  exception propagating out of a `with Arm() as arm:` block, and normal
  `__exit__` all call `unload_all()` before the process/context actually ends.
  Each of these is exercised by a dedicated test
  (`tests/test_unload_on_exit_and_exception.py`,
  `tests/test_unload_on_sigint.py`).

One real subtlety, worth documenting because it is not obvious from reading
`_on_signal` alone: `SafetyLayer` installs its SIGINT/SIGTERM handlers with
`signal.signal(sig, self._on_signal)`, which returns whatever handler was
previously installed. In a plain Python process, that is normally
`signal.default_int_handler` — itself callable. So after unloading,
`_on_signal` *chains* to it, which raises `KeyboardInterrupt` at the
interrupted call site, exactly as if `SafetyLayer` were not there at all —
except that by the time that exception is raised, every servo has already
been unloaded. Only if there truly was no previous handler does it fall back
to `raise SystemExit(128 + signum)`. The guarantee is "unload happens before
anything else can react to the signal," not any particular exit code.

### Layer 3 — vision stall detection

There is no position feedback, so the only real signal about whether a
commanded move actually happened is a camera watching the arm. `robohand/guard.py`
implements `VisionGuard`:

1. Capture a frame before the move.
2. Command the move (through Layer 1+2 as normal).
3. Capture a frame after.
4. Restrict comparison to a configurable ROI (`vision.roi` in `config.json`,
   fractional or pixel coordinates), take `cv2.absdiff`, Gaussian-blur to
   suppress sensor noise, threshold, and count changed pixels
   (`vision.diff_threshold`, `vision.min_changed_pixels`).
5. If the changed-pixel count is below threshold where motion was commanded,
   conclude **stall or obstruction**, unload the implicated servo(s)
   immediately, and raise `guard.StallDetected`.

This degrades safely by construction: `VisionGuard.available()` is `False`
whenever no `frame_source` was supplied (no camera opened), and every
`check()` call is then a no-op — Layers 1 and 2 remain fully active on their
own, and `Arm`/`cli.py doctor` print an explicit warning that the process is
running without visual stall detection. Vision guard is never required for
Layers 1/2 to work; it is a strict addition when available.

`Arm` auto-disables vision guard when running against the simulator unless a
caller explicitly asks for it (`use_vision=True`): a real camera pointed at an
empty desk while a fake transport pretends to move would otherwise report
every simulated move as a false stall.

### Gripper-specific: the incremental grip ladder

`close_grip()`/`open_grip()` never command an absolute target — that is
precisely the shape of the original incident. Instead `Arm._grip_ladder()`
steps the gripper by a small increment (`gripper.step`, default 40 units),
checking the vision guard after each step (when available). The first step
that produces no visible motion is treated as contact with an object *or*
arrival at the mechanical end stop — either way, the ladder immediately backs
off one increment and unloads. `gripper.grip_min_safe` /
`gripper.grip_max_safe` bound the ladder unconditionally, independent of
whether a camera is attached at all: with no vision guard, the ladder still
stops at these hard backstops rather than driving toward the mechanical stop.

Ladder steps deliberately bypass `SafetyLayer.move()`'s own *automatic*,
hard-fail vision check (`Arm.set_positions(..., vision_guard=None)`) — that
automatic check is designed for arbitrary one-shot moves, where "no observed
motion" always means something is wrong. For the ladder, "no motion" on a
given step is the *expected, successful* way for the ladder to end (contact),
so the ladder implements its own graceful handling instead of racing against,
and being pre-empted by, the stricter automatic check.

## Simulator

`robohand/sim.py` provides `SimTransport`, a drop-in replacement for
`HidTransport` implementing the identical interface (`move_servos`,
`unload_servos`, `get_positions`, `get_battery_voltage`). It models:

- An **electrical range** per joint (500..2500) and a narrower **mechanical**
  range (`DEFAULT_MECHANICAL_LIMITS`, e.g. the gripper's jaws physically stop
  at 1150/1850).
- **Locked-rotor stall**: commanding beyond the mechanical range accrues
  `stall_ms` equal to the full commanded duration (the sim has no idea it
  failed either — same as the real open-loop controller).
- An optional **obstruction** (`set_obstruction()`), representing a grasped
  object short of the mechanical stop. Contact with an obstruction does *not*
  accrue stall time: gripping and holding against an object is intended
  gripper behaviour, not a destructive event. This distinction is what lets
  `tests/test_grip_ladder.py` assert the ladder correctly stops on contact
  with **zero** stall time.
- `get_positions()` always raises `NoResponse` — matching the verified real
  hardware behaviour, so no test can accidentally depend on position feedback
  that does not exist.
- A full command log (`.log`) and `stall_ms_total()` / `any_torque_on()` for
  test assertions.
- The exact same `require_motion_token` choke point as the real transport.

Selected via `ROBOHAND_TRANSPORT=sim` or `--sim`. The real `HidTransport`
remains the default so nobody accidentally ships simulator behaviour.
`--dry-run` layers a `DryRunTransport` on top that also prints every packet
and never persists shadow state to disk.

## What is proven, and what is not

`docs/verification-sdk.md` has the full pytest output. In short: the exact
incident (raw 700/2200 to GRIP) is refused, zero stall time is ever recorded
against the simulator, the choke point cannot be bypassed, every exit path
unloads, the watchdog unloads on idle, large moves are ramped, uncalibrated
joints refuse out-of-band requests, vision guard detects synthetic no-motion,
and the grip ladder stops/backs off/unloads on simulated contact.

This does **not** prove the real servos behave identically to the simulator's
model — the mechanical stop positions in `sim.py` are plausible estimates, not
measured values (measuring them safely requires the calibration procedure
above, done by hand with a human watching). It does not cover self-collision
between links, a rigid object being crushed with vision guard unavailable, or
the arm falling under gravity the instant a joint unloads. See
`docs/verification-sdk.md` for the full, blunt list of residual risk.
