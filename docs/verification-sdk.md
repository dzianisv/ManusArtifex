# Verification — robohand safety redesign

All validation in this document runs against `robohand.sim.SimTransport`. **No
command in this document, or in `tests/`, ever opens the real HID device (VID
0x0483 / PID 0x5750).** The arm's power was switched off by the operator after
the incident and stayed off for the entirety of this work.

Environment: Python 3.10.19, pytest 9.0.2, numpy 2.2.6, opencv-python
4.13.0.92, hidapi 0.15.0, macOS (Darwin).

## How to reproduce

```bash
cd robohand-repo   # this project's root
python3 -m pytest tests/ -v
```

(`pyproject.toml` sets `addopts = "-p no:agentprobe"` to route around a stray,
unrelated, non-importable global pytest plugin present in this particular
sandbox environment. It has no effect on test behavior; it exists only so the
bare command above works without needing an extra flag.)

## Full pytest output (30/30 passed)

```
$ python3 -m pytest tests/ -v
============================= test session starts ==============================
platform darwin -- Python 3.10.19, pytest-9.0.2, pluggy-1.6.0 -- /Users/engineer/.venv/bin/python3
cachedir: .pytest_cache
rootdir: /Users/engineer/workspace/ManusArtifex/.worktrees/arm
configfile: pyproject.toml
plugins: anyio-4.12.1, mock-3.15.1, repeat-0.9.4, langsmith-0.10.9, xdist-3.8.0, a-test-0.0.1.dev30+g70229f16e, typeguard-4.5.1, asyncio-1.3.0, deepeval-3.8.8, rerunfailures-16.1
asyncio: mode=strict, debug=False, asyncio_default_fixture_loop_scope=None, asyncio_default_test_loop_scope=function
collecting ... collected 30 items

tests/test_calibration.py::test_uncalibrated_joint_refuses_out_of_band_move PASSED [  3%]
tests/test_calibration.py::test_uncalibrated_joint_allows_in_band_move PASSED [  6%]
tests/test_calibration.py::test_cli_move_refuses_and_exits_non_zero PASSED [ 10%]
tests/test_grip_ladder.py::test_grip_ladder_stops_on_contact PASSED      [ 13%]
tests/test_grip_ladder.py::test_grip_ladder_respects_hard_backstop_without_vision PASSED [ 16%]
tests/test_grip_ladder.py::test_grip_ladder_open_direction_stops_on_contact PASSED [ 20%]
tests/test_gripper_incident.py::test_gripper_cannot_reach_end_stop PASSED [ 23%]
tests/test_gripper_incident.py::test_gripper_cannot_reach_end_stop_even_with_override PASSED [ 26%]
tests/test_gripper_incident.py::test_gripper_floor_enforced PASSED       [ 30%]
tests/test_hid_framing.py::test_unsolicited_action_group_complete_is_drained_not_misparsed PASSED [ 33%]
tests/test_hid_framing.py::test_stray_frames_do_not_consume_the_retry_budget PASSED [ 36%]
tests/test_hid_framing.py::test_genuine_non_reply_still_times_out PASSED [ 40%]
tests/test_hid_framing.py::test_init_drains_stale_queue_before_first_request PASSED [ 43%]
tests/test_no_bypass.py::test_no_path_bypasses_safety_missing_token PASSED [ 46%]
tests/test_no_bypass.py::test_no_path_bypasses_safety_wrong_transport_token PASSED [ 50%]
tests/test_no_bypass.py::test_no_path_bypasses_safety_reused_token PASSED [ 53%]
tests/test_poses.py::test_all_poses_within_safe_band PASSED              [ 56%]
tests/test_poses.py::test_all_poses_within_global_guard PASSED           [ 60%]
tests/test_ramping.py::test_large_move_is_ramped PASSED                  [ 63%]
tests/test_ramping.py::test_small_move_is_a_single_step PASSED           [ 66%]
tests/test_unload_on_exit_and_exception.py::test_unload_on_exit PASSED   [ 70%]
tests/test_unload_on_exit_and_exception.py::test_unload_on_exception PASSED [ 73%]
tests/test_unload_on_exit_and_exception.py::test_unload_on_exit_via_arm PASSED [ 76%]
tests/test_unload_on_exit_and_exception.py::test_unload_on_exception_via_arm PASSED [ 80%]
tests/test_unload_on_sigint.py::test_unload_on_sigint PASSED             [ 83%]
tests/test_vision_guard.py::test_vision_guard_detects_no_motion PASSED   [ 86%]
tests/test_vision_guard.py::test_vision_guard_detects_motion_and_does_not_raise PASSED [ 90%]
tests/test_vision_guard.py::test_vision_guard_is_a_noop_without_a_camera PASSED [ 93%]
tests/test_watchdog.py::test_watchdog_unloads_when_idle PASSED           [ 96%]
tests/test_watchdog.py::test_watchdog_does_not_unload_while_active PASSED [100%]

============================= 30 passed in 18.73s ==============================
```

(RequestsDependencyWarning line from an unrelated globally-installed package
omitted above; it precedes the `test session starts` banner and has no bearing
on these results.)

## 🔴 Mid-project correction: the gripper's real range is 1500..2500, not 500..2500

After the first 25-test pass above, hardware research (reference firmware +
an independent LeArm owner report — see `docs/research-hardware.md`)
established that GRIP (servo 1) is a **LeArm-family gripper servo** whose own
valid range is **1500..2500**, not the generic 500..2500 assumed for every
other joint, and that **lower raw = MORE OPEN**. This means the original
incident's `close_grip=700` command did not drive the gripper toward a closed
stop — it was 800 units below the gripper's real floor, driving the servo
hard past the **open** mechanical stop, matching the video evidence (jaws
splayed open, held, grinding). The originally-flagged `open_grip=2200` was, in
hindsight, never the dangerous value — it's inside GRIP's real band.

This required five structural changes, all now in this codebase and covered
by the 5 new/updated tests below:

1. **Per-servo absolute floors/ceilings**, not one flat global pair.
   `config.ABSOLUTE_MIN_OVERRIDE = {"GRIP": 1500}` (hardcoded Python, like the
   flat guard — never JSON) tightens the generic 700..2300 global guard for
   GRIP specifically. `config.absolute_bounds(name)` returns the intersection.
   A load-time tripwire raises `ValueError` if `config.json`'s
   documentation-only `gripper_absolute_floor` field ever disagrees with the
   hardcoded value.
2. **Simulator model updated**: `sim.py`'s `DEFAULT_MECHANICAL_LIMITS[1]`
   changed from the old, wrong `(1150, 1850)` to `(1500, 2500)`.
3. **`test_gripper_cannot_reach_end_stop` rewritten** to replay both incident
   values: 700 (refused outright — this is the value that caused real damage)
   and 2200 (legitimately in-band under the corrected range, proceeds without
   raising). Both assert `stall_ms_total() == 0`.
4. **New `test_gripper_floor_enforced`**: direct requests of 700/1000/1499 to
   GRIP are proven to never reach the transport (zero MOVE packets sent)
   without an override, and are proven to clamp to ≥1600 — never below the
   1500 absolute floor — even under an explicit override.
5. **Grip ladder direction fixed**: `Arm._grip_ladder()` had the sign
   backwards under the old (wrong) understanding. Now `close_grip()`
   increases the raw value toward `grip_max_safe` (2300) and `open_grip()`
   decreases it toward `grip_min_safe` (1600), matching "lower = more open."
   `tests/test_grip_ladder.py` was rewritten with the corrected direction
   semantics and new obstruction values.

Two more research findings were also addressed:

- **cmd 8 (`ACTION_GROUP_COMPLETE`) arrives unsolicited** from the controller
  and, if not drained, can desynchronize the reply parser. `protocol.py`'s
  `HidTransport.recv()` was rewritten to a deadline-based loop where frames
  that arrive but don't match the requested `cmd` (including cmd 8) are
  discarded *without* consuming the genuine-miss retry budget, and
  `HidTransport.__init__()` now calls `self.drain()` once at connect time.
  Covered by the 4 new tests in `tests/test_hid_framing.py`, using a fully
  mocked `hid.device` (no real hardware touched).
- **The claw servo (LFD family) claims built-in anti-stall protection; the
  LDX-218 used on every other joint makes no such claim.** No relaxation was
  made to Layer 2's auto-unload for the non-gripper joints — if anything they
  are the *more* fragile ones for sustained stall, and this redesign already
  treats every joint identically for auto-unload purposes.

## Mapping to the 9 required proofs (updated)

| # | Requirement | Test(s) | Result |
|---|---|---|---|
| 1 | `test_gripper_cannot_reach_end_stop` | `tests/test_gripper_incident.py` (3 tests: both incident values, override-proof, floor-enforced) | PASS — 700 refused, 2200 legitimately in-band, 0 stall_ms in every case; 700/1000/1499 never reach the transport even under override |
| 2 | `test_all_poses_within_safe_band` | `tests/test_poses.py` (2 tests) | PASS — every pose in `data/poses.json` inside every joint's effective band and its own per-servo `absolute_bounds` |
| 3 | `test_no_path_bypasses_safety` | `tests/test_no_bypass.py` (3 tests: no token, wrong-transport token, reused token) | PASS — all raise `SafetyBypassError` |
| 4 | `test_unload_on_sigint` | `tests/test_unload_on_sigint.py` | PASS — real subprocess, real `SIGINT`, sim log confirms all servos unloaded |
| 4 | `test_unload_on_exception` | `tests/test_unload_on_exit_and_exception.py::test_unload_on_exception[_via_arm]` | PASS |
| 4 | `test_unload_on_exit` | `tests/test_unload_on_exit_and_exception.py::test_unload_on_exit[_via_arm]` | PASS |
| 5 | `test_watchdog_unloads_when_idle` | `tests/test_watchdog.py` (+ sanity complement that it does NOT fire while active) | PASS |
| 6 | `test_large_move_is_ramped` | `tests/test_ramping.py` | PASS — a 600-unit move on GRIP (1650→2250, its uncalibrated provisional band) with `max_step_per_move=100` produces multiple stepped packets, no step exceeding 100 units |
| 7 | `test_uncalibrated_joint_refuses_out_of_band_move` | `tests/test_calibration.py` (3 tests, incl. CLI-level exit-code check) | PASS |
| 8 | `test_vision_guard_detects_no_motion` | `tests/test_vision_guard.py` (+ 2 sanity complements: real motion doesn't raise, no-camera is a no-op) | PASS |
| 9 | `test_grip_ladder_stops_on_contact` | `tests/test_grip_ladder.py` (3 tests: close-direction contact stop, hard-backstop-without-vision, open-direction contact stop) | PASS — 0 stall_ms in every case |
| — | (not in the original 9, added for the cmd-8 research finding) | `tests/test_hid_framing.py` (4 tests) | PASS — unsolicited cmd 8 frames are drained/discarded without corrupting the next reply or exhausting the retry budget |

30 tests total (9 required scenarios, their reasonable sanity complements, and
4 additional tests for the cmd-8 protocol-framing fix that came out of the
gripper-range correction research).

## Proof the tests are not vacuous: old-design counterfactual

Test 1 is the one that most directly encodes "this incident cannot happen
again." To show it is not vacuously passing (i.e. that it really would have
failed against the code that caused the incident), here is a direct replay of
the incident's exact command against the simulator's mechanical model, with
the safety layer's calibration/choke-point gate removed — i.e., exactly what
the pre-redesign code path did: forward a raw command straight to the
transport with no calibration check.

```
$ python3 -c "
from robohand.sim import SimTransport
from robohand.protocol import MotionToken

transport = SimTransport()
token = MotionToken(id(transport))          # bypasses SafetyLayer entirely
transport.move_servos([(1, 700)], 600, token=token)   # incident: GRIP -> 700
print('stall_ms:', transport.stall_ms_total())
"
stall_ms: 600.0
```

Against the new design, the identical command (`safety.move({'GRIP': 700},
duration_ms=600)`) raises `CalibrationRequired` before anything is sent, and
`stall_ms_total()` is `0.0` — this is exactly what
`test_gripper_cannot_reach_end_stop` asserts. The difference between `600.0`
and `0.0` above is the entire point of Layer 1 — and it holds under the
*corrected* mechanical model (GRIP's open stop at 1500) exactly as it did
under the original, wrong model, because 700 is far outside both.

## Honest residual risk — not covered by this redesign

Being direct about what is **not** proven, per the task's explicit request not
to oversell:

1. **The simulator's mechanical stop positions are mostly still estimates, not
   measurements — GRIP's open-side stop is the one exception.** GRIP's open
   mechanical stop (1500) is now backed by the reference firmware and an
   independent LeArm owner's report, not just guesswork — but its closed-side
   stop (modeled at 2500) and every other joint's mechanical stop in
   `sim.py`'s `DEFAULT_MECHANICAL_LIMITS` remain plausible guesses, not values
   measured off the real arm. Real calibration (hand-teaching each joint's
   actual range with torque off, per `docs/safety.md`) has not been done and
   cannot safely be automated on this hardware (no position feedback). Every
   joint remains `calibrated: false` and clamped to its provisional band on
   real hardware until a human does this by hand.
2. **Self-collision between links is not modeled or checked anywhere.** Layers
   1–3 all reason about individual joints in isolation. Nothing prevents
   commanding a combination of joint angles that would drive the arm into
   itself, into the table, or into an object placed near it, as long as each
   individual joint's target is within its own band. This would require a
   kinematic/geometric model of the arm and its environment, which does not
   exist in this codebase.
3. **Gripper closing on a rigid, immovable object with vision guard
   unavailable degrades to Layers 1+2 only.** If there is no camera (or it
   fails to open), the grip ladder still stops at `grip_min_safe`/
   `grip_max_safe` — but if the object is rigid and sits *between* the
   provisional band and those backstops, the ladder will drive the full
   distance to the backstop pressing against it, holding torque until
   `hold_ms`/the watchdog unloads it. That is bounded (seconds, not
   indefinite, thanks to Layer 2) but it is still real, sustained torque
   against an obstruction with zero visual confirmation that stopping was
   the right call. Layer 3 is what actually prevents grinding past the
   backstop in the ideal case; without it you get "bounded pressure," not
   "no unwanted pressure."
4. **The arm has no protection against falling under gravity when unloaded.**
   Layer 2's entire safety model is "torque off = safe." For a gripper joint,
   or for any joint that must actively resist gravity to hold the arm's own
   weight (e.g. shoulder/elbow in an extended pose), unloading it is not
   necessarily benign — the arm could sag, drop, or swing under its own
   weight, potentially damaging itself or anything beneath it, the instant
   `hold_ms`/idle/exception/signal unload logic fires. This redesign does not
   distinguish "safe to unload" from "unsafe to unload" per joint or per
   pose; it always treats "shorter torque time" as strictly safer, which is
   true for stall/gear-stripping risk but not for gravity-drop risk. Any pose
   where a joint must hold position against gravity needs this weighed
   explicitly before relying on this package's default unload behavior.
5. **Vision guard is environment-fragile by nature.** ROI and thresholds
   (`config.json`'s `vision` section) are tuned for *a* camera placement and
   lighting condition, not *any* placement. A moved camera, changed lighting,
   reflections, or something else entering the frame between the two capture
   points can produce false positives (spurious `StallDetected` on a
   perfectly good move) or false negatives (real stall missed because the
   frame diff happened to clear threshold anyway, e.g. a shadow moving
   independently of the arm). It is a genuine additional signal, not a
   guarantee — Layers 1+2 are the ones that provide a hard bound regardless
   of what the camera sees or doesn't see.
6. **`pushButtons.py` at the repository root completely bypasses everything in
   this redesign.** It is a separate, older script using a different `xarm`
   library and issuing raw absolute servo positions directly — including,
   per its own contents, positions in the same dangerous range as the
   incident. It was explicitly out of scope for this task (not one of the
   files listed for modification), but it means the *codebase* as a whole
   still contains an unguarded path to the exact same failure mode if anyone
   runs it. It should be deleted, rewritten against `robohand.arm.Arm`, or at
   minimum clearly marked as dangerous/deprecated in a follow-up.
7. **The watchdog and hold timers are wall-clock, best-effort, single-process.**
   If the Python process is itself frozen (e.g. by the OS, a debugger
   breakpoint, or being suspended with `SIGSTOP`, which cannot be trapped),
   no unload logic runs at all until the process resumes — Layer 2 assumes
   the process is scheduled and running. `reset_arm.py`'s independence from
   the rest of the package (a completely separate, minimal script) exists
   specifically as a get-out-of-jail path for this scenario: it can still be
   run by a human from another shell even if the main control process is
   stuck, because it depends on nothing this package does or tracks.
8. **This whole test suite validates the simulator's model of the hardware, not
   the hardware itself.** The simulator is only as good as the assumptions
   baked into it (item 1). No test in this suite, and nothing in this
   session, has touched the real arm — which was the explicit, hard
   constraint of this task, but it also means final confidence still
   requires careful, human-supervised, torque-limited testing on the real
   device before fully trusting any of this in production.
9. **The gripper-range correction was itself a research-derived fix, not a
   hardware-measured one.** The 1500..2500 range and "lower = more open"
   direction come from a reference firmware source and an independent
   owner's forum report for the same servo controller family — both credible
   and independently verified during this session (the firmware source was
   fetched and its exact clamp logic confirmed), but neither is a direct
   measurement of *this specific* arm. It is possible (though considered
   unlikely, given the video evidence matches this theory precisely) that
   this individual unit's servo has been reprogrammed or differs from the
   reference. The calibration procedure in `docs/safety.md` is what actually
   settles this for good, and it still has not been performed.

## 2026-07-28 (session 2): full entry-point hardening — reject over clamp, everywhere

Follow-up task: audit and close *every* remaining path by which an
out-of-electrical-range servo position could reach a servo, from any entry
point (`config.json`, `poses.json`, sequence/`play` files, CLI parsing,
`pose --save`/`teach` write paths, `--force`/override flags, the grip ladder,
ramp interpolation, and the transport layer itself). As before, **all
validation in this section runs against `robohand.sim.SimTransport` only —
no command here ever opened the real HID device (VID 0x0483 / PID 0x5750).**

### Design change: one hardcoded source of truth for electrical limits

`robohand/config.py` now defines:

```python
CANONICAL_JOINTS: Dict[str, int] = {"GRIP": 1, "WRIST_ROLL": 2, ...}
ELECTRICAL_LIMITS: Dict[int, Tuple[int, int]] = {
    1: (1500, 2500),  # GRIP
    2: (500, 2500), 3: (500, 2500), 4: (500, 2500),
    5: (500, 2500), 6: (500, 2500),
}
```

Every other bound in the package (`GLOBAL_GUARD_MIN/MAX`, per-joint configured
min/max, the provisional band, the grip ladder backstops, the transport-layer
assertion) is derived from this table and can only ever be equal to or
narrower than it. This replaces the old `ABSOLUTE_MIN_OVERRIDE`/
`ABSOLUTE_MAX_OVERRIDE` dicts, which existed but were not consistently
consulted by every entry point.

### Entry points found and how each is now guarded

| # | Entry point | Guard added/verified | Previously guarded? |
|---|---|---|---|
| 1 | `config.json` loading (`config._validate_servo_spec`) | Rejects (raises `ConfigError`, naming the offending key) an unknown joint name, a servo id that disagrees with fixed wiring, missing/non-integer min/max, inverted min/max, a range wider than `ELECTRICAL_LIMITS`, a center outside its own configured band, a non-bool `calibrated`. Tripwires cross-check `hardware_limits.min/max` and `safety.gripper_absolute_floor` against the hardcoded constants. | **Partially** — some checks existed; inverted min/max, out-of-electrical-range widening, and unknown-joint-name were **not** previously rejected (silently intersected or accepted). |
| 2 | `poses.json` loading (`poses.load_poses`) | Every pose's `positions`/`duration_ms` validated via `config.validate_positions`/`validate_duration_ms` at load time; raises `InvalidPosition` naming the pose and joint. | **No** — previously loaded with zero validation; an illegal value would only have been caught later, at move time. |
| 3 | `pose --save` / `teach` write paths (`poses.save_pose`, `cli.cmd_teach`) | `save_pose` validates before writing (a rejected save touches nothing on disk). `cmd_teach`'s hand-typed-input path now calls `config.validate_position_value` and **rejects** (previously called `config.clamp`, i.e. silently substituted a different value). | **No** — `save_pose` had no validation; `cmd_teach` silently clamped illegal hand-typed input. |
| 4 | Sequence/`play` files (`sequences.py`) | New `validate_sequence(steps)` validates every step's positions/pose-reference/duration up front; called by `load_sequence` (load time) *and* again at the start of `play_sequence` (defense in depth) so a bad step anywhere rejects the **whole** sequence before any motion starts. `save_sequence`/`record_interactive` also validate before writing. | **No** — `play_sequence` previously validated/executed steps lazily one at a time, so steps before a bad one would already have moved the arm ("half-executed"). |
| 5 | CLI parsing (`cli.py`: `_parse_joints`, `cmd_move`, `cmd_grip` `--step`) | `_parse_joints` catches `json.JSONDecodeError` and routes through `config.validate_positions` (rejects non-integers, out-of-range, unknown names). `cmd_move`'s `--servo/--pos` branch validates before constructing `Arm`. `cmd_grip`'s `--step` (via `arm._grip_ladder`) now rejects non-positive step values instead of silently `abs()`-ing them. `main()` now catches `config.InvalidPosition`/`ConfigError`, `json.JSONDecodeError`, `KeyError`, and generic `ValueError`, printing a clear message and exiting non-zero. | **No** — `_parse_joints` let raw `JSONDecodeError`/`ValueError`/`KeyError` escape uncaught; `--step` silently flipped the sign of a negative value. |
| 6 | `--override-reason` (`safety.SafetyLayer.move`) | Rewritten to **never** clamp a user-supplied final target: out-of-band requests raise `CalibrationRequired` (uncalibrated, no override) or the new `PositionOutOfRange` (calibrated, or override supplied but still out of range) — including when the override is used. An override can still only widen *up to* the joint's own configured min/max (itself always ⊆ the electrical range) — it can never reach past `ELECTRICAL_LIMITS`, and GRIP can never go below 1500 even with an override. | **No — this was the most significant gap.** The prior implementation *silently clamped* an out-of-band override request to the nearest legal value instead of refusing — the same failure class as the original incident (a bad input silently turned into a different, unnoticed value). Two existing tests (`test_gripper_incident.py::test_gripper_floor_enforced`, `test_gripper_cannot_reach_end_stop_even_with_override`) asserted this clamp behavior; both were rewritten to assert rejection instead. |
| 7 | Grip ladder (`arm._grip_ladder`) | Every rung (including the back-off step) already routed through `Arm.set_position` → `SafetyLayer.move`, which now rejects rather than clamps out-of-band requests; `--step` itself is now validated (see #5). | **Yes** (rungs were already range-checked structurally) — only the `--step` sign-flip was a gap. |
| 8 | Ramp interpolation (`safety.SafetyLayer.move`, inner loop) | Intermediate steps are still clamped (the *only* place clamping remains legal, per the task's design mandate) but now `logger.warning(...)` fires whenever a clamp actually changes a value — this should be mathematically unreachable for a legal start/target pair, so a warning firing is itself a signal something upstream is wrong. | **Yes**, structurally safe already; logging was the only gap. |
| 9 | Transport layer (`protocol.py`, `sim.py`) | New `assert_targets_within_electrical_range()` / `IllegalServoTarget` exception, called by **both** `HidTransport.move_servos` and `SimTransport.move_servos` immediately after the `MotionToken` check and before any bytes/state changes — validates every `(servo_id, position)` pair against `ELECTRICAL_LIMITS` and `duration_ms` is a sane positive int. Proven to reject even a **valid, correctly-minted `MotionToken`** carrying an illegal position. | **No — this was a real gap.** `move_servos` on both transports previously checked only for a valid token; a valid token carrying an illegal position would have gone straight to the wire (or the simulator). |
| 10 | Duration/time parameters (`config.validate_duration_ms`) | Non-positive, non-integer, or absurd (>60s) durations raise `InvalidPosition` — enforced at `SafetyLayer.move` and again at the transport backstop. | **No** — durations were passed through with no positivity/sanity check. |

### Previously-unguarded paths found (honest summary)

The most significant finding: **`safety.SafetyLayer.move` silently clamped an
out-of-band override request instead of rejecting it** — this is the exact
failure mode that caused the original incident (a numeric input that didn't
match intent was silently turned into *some* other value rather than refused).
Beyond that: the transport layer had **no position-range check at all** (item
9); sequences could be **half-executed** before hitting a bad step (item 4);
`poses.json` and `config.json` were both loaded with **materially incomplete
validation** (items 1–2); `cmd_teach` used `config.clamp` on hand-typed input
(item 3); CLI JSON/type errors could **escape uncaught** (item 5); and the
grip ladder's `--step` silently flipped the sign of a negative value instead
of rejecting it (item 7).

### New test suite: `tests/test_range_enforcement.py`

134 new table-driven tests covering: GRIP at 700/1000/1499/2501/-5/0/"abc"/1e9
rejected via direct API, CLI `move --joints`, CLI `move --servo/--pos`, pose
file load, sequence file load, `pose --save`, `teach`, and raw transport call;
every non-gripper servo at 499/2501 rejected the same way; malformed
`config.json` (inverted min/max, out-of-electrical-range, unknown joint name,
non-integer value, out-of-band center) rejected at import time with a clear
message naming the offending key; override cannot escape the electrical range
or take GRIP below 1500 (but still succeeds for an in-band override request);
ramp intermediates proven in-range for two independent long moves; a sequence
with one illegal step proven to produce **zero** `MOVE` packets; `stall_ms_total()
== 0` asserted wherever motion actually occurs; non-positive/absurd durations
rejected at both `SafetyLayer` and the transport backstop.

### Full pytest output (164/164 passed)

```
$ python3 -m pytest tests/ -v
============================= test session starts ==============================
platform darwin -- Python 3.10.19, pytest-9.0.2, pluggy-1.6.0 -- /Users/engineer/.venv/bin/python3
cachedir: .pytest_cache
rootdir: /Users/engineer/workspace/ManusArtifex
configfile: pyproject.toml
plugins: anyio-4.12.1, mock-3.15.1, repeat-0.9.4, langsmith-0.10.9, xdist-3.8.0, a-test-0.0.1.dev30+g70229f16e, typeguard-4.5.1, asyncio-1.3.0, deepeval-3.8.8, rerunfailures-16.1
asyncio: mode=strict, debug=False, asyncio_default_fixture_loop_scope=None, asyncio_default_test_loop_scope=function
collecting ... collected 164 items

tests/test_calibration.py::test_uncalibrated_joint_refuses_out_of_band_move PASSED [  0%]
tests/test_calibration.py::test_uncalibrated_joint_allows_in_band_move PASSED [  1%]
tests/test_calibration.py::test_cli_move_refuses_and_exits_non_zero PASSED [  1%]
tests/test_grip_ladder.py::test_grip_ladder_stops_on_contact PASSED      [  2%]
tests/test_grip_ladder.py::test_grip_ladder_respects_hard_backstop_without_vision PASSED [  3%]
tests/test_grip_ladder.py::test_grip_ladder_open_direction_stops_on_contact PASSED [  3%]
tests/test_gripper_incident.py::test_gripper_cannot_reach_end_stop PASSED [  4%]
tests/test_gripper_incident.py::test_gripper_cannot_reach_end_stop_even_with_override PASSED [  4%]
tests/test_gripper_incident.py::test_gripper_floor_enforced PASSED       [  5%]
tests/test_hid_framing.py::test_unsolicited_action_group_complete_is_drained_not_misparsed PASSED [  6%]
tests/test_hid_framing.py::test_stray_frames_do_not_consume_the_retry_budget PASSED [  6%]
tests/test_hid_framing.py::test_genuine_non_reply_still_times_out PASSED [  7%]
tests/test_hid_framing.py::test_init_drains_stale_queue_before_first_request PASSED [  7%]
tests/test_no_bypass.py::test_no_path_bypasses_safety_missing_token PASSED [  8%]
tests/test_no_bypass.py::test_no_path_bypasses_safety_wrong_transport_token PASSED [  9%]
tests/test_no_bypass.py::test_no_path_bypasses_safety_reused_token PASSED [  9%]
tests/test_poses.py::test_all_poses_within_safe_band PASSED              [ 10%]
tests/test_poses.py::test_all_poses_within_global_guard PASSED           [ 10%]
tests/test_ramping.py::test_large_move_is_ramped PASSED                  [ 11%]
tests/test_ramping.py::test_small_move_is_a_single_step PASSED           [ 12%]
tests/test_range_enforcement.py::test_direct_api_rejects_illegal_grip_int[700] PASSED [ 12%]
tests/test_range_enforcement.py::test_direct_api_rejects_illegal_grip_int[1000] PASSED [ 13%]
tests/test_range_enforcement.py::test_direct_api_rejects_illegal_grip_int[1499] PASSED [ 14%]
tests/test_range_enforcement.py::test_direct_api_rejects_illegal_grip_int[2501] PASSED [ 14%]
tests/test_range_enforcement.py::test_direct_api_rejects_illegal_grip_int[-5] PASSED [ 15%]
tests/test_range_enforcement.py::test_direct_api_rejects_illegal_grip_int[0] PASSED [ 15%]
tests/test_range_enforcement.py::test_direct_api_rejects_illegal_grip_non_int[abc] PASSED [ 16%]
tests/test_range_enforcement.py::test_direct_api_rejects_illegal_grip_non_int[1000000000.0] PASSED [ 17%]
tests/test_range_enforcement.py::test_direct_api_rejects_illegal_grip_non_int[1500.0] PASSED [ 17%]
tests/test_range_enforcement.py::test_direct_api_rejects_illegal_grip_non_int[True] PASSED [ 18%]
tests/test_range_enforcement.py::test_direct_api_safety_layer_rejects_illegal_grip[700] PASSED [ 18%]
tests/test_range_enforcement.py::test_direct_api_safety_layer_rejects_illegal_grip[1000] PASSED [ 19%]
tests/test_range_enforcement.py::test_direct_api_safety_layer_rejects_illegal_grip[1499] PASSED [ 20%]
tests/test_range_enforcement.py::test_direct_api_safety_layer_rejects_illegal_grip[2501] PASSED [ 20%]
tests/test_range_enforcement.py::test_direct_api_safety_layer_rejects_illegal_grip[-5] PASSED [ 21%]
tests/test_range_enforcement.py::test_direct_api_safety_layer_rejects_illegal_grip[0] PASSED [ 21%]
tests/test_range_enforcement.py::test_direct_api_safety_layer_rejects_illegal_grip_non_int[abc] PASSED [ 22%]
tests/test_range_enforcement.py::test_direct_api_safety_layer_rejects_illegal_grip_non_int[1000000000.0] PASSED [ 23%]
tests/test_range_enforcement.py::test_direct_api_rejects_non_grip_out_of_electrical_range[499-WRIST_ROLL] PASSED [ 23%]
tests/test_range_enforcement.py::test_direct_api_rejects_non_grip_out_of_electrical_range[499-WRIST_FLEX] PASSED [ 24%]
tests/test_range_enforcement.py::test_direct_api_rejects_non_grip_out_of_electrical_range[499-ELBOW] PASSED [ 25%]
tests/test_range_enforcement.py::test_direct_api_rejects_non_grip_out_of_electrical_range[499-SHOULDER_LIFT] PASSED [ 25%]
tests/test_range_enforcement.py::test_direct_api_rejects_non_grip_out_of_electrical_range[499-SHOULDER_PAN] PASSED [ 26%]
tests/test_range_enforcement.py::test_direct_api_rejects_non_grip_out_of_electrical_range[2501-WRIST_ROLL] PASSED [ 26%]
tests/test_range_enforcement.py::test_direct_api_rejects_non_grip_out_of_electrical_range[2501-WRIST_FLEX] PASSED [ 27%]
tests/test_range_enforcement.py::test_direct_api_rejects_non_grip_out_of_electrical_range[2501-ELBOW] PASSED [ 28%]
tests/test_range_enforcement.py::test_direct_api_rejects_non_grip_out_of_electrical_range[2501-SHOULDER_LIFT] PASSED [ 28%]
tests/test_range_enforcement.py::test_direct_api_rejects_non_grip_out_of_electrical_range[2501-SHOULDER_PAN] PASSED [ 29%]
tests/test_range_enforcement.py::test_cli_move_joints_rejects_illegal_grip[700] PASSED [ 29%]
tests/test_range_enforcement.py::test_cli_move_joints_rejects_illegal_grip[1000] PASSED [ 30%]
tests/test_range_enforcement.py::test_cli_move_joints_rejects_illegal_grip[1499] PASSED [ 31%]
tests/test_range_enforcement.py::test_cli_move_joints_rejects_illegal_grip[2501] PASSED [ 31%]
tests/test_range_enforcement.py::test_cli_move_joints_rejects_illegal_grip[-5] PASSED [ 32%]
tests/test_range_enforcement.py::test_cli_move_joints_rejects_illegal_grip[0] PASSED [ 32%]
tests/test_range_enforcement.py::test_cli_move_joints_rejects_illegal_grip_non_int["abc"] PASSED [ 33%]
tests/test_range_enforcement.py::test_cli_move_joints_rejects_illegal_grip_non_int[1e9] PASSED [ 34%]
tests/test_range_enforcement.py::test_cli_move_joints_rejects_malformed_json PASSED [ 34%]
tests/test_range_enforcement.py::test_cli_move_joints_rejects_unknown_joint_name PASSED [ 35%]
tests/test_range_enforcement.py::test_cli_move_joints_rejects_non_grip_out_of_range[499-WRIST_ROLL] PASSED [ 35%]
tests/test_range_enforcement.py::test_cli_move_joints_rejects_non_grip_out_of_range[499-WRIST_FLEX] PASSED [ 36%]
tests/test_range_enforcement.py::test_cli_move_joints_rejects_non_grip_out_of_range[499-ELBOW] PASSED [ 37%]
tests/test_range_enforcement.py::test_cli_move_joints_rejects_non_grip_out_of_range[499-SHOULDER_LIFT] PASSED [ 37%]
tests/test_range_enforcement.py::test_cli_move_joints_rejects_non_grip_out_of_range[499-SHOULDER_PAN] PASSED [ 38%]
tests/test_range_enforcement.py::test_cli_move_joints_rejects_non_grip_out_of_range[2501-WRIST_ROLL] PASSED [ 39%]
tests/test_range_enforcement.py::test_cli_move_joints_rejects_non_grip_out_of_range[2501-WRIST_FLEX] PASSED [ 39%]
tests/test_range_enforcement.py::test_cli_move_joints_rejects_non_grip_out_of_range[2501-ELBOW] PASSED [ 40%]
tests/test_range_enforcement.py::test_cli_move_joints_rejects_non_grip_out_of_range[2501-SHOULDER_LIFT] PASSED [ 40%]
tests/test_range_enforcement.py::test_cli_move_joints_rejects_non_grip_out_of_range[2501-SHOULDER_PAN] PASSED [ 41%]
tests/test_range_enforcement.py::test_cli_move_servo_pos_rejects_illegal_grip[700] PASSED [ 42%]
tests/test_range_enforcement.py::test_cli_move_servo_pos_rejects_illegal_grip[1000] PASSED [ 42%]
tests/test_range_enforcement.py::test_cli_move_servo_pos_rejects_illegal_grip[1499] PASSED [ 43%]
tests/test_range_enforcement.py::test_cli_move_servo_pos_rejects_illegal_grip[2501] PASSED [ 43%]
tests/test_range_enforcement.py::test_cli_move_servo_pos_rejects_illegal_grip[-5] PASSED [ 44%]
tests/test_range_enforcement.py::test_cli_move_servo_pos_rejects_illegal_grip[0] PASSED [ 45%]
tests/test_range_enforcement.py::test_cli_move_servo_pos_rejects_non_integer PASSED [ 45%]
tests/test_range_enforcement.py::test_cli_grip_rejects_non_positive_step[-5] PASSED [ 46%]
tests/test_range_enforcement.py::test_cli_grip_rejects_non_positive_step[0] PASSED [ 46%]
tests/test_range_enforcement.py::test_pose_file_rejects_illegal_grip_at_load[700] PASSED [ 47%]
tests/test_range_enforcement.py::test_pose_file_rejects_illegal_grip_at_load[1000] PASSED [ 48%]
tests/test_range_enforcement.py::test_pose_file_rejects_illegal_grip_at_load[1499] PASSED [ 48%]
tests/test_range_enforcement.py::test_pose_file_rejects_illegal_grip_at_load[2501] PASSED [ 49%]
tests/test_range_enforcement.py::test_pose_file_rejects_illegal_grip_at_load[-5] PASSED [ 50%]
tests/test_range_enforcement.py::test_pose_file_rejects_illegal_grip_at_load[0] PASSED [ 50%]
tests/test_range_enforcement.py::test_pose_file_rejects_non_grip_out_of_range_at_load[499-WRIST_ROLL] PASSED [ 51%]
tests/test_range_enforcement.py::test_pose_file_rejects_non_grip_out_of_range_at_load[499-WRIST_FLEX] PASSED [ 51%]
tests/test_range_enforcement.py::test_pose_file_rejects_non_grip_out_of_range_at_load[499-ELBOW] PASSED [ 52%]
tests/test_range_enforcement.py::test_pose_file_rejects_non_grip_out_of_range_at_load[499-SHOULDER_LIFT] PASSED [ 53%]
tests/test_range_enforcement.py::test_pose_file_rejects_non_grip_out_of_range_at_load[499-SHOULDER_PAN] PASSED [ 53%]
tests/test_range_enforcement.py::test_pose_file_rejects_non_grip_out_of_range_at_load[2501-WRIST_ROLL] PASSED [ 54%]
tests/test_range_enforcement.py::test_pose_file_rejects_non_grip_out_of_range_at_load[2501-WRIST_FLEX] PASSED [ 54%]
tests/test_range_enforcement.py::test_pose_file_rejects_non_grip_out_of_range_at_load[2501-ELBOW] PASSED [ 55%]
tests/test_range_enforcement.py::test_pose_file_rejects_non_grip_out_of_range_at_load[2501-SHOULDER_LIFT] PASSED [ 56%]
tests/test_range_enforcement.py::test_pose_file_rejects_non_grip_out_of_range_at_load[2501-SHOULDER_PAN] PASSED [ 56%]
tests/test_range_enforcement.py::test_pose_file_shipped_data_still_loads PASSED [ 57%]
tests/test_range_enforcement.py::test_sequence_file_rejects_illegal_grip_at_load[700] PASSED [ 57%]
tests/test_range_enforcement.py::test_sequence_file_rejects_illegal_grip_at_load[1000] PASSED [ 58%]
tests/test_range_enforcement.py::test_sequence_file_rejects_illegal_grip_at_load[1499] PASSED [ 59%]
tests/test_range_enforcement.py::test_sequence_file_rejects_illegal_grip_at_load[2501] PASSED [ 59%]
tests/test_range_enforcement.py::test_sequence_file_rejects_illegal_grip_at_load[-5] PASSED [ 60%]
tests/test_range_enforcement.py::test_sequence_file_rejects_illegal_grip_at_load[0] PASSED [ 60%]
tests/test_range_enforcement.py::test_sequence_with_one_illegal_step_performs_zero_motion PASSED [ 61%]
tests/test_range_enforcement.py::test_sequence_file_rejects_unknown_pose_reference PASSED [ 62%]
tests/test_range_enforcement.py::test_pose_save_refuses_to_write_illegal_grip[700] PASSED [ 62%]
tests/test_range_enforcement.py::test_pose_save_refuses_to_write_illegal_grip[1000] PASSED [ 63%]
tests/test_range_enforcement.py::test_pose_save_refuses_to_write_illegal_grip[1499] PASSED [ 64%]
tests/test_range_enforcement.py::test_pose_save_refuses_to_write_illegal_grip[2501] PASSED [ 64%]
tests/test_range_enforcement.py::test_pose_save_refuses_to_write_illegal_grip[-5] PASSED [ 65%]
tests/test_range_enforcement.py::test_pose_save_refuses_to_write_illegal_grip[0] PASSED [ 65%]
tests/test_range_enforcement.py::test_pose_save_refuses_illegal_value_without_disturbing_existing_file PASSED [ 66%]
tests/test_range_enforcement.py::test_cli_teach_rejects_illegal_hand_typed_position PASSED [ 67%]
tests/test_range_enforcement.py::test_cli_teach_rejects_non_integer_hand_typed_position PASSED [ 67%]
tests/test_range_enforcement.py::test_cli_pose_save_writes_only_legal_positions PASSED [ 68%]
tests/test_range_enforcement.py::test_config_json_rejects_inverted_min_max PASSED [ 68%]
tests/test_range_enforcement.py::test_config_json_rejects_out_of_electrical_range PASSED [ 69%]
tests/test_range_enforcement.py::test_config_json_rejects_grip_below_electrical_floor PASSED [ 70%]
tests/test_range_enforcement.py::test_config_json_rejects_unknown_joint_name PASSED [ 70%]
tests/test_range_enforcement.py::test_config_json_rejects_string_value PASSED [ 71%]
tests/test_range_enforcement.py::test_config_json_rejects_center_outside_own_band PASSED [ 71%]
tests/test_range_enforcement.py::test_config_json_shipped_file_still_loads_cleanly PASSED [ 72%]
tests/test_range_enforcement.py::test_override_cannot_take_grip_below_1500 PASSED [ 73%]
tests/test_range_enforcement.py::test_override_cannot_escape_electrical_range[WRIST_ROLL] PASSED [ 73%]
tests/test_range_enforcement.py::test_override_cannot_escape_electrical_range[WRIST_FLEX] PASSED [ 74%]
tests/test_range_enforcement.py::test_override_cannot_escape_electrical_range[ELBOW] PASSED [ 75%]
tests/test_range_enforcement.py::test_override_cannot_escape_electrical_range[SHOULDER_LIFT] PASSED [ 75%]
tests/test_range_enforcement.py::test_override_cannot_escape_electrical_range[SHOULDER_PAN] PASSED [ 76%]
tests/test_range_enforcement.py::test_override_grip_within_configured_band_still_succeeds PASSED [ 76%]
tests/test_range_enforcement.py::test_ramp_intermediates_all_within_electrical_and_effective_bounds PASSED [ 77%]
tests/test_range_enforcement.py::test_ramp_intermediates_within_range_for_calibrated_style_long_move PASSED [ 78%]
tests/test_range_enforcement.py::test_transport_backstop_rejects_illegal_grip_even_with_valid_token[700] PASSED [ 78%]
tests/test_range_enforcement.py::test_transport_backstop_rejects_illegal_grip_even_with_valid_token[1000] PASSED [ 79%]
tests/test_range_enforcement.py::test_transport_backstop_rejects_illegal_grip_even_with_valid_token[1499] PASSED [ 79%]
tests/test_range_enforcement.py::test_transport_backstop_rejects_illegal_grip_even_with_valid_token[2501] PASSED [ 80%]
tests/test_range_enforcement.py::test_transport_backstop_rejects_illegal_grip_even_with_valid_token[-5] PASSED [ 81%]
tests/test_range_enforcement.py::test_transport_backstop_rejects_illegal_grip_even_with_valid_token[0] PASSED [ 81%]
tests/test_range_enforcement.py::test_transport_backstop_rejects_illegal_grip_non_int[abc] PASSED [ 82%]
tests/test_range_enforcement.py::test_transport_backstop_rejects_illegal_grip_non_int[1000000000.0] PASSED [ 82%]
tests/test_range_enforcement.py::test_transport_backstop_rejects_illegal_grip_non_int[True] PASSED [ 83%]
tests/test_range_enforcement.py::test_transport_backstop_rejects_non_grip_out_of_range[499-WRIST_ROLL] PASSED [ 84%]
tests/test_range_enforcement.py::test_transport_backstop_rejects_non_grip_out_of_range[499-WRIST_FLEX] PASSED [ 84%]
tests/test_range_enforcement.py::test_transport_backstop_rejects_non_grip_out_of_range[499-ELBOW] PASSED [ 85%]
tests/test_range_enforcement.py::test_transport_backstop_rejects_non_grip_out_of_range[499-SHOULDER_LIFT] PASSED [ 85%]
tests/test_range_enforcement.py::test_transport_backstop_rejects_non_grip_out_of_range[499-SHOULDER_PAN] PASSED [ 86%]
tests/test_range_enforcement.py::test_transport_backstop_rejects_non_grip_out_of_range[2501-WRIST_ROLL] PASSED [ 87%]
tests/test_range_enforcement.py::test_transport_backstop_rejects_non_grip_out_of_range[2501-WRIST_FLEX] PASSED [ 87%]
tests/test_range_enforcement.py::test_transport_backstop_rejects_non_grip_out_of_range[2501-ELBOW] PASSED [ 88%]
tests/test_range_enforcement.py::test_transport_backstop_rejects_non_grip_out_of_range[2501-SHOULDER_LIFT] PASSED [ 89%]
tests/test_range_enforcement.py::test_transport_backstop_rejects_non_grip_out_of_range[2501-SHOULDER_PAN] PASSED [ 89%]
tests/test_range_enforcement.py::test_transport_backstop_accepts_legal_value PASSED [ 90%]
tests/test_range_enforcement.py::test_safety_layer_rejects_non_positive_duration[0] PASSED [ 90%]
tests/test_range_enforcement.py::test_safety_layer_rejects_non_positive_duration[-1] PASSED [ 91%]
tests/test_range_enforcement.py::test_safety_layer_rejects_non_positive_duration[-1000] PASSED [ 92%]
tests/test_range_enforcement.py::test_safety_layer_rejects_absurd_duration PASSED [ 92%]
tests/test_range_enforcement.py::test_transport_backstop_rejects_non_positive_duration[0] PASSED [ 93%]
tests/test_range_enforcement.py::test_transport_backstop_rejects_non_positive_duration[-1] PASSED [ 93%]
tests/test_unload_on_exit_and_exception.py::test_unload_on_exit PASSED   [ 94%]
tests/test_unload_on_exit_and_exception.py::test_unload_on_exception PASSED [ 95%]
tests/test_unload_on_exit_and_exception.py::test_unload_on_exit_via_arm PASSED [ 95%]
tests/test_unload_on_exit_and_exception.py::test_unload_on_exception_via_arm PASSED [ 96%]
tests/test_unload_on_sigint.py::test_unload_on_sigint PASSED             [ 96%]
tests/test_vision_guard.py::test_vision_guard_detects_no_motion PASSED   [ 97%]
tests/test_vision_guard.py::test_vision_guard_detects_motion_and_does_not_raise PASSED [ 98%]
tests/test_vision_guard.py::test_vision_guard_is_a_noop_without_a_camera PASSED [ 98%]
tests/test_watchdog.py::test_watchdog_unloads_when_idle PASSED           [ 99%]
tests/test_watchdog.py::test_watchdog_does_not_unload_while_active PASSED [100%]

============================= 164 passed in 20.29s =============================
```

(RequestsDependencyWarning line from an unrelated globally-installed package
omitted above, as in the first run; it has no bearing on these results.)

### Still unguarded, and why

1. **`pushButtons.py` at the repository root** — as flagged in the previous
   session, this is a separate, older script using a different `xarm`
   library that issues raw absolute servo positions directly, completely
   bypassing `robohand.config`/`safety`/`protocol`. It remains out of scope:
   it was not one of the files named in this task (`config.json`,
   `poses.json`, `sequences.py`, `cli.py`, `arm.py`, `protocol.py`, `sim.py`),
   and rewriting/deleting an unrelated standalone script was not requested.
   It should still be deleted, rewritten against `robohand.arm.Arm`, or at
   minimum marked deprecated/dangerous.
2. **Judgment call on item 6's exact scope**: the task says "an override may
   widen up to the *electrical* range for a calibrated joint." No joint
   currently ships `calibrated: true` in `config.json`, and the override
   mechanism as implemented widens only to a joint's own *configured*
   min/max (itself always inside the electrical range), never all the way
   to the raw electrical range itself. This is the more conservative, safer
   reading and was not changed, but it is worth flagging explicitly: if a
   future calibrated joint is meant to be overridable all the way to its
   electrical limits, that would be a deliberate, separate design change,
   not something this task's mandate ("never permit anything outside the
   electrical range... GRIP below 1500") required loosening.
3. **Shadow/last-commanded state (`~/.robohand/state.json`)**: now validated
   on load (`arm._load_shadow`) — an illegal or corrupted entry is discarded
   and logged rather than trusted — but this is a defense-in-depth measure,
   not a new primary guard; the ramp/effective-bounds checks downstream were
   already safe against a bad "current" value even before this change.
