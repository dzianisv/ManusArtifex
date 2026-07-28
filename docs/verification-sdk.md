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
