# robohand SDK

Architecture, extension points, and how to run everything without touching real
hardware. For the safety design specifically (three layers, choke point, incident
history), see `docs/safety.md` — this document assumes you've read that.

## Module map

```
robohand/
  protocol.py   HID wire protocol (LSC-6). MotionToken / SafetyBypassError /
                require_motion_token live here. HidTransport is the ONLY thing
                that talks to real hardware.
  sim.py        SimTransport / DryRunTransport — same interface as HidTransport,
                models mechanical end stops, no real device involved.
  safety.py     SafetyLayer — the single choke point, watchdog thread, exit
                handling. Owns a transport (real or sim), nothing else does.
  config.py     Loads data/config.json. Effective-bounds computation
                (global guard ∩ configured min/max ∩ provisional band).
  guard.py      VisionGuard / StallDetected — Layer 3, frame-diff stall
                detection. Camera-agnostic (takes any frame_source callable).
  arm.py        Arm — the public, high-level API. Shadow position tracking,
                grip ladder, doctor introspection. Never touches a transport
                directly; every motion call goes through self._safety.move().
  poses.py      Named-pose JSON storage (data/poses.json).
  sequences.py  Ordered pose/position playback (used by `robohand play`).
  cli.py        Argument parsing and subcommands. Thin wrapper over Arm.
  camera.py     Existing camera capture (not modified this project — used by
                guard.py's default frame source when a real camera is present).
  vision.py     Existing CV helpers (not modified this project).
data/
  config.json   Per-joint limits/calibration, safety/vision/gripper tunables.
  poses.json    Named poses (home, open_grip, close_grip, ...).
reset_arm.py    Standalone emergency stop. Deliberately has ZERO dependency on
                the robohand package (only imports `hid`) so it still works if
                anything else in the package is broken.
tests/          pytest suite, all against sim.SimTransport. See
                docs/verification-sdk.md for the last full run's output.
```

## The safety token, precisely

`protocol.MotionToken` is an opaque object with two pieces of state: the
`id()` of the transport it was minted for, and a `_spent` flag. It is
constructed exactly once per outgoing packet, inside
`SafetyLayer._mint_token()`:

```python
def _mint_token(self) -> MotionToken:
    return MotionToken(id(self._transport))
```

Both transports call the same shared function before sending a SERVO_MOVE:

```python
def require_motion_token(transport, token):
    if not isinstance(token, MotionToken):
        raise SafetyBypassError(...)
    token._redeem(transport)   # raises if spent, or minted for a different transport
```

There is no public way to construct a token that passes `_redeem()` except by
being handed one from `SafetyLayer._mint_token()` immediately before a
transport call — and `_mint_token()` is only ever invoked from inside
`SafetyLayer.move()`, after every Layer-1 check has already passed. This means:

- You cannot call `transport.move_servos(...)` from anywhere else in the
  codebase, in a test, or in a REPL, and have it succeed — try it, it raises
  `SafetyBypassError` (`tests/test_no_bypass.py` does exactly this).
- A token cannot be replayed: `_redeem()` sets `_spent = True` on first use.
- A token minted for one transport instance cannot be handed to another
  (`id(transport)` mismatch) — relevant if a process somehow ends up holding
  two transports (e.g. a test constructing two `SimTransport()`s).

If you ever find yourself wanting to call `move_servos` directly to "just get
this one thing working," that is the signal that the thing you're building
belongs inside `SafetyLayer.move()`, not around it.

## How to add a new joint

1. Add an entry to `robohand/data/config.json` → `servos`: `id` (raw servo
   number on the LSC-6 bus), `min`/`max`/`center` (start with a conservative
   provisional band — do not guess mechanical limits), `calibrated: false`.
2. Add plausible mechanical stops for it to `sim.py`'s
   `DEFAULT_MECHANICAL_LIMITS` (used only by the simulator — pick values
   narrower than the electrical 500..2500 range; these are *estimates* for
   testing, not a substitute for real calibration).
3. That's it for wiring — `config.JOINT_NAMES`, `config.JOINT_LIMITS`,
   `config.resolve_servo()`, ramping, calibration checks, and the CLI's
   `--servo`/`--joints` arguments all read the servo map dynamically from
   `config.json`; nothing is hardcoded to the current 6 joints.
4. Calibrate it for real per `docs/safety.md`'s calibration procedure before
   setting `calibrated: true` — do this by hand, torque off, with a human
   watching, exactly as for every other joint. There is no automated way to
   discover mechanical limits without either position feedback (which does
   not exist) or moving the joint until something stalls (which is the exact
   failure mode this whole redesign exists to prevent).

## How to run the simulator

Three equivalent ways to force sim mode, from least to most invasive:

```bash
# 1. Environment variable (affects any script/process in that shell)
export ROBOHAND_TRANSPORT=sim
python3 -m robohand.cli move --servo GRIP --pos 1700

# 2. CLI flag (affects just this invocation)
python3 -m robohand.cli --sim move --servo GRIP --pos 1700

# 3. Programmatic (affects just this Arm instance)
from robohand.arm import Arm
with Arm(sim=True) as arm:
    arm.set_position("GRIP", 1700)
```

`--dry-run` implies `--sim` and additionally: prints every packet the safety
layer would have sent (`[dry-run] SERVO_MOVE ...` / `[dry-run] MULT_SERVO_UNLOAD
...`), and never persists shadow-position state to `~/.robohand/state.json`.
Use it to see exactly what a command *would* do without even touching the
simulator's internal state being relied on by anything else.

The real `HidTransport` — and therefore any actual write to VID 0x0483 / PID
0x5750 — is only ever reached when neither `--sim`, `--dry-run`, nor
`ROBOHAND_TRANSPORT=sim` is set. This project's own validation never runs
without one of those three switches active; see `tests/conftest.py`, which
forces `ROBOHAND_TRANSPORT=sim` for every test as a second, redundant guard on
top of the tests themselves passing `sim=True` explicitly.

`robohand.sim.SimTransport` exposes test/demo-only helpers beyond the shared
interface:

- `set_obstruction(servo_id, min=None, max=None)` — simulate a rigid object
  (e.g. something in the gripper) blocking travel short of the mechanical
  stop. Contact here does not count as a stall (see `docs/safety.md`).
- `stall_ms_total()` / `joints[id].stall_ms` — accumulated locked-rotor time.
- `any_torque_on()` — whether anything is still energized.
- `.log` — the full list of every command sent, in order, with achieved
  positions.

## Running the tests

```bash
python3 -m pytest tests/ -v
```

Everything in `tests/` runs against `SimTransport`; nothing in the suite opens
a real HID device. `pyproject.toml` sets `addopts = "-p no:agentprobe"` to work
around a stray, non-importable global pytest plugin in this particular
environment — unrelated to robohand, safe to remove if that plugin is fixed or
absent in another environment.

## `robohand doctor`

`python3 -m robohand.cli [--sim|--dry-run] doctor [--json]` reports, without
commanding any motion:

- Device presence (a battery-voltage read — `cmd 15`, verified to reply
  reliably — only attempted against real hardware, never in `--sim`/`--dry-run`).
- Per-joint calibration status and effective bounds.
- Camera availability (via `robohand.camera`).
- Which safety layers are active (1 and 2 always; 3 only if a camera opened
  successfully and vision guard is in use).

Use this before anything else when picking the arm back up after it's been
idle — it is the one command guaranteed to never move a servo.
