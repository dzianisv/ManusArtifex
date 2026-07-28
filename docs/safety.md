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
