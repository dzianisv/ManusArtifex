# LeArm / LSC-6 hardware research

**Scope:** Hiwonder / LewanSoul **LeArm** 6-DOF arm, LSC-6 controller, USB-HID `0x0483:0x5750`.
**Purpose:** prevent a repeat of the 2026-07-28 gripper stall (see [safety.md](safety.md)).

---

## 1. Verdict on position feedback — READ THIS FIRST

> **Position read-back (`cmd 21`) is physically impossible on this hardware.**
> Our empirical result — zero bytes returned, 3/3 attempts — is expected. It is
> not a bug, not a timing issue, and not fixable in software.

Confidence: **HIGH**, four independent sources.

1. The reference library's own example says so in a comment:
   `// The getPosition function will work for xArm but not LeArm.`
   — [GetPosition.ino](https://raw.githubusercontent.com/ccourson/Hiwonder-xArm1S/main/Arduino/xArmServoController/examples/GetPosition/GetPosition.ino)

2. A LeArm owner hitting *our exact* `IndexError` on a short buffer:
   "No, you can't read the position on LeArm; that feature only works on XArm."
   — [ccourson/Hiwonder-xArm1S#5](https://github.com/ccourson/Hiwonder-xArm1S/issues/5#issuecomment-1908425827)

3. **The physical reason.** LeArm uses plain 3-wire **PWM** hobby servos. A PWM
   servo has no data line back to the controller — there is no channel on which
   to ask "where are you now". Contrast Hiwonder's **LX-15D** *serial bus* servo,
   which advertises "temperature, position feedback" precisely because it is a
   different product line.
   [LDX-218](https://www.hiwonder.com/search/suggest.json?q=LDX-218&resources[type]=product) ·
   [LX-15D](https://www.hiwonder.com/search/suggest.json?q=LX-15D&resources[type]=product)

4. Hiwonder's current LSC-family protocol doc contains **no position-read command
   at all** — 5 host→controller commands, 3 notifications.
   [LSC-16 protocol](https://wiki.hiwonder.com/projects/16-Channel-Servo-Controller/en/latest/docs/2_Communication_Protocol.html)

**Consequence for this repo:** software shadow-tracking of position is the only
correct architecture. Do not spend further effort on `cmd 21`. The only true
feedback available is **the camera**.

---

## 2. Command reference

Framing for every command: `0x55 0x55 <Length> <Cmd> <Params...>`, where
`Length = n_params + 2`.

| Cmd | Name | Direction | Layout | Status here |
|----:|------|-----------|--------|-------------|
| 0 | `BEEP` | H→C | none | untested |
| 3 | `SERVO_MOVE` | H→C | count; time_ms LE; then per servo: id, pos LE | **works** |
| 6 | `ACTION_GROUP_RUN` | H→C, C→H echo | group id; repeat LE (0=∞) | used by `pushButtons.py` |
| 7 | `ACTION_GROUP_STOP` | H→C, C→H echo | none | **works** (`reset_arm.py`) |
| 8 | `ACTION_GROUP_COMPLETE` | **C→H only**, unsolicited | as RUN | not yet handled — should be |
| 11 | `ACTION_GROUP_SPEED` | H→C | group id (0xFF=all); speed % LE | matches our impl |
| 15 | `GET_BATTERY_VOLTAGE` | H→C, **replies** | reply = millivolts LE | **works** (7396 mV) |
| 20 | `SERVO_STOP` / unload | H→C | count, then ids | **works** — servos go limp |
| 21 | `GET_SERVO_POSITION` | H→C, reply expected | count, then ids | **never replies** — see §1 |
| 27 | `SERVO_ID_WRITE` | H→C | writes ID to bus servos | **not applicable** — bus-servo only. Do not send. |

**Transport note.** Hiwonder documents this as a 9600-baud *serial* protocol.
Our LSC-6 carries the identical `0x55 0x55` byte stream inside **USB-HID
reports** (leading `0x00` report-ID byte on output). Same logical protocol,
different transport — do not conflate the two.

**Deviation calibration (≠ feedback).** A stored per-servo offset in the range
**−100..+100** exists to correct servo-horn mounting misalignment. Reading it
back does *not* report the live angle.
[LSC user manual](https://wiki.hiwonder.com/projects/16-Channel-Servo-Controller/en/latest/docs/1_User_Manual_formatted.html)

---

## 3. Servos and joint mapping

### Which servos

Per Hiwonder's own [Amazon listing](https://www.amazon.com/dp/B074T6DPKX) for this kit:

| Part | Used for | Key specs | Source |
|---|---|---|---|
| **LDX-218** | main joints | PWM, 500–2500 µs → 0–180°, 17 kg·cm @7.4 V, min working current **1 A** | [datasheet](https://www.hiwonder.com/search/suggest.json?q=LDX-218&resources[type]=product) |
| **LFD-06** | metal claw / gripper | PWM; closest documented relative **LFD-01M**: locked-rotor current **≤1.1 A**, claims "built-in anti-blocking algorithm" | [LFD-01M](https://www.hiwonder.com/search/suggest.json?q=LFD-01M&resources[type]=product) |
| **LD-1501MG** | large bottom plate | **UNKNOWN** — no datasheet found; assume LDX-218-like | — |

**Raw units are literally PWM pulse width in microseconds.** 500–2500 µs maps to
0–180° of *servo* rotation. This is the **electrical** range. The **mechanical**
range of each joint — limited by linkages, the chassis and the claw geometry — is
narrower, and that gap is exactly what destroyed us on 2026-07-28.

### Servo-ID → joint mapping (independently cross-validated)

Our mapping is confirmed by `andrewda/learm_ros2`, an unrelated ROS2 driver that
arrived at the identical assignment:

| ID | Joint | learm_ros2 name |
|---:|---|---|
| 1 | GRIP | `grip_right` (note: "open is close", axis flipped in their URDF) |
| 2 | WRIST_ROLL | `wrist_roll` |
| 3 | WRIST_FLEX | `wrist_flex` |
| 4 | ELBOW | `elbow` |
| 5 | SHOULDER_LIFT | `shoulder_lift` |
| 6 | SHOULDER_PAN | `shoulder_pan` |

---

## 4. Implications for our SDK

1. **Never trust the 500–2500 range.** It is electrical, not mechanical.
2. **Shadow-track position in software**; there is no alternative.
3. **Unload (cmd 20) is the safety primitive.** It works, needs no feedback, and
   is the only way to end a stall. Use it liberally.
4. **The camera is the only real feedback loop.** Treat vision as a sensor, not a
   nicety.
5. **Handle cmd 8** (`ACTION_GROUP_COMPLETE`) — it arrives unsolicited and will
   otherwise desynchronise the input report parser.
6. **Never send cmd 27.** It targets bus servos we do not have.
7. Consider the **deviation offset** (−100..+100) for centring calibration, but
   do not mistake it for feedback.

---

## 5. Open questions

- Per-joint mechanical limits in raw units — **UNKNOWN**, must be measured by
  hand with torque off. This is the single biggest gap.
- LD-1501MG specifications — **UNKNOWN**.
- Whether the claw servo's "anti-blocking algorithm" actually cuts drive on
  stall, and whether LDX-218 has it — under investigation.
- Whether battery voltage (cmd 15) sags measurably under stall, which would give
  us a second, non-visual stall detector — under investigation.
