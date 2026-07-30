# Robot brain design

Date: 2026-07-30. Hardware: Hiwonder LeArm, LSC-6 controller, 6 PWM servos, 1 USB webcam, MacBook.

## The constraint

The servos report nothing. No position, no current, no temperature. The controller only sends PWM out.

The camera is the only sensor.

## What was wrong

Every vision decision asked one question: how many pixels changed between two frames?

That question is about the room, not the arm. Measured 2026-07-30, arm held still:

| Scene | Noise floor (p95) | Gate at threshold 3,000 |
|---|---:|---|
| Empty room | 1,243 changed px | PASS |
| One person in frame | 43,335 | FAIL |
| Person cropped out of the ROI | 5,869–13,676 | FAIL |
| Tight crop on the arm only | 6,375 | FAIL |

A person shifting in a chair beats the signal from a moving joint.

Three fixes were tried. All three failed:

- Tighter ROI. The person's reflection is in the TV. Auto-exposure reacts to them and shifts the whole frame.
- Brightness normalisation (`vision.normalize=zscore`). Worse: 10,000 vs 6,375.
- Higher threshold. It would then sit above real motion too.

No threshold separates "the arm moved" from "a human moved" when the measurement is "some pixels differ".

## The three options, and which one we can use

**Joint encoders.** What LeRobot-supported arms use. `obs = robot.get_observation()` → `select_action`. Closed to us: the LSC-6 reads nothing back. This also rules out imitation learning (ACT, diffusion, OpenVLA, π0) — those need readable joint state to record demos and to execute. Opening it means replacing all six servos with bus servos plus a new controller. Multi-week.

**Fiducial markers.** A printed pattern with a known ID. The detector looks for that pattern only. Ships in OpenCV as `cv2.aruco`. This is the standard substitute when there are no encoders. **Chosen.**

**A VLM.** Good at meaning. Bad at geometry. Tested on this arm 2026-07-28: Gemini Robotics-ER 1.6 missed the gripper by 20–30 px and reported a graspable object in a frame that had none. Google's own guidance is to pair ER with a low-level controller you supply.

## Design

```
   YOU
    │  "pick up the bottle"
    ▼
┌─────────────────────────────────────────────────────┐
│ MEANING          LLM agent (pi) + VLM               │
│                  Which object. What order. When to  │
│                  stop. Reads an annotated frame.    │
│                  1–3 s. Costs money. Can be wrong.  │
└─────────────────────────┬───────────────────────────┘
                          │  names a target: "marker 1"
                          │  never a joint angle
                          ▼
┌─────────────────────────────────────────────────────┐
│ GEOMETRY         ArUco markers (robohand/markers.py)│
│                  marker 0 = gripper, at (412, 233)  │
│                  marker 1 = object,  at (455, 240)  │
│                  10 ms. Free. Deterministic.        │
│                  A person is not a marker.          │
└─────────────────────────┬───────────────────────────┘
                          │  pixel error: 43 px right, 7 px down
                          ▼
┌─────────────────────────────────────────────────────┐
│ CONTROL          Visual servo loop                  │
│                  Step the joint. Re-measure. Repeat.│
│                  No IK. No hand-eye calibration.    │
└─────────────────────────┬───────────────────────────┘
                          │  requested move
                          ▼
┌─────────────────────────────────────────────────────┐
│ SAFETY           robohand safety layer (unchanged)  │
│                  Joint bands. Absolute rails.       │
│                  Single-use move tokens.            │
│                  Torque time budget. CAN REFUSE.    │
└─────────────────────────┬───────────────────────────┘
                          ▼
                       SERVOS
```

The model names a marker. It never sends a number to a servo. A wrong answer from the model cannot become a bad servo command.

## Evidence

Same two frames. Arm still. Large clutter moving elsewhere.

```
absdiff  ->  106,750 changed px   threshold 3,000  ->  "MOTION"     wrong
markers  ->        0.00 px moved  threshold   3.0  ->  "no motion"  right
```

`robohand markers` on the live camera with a person in frame: 0 markers found. Not below threshold. Absent.

## Rules

- A marker that is not detected raises an error. It never reports "no motion". `calibrate` infers mechanical stops from repeated no-motion, so a marker falling off would otherwise be written into `config.json` as a joint limit.
- The VLM never returns a coordinate that reaches a servo.
- The safety layer is not modified by any of this.
- The absdiff path stays. It is still a cheap stall check during one commanded move, where a false positive is harmless and the marker may be hidden by the arm.

## Not built

- No `solvePnP` pose estimation. It needs camera intrinsics. Nobody has run a checkerboard calibration on this webcam.
- No hand-eye calibration. No inverse kinematics. The loop servos in image space.
- No VLA. Requires joint state we do not have.

## Cost

Markers must be printed, taped on, and stay stuck. They must be unoccluded at the poses that matter. That is recurring physical maintenance, like camera aim.

## Status

| Item | State |
|---|---|
| `robohand/markers.py` + `ArucoMotionObserver` | Built. 15 tests. |
| `robohand markers --generate` / `robohand markers` | Built. |
| `robohand calibrate --observer aruco` | Built. |
| Markers physically taped to arm | **Operator. 2 minutes.** |
| WRIST_ROLL calibration | Blocked on the above. |
| SHOULDER_LIFT (servo 5) | **Broken. Intermittent.** See docs/hardware-fault-servo5.md. |

## Next

1. `robohand markers --generate sheet.png`. Print. Tape marker 0 to the gripper, marker 1 to the bottle.
2. `robohand markers` to confirm both are seen.
3. `robohand calibrate --observer aruco --joint WRIST_ROLL --i-am-present-and-watching`.
4. Close the pick loop on marker 0 vs marker 1.

## References

Fetched and read 2026-07-30 unless noted.

**Approach we chose**

- OpenCV, *Detection of ArUco Markers*. <https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html> — the detector, the dictionaries, and the quiet-zone requirement.

**LLM as planner, not as controller** — the pattern this design follows

- Liang et al., *Code as Policies: Language Model Programs for Embodied Control*, arXiv:2209.07753. <https://arxiv.org/abs/2209.07753> — the LLM writes a program that calls perception and control APIs. It does not emit motor commands.
- Ahn et al., *Do As I Can, Not As I Say: Grounding Language in Robotic Affordances* (SayCan), arXiv:2204.01691. <https://arxiv.org/abs/2204.01691> — the model proposes, a separate layer decides what is physically possible.
- Google, *Gemini Robotics ER*, Gemini API docs. <https://ai.google.dev/gemini-api/docs/robotics-overview> — ER outputs points and boxes, and the docs say to pair it with a low-level controller you supply.

**End-to-end learned control** — surveyed and ruled out, all need joint state

- Kim et al., *OpenVLA: An Open-Source Vision-Language-Action Model*, arXiv:2406.09246. <https://arxiv.org/abs/2406.09246>
- Black et al., *π₀: A Vision-Language-Action Flow Model for General Robot Control*, arXiv:2410.24164. <https://arxiv.org/abs/2410.24164>
- Hugging Face, *LeRobot*. <https://github.com/huggingface/lerobot> — supported arms (SO-100, Koch, OMX, Reachy2) all report joint state; the standard loop is `get_observation` → `select_action` → `send_action`.

**This project's own measurements**

- `docs/cocktail-robot.md` — the 2026-07-28 Gemini ER 1.6 test on a real photo of this arm: gripper missed by 20–30 px, hallucinated object.
- `docs/camera-field-of-view.md` — the noise-floor table above.
- `docs/research-hardware.md` — what the LSC-6 can and cannot report.
