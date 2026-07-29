# Cocktail robot — feasibility and design

Date: 2026-07-28. Hardware: Hiwonder LeArm (LSC-6 USB-HID controller, 6 PWM servos, no
position feedback) + one USB webcam + MacBook (no NVIDIA GPU).

## TL;DR

| Question | Answer |
|---|---|
| Can I use Gemini Robotics? | **Partly.** The action-output VLA is trusted-tester only. `gemini-robotics-er-1.6-preview` (perception/reasoning) **is** on our API key and **verified working** — see below. |
| Can any VLA drive this arm? | **No.** Every VLA surveyed requires readable joint state. This arm has none, by construction. |
| Can the arm pour from a bottle? | **No.** A full 750 ml bottle (~1.2 kg) is ~5–8× the estimated safe payload. |
| So what do we build? | LLM plans → classical scripted controller executes (the *Code-as-Policies* pattern). Bottles fixed + pumps. Arm moves the **glass** only. |

## 1. Gemini Robotics: what we can and cannot use

| Tier | Access | Output | Us |
|---|---|---|---|
| Gemini Robotics-ER 1.6 | ✅ public API | 2D points `[y,x]`/1000, boxes, image-space trajectories, 3D boxes | ✅ **usable** |
| Gemini Robotics (full VLA) | ❌ trusted testers (Boston Dynamics, Apptronik, Agility…) | robot actions | ❌ |
| Gemini Robotics On-Device | ❌ trusted-tester SDK, validated hardware only | robot actions | ❌ |

Source: <https://ai.google.dev/gemini-api/docs/robotics-overview>,
<https://deepmind.google/blog/gemini-robotics-brings-ai-into-the-physical-world/>

**ER 1.6 is not a controller.** It outputs where things are, not what the servos should do.
Google's own docs say to combine it with "a low-level controller" you supply. That is exactly
what `robohand` already is.

### Verified on our hardware, 2026-07-28

Key: Bitwarden item `GEMINI_API_KEY` (value is in the item **notes**, not the password field).
The stale key in `~/.env.d/google_genai.env` is **invalid** — do not use it.

```bash
source ~/.env.d/bitwarden.env
K=$(bw get notes GEMINI_API_KEY | tr -d '\n' | grep -oE 'AIza[A-Za-z0-9_-]{20,}' | head -1)
curl -s "https://generativelanguage.googleapis.com/v1beta/models?key=$K&pageSize=500" \
  | python3 -c "import sys,json;print([m['name'] for m in json.load(sys.stdin)['models'] if 'robot' in m['name']])"
# -> ['models/gemini-robotics-er-1.5-preview', 'models/gemini-robotics-er-1.6-preview']
```

Pointing test against `docs/media/snapshot.jpg` (real photo of our arm), 1153 prompt tokens:

```json
[{"point": [542, 321], "label": "robot arm's gripper/claw tip"},
 {"point": [552, 381], "label": "graspable object on the table"}]
```

Rendered result: `docs/media/gemini_er_pointing.jpg`.

**Honest assessment of that result:**
- gripper tip → px (410, 390): roughly right, **~20–30 px off** the actual claw.
- "graspable object" → px (487, 397): **a hallucination.** There is no object in that frame; it
  pointed at a dark box in the background. ER will invent an answer rather than say "none".

Conclusion: ER is good for *coarse semantic localization* ("which one is the gin", "where is the
glass"). It is **not** good enough for the final centimetres of a grasp, and it must never be
trusted to assert an object exists. Guard every ER call with an explicit "return [] if absent"
instruction plus a sanity check against classical detection.

## 2. Why no VLA can drive this arm

Not an access problem — an architectural one.

```
VLA (OpenVLA / π0 / Octo / SmolVLA / GR00T / Gemini Robotics)
        │  requires
        ▼
  observation = { images , JOINT STATE }        ← we cannot produce this
        │
        ▼
  action = joint targets @ 30-50 Hz closed loop
```

LeRobot proves it at the code level — `so_follower.py::get_observation()` does
`self.bus.sync_read("Present_Position")`, a real hardware bus read. Our PWM servos
(LDX-218 / LFD-06 / LD-1501MG) have **no return data line at all**; `cmd 21` never replies
(see `docs/research-hardware.md`). You cannot write a LeRobot driver for this arm without
faking ground truth, which poisons everything downstream.

| Model | Open | Runs on our Mac | Needs joint state | Verdict |
|---|---|---|---|---|
| Gemini Robotics-ER 1.6 | ✅ API | ✅ cloud | no | ✅ use as planner |
| Gemini Robotics (VLA / on-device) | ❌ | – | yes | ❌ |
| OpenVLA (7B) | ✅ | ❌ CUDA + flash-attn, no aarch64 wheels | **no** — image+prompt only | ❌ *see below* |
| Octo | ✅ | ❌ GPU/TPU | yes | ❌ |
| π0 / openpi | ✅ weights | ❌ **NVIDIA + Ubuntu only**, 8–70 GB VRAM | yes | ❌ |
| SmolVLA (450M) | ✅ | ✅ **CPU/Mac ok** | yes | ❌ *compute fine, feedback fatal* |
| GR00T N1 | ✅ | ❌ GPU | yes | ❌ |
| VLM → scripted primitives | ✅ | ✅ | **no** | ✅ **the only sound path** |

SmolVLA is the instructive one: the compute objection dissolves and it *still* doesn't work.
The blocker is the missing sensor, not the missing GPU.

Reference architecture to copy instead: **Code-as-Policies**
(<https://code-as-policies.github.io>) and **VoxPoser** (<https://voxposer.github.io>) — LLM
writes/selects calls into a classical motion library. The Gemini Robotics paper itself cites
Code-as-Policies for its zero-shot mode (arXiv:2503.20020).

## 3. The payload problem — why the design must invert

- LDX-218 / LD-1501MG: **17 kg·cm stall** torque. Stall torque is the *failure* point, not a
  working rating (we already stalled a servo once — `docs/safety.md`).
- Reach ~15–20 cm → optimistic holding capacity **~0.15–0.3 kg at extension**; safe working
  payload realistically **under 200 g**.
- Full 750 ml bottle ≈ **1.2–1.3 kg** (estimate: ~500 g glass + 750 g liquid).
- No vendor whole-arm payload spec exists — Hiwonder's pages are unreadable. This is an
  **estimate**, flagged as such. But there is no reading of 17 kg·cm that reaches 1.2 kg.

Corroboration from the real world — nobody does it the naive way:

| System | Approach |
|---|---|
| Makr Shakr ("Toni") | **industrial KUKA arms** grip and tip real bottles |
| Bartendro (open source) | **no arm at all** — peristaltic pumps |

Both ends of the market avoid "small arm lifts full bottle". So do we.

## 4. Recommended architecture

```
   ┌─────────────────────────────────────────────┐
   │  Gemini ER 1.6 / any VLM   (~1 call/step)   │  semantic: which bottle, where's the glass
   └───────────────┬─────────────────────────────┘
                   │ 2D points (0-1000)
   ┌───────────────▼─────────────────────────────┐
   │  ArUco tags + homography (robohand/vision)  │  precise ID + pose on the table plane
   └───────────────┬─────────────────────────────┘
                   │ workspace X,Y
   ┌───────────────▼─────────────────────────────┐
   │  robohand safety layer (limits/ladder/      │  ← the only thing that talks to servos
   │  watchdog/vision stall guard/MotionToken)   │
   └───────────────┬─────────────────────────────┘
                   │ open-loop PWM
   ┌───────────────▼─────────────────────────────┐
   │  LeArm — moves the GLASS only               │
   └─────────────────────────────────────────────┘

   Liquid path is entirely separate:
   fixed bottles ──▶ peristaltic pumps / optics ──▶ nozzle over glass
                     triggered by script, volume by timed pour + load cell
```

Key decisions:
1. **Arm carries the glass, never a bottle.** Empty/half-full glass ≈ 100–300 g, borderline but
   plausible; a bottle is not.
2. **Fixed bottles + pumps** (Bartendro precedent). Removes the payload problem *and* the
   pour-orientation problem *and* the spill risk in one move.
3. **ArUco tags** on glass and bottles (`cv2.aruco`) for robust ID + orientation — more reliable
   than colour thresholding under bar lighting, and gives orientation which ER does not.
4. **Look-move-look-again visual servoing** for the last few cm. The camera closes the loop the
   servos can't. Works with a bad kinematic model; needs no encoders. This is the single most
   important technique for this hardware.
5. **Volume**: timed pour (primary) + HX711 load cell under the glass (closed loop). VLM
   liquid-level estimate as a *sanity check only*, never as control.

## 5. Phased plan

| Phase | Scope | New hardware |
|---|---|---|
| **0** *(blocked)* | Operator hand-calibrates joint limits with torque off; first verified pick of a light object on video | none |
| **1** | ER/VLM finds glass → homography → scripted glass pick/place → pump triggered for a timed pour → scripted stir. Legitimate "AI-directed cocktail assembly" demo. | pump(s) |
| **2** | ArUco tags; look-move-look-again servoing on final approach; load-cell volume closed loop | tags, HX711 (~$5–10) |
| **3** *(only if we want real VLA/imitation learning)* | Swap to feedback-capable bus servos → unlocks LeRobot / SmolVLA / π0 | Hiwonder LX-224HV retrofit, or SO-101 arm (~$120–230; verified $229.88 for the leader+follower bundle) |

Phase 3 is the honest answer to "I want Gemini Robotics": **buy servos that can report their
position.** No amount of software gets us there on the current arm.

## 6. Safety additions for liquid

Beyond the existing three layers (`docs/safety.md`):
- Splash guard / drip tray; no connectors under the pour point.
- Independent **time-based** cutoff on every pump, not just the vision guard — vision fails on
  occlusion and lighting, and a tipping glass is faster than a frame-diff detector.
- `reset_arm.py` reachable at all times (safe to run mid-stall).

## 7. Known gaps

- No vendor payload spec for LeArm — payload figure is an estimate from servo torque.
- 750 ml bottle weight is a calculated estimate, not a sourced spec.
- ER 1.6 pricing / rate limits for the preview model: not published in static docs.
- No public example found of anyone driving a LeArm/Braccio-class arm with an LLM end to end.
  Only a non-LLM ROS2 driver exists: <https://github.com/andrewda/learm_ros2>.
- `cv2.calibrateHandEye` needs *achieved* poses we cannot read — textbook hand-eye calibration
  is unavailable to us. Visual servoing replaces it.

## Appendix — Jetson? (2026-07-28)

**No. Not until the servos are replaced.** A Jetson fixes compute. Our blocker is a sensor.

### Correction to the table above

**OpenVLA is not state-conditioned.** Its documented inference call takes image + prompt only —
it is the one surveyed action-output VLA that does not structurally require proprioception. An
earlier version of this doc said otherwise; that was wrong.

It still doesn't work for us, one layer down: it emits **7-DoF end-effector deltas**, and you
cannot convert a delta into a joint command without knowing your current pose — which is exactly
what we can't measure. Plus 7B Llama-2 (~15 GB of weights) pinned to `flash-attn 2.5.5` /
`torch 2.2.0` with no aarch64 wheels. Right conclusion, wrong reason, now fixed.

### Memory floor vs. what "a small Jetson" means

GR00T N1.7 README, Hardware Requirements: *"Inference: 1 GPU with 16 GB+ VRAM."*

| Module | Memory | GR00T (16 GB floor) |
|---|---|---|
| Jetson Nano (original, 4 GB) | 4 GB | dead — stuck on the JetPack 4.6.x line |
| **Orin Nano 8 GB / "Super" ($249)** | 8 GB | **blocked** — this is what "a small Jetson" usually means |
| Orin NX 8 GB | 8 GB | blocked |
| Orin NX 16 GB | 16 GB | exactly at the floor; in *no* NVIDIA benchmark or install script |
| AGX Orin 64 GB | 64 GB | works at **2.9–4.6 Hz** |
| AGX Thor (128 GB) | 128 GB | 10.7 Hz w/ TensorRT |

NVIDIA's own benchmark table labels the AGX Orin row *"Legacy Jetson edge… only suitable for
slow, non-reactive tasks."* That is their words about their $2k-class module.

GR00T does ship real Jetson support (`scripts/deployment/orin/`, `thor/`, Docker profiles
`gr00t-orin` / `gr00t-thor`, tested on JetPack 6.2 / 7.1) via the Jetson AI Lab wheel index —
but `torchcodec` builds from source on aarch64, and `uv run python` there will silently destroy
the platform environment by re-syncing against the x86_64 `pyproject.toml`.

### The state requirement, restated at the source

- GR00T's observation dict is `{video, state, language}`; `state` is mandatory and validated at
  runtime under `strict=True`. N1.7's relative-EEF action space is *defined* as deltas from
  current pose.
- SmolVLA takes "the robot's current sensorimotor state" as one of three required inputs.
- LeRobot's `SOFollower.get_observation()` is one line: `sync_read("Present_Position")`. There is
  **no stateless robot class anywhere** in the `robots/` tree.

### What to buy instead

**~$122, and none of it a Jetson.** SO-101 *follower-only* (LeRobot-native, Feetech STS3215
serial bus servos with position readback):

| Item | Cost |
|---|---|
| 6× STS3215 @ $13.89 | $83.34 |
| Waveshare bus servo driver board | $10.60 |
| cable / PSU / clamps / drivers | $28.00 |
| **total** | **$121.94** + 3D-printed parts (budget $30–60 if outsourced) |

Compute stays as-is: SmolVLA is 450 M params and HuggingFace states it runs "even a MacBook".
Fine-tune on a rented A100 (~4 h / 20k steps). GR00T, openpi and SmolVLA all ship remote
policy-server transports (ZMQ/websocket) *precisely so the GPU need not be on the robot* — so
even in the good case the Jetson is optional.

### Order of operations

1. Buy the SO-101 follower kit. It is the cheapest line item here and the only one that removes
   the actual blocker.
2. Keep the LeArm on the Gemini-ER + scripted-controller path. It matches what an open-loop arm
   can actually do, and "pick up an object" needs no 30 Hz loop. Safety layer and sim carry over.
3. On arrival: calibrate, teleoperate **~50 episodes** (5 object positions × 10 — HF explicitly
   reports 25 episodes *failing*), push the dataset.
4. Fine-tune `lerobot/smolvla_base`; run rollouts from the MacBook.
5. Only if you can *name* a measured latency problem, price a Jetson — against a cloud policy
   server, which is what NVIDIA's own PolicyServer/PolicyClient architecture exists for.

### Unverified

- LX-224HV as a drop-in for LDX-218 in a LeArm chassis: no dimensional drawings found, and it's
  an 11.1 V HV servo against the LeArm's 6–7.4 V rail. **Treat the retrofit as unproven** — the
  SO-101 route avoids the question entirely.
- Orin NX 16 GB actually loading GR00T alongside an OS: UNKNOWN, untested by NVIDIA.
- Jetson Nano's exact terminal JetPack version (EOL direction confirmed; "4.6.7" not confirmed).
- 3D-printing cost, and current Koch v1.1 / Moss / Trossen prices.
