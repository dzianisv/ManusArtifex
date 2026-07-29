# Verification — camera-health gate and robust motion metric

This document records the empirical basis for the camera-health gate
(`robohand/camera_health.py`), the opt-in brightness-normalised motion metric
(`guard.VisionGuard.normalize`), and their wiring into calibration, the CLI, and
the safety layer.

**No command here, and no test in `tests/`, ever opens the real HID device (VID
0x0483 / PID 0x5750) or commands any arm motion.** The camera (index 0, the
built-in macOS webcam) is camera-only — no HID involved. Automated tests use
`sim.SimTransport` and synthetic frames exclusively; the real-camera measurements
below were taken by hand via `robohand camera-check` and a scratch metric script,
and are reproduced here for the record.

Environment: Python 3.10, pytest 9.0.2, numpy, opencv-python 4.x, macOS (Darwin),
camera index 0 via `cv2.CAP_AVFOUNDATION` (auto-exposure/auto-focus **cannot** be
locked — `cap.set(CAP_PROP_AUTO_EXPOSURE/EXPOSURE/AUTOFOCUS)` all return `False`).

> **⚠️ Read this first.** The
> **[Full-arm-in-frame measurement](#full-arm-in-frame-measurement-authoritative--supersedes-all-prior-rounds)**
> section immediately below is the **authoritative** empirical basis for the shipped
> `robohand/data/config.json` values (`min_changed_pixels=3000`, `roi=[0,0,1.0,1.0]`
> = full frame, `health_interval_s=0.8`). It **supersedes all** earlier rounds,
> including the **[Operator re-measurement](#operator-re-measurement-superseded-by-full-arm-in-frame-measurement)**
> section (which used the now-obsolete narrow ROI `[0,300,520,420]` and
> `min_changed_pixels=500`) and the original task-prompt numbers. Those earlier
> tables are kept, marked superseded, for the paper trail only.

---

## Full-arm-in-frame measurement (authoritative — supersedes all prior rounds)

**Today's round.** The physical camera mount was **improved**: it now frames the
**ENTIRE arm — base, all links, and the gripper — well lit, in the full 1280×720
frame.** The old narrow ROI `[0,300,520,420]` (tuned for a mount that could only see
the rotating base in a dark corner) is **obsolete** and is **no longer the default**.
Full-frame raw absdiff is now sufficient on its own.

**This changes the gate's role** from *"workaround for a permanently blind/noisy
camera that could never reliably see real motion"* to *"regression guard against
re-aiming the camera, lighting changes, or scene obstruction."* The underlying
problem the gate was originally built for is **resolved on this specific
deployment** — but the gate **remains valuable and required** precisely because that
good state can regress: someone re-aims the laptop, the room goes dark, or a person
walks in front of the lens. Those must be **caught loudly**, not silently produce
bad calibration data.

### Protocol

- **20 warm-up reads** after `cv2.VideoCapture(0)` opens (matches the shipped
  `CameraConfig.warmup_frames=20` in `robohand/camera.py` — **no camera.py change
  was needed**).
- **8 flush reads before every captured frame** (matches shipped
  `CameraConfig.flush_reads=8`).
- **Plain `absdiff>25` over the FULL frame — NO ROI** (`diff_threshold=25` matches
  this protocol).

### Result (operator's real-hardware numbers, verbatim)

| sample | changed px | separation vs 577 noise |
|---|---:|---:|
| no-motion control (zero commanded motion), sample 1 | 454 | — |
| no-motion control (zero commanded motion), sample 2 | **577** (worst noise) | — |
| SHOULDER_PAN commanded 60 raw units, 1200 ms | **78,116** | **135×** |
| **GRIP** commanded move, 1200 ms | **25,746** | **45× — WORST-CASE real signal** |

**GRIP is the worst case and the one that matters most:** it has the smallest visual
signature of any joint, *and* it is the joint that was damaged in the original
2026-07-28 incident. Even so, at 25,746 changed px it clears the 577 noise floor by
45×.

### Shipped `config.json` values and their justification

| key | prior (narrow-ROI round) | **new (full-frame)** | why |
|---|---|---|---|
| `vision.roi` | `[0,300,520,420]` (pixels, narrow) | **`[0,0,1.0,1.0]`** (fractional = full frame) | camera now frames the whole arm; `guard._resolve_roi` maps values in (0,1] to frame fractions, so this is x=0,y=0,w=1280,h=720 = no cropping. A narrower ROI stays available via `camera-check --roi` and would widen the GRIP margin, but is **not required** and **not the default**. |
| `vision.min_changed_pixels` | 500 | **3000** | ~5.2× above the worst no-motion sample (577) and ~8.6× below the worst-case real signal (GRIP, 25,746); sits near the geometric mean √(577·25746) ≈ 3,854 — comfortably separated from both noise and the weakest real signal on a log scale. |
| `vision.health_margin` | 1.5 (kept) | **1.5** (kept) | required = 1.5 × 577 ≈ **865.5**, comfortably below 3000 → a further **~3.5×** buffer beyond the bare minimum. Passes comfortably now, still rejects a genuine dark-scene regression. |
| `vision.health_separation` | 3.0 (kept) | **3.0** (kept) | floor = 3 × 577 ≈ **1731**; GRIP (25,746) clears it by **~14.9×** → the visibility spot-check also passes comfortably. |
| `vision.health_interval_s` | 0.8 (kept) | **0.8** (kept) | validated in the prior round; unchanged. |
| `vision.health_frames` | 15 (kept) | **15** (kept) | unchanged. |
| `vision.diff_threshold` | 25 (kept) | **25** (kept) | matches the operator's `absdiff>25` protocol. |

**Only `robohand/data/config.json` (the live config for *this* arm+camera) carries
these values.** `robohand/config.py`'s generic code-level fallbacks
(`150` / `[0.15,0.1,0.7,0.8]`) are **left untouched** — they serve other, unmeasured
deployments.

### PASS-case arithmetic (default config, new good setup)

```
required_threshold = health_margin × p95_noise = 1.5 × 577 ≈ 865.5
gate PASSES iff  min_changed_pixels ≥ required_threshold
                 3000 ≥ 865.5   ✓  PASS
headroom beyond bare minimum:   3000 / 865.5 ≈ 3.5×
headroom over the raw noise:    3000 / 577   ≈ 5.2×

visibility floor = max(threshold, health_separation × p95_noise)
                 = max(3000, 3.0 × 577) = max(3000, 1731) = 3000
GRIP motion 25,746 ≥ 3000  ✓  visible (and 25,746 / 577 ≈ 45× over raw noise,
                                        25,746 / 1731 ≈ 14.9× over the 3× floor)
```

### FAIL-case arithmetic (illustrative regression back to a dark/exposure-hunting scene)

If the camera is re-aimed into a dark corner, the room goes dark, or exposure
hunting resumes, the no-motion floor climbs far above the ~577 seen here. **Any
no-motion floor above roughly `3000 / 1.5 ≈ 2000` changed px makes the gate FAIL:**

```
Illustrative severe exposure hunt, p95_noise = 200,000 changed px
  (cf. the ORIGINAL uncorrected numbers: 268,518 / 546,921 / 550,276 no-motion px)
required_threshold = 1.5 × 200,000 = 300,000
gate PASSES iff  3000 ≥ 300,000   ✗  FAIL (loudly refuses, writes nothing)

Even a milder regression, p95_noise = 2,001 changed px:
required_threshold = 1.5 × 2,001 = 3,001.5
                     3000 ≥ 3,001.5   ✗  FAIL  (threshold just barely undershot)
```

The boundary is exactly `min_changed_pixels / health_margin = 3000 / 1.5 = 2000`
changed px of no-motion noise. This is also enforced by a regression test — see
`tests/test_camera_health.py::test_full_frame_shipped_numbers_pass_and_dark_regression_fails`.

**ROI restriction is no longer *required* for correct operation.** It remains
available (`robohand camera-check --roi X Y W H`) and would further widen the GRIP
margin if persistent room clutter / people traffic ever becomes a noise source; it
is documented as an **optional hardening step, not the default**. If the camera or
arm mount is moved again, re-measure the ROI and thresholds with `robohand
camera-check`.

### Live `robohand camera-check` output (this machine, full frame, after the config change)

Fresh evidence captured on camera index 0 with the new full-frame config (no `--roi`
override). The quiet-room noise floor here (p95 ≈ 37) is *lower* than the operator's
on-site 577 — exactly the site-to-site variability the gate exists to measure — and
both correctly PASS:

```
$ python3 -m robohand.cli camera-check
Camera health: PASS
  frames sampled:   15 (interval 0.8s, 14 pairs)
  metric:           normalize=none
  no-motion noise:  p50=7  p95=37  max=43 changed px
  motion threshold: 3000 changed px
  required (>= p95 x 1.5): 56
  headroom (threshold / p95 noise): 80.8x
Camera is usable for vision-guided calibration.
```

`--sim doctor` on the same machine likewise reports `Camera health: PASS`,
`motion threshold: 3000`, and `layer3_vision_guard: ACTIVE`.

---

## Operator re-measurement (SUPERSEDED by Full-arm-in-frame measurement)

> **⚠️ Superseded (see [Full-arm-in-frame measurement](#full-arm-in-frame-measurement-authoritative--supersedes-all-prior-rounds) above).**
> This round was measured with the **now-obsolete narrow ROI `[0,300,520,420]`**,
> back when the camera mount could physically see **only** the rotating base in a
> dark corner. The mount has since been improved to frame the **entire arm** (base,
> all links, gripper), so full-frame operation is now the shipped default and
> `min_changed_pixels` moved from 500 → 3000. The `roi`/`min_changed_pixels`
> conclusions below no longer describe the shipped config; the *methodology*
> (warm-up/flush protocol, measure the live floor on site) remains valid and is
> carried forward. Kept for the paper trail.

The human operator, who physically holds the real arm and camera, re-measured the
camera far more carefully than either the original bug report **or** the prior
"fixed capture path" round below. **These numbers are authoritative and supersede
both.**

### Protocol (the corrected, thorough one)

- **15 warm-up reads** immediately after `cv2.VideoCapture(0)` opens (the shipped
  code uses `warmup_frames=20`, a safe superset — left unchanged).
- **8 flush reads before *every* captured frame** (matches the shipped
  `CameraConfig.flush_reads=8`; see *Correction — capture-pipeline staleness*).
- **~0.8 s between samples** (the shipped health-gate code default was 0.4 s —
  *faster* than what the operator validated; `config.json` now overrides it to 0.8 s).
- **ROI = pixel rows 300:720, cols 0:520 in a 1280×720 frame** — in this codebase's
  `(x, y, w, h)` pixel convention, `(0, 300, 520, 420)`. This is the **arm's base
  region**, the *only* part of the arm this camera mount can physically see.
- **Motion sample** = the genuine SHOULDER_PAN evidence pair
  `/tmp/t_before.jpg` → `/tmp/t_after.jpg` (read-only). **Noise** = freshly captured
  zero-motion control frames through the corrected protocol.

### Result (verbatim)

| metric | motion (changed px) | noise (no-motion, two samples) | separation |
|---|---:|---:|---:|
| raw absdiff>25, FULL frame | 46,522 | 416, 426 | 110× |
| **raw absdiff>25, ROI (0,300,520,420)** | **33,615** | **99, 98** | **340× — BEST** |
| mean-normalised, FULL frame | 42,989 | 464, 478 | 93× |
| mean-normalised, ROI | 33,251 | 106, 115 | 310× |
| gradient, FULL frame | 14,711 | 12,170, 12,257 | **FAIL (~1.02×, no real separation at all)** |
| gradient, ROI | 12,697 | 1,158, 1,223 | 11× |

### Verdict (operator's, followed to the letter)

1. **The vision guard is NOT fundamentally broken.** Plain **raw absdiff restricted
   to the ROI** is the best *and* sufficient metric (340× separation), and it is
   *already* the shipped default (`VisionGuard.normalize="none"`) — **no metric code
   change was made or is needed.**
2. **Mean-normalisation is NOT clearly better** here (93–310× vs raw's 110–340×) —
   comparable at best, worse at times.
3. **Gradient is DISQUALIFIED.** On the full frame it fails outright (~1.02×
   separation — 14,711 signal vs ~12,200 noise, essentially no signal); even
   restricted to the ROI it manages only 11×, far below raw's 340×. **No gradient
   metric was or will be implemented in code.** This is evidence-based rejection,
   not a gap.
4. The dominant real noise sources are **(a)** missing warm-up / stale buffered
   frames right after open (fixed in code — see the capture-staleness correction
   below), and **(b)** people/objects moving *elsewhere in the room*, which the ROI
   restriction hugely suppresses (compare raw FULL noise 416/426 vs raw ROI 99/98).

### Shipped `config.json` values and their justification

| key | old (generic) | **new (site-specific)** | why |
|---|---|---|---|
| `vision.min_changed_pixels` | 150 | **500** | ~5× above the honest measured noise floor (p95 ≈ 99) and ~67× below the real motion signal (33,615) — genuine headroom both directions, deliberately conservative in the "declare no-motion/stall" direction (the safety-preferred failure mode on hardware with zero position feedback: too-low defeats stall detection — the exact original defect; too-high just protects slightly more eagerly). |
| `vision.roi` | `[0.15,0.1,0.7,0.8]` (fractional placeholder) | **`[0,300,520,420]`** (pixels) | the operator-measured base-region framing for this specific 1280×720 mount; suppresses room/people motion; gave the best raw separation (340×). |
| `vision.health_interval_s` | 0.4 (code default) | **0.8** | the operator's validated, more-thorough spacing; spans more of an auto-exposure hunting cycle and surfaces a materially higher, more honest noise floor than 0.4 s did. |
| `vision.health_margin` | 1.5 | 1.5 (kept) | now a **secondary** day-to-day factor layered on the primary absolute 500 threshold: required = 1.5 × 99 ≈ 148.5, comfortably below 500 → a further ~3.4× buffer. Intentional layered defense. |
| `vision.diff_threshold` | 25 | 25 (kept) | matches the operator's `absdiff>25` protocol. |

**Only `robohand/data/config.json` (the live config for *this* arm+camera) carries
these values.** `robohand/config.py`'s generic code-level fallbacks
(`150` / `[0.15,0.1,0.7,0.8]` / `0.4`) are **left untouched** — they exist for other,
unmeasured deployments of this library and must stay conservative/generic.

### Live corroboration on this machine (after the config change)

The **motion** measurement is deterministic — it scores the *same* shared evidence
JPEGs through the shipped guard, so it should reproduce the operator's numbers, and
it does (the small delta is the guard's `blur_ksize=5` Gaussian blur vs the
operator's unblurred `absdiff>25`):

```
$ robohand camera-check --before /tmp/t_before.jpg --after /tmp/t_after.jpg --roi 0 0 1280 720
  known-real move:  47469 changed px          (operator FULL: 46,522)

$ robohand camera-check --before /tmp/t_before.jpg --after /tmp/t_after.jpg   # ROI from config.json
  known-real move:  35033 changed px          (operator ROI:  33,615)
```

The **live noise floor measured here is lower** (p95 ≈ 4–27) than the operator's
(≈ 99) **because this machine's scene is quiet and empty**; the operator's room had
real environmental motion. That is not a contradiction — it is exactly the
variability the gate exists to measure *on site*: the same 500 threshold that sits
~5× above the operator's busy-room floor sits ~20–100× above this quiet-room floor,
and both correctly PASS. Full live output is pasted in
[Real `robohand camera-check` output](#real-robohand-camera-check-output-live-camera-index-0--after-the-fix) below.

### Why the gate exists (reframed per the operator)

1. **Force the warm-up/flush protocol.** Every `camera-check` / `calibrate()`
   invocation opens a *fresh* `Camera`, which always runs the one-time 20-frame
   warm-up, and `grab()` always flushes 8 stale frames on *every* call — so the
   corrected protocol is structurally unavoidable, not opt-in.
2. **Measure the LIVE no-motion floor at the moment of use.** `assess_camera_health`
   samples fresh frames every time it runs; nothing is cached across separate
   processes. A person walking through the room *during the sampling window*
   inflates several of the N−1 consecutive-pair scores, raising the measured p95
   above the demanding `500 / 1.5`-derived bar and making the gate **FAIL loudly**,
   not silently pass.
3. **Refuse when the achievable separation is too small.** The 500 threshold and the
   1.5× margin are demanding but comfortably achievable on a correctly-aimed,
   ROI-restricted camera (operator: 99 floor → 500 threshold = 5× headroom).

---

## The defect

The arm has **no position feedback** (`protocol.NoResponse`) — the camera is the
only signal for "did the joint move". `guard.VisionGuard` answers that by counting
changed pixels between a before/after frame and comparing to
`VISION_MIN_CHANGED_PIXELS` (default 150). That comparison is only meaningful if
the camera's *no-motion* noise floor sits **below** the threshold.

On a camera that cannot lock exposure, in a dark scene, it does not. Auto-exposure
hunts continuously, changing global brightness frame-to-frame, which raw absolute
differencing conflates with real motion. The originally reported deployment
numbers (raw absdiff, default ROI/threshold/blur):

| sample | changed px |
|---|---|
| **real** 2 s SHOULDER_PAN move | **46,411** |
| no motion (arm still), pair 1 | 268,518 |
| no motion (arm still), pair 2 | 546,921 |
| no motion (arm still), pair 3 | 550,276 |

The no-motion noise is **6–12× larger** than the real motion signal. Every frame
pair reads as "motion", so during calibration `consecutive_misses` never
accumulates, the observer-reliability burst is never triggered, and
`_search_direction` walks every joint to its absolute rail — reproducing the
2026-07-28 servo-stall incident, once per joint, on every run. The existing
reliability burst only detects the *opposite* fault (an observer that misses real
motion), so it is structurally blind to this one.

> **Correction (follow-up round, 2026-07-28 evening).** The table above and the
> "6–12×" figure were measured through a **defective capture path** and are
> inflated by a capture bug, not purely by lighting. See
> **[Correction — capture-pipeline staleness](#correction--capture-pipeline-staleness-follow-up-round)**
> below. The *gate's design is unaffected and correct*; what was wrong was the
> `Camera.grab()` staleness that made the measured noise floor unstable and far too
> high. With the capture fixed, raw absdiff within the ROI is a clean, sufficient
> metric (p95 ≈ 27–40 changed px vs a 150 threshold), and the z-score option — while
> retained — is **not** required by default.

The two real evidence frames confirm the mechanism: mean pixel brightness shifts
**35.8 → 44.7** between `t_before.jpg` and `t_after.jpg` — a global exposure
change, not a local structural one.

---

## Correction — capture-pipeline staleness (follow-up round)

The original measurements above (and the prior round's z-score-vs-raw comparison)
were **confounded by a capture bug in `camera.py`**, now fixed. `Camera.open()` did
an 8-frame warm-up **once**, but `Camera.grab()` then did a **single** `cap.read()`
per call with **no per-grab flush**. On this camera/AVFoundation combo a single
`read()` after a time gap — exactly the gap between a move's before/after frames,
and between the health gate's noise-floor samples (which sleep `interval_s` between
captures) — can return a **stale buffered frame** captured seconds earlier.
Diffing a stale frame against a fresh one inflates the measured *no-motion* noise
enormously, and makes it unstable run-to-run.

The tell was already in this document: the *same nominal setup* produced **three
different noise floors** — raw-absdiff+ROI p95 = **366** (research capture, above),
**2 972** (live `camera-check`, below), and **~4–27** (a hand probe with heavy
flushing). Three answers from one setup is not a stable property of the scene; it
is an unstable capture path. An independent real-hardware session reached the same
conclusion and fixed it the same way — "20 warm-up reads after open and 8 flush
reads before every captured frame" — after which *plain raw absdiff > 25* gave a
clean 45–240× separation with no normalisation (`docs/hardware-fault-servo5.md`).

**The fix** (`robohand/camera.py`): `CameraConfig.flush_reads = 8` (new) discards 8
buffered frames at the start of every `grab()`; `warmup_frames` raised `8 → 20`.
Both are dataclass defaults, so `arm._open_vision_guard()` and
`cli._open_camera_observer()` — which build `CameraConfig(index=0)` with no other
args — pick them up with **zero call-site changes**.

**Effect, lighting held constant** (15 frames @ 0.4 s, raw absdiff + ROI, same
scene, back-to-back recipes):

| capture recipe | no-motion p50 | p95 | max |
|---|---:|---:|---:|
| **current bug** (warmup 8, **flush 0**) | 22 | 34 | 35 |
| warmup 8, flush 8 | 1 | 28 | 29 |
| warmup 20, flush 8 (**shipped**) | 1 | 29 | 31 |
| warmup 20, flush 12 | 0 | 27 | 33 |

Flushing drops the *median* no-motion noise ~20× (22 → 1) with lighting unchanged —
that gap is pure capture staleness. In this stable evening scene the p95 is
dominated by genuine small scene changes (~30 changed px), not staleness, and every
recipe is far below the 150 threshold; the huge historical numbers (366 / 2 972)
needed the *unflushed* path to appear. `flush_reads = 8` removes the artifact and
matches the independently-validated recipe; `warmup 20` is a cheap one-time settle
margin (my data shows ≥ 8 warm-up is adequate once flushing is present, so 20 is
comfortably safe). **Shipping 0 flush is the original bug — never do it.**

---

## Metric research (item 1) — re-run through the fixed capture path

> **⛔ Superseded by
> [Operator re-measurement](#operator-re-measurement-authoritative--supersedes-all-prior-rounds).**
> The numbers in this section were measured on a *different, quieter local scene* at
> the **0.4 s** interval — which *undersamples* the auto-exposure hunting cycle and
> yields an artificially low noise floor (p95 ≈ 27–40) versus the operator's honest
> **≈ 99** at the validated **0.8 s** spacing, and used a looser fractional ROI.
> The section's **conclusion is unchanged and correct** — raw absdiff + ROI is the
> right, sufficient metric — but the specific separation figures below are *not* the
> authoritative ones. They are retained for the paper trail. The gradient metric the
> operator later disqualified was never implemented here; z-score remains an
> unused-by-default opt-in.

Signal = the real evidence frames (`/tmp/t_before.jpg` → `/tmp/t_after.jpg`, a
genuine SHOULDER_PAN move, read-only). Noise = **freshly captured no-motion control
frames through the fixed `Camera` path** (warmup 20, flush 8; 15 frames @ 0.4 s),
scored on every consecutive pair (a real before/after check judges one frame
against the very next, so consecutive pairs sample exactly that noise).

### Candidates

- **(a) raw absdiff** — the shipped baseline.
- **(b) per-frame mean subtraction** — cancels a global brightness *offset*.
- **(c) z-score normalisation** (subtract mean, divide by std, per frame) —
  cancels the full *affine* `gain·x + offset` transform, the first-order model of
  what an auto-exposure loop does to every pixel at once.
- each of the above, restricted to `config.VISION_ROI = (0.15, 0.1, 0.7, 0.8)`.

### Result — corrected, reproducible separation (two consecutive runs)

| metric | motion (signal) | noise p50 | noise p95 | noise max | motion / p95 |
|---|---:|---:|---:|---:|---:|
| raw absdiff (FULL) | 47,469 | 9 / 21 | 40 / 38 | 47 / 39 | 1172× / 1259× |
| **raw absdiff (ROI)** | **22,583** | 3 / 16 | **35 / 32** | 37 / 33 | **644× / 698×** |
| z-score (FULL) | 67,896 | 0 / 8 | 18 / 20 | 19 / 22 | 3700× / 3386× |
| z-score (ROI) | 29,830 | 0 / 9 | 19 / 22 | 20 / 24 | 1542× / 1353× |

Both runs agree to within a few changed pixels — the run-to-run instability that
plagued the unflushed path is gone. **Raw absdiff within the ROI is now sufficient
on its own**: a no-motion p95 of ~32–35 changed px against a real move of 22,583
(≈ 650–700× separation) and against the 150-changed-px threshold (which clears the
p95 with a 4–5× margin, well above the gate's required 1.5×). No normalisation is
needed to make the gate pass in normal lighting — matching the independent
hardware session's finding that "plain absdiff > 25 beat both mean-normalised and
gradient metrics" once capture was fixed.

### Why the z-score option is *retained but not the default*

The prior round argued z-score is "structurally required." **That conclusion was an
artifact of the stale capture path** — it was comparing metrics on an unstable,
inflated noise floor. Corrected, raw+ROI is fine. Z-score is kept only as an
**opt-in** hedge for genuinely adverse lighting (a dark scene whose auto-exposure
hunts hard), because it is *structurally* invariant to an affine exposure change.
That structural argument still holds — applying a synthetic affine exposure swing
to a single static frame (pure exposure hunting, **zero real motion**):

| pair (ROI, synthetic) | raw absdiff | z-score (0.5 std) |
|---|---:|---:|
| exposure hunt, gain 1.3 offset 15 (no real motion) | 216,543 | 483 |
| exposure hunt, gain 1.6 offset 30 (no real motion) | 515,881 | 1,423 |
| **real** motion (`t_before` → `t_after`) | **22,583** | **29,830** |

Under a *severe* hunt raw absdiff inverts the ordering (calls 515k px of pure
exposure "more motion than a real move"); z-score keeps it right. But that is a
worst-case dark-scene hedge, **not** the normal condition — and the *gate itself*
(measure the noise floor, refuse if the threshold cannot clear it) protects against
exactly this regardless of which metric is chosen. The gate is the load-bearing
fix; the metric is secondary.

### Decision

- **Default `VISION_NORMALIZE="none"` (raw absdiff) is not just compatible, it is
  sufficient** — confirmed by the corrected numbers above. All existing
  moves/grip-ladder/calibration behaviour and tests remain byte-identical.
- Keep `zscore` (and `mean`) as **opt-in** `config.json` `vision.normalize` modes
  for deployments on cameras that cannot lock exposure in a dark scene. When
  `zscore` is active, `VISION_MIN_CHANGED_PIXELS` must be re-tuned above the
  measured noise floor with `robohand camera-check` first (150 is a grey-level
  count, not a std-unit count). This machinery is harmless and may help in adverse
  lighting; it is simply **no longer claimed to be required**.

---

## The gate (items 2 & 3)

`robohand/camera_health.py`:

1. **Noise floor** (`assess_camera_health`): capture N frames with the observer,
   **zero motion commanded**, score every consecutive pair, take the p95. Require
   `motion_threshold ≥ p95 · margin`. Otherwise raise `CameraUnusableError` (an
   `ObserverError` subclass, so every fail-closed `except ObserverError` site
   handles it) and refuse.
2. **Visibility** (`verify_motion_visibility`): a noise-floor check alone passes a
   camera pointed at a wall (near-zero noise *and* signal). Given a known-real
   move, require its score to clear the noise floor by a `separation` factor.
   Deliberately a no-op when the measured noise floor is zero (a synthetic/blind
   observer) — that degenerate case is already owned, with a better diagnosis, by
   calibration's observer-reliability burst.

`MotionObserver` gained `score()` (continuous) and `motion_threshold()`, defaulted
so every existing observer keeps working, and overridden in `VisionMotionObserver`
to expose the guard's real changed-pixel count and `min_changed_pixels`.

Wiring:

- **`calibrate()`** runs the noise-floor gate once, before any joint's search
  (`require_camera_health`), and threads the report into each `calibrate_joint`; a
  direct `calibrate_joint` call runs the gate itself (no bypass). The visibility
  spot-check piggy-backs on the start-of-direction move `_search_direction`
  *already* makes — **zero extra torque, zero extra packets** — preserving every
  torque/packet-accounting assertion in `tests/test_auto_calibrate.py`.
- **`SafetyLayer.move()`** checks health lazily, once, cached — and *degrades
  safely* (treats a proven-unhealthy camera as no camera; see `docs/safety.md`).
- **`robohand camera-check`** and **`robohand doctor`** expose it to the operator.

Code-level fallback defaults (`robohand/config.py`, for *unmeasured* deployments):
`health_frames=15`, `health_interval_s=0.4`, `health_margin=1.5`,
`health_separation=3.0`. The live `robohand/data/config.json` for **this** arm+camera
overrides `health_interval_s` to **0.8** and `min_changed_pixels` to **500** — see
[Operator re-measurement](#operator-re-measurement-authoritative--supersedes-all-prior-rounds).

---

## Real `robohand camera-check` output (live camera, index 0) — after the fix

Run by hand (this is **not** a pytest — real camera hardware is not available in
CI), **with the new `config.json` values in place** (`min_changed_pixels=500`,
`roi=[0,300,520,420]`, `health_interval_s=0.8`). The gate **passes** this camera and
now prints an explicit **headroom** ratio (threshold ÷ live p95 noise):

```
$ robohand camera-check
Camera health: PASS
  frames sampled:   15 (interval 0.8s, 14 pairs)
  metric:           normalize=none
  no-motion noise:  p50=2  p95=16  max=27 changed px
  motion threshold: 500 changed px
  required (>= p95 x 1.5): 24
  headroom (threshold / p95 noise): 31.3x
Camera is usable for vision-guided calibration.
```

Supplying the genuine SHOULDER_PAN evidence pair via `--before/--after` measures the
**real** motion separation from the live noise floor (a diagnostic that reads image
files only — it commands no motion and never touches the HID device). The ROI comes
from `config.json`:

```
$ robohand camera-check --before /tmp/t_before.jpg --after /tmp/t_after.jpg
Camera health: PASS
  frames sampled:   15 (interval 0.8s, 14 pairs)
  metric:           normalize=none
  no-motion noise:  p50=4  p95=27  max=30 changed px
  motion threshold: 500 changed px
  required (>= p95 x 1.5): 41
  headroom (threshold / p95 noise): 18.2x
  known-real move:  35033 changed px
  measured separation vs real motion (move / p95 noise): 1278.6x
  visibility: PASS (need move >= 500)
Camera is usable for vision-guided calibration.
```

The `known-real move: 35033` reproduces the operator's ROI figure of **33,615**
(the small delta is the guard's `blur_ksize=5` blur vs the operator's raw
`absdiff>25`); the full-frame form (`--roi 0 0 1280 720`) gives **47,469** vs the
operator's **46,522**. The **live noise floor here (p95 ≈ 16–27) is lower** than the
operator's ≈ 99 because this machine's scene is quiet and empty — see
[Live corroboration](#live-corroboration-on-this-machine-after-the-config-change).

`robohand --sim doctor` surfaces the same verdict without hanging (5 frames, 0.1 s)
and never fails just because the camera is absent/unusable:

```
Camera available: True
Camera health: PASS
  frames sampled:   5 (interval 0.1s, 4 pairs)
  metric:           normalize=none
  no-motion noise:  p50=2  p95=27  max=31 changed px
  motion threshold: 500 changed px
  required (>= p95 x 1.5): 40
  headroom (threshold / p95 noise): 18.7x
Safety layers:
  layer1_limits_and_calibration: ACTIVE
  layer2_bounded_torque_watchdog: ACTIVE
  layer3_vision_guard: ACTIVE
```

The `health_margin=1.5` and `health_separation=3.0` factors are **kept**; only
`min_changed_pixels` (150 → **500**) and `health_interval_s` (0.4 → **0.8**) and the
ROI (fractional placeholder → **`[0,300,520,420]`**) were re-tuned, per the operator
measurement. With the operator's honest p95 ≈ 99, the 500 threshold sits at ~5×
(and the `1.5 × 99 ≈ 148.5` required floor a further ~3.4× below 500).

---

## Automated tests

`tests/test_camera_health.py` (27 tests, all `SimTransport`/synthetic — no real
camera):

- z-score metric is invariant to affine exposure yet sees real motion; the
  `normalize="none"` default is byte-identical to the original.
- `camera_health` primitives: noise floor, healthy pass, noisy fail +
  `require_*` raises, unavailable-is-reported-not-raised, visibility
  pass/fail/skip, `CameraUnusableError` is an `ObserverError`.
- **defect reproduction**: a `NoisyMotionObserver` whose no-motion noise exceeds
  its own threshold makes `calibrate()` raise `CameraUnusableError` **before any
  motion** (no `MOVE` commanded, no torque), writing **no** file — and it is
  catchable as `ObserverError`.
- **no bypass**: a direct `calibrate_joint()` call is gated too.
- **regression guard**: a blind `NullMotionObserver` is *still* refused, and
  still with the original `CalibrationAborted` "cannot reliably see this joint"
  diagnosis — **not** a `CameraUnusableError`. The new gate did not cannibalise
  the existing blind-camera protection.
- healthy `SimMotionObserver` still calibrates end-to-end (the gate does not newly
  refuse a healthy observer).
- CLI: `camera-check` PASS/FAIL/no-camera + `--json`; `doctor` reports
  `camera_health` and degrades gracefully with no camera.
- **CLI diagnostics (new):** `camera-check --roi X Y W H` threads the override
  through `_open_camera_observer` to the constructed `VisionGuard` (falling back to
  `config.VISION_ROI` when absent); `--before/--after` load two on-disk evidence
  images (files only — no camera, no HID) and report the measured motion
  separation; a missing/unreadable file exits cleanly (rc 2, no traceback); the
  `headroom` ratio appears in both human-readable and `--json` output; the
  `motion_separation` and `headroom` report fields are computed with an epsilon
  floor so a noise-free observer yields a finite ratio, not a division by zero.

Full suite: **370 passed** (`python3 -m pytest tests/ -q`), 0 failures.

---

## What this does NOT protect against (residual risk)

Stated plainly, in the spirit of `docs/calibration.md`'s "Failure modes":

1. **It is a point-in-time check, not continuous monitoring.** The gate runs once,
   before the search (and `SafetyLayer.move` caches its verdict for the whole
   session). Lighting that degrades *after* it passes — a cloud, a monitor waking,
   a lamp switched off mid-run — is not caught by the gate. Past that point only
   the per-step reliability burst and the torque/stall budgets bound the damage.
2. **The noise floor still has some window-dependence.** With capture staleness
   fixed the floor is far steadier run-to-run. What remains is genuine
   *auto-exposure hunting* in a dark scene, which is bursty on its own; a sample
   that lands in a stable stretch can still pass a camera that would fail over a
   longer window. The live `config.json` window is now 15 × **0.8 s ≈ 12 s** (the
   operator's validated spacing, which surfaces a higher, more honest floor than the
   0.4 s code default did); it is a heuristic, not a guarantee. `SafetyLayer.move`'s
   zero-interval sample is shorter still and catches only the *gross* fault.
3. **A healthy camera still has a real, non-zero, scene-dependent noise floor.**
   Flushing removes the *capture-staleness* inflation; it does not remove the
   camera's genuine sensor/lighting noise. In an empty static scene I measured p95
   ~4–27 changed px; the independent hardware session measured **230–335** changed
   px with a *person standing near the arm* (real environmental motion) against
   per-joint signals of 15k–73k px. That floor drifts with lighting, occupancy, and
   framing from one deployment to the next. **The gate's whole job is to catch when
   that floor drifts too close to the threshold in a given room** — this fix does
   not eliminate that risk, it only measures it *correctly* (the pre-fix path
   mis-measured it, wildly and unstably). Re-run `robohand camera-check` on site
   whenever lighting or camera placement changes.
4. **The visibility check is a spot-check, not a certificate.** It reuses whatever
   motion happens to occur at the start of each search direction. It does not
   prove every joint is visible across its whole travel, and it degrades to a
   no-op when no start move happens or the noise floor is unmeasurable.
5. **⚠️ Per-joint ROI tuning is USELESS for a joint the camera cannot physically
   frame.** This is the single most important limitation of *this* deployment. The
   ROI knob only chooses *where in the frame* to look — it cannot conjure a joint
   that is *not in the frame at all*. **On this camera mount, only SHOULDER_PAN
   (the rotating base) is visible.** `WRIST_ROLL`, `WRIST_FLEX`, `ELBOW`,
   `SHOULDER_LIFT`, and `GRIP` are physically out of shot, and **no ROI choice, and
   no software change, makes them visible** — only *physically re-aiming the camera*
   does, which is the human operator's job, not something achievable in code.
   Vision-guided calibration/verification is therefore meaningful *only* for
   SHOULDER_PAN on this mount; for the other five joints the visibility check exists
   precisely to make the software **refuse** rather than silently "calibrate" a
   joint it is blind to — it does not, and cannot, make them visible.
6. **It cannot distinguish "gripper closed on an object" from "gripper at its
   mechanical stop"** — no vision-only method can (see `docs/calibration.md`). A
   healthy camera does not make this decidable.
7. **It does not detect a wrong-but-consistent camera.** A camera aimed at a
   *moving* distractor (a fan, a screen, a reflection of the operator) that
   correlates with commanded moves can pass both noise-floor and visibility checks
   while measuring the wrong thing.
8. **Gravity sag after unload** still reads as real motion at the moment the
   routine asks "did it move?" (see `docs/calibration.md`). Unrelated to camera
   health; a healthy camera makes it *more* likely to be believed, not less.
9. **The opt-in `zscore` metric is not a free lunch.** It is invariant to affine
   (`gain·x+offset`) exposure changes — the first-order model — but not to
   non-affine effects: local auto-white-balance, vignetting shifts, rolling
   shutter, or motion blur. It also requires re-tuning `min_changed_pixels`; a
   deployment that flips `normalize` without re-running `camera-check` can make
   the gate meaningless. The operator's re-measurement also confirmed a **gradient**
   metric is *worse* than raw absdiff (it fails outright on the full frame) — so no
   gradient metric was implemented; raw absdiff + ROI is the shipped, sufficient
   choice.
10. **Simulator ≠ hardware.** `SimMotionObserver` is noise-free by construction, so
   the automated suite proves the *logic* (the gate fires on a synthetic noisy
   observer, passes a clean one, doesn't bypass), not that any particular real
   camera in any particular room will pass or fail. That can only be established
   with `robohand camera-check` on site.
