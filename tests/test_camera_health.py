"""Layer-3 pre-flight proof: the camera-health gate refuses a camera that cannot
tell motion from its own noise, without weakening the existing blind-camera net.

Everything here runs against ``sim.SimTransport``/synthetic frames -- no camera and
no HID device, exactly like the rest of the suite (``tests/conftest.py`` forces
``ROBOHAND_TRANSPORT=sim``). The real fault this guards against (auto-exposure
hunting on a dark scene producing a no-motion "noise" larger than a real move) is
reproduced synthetically with ``observer.NoisyMotionObserver`` and with affine
brightness transforms fed through ``guard.VisionGuard``'s z-score metric.
"""

from __future__ import annotations

import json

import cv2
import numpy as np
import pytest

from robohand import calibrate as calibration
from robohand import camera_health, config
from robohand.arm import Arm
from robohand.camera_health import CameraUnusableError
from robohand.guard import VisionGuard
from robohand.observer import (
    MotionObserver,
    NoisyMotionObserver,
    NullMotionObserver,
    ObserverError,
    SimMotionObserver,
)
from robohand.sim import SimTransport

GRIP_SERVO_ID = 1


# ---------------------------------------------------------------------------
# helpers / fixtures
# ---------------------------------------------------------------------------


def make_arm() -> Arm:
    return Arm(sim=True, use_vision=False, install_signal_handlers=False)


@pytest.fixture
def fast(monkeypatch):
    monkeypatch.setattr(config, "MIN_DURATION_MS", 20)
    monkeypatch.setattr(config, "SETTLE_MARGIN_MS", 0)
    return calibration.CalibrationSettings(step_duration_ms=20)


def _no_sleep(_seconds):
    return None


def _textured(seed: int = 0) -> np.ndarray:
    """A deterministic, textured BGR frame in a mid range (no uint8 saturation when
    an affine gain/offset is later applied)."""
    rng = np.random.default_rng(seed)
    gray = rng.integers(40, 120, size=(120, 160), dtype=np.uint8)
    return np.repeat(gray[:, :, None], 3, axis=2)


def _affine(frame: np.ndarray, gain: float, offset: float) -> np.ndarray:
    """Model a global exposure change: every pixel -> gain*pixel + offset."""
    return np.clip(frame.astype(np.float32) * gain + offset, 0, 255).astype(np.uint8)


def _with_bar(frame: np.ndarray) -> np.ndarray:
    """A real, local, structural change -- a bright bar, like a joint moving."""
    out = frame.copy()
    out[40:80, 60:100] = 255
    return out


class _ScoreStub(MotionObserver):
    """An observer whose continuous ``score`` is fixed, for exercising the gate math
    directly without a guard or frames."""

    def __init__(self, value: float, threshold: float = 1.0) -> None:
        self._value = float(value)
        self._threshold = float(threshold)

    def available(self) -> bool:
        return True

    def capture(self):
        return None

    def observe(self, before, after) -> bool:
        return self._value >= self._threshold

    def score(self, before, after) -> float:
        return self._value

    def motion_threshold(self) -> float:
        return self._threshold


class _ProbeStub(MotionObserver):
    """Returns a low score for captured (noise) pairs and a high score for the
    explicitly-supplied motion-probe pair, so the ``headroom`` and
    ``motion_separation`` report fields can be exercised without a camera."""

    def __init__(self, *, noise: float, motion: float, threshold: float) -> None:
        self._noise = float(noise)
        self._motion = float(motion)
        self._threshold = float(threshold)

    def available(self) -> bool:
        return True

    def capture(self):
        return "noise"

    def observe(self, before, after) -> bool:
        return self.score(before, after) >= self._threshold

    def score(self, before, after) -> float:
        return self._motion if "MOTION" in (before, after) else self._noise

    def motion_threshold(self) -> float:
        return self._threshold


def _move_entries(transport: SimTransport):
    return [e for e in transport.log if e["cmd"] == "MOVE"]


# ---------------------------------------------------------------------------
# 1. the improved (z-score) metric is invariant to exposure hunting
# ---------------------------------------------------------------------------


def test_zscore_metric_is_invariant_to_affine_exposure_but_sees_real_motion():
    base = _textured()
    exposure_only = _affine(base, gain=1.5, offset=20.0)  # pure global brightness change
    real_motion = _with_bar(base)

    raw = VisionGuard(min_changed_pixels=150)  # default "none" behaviour
    z = VisionGuard(normalize="zscore", normalized_threshold=0.5, min_changed_pixels=150)

    # Raw absdiff is completely fooled by a global exposure change...
    raw_exposure = raw.motion_score(base, exposure_only)
    assert raw_exposure > 150, "raw metric should (wrongly) call a pure exposure shift 'motion'"

    # ...but the z-score metric crushes it while still clearly seeing real motion.
    z_exposure = z.motion_score(base, exposure_only)
    z_motion = z.motion_score(base, real_motion)
    assert z_exposure < 150, f"z-score should ignore a pure exposure shift, got {z_exposure}"
    assert z_motion > 150, f"z-score should still see a real move, got {z_motion}"
    assert z_motion > z_exposure * 5, "real motion must separate clearly from exposure noise"


def test_default_metric_is_byte_identical_to_the_original():
    base = _textured(1)
    moved = _with_bar(base)
    default = VisionGuard(min_changed_pixels=150)
    explicit_none = VisionGuard(normalize="none", min_changed_pixels=150)
    assert default.motion_score(base, moved) == explicit_none.motion_score(base, moved)
    assert default.motion_score(base, base) == 0


# ---------------------------------------------------------------------------
# 2. camera_health primitives
# ---------------------------------------------------------------------------


def test_camera_unusable_error_is_an_observer_error():
    assert issubclass(CameraUnusableError, ObserverError)


def test_noise_floor_is_zero_for_a_noise_free_observer():
    obs = SimMotionObserver(SimTransport())
    scores, p50, p95, mx, pairs = camera_health.sample_noise_floor(
        obs, frames=8, interval_s=0.0, sleep=_no_sleep
    )
    assert pairs == 7
    assert p50 == p95 == mx == 0.0


def test_healthy_observer_passes_the_gate():
    obs = SimMotionObserver(SimTransport())
    report = camera_health.assess_camera_health(obs, frames=6, interval_s=0.0, sleep=_no_sleep)
    assert report.available and report.healthy
    assert report.noise_p95 == 0.0
    # returning the report (not raising) is the contract of require_camera_health
    same = camera_health.require_camera_health(obs, frames=6, interval_s=0.0, sleep=_no_sleep)
    assert same.healthy


def test_noisy_observer_fails_the_gate_and_require_raises():
    # noise floor well above its own threshold -- the exact deployment defect.
    obs = NoisyMotionObserver(noise_floor=300_000, threshold=150)
    report = camera_health.assess_camera_health(obs, frames=6, interval_s=0.0, sleep=_no_sleep)
    assert report.available and not report.healthy
    assert report.noise_p95 >= 300_000
    assert "noise floor" in report.reason
    with pytest.raises(CameraUnusableError):
        camera_health.require_camera_health(obs, frames=6, interval_s=0.0, sleep=_no_sleep)


def test_unavailable_observer_is_reported_not_raised():
    class _Down(_ScoreStub):
        def available(self) -> bool:
            return False

    report = camera_health.assess_camera_health(_Down(0), frames=4, interval_s=0.0, sleep=_no_sleep)
    assert report.skipped and not report.available
    # require_ returns the skipped report rather than raising -- "no camera" is a
    # different failure, handled by the caller's own availability check.
    assert camera_health.require_camera_health(
        _Down(0), frames=4, interval_s=0.0, sleep=_no_sleep
    ).skipped


def test_visibility_check_passes_fails_and_skips():
    # noise floor p95 == 50 via a fixed-score stub
    report = camera_health.assess_camera_health(
        _ScoreStub(50), frames=4, interval_s=0.0, sleep=_no_sleep
    )
    assert report.noise_p95 == 50.0

    # a clearly-visible move passes (score 1000 >= 50 * 3)
    assert camera_health.verify_motion_visibility(
        _ScoreStub(1000), None, None, report, separation=3.0
    ) == 1000.0

    # a move that barely clears the noise fails (score 100 < 50 * 3)
    with pytest.raises(CameraUnusableError):
        camera_health.verify_motion_visibility(_ScoreStub(100), None, None, report, separation=3.0)

    # with no measurable noise floor the check is a deliberate no-op (the blind /
    # zero-noise case is owned by the reliability burst, not this check)
    zero = camera_health.assess_camera_health(
        _ScoreStub(0), frames=4, interval_s=0.0, sleep=_no_sleep
    )
    assert zero.noise_p95 == 0.0
    assert camera_health.verify_motion_visibility(_ScoreStub(5), None, None, zero) is None


# ---------------------------------------------------------------------------
# 3. calibrate() integration -- the defect reproduction
# ---------------------------------------------------------------------------


def test_noisy_camera_aborts_calibration_before_any_motion(tmp_path, fast):
    """The defect, as an assertion: an observer whose no-motion noise swamps its own
    threshold must abort calibration BEFORE any joint is touched, writing no file."""
    with make_arm() as arm:
        obs = NoisyMotionObserver(noise_floor=300_000, threshold=150)
        with pytest.raises(CameraUnusableError):
            calibration.calibrate(
                arm, ["ELBOW"], settings=fast, observer=obs, output_dir=tmp_path
            )
        # gate ran before any torque burst -- nothing moved, nothing is loaded.
        assert _move_entries(arm._transport) == []
        assert arm._transport.any_torque_on() is False
    assert list(tmp_path.glob("calibration-*.json")) == []


def test_noisy_camera_abort_is_catchable_as_observer_error(tmp_path, fast):
    """CameraUnusableError must propagate as an ObserverError subclass so every
    existing ``except ObserverError`` site stays fail-closed."""
    with make_arm() as arm:
        obs = NoisyMotionObserver(noise_floor=300_000, threshold=150)
        with pytest.raises(ObserverError):
            calibration.calibrate(arm, ["ELBOW"], settings=fast, observer=obs, output_dir=tmp_path)


def test_direct_calibrate_joint_call_is_also_gated(tmp_path, fast):
    """The bypass path: calling calibrate_joint directly (no calibrate() wrapper) must
    still run the health gate itself -- there is no way to reach the search loop
    without it."""
    with make_arm() as arm:
        obs = NoisyMotionObserver(noise_floor=300_000, threshold=150)
        with pytest.raises(CameraUnusableError):
            calibration.calibrate_joint(arm, "ELBOW", settings=fast, observer=obs)
        assert _move_entries(arm._transport) == []
        assert arm._transport.any_torque_on() is False


def test_healthy_sim_observer_still_calibrates_end_to_end(tmp_path, fast):
    """The gate must not newly refuse a healthy observer."""
    with make_arm() as arm:
        run = calibration.calibrate(
            arm, ["WRIST_ROLL"], settings=fast,
            observer=SimMotionObserver(arm._transport), output_dir=tmp_path,
        )
    assert "WRIST_ROLL" in run.joints
    assert len(list(tmp_path.glob("calibration-*.json"))) == 1


def test_blind_observer_still_aborts_with_the_original_diagnosis(tmp_path, fast):
    """Regression guard: the noise-floor gate must NOT cannibalise the existing
    blind-camera protection. A NullMotionObserver (always 'no motion', zero noise)
    passes the noise-floor gate trivially and must still be caught later by the
    observer-reliability burst -- as a CalibrationAborted, NOT a CameraUnusableError."""
    with make_arm() as arm:
        with pytest.raises(calibration.CalibrationAborted) as excinfo:
            calibration.calibrate(
                arm, ["ELBOW"], settings=fast,
                observer=NullMotionObserver(), output_dir=tmp_path,
            )
        assert not isinstance(excinfo.value, CameraUnusableError)
        assert "cannot reliably see this joint" in str(excinfo.value)
    assert list(tmp_path.glob("calibration-*.json")) == []


# ---------------------------------------------------------------------------
# 4. CLI: camera-check and doctor
# ---------------------------------------------------------------------------


class _FakeCam:
    def __init__(self):
        self.released = False

    def release(self):
        self.released = True


def _patch_camera(monkeypatch, observer):
    from robohand import cli

    cam = _FakeCam()

    def _open(roi=None):
        return (cam, observer) if observer is not None else (None, None)

    monkeypatch.setattr(cli, "_open_camera_observer", _open)
    return cam


def test_cli_camera_check_pass(monkeypatch, capsys):
    from robohand import cli

    cam = _patch_camera(monkeypatch, SimMotionObserver(SimTransport()))
    rc = cli.main(["camera-check", "--frames", "4", "--interval", "0", "--json"])
    assert rc == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["available"] and payload["healthy"]
    assert payload["noise_p95"] == 0.0
    assert cam.released is True


def test_cli_camera_check_fail_on_noisy_camera(monkeypatch, capsys):
    from robohand import cli

    _patch_camera(monkeypatch, NoisyMotionObserver(noise_floor=300_000, threshold=150))
    rc = cli.main(["camera-check", "--frames", "4", "--interval", "0"])
    assert rc == 1
    out = capsys.readouterr().out
    assert "FAIL" in out


def test_cli_camera_check_fail_when_no_camera(monkeypatch, capsys):
    from robohand import cli

    _patch_camera(monkeypatch, None)
    rc = cli.main(["camera-check", "--json"])
    assert rc == 1
    payload = json.loads(capsys.readouterr().out)
    assert payload["available"] is False


def test_cli_doctor_reports_camera_health(monkeypatch, capsys):
    from robohand import cli

    _patch_camera(monkeypatch, SimMotionObserver(SimTransport()))
    rc = cli.main(["--sim", "doctor", "--json"])
    assert rc == 0
    payload = json.loads(capsys.readouterr().out)
    assert "camera_health" in payload
    assert payload["camera_health"]["healthy"] is True
    assert payload["camera"]["available"] is True


def test_cli_doctor_degrades_gracefully_without_a_camera(monkeypatch, capsys):
    from robohand import cli

    _patch_camera(monkeypatch, None)
    rc = cli.main(["--sim", "doctor", "--json"])
    assert rc == 0  # doctor must never fail just because there is no camera
    payload = json.loads(capsys.readouterr().out)
    assert payload["camera"]["available"] is False
    assert payload["camera_health"]["available"] is False
    assert payload["safety_layers"]["layer3_vision_guard"] is False


# ---------------------------------------------------------------------------
# 5. headroom / measured-separation ratios (report fields)
# ---------------------------------------------------------------------------


def test_report_exposes_headroom_ratio_without_a_motion_sample():
    """`headroom` = threshold / max(p95, epsilon) is always computed for an available
    camera and needs no motion sample; measured-separation fields stay None."""
    report = camera_health.assess_camera_health(
        _ScoreStub(50, threshold=200), frames=4, interval_s=0.0, sleep=_no_sleep
    )
    assert report.noise_p95 == 50.0
    assert report.headroom == pytest.approx(200 / 50)  # 4.0x
    assert report.motion_score is None
    assert report.motion_separation is None


def test_headroom_is_finite_for_a_noise_free_observer():
    """A noise-free observer (p95 == 0) must yield a large-but-finite headroom (the
    epsilon floor), never a division by zero."""
    report = camera_health.assess_camera_health(
        SimMotionObserver(SimTransport()), frames=4, interval_s=0.0, sleep=_no_sleep
    )
    assert report.noise_p95 == 0.0
    assert report.headroom is not None
    assert report.headroom == pytest.approx(report.threshold)  # threshold / 1.0


def test_report_exposes_measured_motion_separation_with_a_probe():
    """With a motion probe, `motion_separation` = motion_score / max(p95, epsilon) is
    reported alongside `headroom`, and the two are distinct quantities."""
    obs = _ProbeStub(noise=40, motion=800, threshold=200)
    report = camera_health.assess_camera_health(
        obs, frames=5, interval_s=0.0, sleep=_no_sleep,
        motion_probe=lambda: ("MOTION", "MOTION"),
    )
    assert report.noise_p95 == 40.0
    assert report.headroom == pytest.approx(200 / 40)          # 5.0x
    assert report.motion_score == 800.0
    assert report.motion_separation == pytest.approx(800 / 40)  # 20.0x
    assert report.separation_ok is True


# ---------------------------------------------------------------------------
# 6. CLI: --roi override and --before/--after motion probe
# ---------------------------------------------------------------------------


def test_open_camera_observer_roi_override_reaches_the_guard(monkeypatch):
    """`camera-check --roi` must thread through `_open_camera_observer` to the
    constructed VisionGuard; without it the guard falls back to config.VISION_ROI.
    Exercises the real guard-building path with a fake camera (no real hardware)."""
    from robohand import cli, camera as camera_module

    class _FakeVideoCam:
        def open(self):
            return self

        def grab(self):
            return np.zeros((720, 1280, 3), dtype=np.uint8)

        def release(self):
            pass

    monkeypatch.setattr(camera_module, "Camera", lambda *a, **k: _FakeVideoCam())
    monkeypatch.setattr(camera_module, "CameraConfig", lambda *a, **k: None)

    _cam, obs = cli._open_camera_observer(roi=(10, 20, 30, 40))
    assert tuple(obs.guard.roi) == (10, 20, 30, 40)

    _cam2, obs2 = cli._open_camera_observer()
    assert tuple(obs2.guard.roi) == tuple(config.VISION_ROI)
    assert tuple(obs2.guard.roi) != (10, 20, 30, 40)


def test_cli_camera_check_roi_is_forwarded(monkeypatch, capsys):
    """The `--roi X Y W H` flag is parsed and forwarded to the report builder."""
    from robohand import cli

    captured = {}

    def _open(roi=None):
        captured["roi"] = roi
        return _FakeCam(), SimMotionObserver(SimTransport())

    monkeypatch.setattr(cli, "_open_camera_observer", _open)
    rc = cli.main(["camera-check", "--roi", "0", "300", "520", "420",
                   "--frames", "4", "--interval", "0", "--json"])
    assert rc == 0
    assert captured["roi"] == (0.0, 300.0, 520.0, 420.0)


def test_cli_camera_check_headroom_in_human_and_json_output(monkeypatch, capsys):
    """The headroom ratio is present in both human-readable and --json output."""
    from robohand import cli

    monkeypatch.setattr(
        cli, "_open_camera_observer",
        lambda roi=None: (_FakeCam(), SimMotionObserver(SimTransport())),
    )
    rc = cli.main(["camera-check", "--frames", "4", "--interval", "0"])
    assert rc == 0
    assert "headroom" in capsys.readouterr().out.lower()

    rc = cli.main(["camera-check", "--frames", "4", "--interval", "0", "--json"])
    assert rc == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["headroom"] is not None


def test_cli_camera_check_motion_probe_reports_separation(monkeypatch, capsys, tmp_path):
    """`--before/--after` load two on-disk images (never touching a camera or HID),
    score them through the guard, and report the measured separation from the live
    noise floor."""
    from robohand import cli
    from robohand.guard import VisionGuard
    from robohand.observer import VisionMotionObserver

    const = np.full((720, 1280, 3), 100, dtype=np.uint8)
    guard = VisionGuard(
        frame_source=lambda: const.copy(), roi=(0, 300, 520, 420),
        diff_threshold=25, min_changed_pixels=500, blur_ksize=5,
    )
    obs = VisionMotionObserver(guard)
    monkeypatch.setattr(cli, "_open_camera_observer", lambda roi=None: (_FakeCam(), obs))

    before = np.zeros((720, 1280, 3), dtype=np.uint8)
    after = before.copy()
    after[350:520, 50:400] = 255  # a bright block inside the ROI = "a real move"
    bpath = tmp_path / "before.png"
    apath = tmp_path / "after.png"
    assert cv2.imwrite(str(bpath), before)
    assert cv2.imwrite(str(apath), after)

    rc = cli.main(["camera-check", "--before", str(bpath), "--after", str(apath),
                   "--frames", "4", "--interval", "0", "--json"])
    assert rc == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["motion_score"] > payload["noise_p95"]
    assert payload["motion_separation"] is not None
    # noise-free const source => p95 == 0 => motion_separation == motion_score / 1.0
    assert payload["motion_separation"] == pytest.approx(payload["motion_score"])
    assert payload["separation_ok"] is True
    assert payload["headroom"] is not None


def test_cli_camera_check_missing_motion_file_errors_cleanly(capsys, tmp_path):
    """An unreadable evidence image exits non-zero with a clear message and no
    traceback -- and never opens the camera (the load happens first)."""
    from robohand import cli

    rc = cli.main(["camera-check", "--before", str(tmp_path / "nope.png"),
                   "--after", str(tmp_path / "nada.png")])
    assert rc == 2
    combined = capsys.readouterr()
    text = (combined.out + combined.err).lower()
    assert "could not be read" in text
    assert "traceback" not in text


def test_cli_camera_check_before_after_must_be_paired(capsys, tmp_path):
    """Supplying only one of --before/--after is a clean usage error (exit 2)."""
    from robohand import cli

    rc = cli.main(["camera-check", "--before", str(tmp_path / "only.png")])
    assert rc == 2
    combined = capsys.readouterr()
    assert "must be given together" in (combined.out + combined.err).lower()


# ---------------------------------------------------------------------------
# 7. Full-arm-in-frame regression guard (today's real-hardware numbers)
# ---------------------------------------------------------------------------


def test_shipped_config_is_the_full_frame_deployment():
    """Guard the *shipped* config.json values for the new full-arm-in-frame setup:
    full-frame ROI, min_changed_pixels=3000, margin 1.5, separation 3.0."""
    assert tuple(config.VISION_ROI) == (0, 0, 1.0, 1.0)  # fractional full frame
    assert config.VISION_MIN_CHANGED_PIXELS == 3000
    assert config.VISION_HEALTH_MARGIN == 1.5
    assert config.VISION_HEALTH_SEPARATION == 3.0
    assert config.VISION_DIFF_THRESHOLD == 25


def test_full_frame_shipped_numbers_pass_and_dark_regression_fails():
    """Encode today's real full-arm-in-frame hardware numbers as a regression guard.

    Noise p95 = 577 changed px (worst no-motion control), GRIP motion = 25,746 changed
    px (the WORST-case real signal -- smallest visual signature, the joint damaged in
    the original incident). With the shipped threshold=3000 / margin=1.5 /
    separation=3.0 the health gate PASSES and the GRIP move clears the visibility
    check by ~45x. A dark-scene/exposure-hunting regression (noise floor above
    3000/1.5 = 2000) correctly FAILS.
    """
    NOISE = 577.0
    GRIP_MOTION = 25_746.0
    THRESHOLD = float(config.VISION_MIN_CHANGED_PIXELS)  # 3000, shipped
    MARGIN = config.VISION_HEALTH_MARGIN                 # 1.5, shipped
    SEPARATION = config.VISION_HEALTH_SEPARATION         # 3.0, shipped

    obs = _ProbeStub(noise=NOISE, motion=GRIP_MOTION, threshold=THRESHOLD)
    report = camera_health.assess_camera_health(
        obs, frames=6, interval_s=0.0, margin=MARGIN, separation=SEPARATION,
        motion_probe=lambda: ("MOTION", "MOTION"), sleep=_no_sleep,
    )

    # --- PASS case: default config clears the gate comfortably in the good setup ---
    assert report.available and report.healthy
    assert report.noise_p95 == pytest.approx(NOISE)
    # required = 1.5 * 577 = 865.5, comfortably below the 3000 threshold
    assert report.required_threshold == pytest.approx(MARGIN * NOISE)  # 865.5
    assert report.required_threshold < THRESHOLD
    assert THRESHOLD / report.required_threshold == pytest.approx(3.466, rel=1e-3)  # ~3.5x buffer
    # visibility: GRIP (25,746) clears the floor max(3000, 3*577=1731)=3000 by ~45x over raw noise
    assert report.separation_ok is True
    assert report.separation_floor == pytest.approx(max(THRESHOLD, SEPARATION * NOISE))  # 3000
    assert report.motion_score == pytest.approx(GRIP_MOTION)
    assert report.motion_separation == pytest.approx(GRIP_MOTION / NOISE, rel=1e-3)  # ~44.6x (~45x)
    assert 44.0 < report.motion_separation < 46.0

    # --- FAIL case: a dark-scene/exposure-hunting regression is rejected loudly ---
    dark = _ProbeStub(noise=200_000.0, motion=GRIP_MOTION, threshold=THRESHOLD)
    dark_report = camera_health.assess_camera_health(
        dark, frames=6, interval_s=0.0, margin=MARGIN, separation=SEPARATION, sleep=_no_sleep
    )
    assert dark_report.available and not dark_report.healthy
    with pytest.raises(CameraUnusableError):
        camera_health.require_camera_health(
            dark, frames=6, interval_s=0.0, margin=MARGIN, separation=SEPARATION, sleep=_no_sleep
        )

    # --- boundary: the FAIL threshold is exactly min_changed_pixels / margin = 2000 ---
    boundary = THRESHOLD / MARGIN  # 2000.0
    just_below = _ProbeStub(noise=boundary - 1.0, motion=GRIP_MOTION, threshold=THRESHOLD)
    just_above = _ProbeStub(noise=boundary + 1.0, motion=GRIP_MOTION, threshold=THRESHOLD)
    assert camera_health.assess_camera_health(
        just_below, frames=6, interval_s=0.0, margin=MARGIN, sleep=_no_sleep
    ).healthy is True
    assert camera_health.assess_camera_health(
        just_above, frames=6, interval_s=0.0, margin=MARGIN, sleep=_no_sleep
    ).healthy is False
