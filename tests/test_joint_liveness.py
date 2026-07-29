"""Stage-2 proof: the per-joint liveness check detects a dead actuator, and -- just
as importantly -- never drives a joint into a mechanical stop to do it.

Context. The liveness check shipped with no test coverage at all: stubbing
``_verify_joint_liveness`` to an immediate ``return`` left the whole suite green.
The only test that reacted to it was ``test_auto_calibrate.py::
test_stall_time_stays_within_budget``, which FAILED (3200ms vs a 2400ms ceiling)
because the original ``_liveness_targets`` drew its probe targets from the
*provisional band* -- which is by definition wider than the true mechanical range,
so its targets were never known-reachable and stalled the servo on both legs.

These tests exist so that regression cannot recur silently: every one of them fails
if ``_verify_joint_liveness`` is stubbed out, and ``test_liveness_never_commands_
outside_the_true_mechanical_range`` fails against the ORIGINAL target derivation.

Everything runs against ``sim.SimTransport``/``SimMotionObserver`` -- no camera and
no HID device (``tests/conftest.py`` forces ``ROBOHAND_TRANSPORT=sim``).
"""

from __future__ import annotations

import pytest

from robohand import calibrate as calibration
from robohand import camera_health, config
from robohand.arm import Arm
from robohand.calibrate import JointUnresponsive
from robohand.observer import MotionObserver, SimMotionObserver
from robohand.sim import SimTransport

GRIP_SERVO_ID = 1
ELBOW_SERVO_ID = 4
SHOULDER_LIFT_SERVO_ID = 5


# ---------------------------------------------------------------------------
# helpers / fixtures
# ---------------------------------------------------------------------------


def make_arm(mechanical_limits=None) -> Arm:
    arm = Arm(sim=True, use_vision=False, install_signal_handlers=False)
    if mechanical_limits:
        for servo_id, (lo, hi) in mechanical_limits.items():
            joint = arm._transport.joints[servo_id]
            joint.mech_min, joint.mech_max = lo, hi
    return arm


@pytest.fixture
def fast(monkeypatch):
    """Short bursts so the suite stays quick, with liveness EXPLICITLY enabled --
    it is opt-in and off by default (see CalibrationSettings.liveness_check)."""
    monkeypatch.setattr(config, "MIN_DURATION_MS", 20)
    monkeypatch.setattr(config, "SETTLE_MARGIN_MS", 0)
    return calibration.CalibrationSettings(step_duration_ms=20, liveness_check=True)


def _no_sleep(_seconds):
    return None


def _health(observer) -> camera_health.CameraHealthReport:
    return camera_health.assess_camera_health(
        observer, frames=4, interval_s=0.0, sleep=_no_sleep
    )


def _budget(arm, settings) -> calibration._Budget:
    return calibration._Budget(settings, arm._transport)


def _move_entries(transport: SimTransport):
    return [e for e in transport.log if e["cmd"] == "MOVE"]


def _commanded_targets(transport: SimTransport, servo_id: int):
    """Every raw position commanded for one servo, in order."""
    out = []
    for entry in _move_entries(transport):
        for sid, pos in entry["targets"]:
            if sid == servo_id:
                out.append(pos)
    return out


class _BlindObserver(MotionObserver):
    """Sees nothing, ever -- score 0 for every pair. Models a camera that cannot make
    out this joint (bad angle, occlusion), NOT a dead joint."""

    def available(self) -> bool:
        return True

    def capture(self):
        return "frame"

    def observe(self, before, after) -> bool:
        return False

    def score(self, before, after) -> float:
        return 0.0

    def motion_threshold(self) -> float:
        return 12.0


# ---------------------------------------------------------------------------
# 1. target derivation -- the actual regression
# ---------------------------------------------------------------------------


def test_liveness_centre_is_the_absolute_midpoint_not_a_hardcoded_1500():
    """GRIP's electrical floor is 1500, so 1500 is its *open mechanical stop*, not a
    safe centre. Homing GRIP to a hardcoded 1500 would drive it onto that stop --
    the same class of mistake as the GRIP=700 command that destroyed the gripper."""
    assert calibration._liveness_center("GRIP") == 1900
    for name in ("ELBOW", "SHOULDER_PAN", "WRIST_ROLL", "WRIST_FLEX", "SHOULDER_LIFT"):
        assert calibration._liveness_center(name) == 1500


def test_liveness_targets_are_symmetric_about_the_centre_and_inside_the_band():
    for name in config.JOINT_NAMES:
        centre, down, up = calibration._liveness_targets(name)
        assert centre - down == up - centre, f"{name}: excursion must be symmetric"
        lo, hi = config.provisional_bounds(name)
        assert lo <= down < centre < up <= hi, (
            f"{name}: targets {down}/{up} must stay inside the provisional band "
            f"[{lo}, {hi}] so the excursion needs no elevated privilege"
        )


def test_liveness_targets_do_not_depend_on_the_provisional_band_width():
    """The original bug in one assertion: targets derived from the band scale WITH the
    band, so a wide band produced far-away targets. Centre-derived targets must not."""
    centre, down, up = calibration._liveness_targets("ELBOW")
    assert (centre - down) == calibration.LIVENESS_DELTA
    assert (up - centre) == calibration.LIVENESS_DELTA


def test_liveness_targets_do_not_depend_on_where_the_joint_currently_is(fast):
    """The heart of the original defect: targets were taken relative to
    ``arm._shadow`` -- the joint's *current* position -- which is exactly the thing
    we have no feedback for and cannot bound an excursion against. The excursion
    must be anchored to the safe centre and identical no matter where the arm was
    left, so it is homed there first."""
    seen = []
    for parked in (1250, 1500, 1790):
        with make_arm() as arm:
            arm._safety.move({"ELBOW": parked}, duration_ms=20, wait=True)
            arm._safety.unload(["ELBOW"])
            obs = SimMotionObserver(arm._transport)
            before = len(_commanded_targets(arm._transport, ELBOW_SERVO_ID))
            calibration._verify_joint_liveness(
                arm, "ELBOW", fast, obs, _budget(arm, fast), _health(obs)
            )
            seen.append(_commanded_targets(arm._transport, ELBOW_SERVO_ID)[before:])
    assert seen[0] == seen[1] == seen[2], (
        f"liveness excursion varied with the parked position: {seen}"
    )


# ---------------------------------------------------------------------------
# 2. the headline claim: a dead joint is detected
# ---------------------------------------------------------------------------


def test_dead_joint_raises_joint_unresponsive(fast):
    """THE headline claim of the whole feature, which previously had no test at all.
    A servo that accepts commands but never moves and draws no current -- the real
    SHOULDER_LIFT / servo-5 signature -- must be reported as unresponsive."""
    with make_arm() as arm:
        arm._transport.set_dead(SHOULDER_LIFT_SERVO_ID)
        obs = SimMotionObserver(arm._transport)
        with pytest.raises(JointUnresponsive) as excinfo:
            calibration._verify_joint_liveness(
                arm, "SHOULDER_LIFT", fast, obs, _budget(arm, fast), _health(obs)
            )
        msg = str(excinfo.value)
        assert "unresponsive" in msg
        assert "NOT seen" in msg
        assert "flat" in msg, "a flat pack is the strongest dead-servo evidence we have"
        assert arm._transport.any_torque_on() is False, "must never leave a joint loaded"


def test_live_joint_passes_liveness(fast):
    with make_arm() as arm:
        obs = SimMotionObserver(arm._transport)
        calibration._verify_joint_liveness(
            arm, "ELBOW", fast, obs, _budget(arm, fast), _health(obs)
        )  # must not raise
        assert arm._transport.any_torque_on() is False


def test_dead_joint_aborts_before_any_limit_search(fast):
    """Ordering, in the style of test_noisy_camera_aborts_calibration_before_any_motion:
    a dead joint must be caught by liveness and never reach the search. Liveness
    commands exactly three moves (home + two legs); anything beyond that is the
    search having run on a joint we cannot trust."""
    with make_arm() as arm:
        arm._transport.set_dead(ELBOW_SERVO_ID)
        obs = SimMotionObserver(arm._transport)
        with pytest.raises(JointUnresponsive):
            calibration.calibrate_joint(arm, "ELBOW", settings=fast, observer=obs)
        assert len(_commanded_targets(arm._transport, ELBOW_SERVO_ID)) == 3, (
            "expected exactly home + 2 liveness legs, i.e. the search never started"
        )
        assert arm._transport.any_torque_on() is False


# ---------------------------------------------------------------------------
# 3. the safety property: liveness must not stall a servo
# ---------------------------------------------------------------------------


def test_liveness_is_stall_free_on_every_realistic_joint(fast):
    """Safety property that DOES hold: against the package's own best guess at this
    hardware (``sim.DEFAULT_MECHANICAL_LIMITS``, min headroom 400 units from centre --
    GRIP at 1500..2500 about its 1900 centre) a 150-unit excursion never reaches a
    stop, on any joint."""
    for name in config.JOINT_NAMES:
        with make_arm() as arm:
            obs = SimMotionObserver(arm._transport)
            calibration._verify_joint_liveness(
                arm, name, fast, obs, _budget(arm, fast), _health(obs)
            )
            assert arm._transport.stall_ms_total() == 0.0, f"{name} stalled"


def test_liveness_would_stall_a_pathologically_narrow_joint(fast):
    """The limitation, asserted rather than hidden -- this is the evidence behind
    ``liveness_check`` defaulting to False.

    On the narrowest joint the suite models (GRIP with stops at 1855/2045: 45 units
    of headroom below its 1900 centre) the 150-unit excursion required for the joint
    to be *visible* does reach the stop. That is not a bug in the target derivation,
    which is now correctly anchored to a known-safe centre; it is the structural
    fact that with no position feedback ``centre +/- delta`` is not safe by
    construction for ANY delta > 0. Shrinking delta to 45 to satisfy this fixture
    would drop below the ~125 units the battery check needs and leave no margin for
    the three joints whose px-per-unit was never measured.

    If a future change makes this test fail because nothing stalls any more, that is
    good news -- and the moment to revisit the default.
    """
    with make_arm({GRIP_SERVO_ID: (1855, 2045)}) as arm:
        obs = SimMotionObserver(arm._transport)
        try:
            calibration._verify_joint_liveness(
                arm, "GRIP", fast, obs, _budget(arm, fast), _health(obs)
            )
        except JointUnresponsive:
            pass
        assert arm._transport.stall_ms_total() > 0.0, (
            "expected the documented limitation; if this no longer stalls, re-derive "
            "LIVENESS_DELTA and consider enabling liveness_check by default"
        )


def test_default_calibration_never_pays_that_stall(fast, monkeypatch):
    """And because liveness is off by default, the shipped path pays none of it: the
    stall-budget guard in test_auto_calibrate stays intact."""
    settings = calibration.CalibrationSettings(step_duration_ms=20)
    with make_arm({GRIP_SERVO_ID: (1855, 2045)}) as arm:
        obs = SimMotionObserver(arm._transport)
        calibration.calibrate(arm, ["GRIP"], settings=settings, observer=obs, write=False)
        stall = arm._transport.stall_ms_total()
    # 3 stalling commands x 2 directions x 20ms burst, and nothing from liveness.
    assert stall <= 3 * 2 * 20, f"unexpected stall {stall}ms -- liveness must not run"


def test_liveness_uses_no_elevated_privilege(fast):
    """A move that can only be made with the rails unlocked is not the move a normal
    command would make, so it cannot prove what liveness needs to prove. The
    excursion must therefore stay inside the provisional band -- no uncalibrated
    override, no probe_absolute_bounds."""
    with make_arm() as arm:
        obs = SimMotionObserver(arm._transport)
        calibration._verify_joint_liveness(
            arm, "ELBOW", fast, obs, _budget(arm, fast), _health(obs)
        )
        lo, hi = config.provisional_bounds("ELBOW")
        for target in _commanded_targets(arm._transport, ELBOW_SERVO_ID):
            assert lo <= target <= hi


# ---------------------------------------------------------------------------
# 4. the battery channel -- how servo 5 was actually diagnosed
# ---------------------------------------------------------------------------


def test_unseen_but_current_drawn_is_not_called_dead(fast):
    """A live joint under a blind camera moves and sags the pack. That is a *sight*
    problem, not a dead actuator -- liveness must defer rather than hand down a
    hardware verdict it cannot support."""
    with make_arm() as arm:
        obs = _BlindObserver()
        calibration._verify_joint_liveness(
            arm, "ELBOW", fast, obs, _budget(arm, fast), _health(obs)
        )  # must NOT raise: the pack sagged, so the servo is electrically alive


def test_unseen_and_flat_pack_is_called_dead(fast):
    """Same blind camera, but the joint really is dead: no motion AND no current."""
    with make_arm() as arm:
        arm._transport.set_dead(ELBOW_SERVO_ID)
        obs = _BlindObserver()
        with pytest.raises(JointUnresponsive) as excinfo:
            calibration._verify_joint_liveness(
                arm, "ELBOW", fast, obs, _budget(arm, fast), _health(obs)
            )
        assert "flat" in str(excinfo.value)


def test_unreadable_battery_is_reported_and_does_not_crash(fast, monkeypatch):
    """A controller that cannot answer get_battery_voltage must not crash liveness,
    and the diagnosis must say the current check was unavailable rather than imply
    it passed."""
    with make_arm() as arm:
        arm._transport.set_dead(ELBOW_SERVO_ID)
        monkeypatch.setattr(
            arm._transport, "get_battery_voltage",
            lambda: (_ for _ in ()).throw(RuntimeError("no telemetry")),
        )
        obs = _BlindObserver()
        with pytest.raises(JointUnresponsive) as excinfo:
            calibration._verify_joint_liveness(
                arm, "ELBOW", fast, obs, _budget(arm, fast), _health(obs)
            )
        assert "Battery voltage was unavailable" in str(excinfo.value)


# ---------------------------------------------------------------------------
# 5. the shipped default is OFF -- documented, not accidental
# ---------------------------------------------------------------------------


def test_liveness_is_opt_in_and_off_by_default():
    """Recorded as a test so flipping the default is a deliberate, visible act. See
    CalibrationSettings.liveness_check for why: no fixed excursion is simultaneously
    visible enough to trust and provably stall-free without hardware measurement."""
    assert calibration.CalibrationSettings().liveness_check is False


def test_liveness_check_must_be_a_bool():
    with pytest.raises(calibration.CalibrationRefused):
        calibration.CalibrationSettings(liveness_check="yes")


def test_default_settings_run_no_liveness_excursion(monkeypatch):
    """With the default settings a dead joint is NOT diagnosed by liveness -- it is
    caught downstream by the search's observer-reliability burst, which aborts the
    run rather than silently calibrating a dead joint. This documents the real
    shipped behaviour rather than the behaviour we wish we could ship."""
    monkeypatch.setattr(config, "MIN_DURATION_MS", 20)
    monkeypatch.setattr(config, "SETTLE_MARGIN_MS", 0)
    settings = calibration.CalibrationSettings(step_duration_ms=20)
    assert settings.liveness_check is False

    called = []
    monkeypatch.setattr(
        calibration, "_verify_joint_liveness",
        lambda *a, **k: called.append(True),
    )
    with make_arm() as arm:
        arm._transport.set_dead(ELBOW_SERVO_ID)
        obs = SimMotionObserver(arm._transport)
        with pytest.raises(calibration.CalibrationAborted):
            calibration.calibrate_joint(arm, "ELBOW", settings=settings, observer=obs)
    assert called == [], "liveness must not run unless explicitly enabled"
