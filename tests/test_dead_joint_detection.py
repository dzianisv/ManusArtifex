"""Proof that a failed-open actuator is diagnosed as such, at zero extra motion.

The problem this solves. A dead servo (the real SHOULDER_LIFT / servo-5 fault) and a
camera that cannot see a joint emit the *identical* signal: "commanded motion, saw
nothing". The search's observer-reliability burst used to attribute that signal
unconditionally to the camera, so a disconnected servo produced

    ELBOW: ... It cannot reliably see this joint -- occlusion, a bad ROI, a dead
    camera, lighting, ...

which sends an operator to inspect the wrong component entirely. The camera is fine.

Why the check lives where it does. A failed-open joint can never *complete* the
limit search -- it aborts inside the reliability burst -- so there is no post-search
band to inspect. The burst is also the one place in the run that has known-real
motion to reason about (it oscillates between two positions the joint has already
been watched to reach). Reading pack voltage and stall time either side of moves the
search is *already making* therefore costs no extra motion, which is the entire
point: no excursion, so no excursion size to justify, so no stall risk.

Everything runs against ``sim.SimTransport`` -- no camera, no HID device
(``tests/conftest.py`` forces ``ROBOHAND_TRANSPORT=sim``).
"""

from __future__ import annotations

import pytest

from robohand import calibrate as calibration
from robohand import config
from robohand.arm import Arm
from robohand.calibrate import JointUnresponsive
from robohand.observer import MotionObserver, SimMotionObserver
from robohand.sim import SimTransport

ELBOW_SERVO_ID = 4
NOISE_FLOOR = calibration._BATTERY_NOISE_FLOOR_V
#: A burst comfortably large enough for the battery signal to be trusted.
TRUSTED_TRAVEL = 240


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


class _BlindObserver(MotionObserver):
    """A camera that cannot make out this joint (bad angle, occlusion, lens cap).
    The joint itself is perfectly alive."""

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


def _move_count(transport: SimTransport) -> int:
    return len([e for e in transport.log if e["cmd"] == "MOVE"])


# ---------------------------------------------------------------------------
# 1. the headline claim
# ---------------------------------------------------------------------------


def test_failed_open_joint_is_diagnosed_as_unresponsive(fast):
    """A servo that accepts commands, never moves, draws no current and never stalls
    must be named as failed-open -- not blamed on the camera."""
    with make_arm() as arm:
        arm._transport.set_dead(ELBOW_SERVO_ID)
        obs = SimMotionObserver(arm._transport)
        with pytest.raises(JointUnresponsive) as excinfo:
            calibration.calibrate_joint(arm, "ELBOW", settings=fast, observer=obs)
        msg = str(excinfo.value)
        assert "failed-open" in msg
        assert "stayed flat" in msg, "the flat pack is the load-bearing evidence"
        assert "servo-5" in msg, "should point at the known fault and its runbook"


def test_healthy_joint_calibrates_normally(fast):
    """The check must not newly refuse a good arm."""
    with make_arm() as arm:
        obs = SimMotionObserver(arm._transport)
        result = calibration.calibrate_joint(arm, "ELBOW", settings=fast, observer=obs)
        assert result.max > result.min


def test_dead_joint_is_isolated_and_the_rest_of_the_run_survives(fast, tmp_path):
    """A dead actuator invalidates one joint, not the camera and not the other
    joints, so the run records it and continues rather than aborting."""
    with make_arm() as arm:
        arm._transport.set_dead(ELBOW_SERVO_ID)
        obs = SimMotionObserver(arm._transport)
        run = calibration.calibrate(
            arm, ["ELBOW", "WRIST_ROLL"], settings=fast, observer=obs, output_dir=tmp_path
        )
    assert "ELBOW" in run.failed, "the dead joint must be recorded"
    assert "ELBOW" not in run.joints, "and must NOT be written as if it were measured"
    assert "WRIST_ROLL" in run.joints, "the healthy joint must still be calibrated"


# ---------------------------------------------------------------------------
# 2. the two faults that must NOT be misdiagnosed
# ---------------------------------------------------------------------------


def test_blind_camera_on_a_live_joint_is_not_called_dead(fast):
    """The joint moves and sags the pack; only the camera is at fault. This must stay
    a CalibrationAborted about sight, never a hardware verdict."""
    with make_arm() as arm:
        obs = _BlindObserver()
        with pytest.raises(calibration.CalibrationAborted) as excinfo:
            calibration.calibrate_joint(arm, "ELBOW", settings=fast, observer=obs)
        assert not isinstance(excinfo.value, JointUnresponsive)
        assert "cannot reliably see this joint" in str(excinfo.value)
        assert "actuator itself is alive" in str(excinfo.value)


def test_joint_jammed_against_a_mechanical_stop_is_not_called_dead(fast):
    """A jammed-but-live servo is electrically alive and pushes against the stop, so
    the transport charges stall time. Stall must acquit it of being failed-open --
    otherwise a jam gets an operator sent to replace a perfectly good servo."""
    with make_arm() as arm:
        joint = arm._transport.joints[ELBOW_SERVO_ID]
        joint.mech_min = joint.mech_max = 1500  # pinned: every burst target stalls
        obs = _BlindObserver()
        with pytest.raises(calibration.CalibrationAborted) as excinfo:
            calibration.calibrate_joint(arm, "ELBOW", settings=fast, observer=obs)
        assert not isinstance(excinfo.value, JointUnresponsive)
        assert "stall time accrued" in str(excinfo.value)


# ---------------------------------------------------------------------------
# 3. the discriminator, exercised AT its boundary
# ---------------------------------------------------------------------------


def _verdict(*, sag: float | None, stall_delta: float | None, travel: int = TRUSTED_TRAVEL):
    """Run the discriminator alone and report which fault it names."""
    v_before = None if sag is None else 7.40
    v_after = None if sag is None else 7.40 - sag
    stall_before = None if stall_delta is None else 100.0
    stall_after = None if stall_delta is None else 100.0 + stall_delta
    try:
        calibration._raise_unseen_motion(
            "ELBOW", 6, 6, calibration.CalibrationSettings(),
            v_before, v_after, stall_before, stall_after, travel,
        )
    except JointUnresponsive:
        return "dead"
    except calibration.CalibrationAborted:
        return "alive"
    raise AssertionError("_raise_unseen_motion must never return")


def test_battery_noise_floor_boundary_is_exercised_from_both_sides():
    """Just under the floor reads as a flat pack (dead); just over reads as current
    drawn (alive). Anything else means the threshold is not doing any work."""
    assert _verdict(sag=NOISE_FLOOR - 0.001, stall_delta=0.0) == "dead"
    assert _verdict(sag=NOISE_FLOOR + 0.001, stall_delta=0.0) == "alive"
    # exactly at the floor is NOT current draw: the comparison is strict, so the
    # ambiguous reading falls to the conservative side of "no sag measured".
    assert _verdict(sag=NOISE_FLOOR, stall_delta=0.0) == "dead"


def test_any_stall_at_all_acquits_regardless_of_the_pack():
    """Stall is an independent 'electrically alive' signal: a flat pack plus any
    stall must not convict."""
    assert _verdict(sag=0.0, stall_delta=0.0) == "dead"
    assert _verdict(sag=0.0, stall_delta=0.1) == "alive"


def test_unreadable_battery_never_convicts():
    """A controller that cannot report voltage must not crash -- and must not convict.

    Without the pack reading the remaining evidence is "commanded motion, saw
    nothing, no stall", which is *equally* the signature of a camera that cannot see
    the joint. Convicting on it would be asserting a hardware fault from evidence
    that does not distinguish the two, which is the exact misdiagnosis this whole
    check exists to remove.
    """
    assert _verdict(sag=None, stall_delta=0.0) == "alive"
    with pytest.raises(calibration.CalibrationAborted) as excinfo:
        calibration._raise_unseen_motion(
            "ELBOW", 6, 6, calibration.CalibrationSettings(),
            None, None, 100.0, 100.0, TRUSTED_TRAVEL,
        )
    msg = str(excinfo.value)
    assert "could NOT be cleared or convicted" in msg
    assert "voltage could not be read" in msg


def test_unknown_stall_is_treated_as_unknown_not_as_zero():
    """Real hardware cannot report stalls, so stall_ms_total is None there. That must
    not be read as 'no stall', but it also must not block the diagnosis -- the pack
    reading carries it."""
    assert _verdict(sag=0.0, stall_delta=None) == "dead"
    assert _verdict(sag=NOISE_FLOOR + 0.01, stall_delta=None) == "alive"


def test_read_stall_total_is_none_for_a_transport_that_cannot_report():
    class _NoStall:
        pass

    class _FakeArm:
        _transport = _NoStall()

    assert calibration._read_stall_total(_FakeArm()) is None


# ---------------------------------------------------------------------------
# 4. the design property: zero extra commanded motion
# ---------------------------------------------------------------------------


def test_detection_adds_no_commanded_motion(fast, monkeypatch):
    """The whole reason this design beats a probe excursion. Detecting the dead joint
    must cost exactly the same number of commanded moves as not detecting it -- the
    verdict is read from telemetry around moves the search was making anyway."""

    def _run() -> tuple[int, type]:
        with make_arm() as arm:
            arm._transport.set_dead(ELBOW_SERVO_ID)
            obs = SimMotionObserver(arm._transport)
            with pytest.raises(calibration.CalibrationError) as excinfo:
                calibration.calibrate_joint(arm, "ELBOW", settings=fast, observer=obs)
            return _move_count(arm._transport), type(excinfo.value)

    with_detection, kind_with = _run()

    # Now neuter the discriminator so it always reports the old blind-camera verdict,
    # i.e. performs no battery/stall reasoning at all, and count again.
    def _old_behaviour(name, misses, moves, settings, *_args):
        raise calibration.CalibrationAborted(f"{name}: cannot reliably see this joint")

    monkeypatch.setattr(calibration, "_raise_unseen_motion", _old_behaviour)
    without_detection, kind_without = _run()

    # Guard against this test passing vacuously: if the monkeypatch never took
    # effect both runs would be identical and the move-count assertion would be
    # comparing a run against itself.
    assert kind_with is JointUnresponsive
    assert kind_without is calibration.CalibrationAborted
    assert kind_with is not kind_without, "the neutered run must differ; patch missed"

    assert with_detection == without_detection, (
        f"detection commanded {with_detection - without_detection} extra move(s); "
        "it must be free"
    )


def test_detection_accrues_no_stall_on_a_dead_joint(fast):
    """A failed-open servo cannot stall (nothing turns, nothing pushes), and the
    detection adds no commands that could."""
    with make_arm() as arm:
        arm._transport.set_dead(ELBOW_SERVO_ID)
        obs = SimMotionObserver(arm._transport)
        with pytest.raises(JointUnresponsive):
            calibration.calibrate_joint(arm, "ELBOW", settings=fast, observer=obs)
        assert arm._transport.stall_ms_total() == 0.0
        assert arm._transport.any_torque_on() is False


# ---------------------------------------------------------------------------
# 5. the burst must be large enough for the pack reading to mean anything
#
# This is the regression that a fixed voltage epsilon hid. The sag a live joint
# produces scales with the burst, so a threshold valid at the shipped step=30 is
# wrong at a finer step -- and wrong in the dangerous direction, convicting a
# healthy servo. Measured in sim on ELBOW: step 30 -> 0.096V, 15 -> 0.048V,
# 10 -> 0.032V, 5 -> 0.016V, 1 -> 0.003V.
# ---------------------------------------------------------------------------


def test_a_burst_too_small_to_sag_the_pack_never_convicts():
    """The core guard. A live joint on a fine step sags less than the noise floor,
    which is indistinguishable from a dead one -- so the check must withhold its
    verdict rather than condemn working hardware."""
    tiny = int(NOISE_FLOOR / calibration._BATTERY_SAG_PER_UNIT_V)  # ~50 units
    assert _verdict(sag=0.0, stall_delta=0.0, travel=tiny) == "alive", (
        "a flat pack over a burst too small to produce sag must NOT convict"
    )


def test_declined_verdict_does_not_claim_the_actuator_is_alive():
    """Withholding the verdict is not the same as clearing the joint. The message
    must not tell an operator the servo is fine when it was never tested."""
    with pytest.raises(calibration.CalibrationAborted) as excinfo:
        calibration._raise_unseen_motion(
            "ELBOW", 6, 6, calibration.CalibrationSettings(),
            7.40, 7.40, 100.0, 100.0, 10,
        )
    msg = str(excinfo.value)
    assert "could NOT be cleared or convicted" in msg
    assert "The actuator itself is alive" not in msg
    assert "too small" in msg


def test_trust_boundary_is_exercised_from_both_sides():
    """The trust rule itself, at its edge."""
    per_unit = calibration._BATTERY_SAG_PER_UNIT_V
    needed = NOISE_FLOOR * calibration._BATTERY_TRUST_MARGIN / per_unit
    assert calibration._battery_signal_is_trustworthy(int(needed) + 1) is True
    assert calibration._battery_signal_is_trustworthy(int(needed) - 1) is False


def test_shipped_default_step_keeps_the_check_armed():
    """A guard that disarms itself under the shipped configuration would be
    decorative. step=30 must remain comfortably inside the trusted region."""
    settings = calibration.CalibrationSettings()
    assert settings.step == 30
    travel = settings.step * (settings.verification_moves + 2)
    assert calibration._battery_signal_is_trustworthy(travel) is True


@pytest.mark.parametrize("step", [15, 30, 60])
def test_a_live_joint_is_never_convicted_at_any_supported_step(fast, step):
    """The false-conviction regression, end to end. A perfectly healthy joint with a
    blind camera must yield the sight diagnosis at every step the project supports --
    never JointUnresponsive."""
    settings = calibration.CalibrationSettings(step_duration_ms=20, step=step)
    with make_arm() as arm:
        with pytest.raises(calibration.CalibrationAborted) as excinfo:
            calibration.calibrate_joint(
                arm, "ELBOW", settings=settings, observer=_BlindObserver()
            )
        assert not isinstance(excinfo.value, JointUnresponsive), (
            f"healthy joint convicted as failed-open at step={step}"
        )
