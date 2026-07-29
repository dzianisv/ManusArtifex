"""Proof that vision-guided auto-calibration discovers real limits, stays inside
them, spends a bounded amount of torque doing it, and fails closed everywhere else.

Everything here runs against ``sim.SimTransport`` (``tests/conftest.py`` forces
``ROBOHAND_TRANSPORT=sim`` for every test). No camera, no HID device, no cv2 window.

The "camera" is ``observer.SimMotionObserver``, which renders each simulated
joint's *achieved* position as a bar and runs those frames through a real
``guard.VisionGuard``. It reads ground truth, never the commanded target, so a
command that the simulated mechanical stop blocked genuinely renders as an
unchanged frame -- the same thing a camera would see.

Speed note: ``SafetyLayer.move`` floors every duration at ``config.MIN_DURATION_MS``
(400 ms shipped) and sleeps for it. Most tests here monkeypatch that floor down and
pass a matching ``step_duration_ms`` so a full two-direction sweep takes under a
second. ``test_stall_time_stays_within_budget`` deliberately does NOT: it runs at
the shipped 400 ms burst so its stall-time assertion is about the real default.
"""

from __future__ import annotations

import json
import re

import pytest

from robohand import calibrate as calibration
from robohand import config
from robohand.arm import Arm
from robohand.observer import (
    FlakyMotionObserver,
    MotionObserver,
    NullMotionObserver,
    ObserverError,
    SimMotionObserver,
)
from robohand.sim import SimTransport

GRIP_SERVO_ID = 1


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------


def make_arm(mechanical_limits=None) -> Arm:
    """An ``Arm`` on a ``SimTransport`` with the requested mechanical end stops.

    The transport is swapped after construction (and handed to the already-running
    ``SafetyLayer``) so each test can model a *different* physical arm -- that is
    what makes these tests prove discovery rather than a coincidental match with
    ``sim.DEFAULT_MECHANICAL_LIMITS``.
    """
    arm = Arm(sim=True, use_vision=False, install_signal_handlers=False)
    if mechanical_limits:
        transport = SimTransport(mechanical_limits=mechanical_limits)
        arm._transport = transport
        arm._safety._transport = transport
    return arm


@pytest.fixture
def fast(monkeypatch):
    """Shrink the safety layer's duration floor so a sweep runs in well under a
    second. Only the *timing* changes -- every check, ramp, token, and unload the
    real routine performs still happens exactly as it would at 400 ms.
    """
    monkeypatch.setattr(config, "MIN_DURATION_MS", 20)
    monkeypatch.setattr(config, "SETTLE_MARGIN_MS", 0)
    return calibration.CalibrationSettings(step_duration_ms=20)


def grip_targets(transport: SimTransport):
    return [pos for entry in transport.log if entry["cmd"] == "MOVE"
            for servo_id, pos in entry["targets"] if servo_id == GRIP_SERVO_ID]


def assert_no_grip_command_below_floor(transport: SimTransport):
    """The 2026-07-28 incident, restated as an assertion.

    GRIP's absolute floor is 1500 (its own electrical limit, stricter than the flat
    700 global guard). The incident commanded 700. Nothing this routine emits may
    ever go below 1500 -- and in practice it stays at 1600, the configured floor.
    """
    floor = config.absolute_bounds("GRIP")[0]
    assert floor == 1500
    below = [p for p in grip_targets(transport) if p < floor]
    assert below == [], f"GRIP commanded below its absolute floor {floor}: {below}"


# ---------------------------------------------------------------------------
# 0: the SafetyLayer capability the search is built on
#
# ``probe_absolute_bounds`` is the only thing that lets this routine look past the
# configured band. These tests pin its exact contract: what it widens, what it
# does NOT widen, and that it cannot be reached by accident.
# ---------------------------------------------------------------------------


def test_probe_absolute_bounds_reaches_where_an_override_alone_cannot(fast):
    """The bug this flag exists to fix, stated as a test.

    ELBOW is uncalibrated with a configured band of 1200..1800, which is
    *numerically identical* to its provisional band -- so
    ``allow_uncalibrated_override=True`` widens nothing at all and 800 is refused.
    Its absolute rail is 700..2300, and 800 is comfortably inside it. Only
    ``probe_absolute_bounds`` can get there.
    """
    from robohand import safety as safety_mod

    assert config.effective_bounds("ELBOW", allow_uncalibrated_override=True) == (1200, 1800)
    assert config.absolute_bounds("ELBOW") == (700, 2300)

    with make_arm() as arm:
        # A legal, ordinary override first, so the audit log has a comparison row.
        arm._safety.move(
            {"ELBOW": 1750},
            allow_uncalibrated_override=True,
            override_reason="ordinary override",
            wait=False,
        )
        assert arm._safety.override_log[-1]["probe_absolute_bounds"] is False

        with pytest.raises(safety_mod.PositionOutOfRange):
            arm._safety.move(
                {"ELBOW": 800},
                allow_uncalibrated_override=True,
                override_reason="override alone must not reach here",
                wait=False,
            )
        assert [(sid, pos) for e in arm._transport.log if e["cmd"] == "MOVE"
                for sid, pos in e["targets"]] == [(4, 1750)]

        result = arm._safety.move(
            {"ELBOW": 800},
            allow_uncalibrated_override=True,
            override_reason="auto-calibration probe (test)",
            probe_absolute_bounds=True,
            wait=False,
        )
        assert result == {"ELBOW": 800}
        assert (4, 800) in [(sid, pos) for e in arm._transport.log if e["cmd"] == "MOVE"
                            for sid, pos in e["targets"]]

        # The audit trail records that the stronger capability was used, through
        # the *existing* override_log -- not a second, parallel logging path.
        assert arm._safety.override_log[-1]["probe_absolute_bounds"] is True


def test_probe_absolute_bounds_still_refuses_a_target_outside_the_absolute_rail():
    """The flag widens which *policy* ring applies. It does not touch the hard rail."""
    from robohand import safety as safety_mod

    with make_arm() as arm:
        for target in (699, 2301, 500, 2500):
            with pytest.raises(safety_mod.SafetyViolation):
                arm._safety.move(
                    {"ELBOW": target},
                    allow_uncalibrated_override=True,
                    override_reason="auto-calibration probe (test)",
                    probe_absolute_bounds=True,
                    wait=False,
                )
        assert [e for e in arm._transport.log if e["cmd"] == "MOVE"] == []


def test_the_absolute_rail_backstop_is_not_gated_on_the_new_flag(monkeypatch):
    """Prove the *final* backstop, not just the range check in front of it.

    With ``probe_absolute_bounds`` the policy ring and the rail coincide, so
    ``PositionOutOfRange`` fires first and ``OutOfGlobalGuard`` is normally
    unreachable -- which is exactly why it needs testing another way. Sabotage the
    policy-ring computation (simulating a future bug in bounds logic, in
    ``calibrate.py`` or anywhere else) and confirm the unconditional
    ``config.absolute_bounds`` check still refuses, *with the new flag set*.
    """
    from robohand import safety as safety_mod

    monkeypatch.setattr(
        safety_mod.SafetyLayer, "_request_bounds",
        staticmethod(lambda key, override, probe: (0, 4000)),
    )
    with make_arm() as arm:
        for joint, target in (("ELBOW", 400), ("ELBOW", 2400), ("GRIP", 1499), ("GRIP", 700)):
            with pytest.raises(safety_mod.OutOfGlobalGuard):
                arm._safety.move(
                    {joint: target},
                    allow_uncalibrated_override=True,
                    override_reason="sabotaged bounds logic",
                    probe_absolute_bounds=True,
                    wait=False,
                )
        assert [e for e in arm._transport.log if e["cmd"] == "MOVE"] == []
        assert_no_grip_command_below_floor(arm._transport)


def test_probe_absolute_bounds_can_never_command_grip_below_its_floor():
    """GRIP's rail floor is its own electrical floor (1500), not the flat 700.

    This is the 2026-07-28 incident value restated against the new capability: even
    the widest thing this package can now ask for stops at 1500, and anything below
    it is refused before a packet is built.
    """
    from robohand import safety as safety_mod

    assert config.absolute_bounds("GRIP") == (1500, 2300)
    with make_arm() as arm:
        for target in (1499, 1000, 700):
            with pytest.raises(safety_mod.SafetyViolation):
                arm._safety.move(
                    {"GRIP": target},
                    allow_uncalibrated_override=True,
                    override_reason="auto-calibration probe (test)",
                    probe_absolute_bounds=True,
                    wait=False,
                )
        # 1500 exactly -- the floor itself -- is reachable, and nothing below it is.
        arm._safety.move(
            {"GRIP": 1500},
            allow_uncalibrated_override=True,
            override_reason="auto-calibration probe (test)",
            probe_absolute_bounds=True,
            wait=False,
        )
        assert_no_grip_command_below_floor(arm._transport)


def test_probe_absolute_bounds_requires_an_audited_override():
    """It cannot be used quietly: no override flag, or no reason, is a hard error."""
    with make_arm() as arm:
        with pytest.raises(ValueError, match="probe_absolute_bounds requires"):
            arm._safety.move({"ELBOW": 800}, probe_absolute_bounds=True, wait=False)
        with pytest.raises(ValueError, match="override_reason"):
            arm._safety.move(
                {"ELBOW": 800},
                allow_uncalibrated_override=True,
                override_reason="",
                probe_absolute_bounds=True,
                wait=False,
            )
        assert [e for e in arm._transport.log if e["cmd"] == "MOVE"] == []


def test_probe_absolute_bounds_defaults_off_everywhere_and_is_not_on_the_cli():
    """Structural: no existing caller can turn it on, and no CLI flag maps to it.

    ``robohand move --override-reason`` keeps exactly its old semantics; the only
    module allowed to set this flag is ``calibrate.py``.
    """
    import inspect

    from robohand import cli
    from robohand import safety as safety_mod

    for func in (safety_mod.SafetyLayer.move, Arm.set_positions):
        param = inspect.signature(func).parameters["probe_absolute_bounds"]
        assert param.default is False
        assert param.kind is inspect.Parameter.KEYWORD_ONLY

    cli_source = inspect.getsource(cli)
    assert "probe_absolute_bounds" not in cli_source, (
        "the absolute-rail probe must not be reachable from any CLI subcommand"
    )
    move_source = inspect.getsource(cli.cmd_move)
    assert "probe" not in move_source

    # And the only module in the package that sets it is calibrate.py.
    import pathlib

    package_dir = pathlib.Path(cli.__file__).parent
    setters = sorted(
        p.name for p in package_dir.glob("*.py")
        if "probe_absolute_bounds=True" in p.read_text()
    )
    assert setters == ["calibrate.py"], setters


def test_uncalibrated_override_semantics_are_unchanged_without_the_flag():
    """Regression fence around the existing ``robohand move --override-reason`` path."""
    from robohand import safety as safety_mod

    with make_arm() as arm:
        # Still bounded by the configured band, exactly as before this feature.
        for joint, inside, outside in (("ELBOW", 1800, 1801), ("GRIP", 1600, 1599)):
            arm._safety.move(
                {joint: inside},
                allow_uncalibrated_override=True,
                override_reason="unchanged behaviour check",
                wait=False,
            )
            with pytest.raises(safety_mod.PositionOutOfRange):
                arm._safety.move(
                    {joint: outside},
                    allow_uncalibrated_override=True,
                    override_reason="unchanged behaviour check",
                    wait=False,
                )



# ---------------------------------------------------------------------------
# 1 + 2: discovery is always inside the true limits, and close to them
# ---------------------------------------------------------------------------


# Each case models a *different physical arm*. The mechanical stops are chosen to
# sit **outside** the joint's provisional/configured band ([1200, 1800] for
# non-GRIP joints, [1600, 2300] for GRIP) but inside its absolute rail -- which is
# the only arrangement that is actually possible on real, not-yet-calibrated
# hardware. The provisional band exists *precisely because* the true limits are
# unknown and assumed to lie outside it; an earlier version of this fixture used
# stops inside the band (e.g. ELBOW 1287/1712), which is a physically impossible
# configuration for an uncalibrated arm and which hid the fact that the search
# could not reach past 1800.
DISCOVERY_CASES = [
    # (joint, servo_id, true mechanical limits, label)
    # sim.DEFAULT_MECHANICAL_LIMITS values -- the package's own best guess at this arm:
    ("ELBOW", 4, (850, 2150), "wide-default"),
    ("SHOULDER_PAN", 6, (800, 2200), "wide-default"),
    ("WRIST_FLEX", 3, (900, 2100), "wide-default"),
    # A different physical arm again, so passing cannot be an echo of the defaults:
    ("WRIST_ROLL", 2, (1035, 1965), "wide-custom"),
    ("SHOULDER_LIFT", 5, (1120, 1880), "wide-narrowish"),
    # GRIP can only be wider than its configured band on the low side (its rail
    # ceiling of 2300 is already stricter than the configured 2300... equal, and
    # stricter than the hardware's 2500 -- see the GRIP asymmetry tests below).
    ("GRIP", 1, (1550, 2200), "grip-low-outside-band"),
    # And one arm that turns out to be MORE conservative than the provisional band
    # assumed. Legitimate, if narrower, and it must still be discovered correctly.
    ("ELBOW", 4, (1287, 1712), "inside-old-provisional-band"),
]


@pytest.mark.parametrize(
    "joint,servo_id,true_limits,label", DISCOVERY_CASES,
    ids=[f"{c[0]}-{c[3]}" for c in DISCOVERY_CASES],
)
def test_discovered_band_is_inside_true_mechanical_limits(joint, servo_id, true_limits, label, fast):
    true_min, true_max = true_limits
    band_lo, band_hi = config.effective_bounds(joint, allow_uncalibrated_override=True)
    if label != "inside-old-provisional-band":
        assert true_min < band_lo, (
            f"{joint} case {label} is not realistic: a true stop at {true_min} is inside the "
            f"configured band [{band_lo}, {band_hi}], which cannot happen on an uncalibrated arm"
        )
    rail_lo, rail_hi = config.absolute_bounds(joint)
    assert rail_lo <= true_min < true_max <= rail_hi

    with make_arm({servo_id: true_limits}) as arm:
        observer = SimMotionObserver(arm._transport)
        run = calibration.calibrate(arm, [joint], settings=fast, observer=observer, write=False)
        result = run.joints[joint]
        assert_no_grip_command_below_floor(arm._transport)

    # SAFE: never wider than the truth. A calibration that claims more range than
    # the joint has is the dangerous failure mode -- it would authorise moves into
    # the stop for the rest of the arm's life.
    assert result.min >= true_min, f"{joint} min {result.min} is BELOW the true stop {true_min}"
    assert result.max <= true_max, f"{joint} max {result.max} is ABOVE the true stop {true_max}"

    # USEFUL: not uselessly narrow either. One probe step can be spent overshooting
    # into the stop and the margin is then applied from the last confirmed-moving
    # position, so the recorded limit lands within step + margin of the truth.
    tolerance = fast.step + fast.margin
    assert result.min - true_min <= tolerance, (
        f"{joint} min {result.min} is more than {tolerance} away from true {true_min}"
    )
    assert true_max - result.max <= tolerance, (
        f"{joint} max {result.max} is more than {tolerance} away from true {true_max}"
    )

    assert result.min_finding == calibration.FINDING_MECHANICAL_STOP
    assert result.max_finding == calibration.FINDING_MECHANICAL_STOP


# ---------------------------------------------------------------------------
# GRIP is asymmetric and must not be papered over
# ---------------------------------------------------------------------------


def test_grip_against_the_default_simulated_arm_reports_both_ends_as_policy_limits(fast):
    """``sim.DEFAULT_MECHANICAL_LIMITS[1] == (1500, 2500)`` -- and neither end of
    that produces a mechanical-stop finding. The two reasons are different, and
    both are permanent properties of this joint, not bugs:

    LOW END. The true open stop (1500) is *exactly* ``absolute_bounds("GRIP")[0]``.
    The search steps down 1950 -> ... -> 1530 -> 1500; the 1500 command is the last
    legal position, it is reachable (not past the stop, exactly at it), so the
    joint visibly moves and ``_search_direction`` takes its ``reached_bound and
    moved`` exit. No no-motion observation ever happens, so the honest finding is
    "ran out of legal room", i.e. the policy limit -- even though the number it
    stopped at happens to coincide with the real stop. The algorithm has no way to
    know that; we know it from the reference firmware (docs/research-hardware.md),
    not from this measurement.

    HIGH END. The true closed stop (2500) is 200 units OUTSIDE our own policy
    ceiling of 2300. It is therefore undiscoverable by this routine, permanently
    and by design -- we refuse to command GRIP that high regardless of what the
    hardware could do.
    """
    assert config.absolute_bounds("GRIP") == (1500, 2300)
    with make_arm() as arm:  # sim.DEFAULT_MECHANICAL_LIMITS, unmodified
        assert (arm._transport.joints[GRIP_SERVO_ID].mech_min,
                arm._transport.joints[GRIP_SERVO_ID].mech_max) == (1500, 2500)
        run = calibration.calibrate(
            arm, ["GRIP"], settings=fast, observer=SimMotionObserver(arm._transport), write=False
        )
        targets = grip_targets(arm._transport)
        assert_no_grip_command_below_floor(arm._transport)
        # Nothing was ever driven past a mechanical stop, in either direction.
        assert arm._transport.stall_ms_total() == 0

    result = run.joints["GRIP"]
    assert result.min_finding == calibration.FINDING_POLICY_LIMIT
    assert result.max_finding == calibration.FINDING_POLICY_LIMIT
    assert min(targets) == 1500, "the low search must reach, and stop at, the rail floor"
    assert max(targets) == 2300, "the high search must reach, and stop at, the rail ceiling"
    assert result.min == 1500 + fast.margin
    assert result.max == 2300 - fast.margin


def test_grip_true_closed_stop_is_permanently_undiscoverable(fast):
    """Stated as its own assertion because it is a known, permanent limitation.

    GRIP's real closed-side stop is modelled at 2500. Our policy ceiling is 2300.
    Whatever the hardware can do above 2300, this routine will never report it --
    the recorded max is pinned at ``2300 - margin`` and labelled a policy limit.
    Do not "fix" this by widening GLOBAL_GUARD_MAX; the ceiling is the point.
    """
    for true_max in (2400, 2500):
        with make_arm({GRIP_SERVO_ID: (1500, true_max)}) as arm:
            run = calibration.calibrate(
                arm, ["GRIP"], settings=fast,
                observer=SimMotionObserver(arm._transport), write=False,
            )
            assert max(grip_targets(arm._transport)) == 2300
            assert_no_grip_command_below_floor(arm._transport)
        result = run.joints["GRIP"]
        assert result.max == 2300 - fast.margin
        assert result.max_finding == calibration.FINDING_POLICY_LIMIT
        assert true_max - result.max > fast.step + fast.margin, (
            "this end is deliberately NOT within tolerance of the truth -- asserting "
            "otherwise would be pretending we measured something we refused to look at"
        )


def test_grip_low_end_is_discoverable_when_the_real_stop_is_above_the_floor(fast):
    """The complement of the above: if GRIP's real open stop is anywhere above
    1500, the search does reach it and reports a genuine mechanical stop.
    """
    with make_arm({GRIP_SERVO_ID: (1610, 2200)}) as arm:
        run = calibration.calibrate(
            arm, ["GRIP"], settings=fast, observer=SimMotionObserver(arm._transport), write=False
        )
        assert_no_grip_command_below_floor(arm._transport)
    result = run.joints["GRIP"]
    assert result.min_finding == calibration.FINDING_MECHANICAL_STOP
    assert 1610 <= result.min <= 1610 + fast.step + fast.margin
    # And note it is BELOW the old, buggy search floor of 1600 -- this stop was not
    # reachable at all before the search was widened to the absolute rail.
    assert config.effective_bounds("GRIP", allow_uncalibrated_override=True)[0] == 1600


def test_stop_outside_the_searchable_band_is_reported_as_policy_not_mechanical(fast):
    """A joint whose mechanical stops are wider than the range we are *willing to
    command* must report ``bounded_by_policy_limit`` -- "we ran out of range we are
    willing to command" is a different claim from "we found the stop", and
    conflating them would let a later reader believe a limit was measured when it
    never was.

    Note what the label does NOT say. ELBOW's electrical range is 500..2500; the
    700/2300 bound this run stops at is our own ``GLOBAL_GUARD_MIN/MAX`` policy
    ring, well inside the hardware's rating. Calling this an "electrical limit"
    (as an earlier version did) would have been a claim about the servo that this
    routine has no evidence for.
    """
    with make_arm({4: (500, 2500)}) as arm:
        run = calibration.calibrate(
            arm, ["ELBOW"], settings=fast, observer=SimMotionObserver(arm._transport), write=False
        )
    result = run.joints["ELBOW"]
    lo, hi = calibration.search_bounds("ELBOW")
    assert (lo, hi) == (700, 2300)
    assert config.electrical_bounds("ELBOW") == (500, 2500), (
        "the policy bound this run stopped at is strictly inside the servo's own "
        "electrical rating -- which is exactly why the finding must not say 'electrical'"
    )
    assert result.min_finding == calibration.FINDING_POLICY_LIMIT
    assert result.max_finding == calibration.FINDING_POLICY_LIMIT
    assert result.min == lo + fast.margin
    assert result.max == hi - fast.margin


def test_finding_names_never_claim_an_electrical_measurement():
    assert calibration.FINDING_POLICY_LIMIT == "bounded_by_policy_limit"
    assert "electrical" not in calibration.FINDING_POLICY_LIMIT
    assert "electrical" not in calibration.FINDING_MECHANICAL_STOP
    assert not hasattr(calibration, "FINDING_ELECTRICAL_LIMIT")


def test_calibration_schema_is_v2_and_v1_files_are_refused(tmp_path):
    """A file produced by the pre-fix version recorded a band ~360 units narrower
    than reality on each side and labelled it a measurement. It must never be
    promotable, so the schema string changed and the existing mismatch check
    refuses the old one.
    """
    assert calibration.SCHEMA == "robohand-calibration-v2"
    stale = tmp_path / "calibration-v1.json"
    stale.write_text(json.dumps({
        "schema": "robohand-calibration-v1",
        "transport": "hardware",
        "joints": {"ELBOW": {"min": 1260, "max": 1740}},
    }))
    with pytest.raises(calibration.CalibrationRefused, match="unknown calibration schema"):
        calibration.load_calibration(stale)
    config_copy = tmp_path / "config.json"
    original = config.load_config(config.DEFAULT_CONFIG_PATH)
    config_copy.write_text(json.dumps(original, indent=2))
    with pytest.raises(calibration.CalibrationRefused, match="unknown calibration schema"):
        calibration.apply_calibration(stale, config_path=config_copy)
    assert json.loads(config_copy.read_text()) == original


def test_grip_search_never_approaches_the_incident_value(fast):
    """GRIP's downward search is floored at 1500 -- its own electrical floor, and
    the confirmed true open stop -- which is 800 units clear of the 700 the
    incident commanded. That value is not expressible by this routine.
    """
    with make_arm({GRIP_SERVO_ID: (1520, 2480)}) as arm:
        run = calibration.calibrate(
            arm, ["GRIP"], settings=fast, observer=SimMotionObserver(arm._transport), write=False
        )
        targets = grip_targets(arm._transport)
        assert_no_grip_command_below_floor(arm._transport)
    assert min(targets) >= 1500
    assert run.joints["GRIP"].min >= 1500
    # The true stop here (1520) is one step inside the rail, so the last commanded
    # position (1500) still produced visible motion and no stop was ever observed:
    # honest answer is "ran out of legal room", not "measured a stop".
    assert run.joints["GRIP"].min_finding == calibration.FINDING_POLICY_LIMIT


# ---------------------------------------------------------------------------
# 3: bounded torque per run
# ---------------------------------------------------------------------------


def test_stall_time_stays_within_budget():
    """Run at the SHIPPED 400 ms burst and assert a specific stall ceiling.

    Derivation of the number, per direction:
      * at most ONE step can be a partial travel that ends against the stop (the
        simulated joint is pinned there by that single command), and
      * exactly ``confirmations`` (2) further steps are commanded past the stop
        before it is believed.
    Every other move in the run -- the initial move to center, the observer
    verification retreat, the back-off to the recorded limit -- targets a position
    inside the mechanical range and therefore accrues no stall time at all.

    So: 3 stalling commands x 2 directions x 400 ms = 2400 ms, and nothing else.
    """
    settings = calibration.CalibrationSettings()
    assert settings.step_duration_ms == config.MIN_DURATION_MS == 400

    with make_arm({GRIP_SERVO_ID: (1855, 2045)}) as arm:
        calibration.calibrate(
            arm, ["GRIP"], settings=settings, observer=SimMotionObserver(arm._transport), write=False
        )
        stall = arm._transport.stall_ms_total()
        assert_no_grip_command_below_floor(arm._transport)
        assert arm._transport.any_torque_on() is False

    assert stall <= 2400, f"stall budget blown: {stall}ms > 2400ms"
    # And, stated as the flat number the design promises rather than as arithmetic:
    assert stall < 2500


def test_every_probe_step_unloads_before_the_next_move(fast):
    """Torque-burst discipline, asserted from the transport log.

    Every *probe* step -- the only kind of move that can be into an unknown stop --
    is a single MOVE followed by an UNLOAD. Two probe steps never occur back to
    back. That is what makes a step into a stop cost one bounded burst instead of
    the continuous torque of the incident.

    Consecutive MOVE packets with no UNLOAD between them DO occur, and are not a
    violation: ``SafetyLayer`` internally ramps any single move whose delta exceeds
    ``max_step_per_move`` (300) into several packets, and the widened search means
    the re-centering move between the two directions can now span up to 800 units.
    Those packets belong to one ``set_position`` call, travel exclusively through
    territory the joint has already been *watched* to traverse (so they cannot be
    into a stop), and are unloaded at the end of the call like everything else.

    The two cases are distinguishable from the log with no ambiguity: a ramp
    increment is ~``max_step_per_move`` units, a probe step is exactly ``step``
    (30). So every unbroken MOVE run of length > 1 must have every internal
    increment strictly larger than one probe step.
    """
    import math as _math

    with make_arm({4: (850, 2150)}) as arm:
        calibration.calibrate(
            arm, ["ELBOW"], settings=fast, observer=SimMotionObserver(arm._transport), write=False
        )
        entries = [e for e in arm._transport.log if e["cmd"] in ("MOVE", "UNLOAD")]
        max_step_per_move = arm._safety.max_step_per_move

    shape = "".join("M" if e["cmd"] == "MOVE" else "U" for e in entries)
    assert shape.count("M") > 5
    assert shape.endswith("U"), f"the run left torque commanded: {shape}"

    runs, current = [], []
    for entry in entries:
        if entry["cmd"] == "MOVE":
            current.append(entry["targets"][0][1])
        elif current:
            runs.append(current)
            current = []
    assert not current, "the log ended on a MOVE with no UNLOAD"
    assert runs

    rail_lo, rail_hi = calibration.search_bounds("ELBOW")
    ramp_cap = _math.ceil((rail_hi - rail_lo) / max_step_per_move)
    multi = 0
    for run in runs:
        if len(run) == 1:
            continue
        multi += 1
        increments = [abs(b - a) for a, b in zip(run, run[1:])]
        assert min(increments) > fast.step, (
            f"two probe-sized steps ({increments}) with no unload between them: {run}"
        )
        assert len(run) <= ramp_cap, f"ramp longer than the rail allows: {run}"
    assert multi <= 2, (
        f"more re-centering ramps than the two direction restarts can explain: {multi}"
    )


def test_torque_budget_abort_writes_nothing(tmp_path, fast, monkeypatch):
    settings = calibration.CalibrationSettings(step_duration_ms=20, max_torque_ms=100)
    with make_arm({4: (850, 2150)}) as arm:
        with pytest.raises(calibration.CalibrationAborted, match="torque"):
            calibration.calibrate(
                arm, ["ELBOW"], settings=settings,
                observer=SimMotionObserver(arm._transport), output_dir=tmp_path,
            )
        assert arm._transport.any_torque_on() is False
    assert list(tmp_path.glob("calibration-*.json")) == []


def test_wall_clock_abort_writes_nothing(tmp_path, fast):
    settings = calibration.CalibrationSettings(step_duration_ms=20, max_wall_clock_s=0.001)
    with make_arm({4: (850, 2150)}) as arm:
        with pytest.raises(calibration.CalibrationAborted, match="wall-clock"):
            calibration.calibrate(
                arm, ["ELBOW"], settings=settings,
                observer=SimMotionObserver(arm._transport), output_dir=tmp_path,
            )
    assert list(tmp_path.glob("calibration-*.json")) == []


# ---------------------------------------------------------------------------
# 4: a flaky observer must not yield a falsely-narrow result
# ---------------------------------------------------------------------------


def test_deterministic_flakiness_is_ridden_through(tmp_path, fast):
    """Every seventh genuine motion report is dropped -- a camera that misses about
    one real motion in seven.

    Two things must hold for the run to survive that. First, with
    ``confirmations=2`` a 1-in-7 pattern can never produce two consecutive misses,
    so it cannot fabricate a stop. Second, the observer-reliability burst is 6
    consecutive known-motion moves and tolerates 1 miss, and any 6 consecutive
    observations contain at most one multiple of 7 -- so the burst passes. The
    result must therefore still land inside the true limits and within one
    step+margin of them, not narrower.
    """
    true_limits = (850, 2150)
    with make_arm({4: true_limits}) as arm:
        observer = FlakyMotionObserver(SimMotionObserver(arm._transport), miss_every=7)
        run = calibration.calibrate(
            arm, ["ELBOW"], settings=fast, observer=observer, write=False
        )
    result = run.joints["ELBOW"]
    assert observer.injected_misses > 0, "the fault injector never fired; test proves nothing"
    assert true_limits[0] <= result.min < result.max <= true_limits[1]
    tolerance = fast.step + fast.margin
    assert result.min - true_limits[0] <= tolerance
    assert true_limits[1] - result.max <= tolerance


@pytest.mark.parametrize("seed", [0, 1, 2, 3, 4, 5, 6, 7])
def test_aggressive_flakiness_never_produces_a_falsely_narrow_result(tmp_path, fast, seed):
    """Half of all genuine motion reports are dropped at random.

    The requirement is a disjunction, and both branches are acceptable: either the
    consecutive-confirmation + observer-verification logic rides through and returns
    a band that is still within tolerance of the truth, or the run aborts and writes
    nothing. What must NOT happen is a completed run that silently records a band
    far narrower than reality and marks it as a measurement.
    """
    true_limits = (850, 2150)
    tolerance = fast.step + fast.margin
    with make_arm({4: true_limits}) as arm:
        observer = FlakyMotionObserver(
            SimMotionObserver(arm._transport), miss_probability=0.5, seed=seed
        )
        try:
            run = calibration.calibrate(
                arm, ["ELBOW"], settings=fast, observer=observer, output_dir=tmp_path
            )
        except calibration.CalibrationAborted:
            assert list(tmp_path.glob("calibration-*.json")) == [], (
                "an aborted run must never leave a result file behind"
            )
            return

    result = run.joints["ELBOW"]
    assert true_limits[0] <= result.min < result.max <= true_limits[1]
    assert result.min - true_limits[0] <= tolerance, (
        f"completed run recorded a falsely-narrow min: {result.min} vs true {true_limits[0]}"
    )
    assert true_limits[1] - result.max <= tolerance, (
        f"completed run recorded a falsely-narrow max: {result.max} vs true {true_limits[1]}"
    )


def test_too_many_ambiguous_observations_aborts(tmp_path, fast):
    settings = calibration.CalibrationSettings(step_duration_ms=20, max_ambiguous_observations=1)
    with make_arm({4: (850, 2150)}) as arm:
        observer = FlakyMotionObserver(SimMotionObserver(arm._transport), miss_every=3)
        with pytest.raises(calibration.CalibrationAborted, match="ambiguous"):
            calibration.calibrate(
                arm, ["ELBOW"], settings=settings, observer=observer, output_dir=tmp_path
            )
    assert list(tmp_path.glob("calibration-*.json")) == []


# ---------------------------------------------------------------------------
# 5: an observer that never sees motion aborts cleanly and writes nothing
# ---------------------------------------------------------------------------


def test_blind_observer_aborts_and_writes_no_file(tmp_path, fast):
    """A lens cap, a dead camera, or a joint outside the ROI reads exactly like
    "every joint is already at its mechanical stop". The routine must refuse to
    record that as a measurement.
    """
    with make_arm({4: (850, 2150)}) as arm:
        observer = NullMotionObserver()
        with pytest.raises(calibration.CalibrationAborted) as excinfo:
            calibration.calibrate(
                arm, ["ELBOW"], settings=fast, observer=observer, output_dir=tmp_path
            )
        assert arm._transport.any_torque_on() is False
        assert arm._transport.stall_ms_total() == 0, (
            "a blind-observer abort should stop long before anything reaches a stop"
        )
    assert "cannot reliably see this joint" in str(excinfo.value)
    assert list(tmp_path.glob("calibration-*.json")) == []


def test_blind_observer_aborts_for_every_joint(tmp_path, fast):
    for joint in config.JOINT_NAMES:
        with make_arm() as arm:
            with pytest.raises(calibration.CalibrationAborted):
                calibration.calibrate(
                    arm, [joint], settings=fast,
                    observer=NullMotionObserver(), output_dir=tmp_path,
                )
            assert_no_grip_command_below_floor(arm._transport)
    assert list(tmp_path.glob("calibration-*.json")) == []


class _ExplodingObserver(MotionObserver):
    """Raises partway through -- a camera that is unplugged mid-run."""

    def __init__(self, inner: MotionObserver, fail_after: int) -> None:
        self._inner = inner
        self._fail_after = fail_after
        self.calls = 0

    def available(self) -> bool:
        return True

    def capture(self):
        return self._inner.capture()

    def observe(self, before, after) -> bool:
        self.calls += 1
        if self.calls > self._fail_after:
            raise ObserverError("camera disconnected")
        return self._inner.observe(before, after)


def test_observer_error_mid_run_aborts_and_writes_nothing(tmp_path, fast):
    with make_arm({4: (850, 2150)}) as arm:
        observer = _ExplodingObserver(SimMotionObserver(arm._transport), fail_after=2)
        with pytest.raises(calibration.CalibrationAborted, match="observer failed"):
            calibration.calibrate(
                arm, ["ELBOW"], settings=fast, observer=observer, output_dir=tmp_path
            )
        assert arm._transport.any_torque_on() is False
    assert list(tmp_path.glob("calibration-*.json")) == []


# ---------------------------------------------------------------------------
# 6: the routine cannot bypass, widen, or out-argue the safety layer
# ---------------------------------------------------------------------------


def test_search_bounds_never_exceed_the_absolute_rail():
    for name in config.JOINT_NAMES:
        lo, hi = calibration.search_bounds(name)
        abs_lo, abs_hi = config.absolute_bounds(name)
        assert abs_lo <= lo <= hi <= abs_hi
        # And they ARE the rail, not the configured band: the configured band is
        # numerically identical to the provisional band for every uncalibrated
        # joint, so searching inside it could never find a real stop.
        assert (lo, hi) == (abs_lo, abs_hi)
    assert calibration.search_bounds("GRIP") == (1500, 2300)
    assert calibration.search_bounds("ELBOW") == (700, 2300)


def test_every_commanded_target_stays_inside_the_absolute_rail(fast):
    """Sweep every joint and assert, from the transport log, that not one packet
    carried a target outside that servo's hardcoded rail.
    """
    # Realistic: every non-GRIP stop is outside the [1200, 1800] configured band,
    # so every joint's search genuinely travels out toward the rail.
    limits = {1: (1550, 2200), 2: (1035, 1965), 3: (900, 2100),
              4: (850, 2150), 5: (1120, 1880), 6: (800, 2200)}
    with make_arm(limits) as arm:
        calibration.calibrate(
            arm, config.JOINT_NAMES, settings=fast,
            observer=SimMotionObserver(arm._transport), write=False,
        )
        moves = [(sid, pos) for e in arm._transport.log if e["cmd"] == "MOVE"
                 for sid, pos in e["targets"]]
        assert_no_grip_command_below_floor(arm._transport)

    assert moves
    for servo_id, pos in moves:
        name = config.ID_TO_NAME[servo_id]
        lo, hi = config.absolute_bounds(name)
        assert lo <= pos <= hi, f"{name} commanded {pos}, outside absolute rail [{lo}, {hi}]"


def test_a_step_refused_by_the_safety_layer_aborts_rather_than_being_swallowed(fast, monkeypatch):
    """If this module ever computes a target the safety layer rejects, that is a bug
    in its own bounds logic -- it must surface as an abort with nothing written, not
    be retried, clamped, or ignored.
    """
    from robohand import safety

    original = Arm.set_position
    state = {"n": 0}

    def sabotaged(self, name, position, *a, **kw):
        state["n"] += 1
        if state["n"] == 3:
            # Simulate the bug: a target outside the joint's effective range.
            position = config.absolute_bounds(name)[1] + 500
            return original(self, name, position, *a, **kw)
        return original(self, name, position, *a, **kw)

    monkeypatch.setattr(Arm, "set_position", sabotaged)
    with make_arm({4: (850, 2150)}) as arm:
        with pytest.raises(calibration.CalibrationAborted, match="refused by the safety layer"):
            calibration.calibrate(
                arm, ["ELBOW"], settings=fast,
                observer=SimMotionObserver(arm._transport), write=False,
            )
        # The illegal packet never reached the transport.
        moves = [pos for e in arm._transport.log if e["cmd"] == "MOVE"
                 for sid, pos in e["targets"]]
        assert all(pos <= config.absolute_bounds("ELBOW")[1] for pos in moves)
    assert issubclass(safety.PositionOutOfRange, safety.SafetyViolation)


def test_transport_still_refuses_a_move_without_a_token(fast):
    """The choke point is unchanged by anything in this feature."""
    from robohand.protocol import SafetyBypassError

    with make_arm() as arm:
        with pytest.raises(SafetyBypassError):
            arm._transport.move_servos([(4, 1500)], 400)


# -- out-of-policy settings fail closed -------------------------------------


@pytest.mark.parametrize("kwargs,pattern", [
    ({"step": 0}, "step must be in"),
    ({"step": 500}, "step must be in"),
    ({"step": 1.5}, "step must be an integer"),
    ({"margin": -1}, "margin must be in"),
    ({"confirmations": 1}, "confirmations must be in"),
    ({"step_duration_ms": 900}, "torque-burst ceiling"),
    ({"step_duration_ms": 10}, "below config.MIN_DURATION_MS"),
    ({"max_wall_clock_s": 0}, "max_wall_clock_s"),
])
def test_out_of_policy_settings_are_refused(kwargs, pattern):
    with pytest.raises(calibration.CalibrationRefused, match=re.escape(pattern)):
        calibration.CalibrationSettings(**kwargs)


def test_absurd_margin_is_refused_before_any_motion(tmp_path):
    """A margin wide enough to swallow the joint's own searchable band must fail
    closed -- refuse, do not quietly shrink the margin and carry on.
    """
    settings = calibration.CalibrationSettings(margin=400, step_duration_ms=config.MIN_DURATION_MS)
    with make_arm() as arm:
        with pytest.raises(calibration.CalibrationRefused, match="margin"):
            calibration.calibrate(
                arm, ["GRIP"], settings=settings,
                observer=SimMotionObserver(arm._transport), output_dir=tmp_path,
            )
        assert [e for e in arm._transport.log if e["cmd"] == "MOVE"] == []
    assert list(tmp_path.glob("calibration-*.json")) == []


def test_coarse_step_is_refused_before_any_motion(tmp_path, fast, monkeypatch):
    """A step too coarse to distinguish travel from a stop must fail closed.

    The shipped provisional bands (600 wide) are large relative to ``MAX_STEP``, so
    this guard cannot fire on the stock config -- squeeze the searchable band to a
    sliver to prove it exists and refuses rather than silently proceeding on two
    steps' worth of evidence.
    """
    monkeypatch.setattr(calibration, "search_bounds", lambda name: (1450, 1550))
    settings = calibration.CalibrationSettings(step=60, step_duration_ms=20, margin=10)
    with make_arm() as arm:
        with pytest.raises(calibration.CalibrationRefused, match="too coarse"):
            calibration.calibrate(
                arm, ["ELBOW"], settings=settings,
                observer=SimMotionObserver(arm._transport), output_dir=tmp_path,
            )
        assert [e for e in arm._transport.log if e["cmd"] == "MOVE"] == []
    assert list(tmp_path.glob("calibration-*.json")) == []


# ---------------------------------------------------------------------------
# output file + promotion
# ---------------------------------------------------------------------------


def test_result_file_records_findings_and_provenance(tmp_path, fast):
    with make_arm({GRIP_SERVO_ID: (1550, 2200)}) as arm:
        run = calibration.calibrate(
            arm, ["GRIP"], settings=fast,
            observer=SimMotionObserver(arm._transport), output_dir=tmp_path,
        )
    assert run.output_path is not None and run.output_path.exists()
    payload = json.loads(run.output_path.read_text())
    assert payload["schema"] == calibration.SCHEMA
    assert payload["transport"] == "sim"
    entry = payload["joints"]["GRIP"]
    for key in ("min", "max", "margin", "min_finding", "max_finding",
                "steps_min", "steps_max", "ambiguous_observations", "torque_ms"):
        assert key in entry, f"result file is missing {key}"
    assert entry["min_finding"] in (
        calibration.FINDING_MECHANICAL_STOP, calibration.FINDING_POLICY_LIMIT
    )
    # config.json must be untouched by a run.
    assert config.JOINT_LIMITS["GRIP"].calibrated is False


def test_promotion_updates_config_and_round_trips(tmp_path, fast, monkeypatch):
    config_copy = tmp_path / "config.json"
    config_copy.write_text(json.dumps(config.load_config(config.DEFAULT_CONFIG_PATH), indent=2))

    with make_arm({GRIP_SERVO_ID: (1550, 2200)}) as arm:
        run = calibration.calibrate(
            arm, ["GRIP"], settings=fast,
            observer=SimMotionObserver(arm._transport), output_dir=tmp_path,
        )

    applied = calibration.apply_calibration(
        run.output_path, config_path=config_copy, allow_simulated=True
    )
    assert applied["GRIP"]["calibrated"] is True
    reloaded = config.load_config(config_copy)
    spec = reloaded["servos"]["GRIP"]
    assert spec["calibrated"] is True
    assert spec["min"] == run.joints["GRIP"].min
    assert spec["max"] == run.joints["GRIP"].max
    assert "PROVISIONAL" not in spec["comment"]
    # And the written file still satisfies the exact validation config.py runs at
    # import time, for every joint -- not just the promoted one.
    for name, joint_spec in reloaded["servos"].items():
        config._validate_servo_spec(name, joint_spec)


def test_promotion_refuses_a_simulated_result_by_default(tmp_path, fast):
    config_copy = tmp_path / "config.json"
    config_copy.write_text(json.dumps(config.load_config(config.DEFAULT_CONFIG_PATH), indent=2))
    with make_arm({GRIP_SERVO_ID: (1550, 2200)}) as arm:
        run = calibration.calibrate(
            arm, ["GRIP"], settings=fast,
            observer=SimMotionObserver(arm._transport), output_dir=tmp_path,
        )
    with pytest.raises(calibration.CalibrationRefused, match="not real hardware"):
        calibration.apply_calibration(run.output_path, config_path=config_copy)
    assert json.loads(config_copy.read_text())["servos"]["GRIP"]["calibrated"] is False


def test_promotion_refuses_a_hand_widened_calibration_file(tmp_path):
    """The dangerous edit: someone hand-writes a calibration file claiming GRIP can
    go to 900. Promotion must refuse it outright, not clamp it and proceed.
    """
    config_copy = tmp_path / "config.json"
    original = config.load_config(config.DEFAULT_CONFIG_PATH)
    config_copy.write_text(json.dumps(original, indent=2))
    forged = tmp_path / "calibration-forged.json"
    forged.write_text(json.dumps({
        "schema": calibration.SCHEMA,
        "transport": "hardware",
        "joints": {"GRIP": {"min": 900, "max": 2400, "margin": 0,
                            "min_finding": calibration.FINDING_MECHANICAL_STOP,
                            "max_finding": calibration.FINDING_MECHANICAL_STOP}},
    }))
    with pytest.raises(calibration.CalibrationRefused, match="absolute rail"):
        calibration.apply_calibration(forged, config_path=config_copy)
    assert json.loads(config_copy.read_text()) == original


def test_promotion_refuses_an_inverted_range(tmp_path):
    config_copy = tmp_path / "config.json"
    original = config.load_config(config.DEFAULT_CONFIG_PATH)
    config_copy.write_text(json.dumps(original, indent=2))
    forged = tmp_path / "calibration-inverted.json"
    forged.write_text(json.dumps({
        "schema": calibration.SCHEMA,
        "transport": "hardware",
        "joints": {"ELBOW": {"min": 1700, "max": 1300}},
    }))
    with pytest.raises(calibration.CalibrationRefused, match="empty or inverted"):
        calibration.apply_calibration(forged, config_path=config_copy)
    assert json.loads(config_copy.read_text()) == original


def test_promotion_refuses_an_unknown_schema(tmp_path):
    bad = tmp_path / "calibration-bad.json"
    bad.write_text(json.dumps({"schema": "something-else", "joints": {}}))
    with pytest.raises(calibration.CalibrationRefused, match="unknown calibration schema"):
        calibration.apply_calibration(bad, config_path=tmp_path / "config.json")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def test_cli_calibrate_runs_under_sim(tmp_path, fast, capsys):
    from robohand import cli

    rc = cli.main([
        "--sim", "calibrate", "--joint", "WRIST_ROLL",
        "--step", "30", "--margin", "60", "--step-duration", "20",
        "--out-dir", str(tmp_path), "--json",
    ])
    assert rc == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["schema"] == calibration.SCHEMA
    assert "WRIST_ROLL" in payload["joints"]
    assert len(list(tmp_path.glob("calibration-*.json"))) == 1


def test_cli_calibrate_refuses_real_hardware_without_the_presence_flag(monkeypatch, capsys):
    """The refusal must happen before the transport is opened.

    ``_make_arm`` and ``protocol.HidTransport`` are both replaced with a landmine:
    if the guard ever failed, this test would fail loudly rather than reaching for
    the real HID device (VID 0x0483 / PID 0x5750).
    """
    from robohand import cli, protocol

    monkeypatch.delenv("ROBOHAND_TRANSPORT", raising=False)

    def landmine(*args, **kwargs):
        raise AssertionError("hardware path reached -- the presence gate did not hold")

    monkeypatch.setattr(cli, "_make_arm", landmine)
    monkeypatch.setattr(protocol, "HidTransport", landmine)

    assert cli.main(["calibrate", "--joint", "GRIP"]) == 2
    err = capsys.readouterr().err
    assert "REFUSING TO RUN" in err
    assert "--i-am-present-and-watching" in err

    assert cli.main(["calibrate", "--all"]) == 2
    assert "present" in capsys.readouterr().err


def test_cli_calibrate_requires_a_target(capsys):
    from robohand import cli

    assert cli.main(["--sim", "calibrate"]) == 2
    assert "--joint" in capsys.readouterr().err
    assert cli.main(["--sim", "calibrate", "--joint", "GRIP", "--all"]) == 2
    assert "mutually exclusive" in capsys.readouterr().err


def test_cli_apply_refuses_a_widened_file(tmp_path, capsys, monkeypatch):
    from robohand import cli

    config_copy = tmp_path / "config.json"
    config_copy.write_text(json.dumps(config.load_config(config.DEFAULT_CONFIG_PATH), indent=2))
    monkeypatch.setenv("ROBOHAND_CONFIG", str(config_copy))
    forged = tmp_path / "calibration-forged.json"
    forged.write_text(json.dumps({
        "schema": calibration.SCHEMA,
        "transport": "hardware",
        "joints": {"GRIP": {"min": 700, "max": 2300}},
    }))
    assert cli.main(["--sim", "calibrate", "--apply", str(forged)]) == 2
    assert "absolute rail" in capsys.readouterr().err
    assert json.loads(config_copy.read_text())["servos"]["GRIP"]["calibrated"] is False
