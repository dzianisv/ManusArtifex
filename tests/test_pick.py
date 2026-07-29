"""Proof that the vision-guided pick routine works, and -- more importantly --
that every way it can fail leaves the arm safe.

Everything runs against ``sim.SimTransport`` + ``sim.SimWorld``
(``tests/conftest.py`` forces ``ROBOHAND_TRANSPORT=sim`` for every test). No
camera, no HID device, no network. The "camera" is ``locate.SimLocator``, which
reads simulator ground truth; the grip ladder's motion observer is
``observer.SimMotionObserver``, which renders each joint's *achieved* position.
Both read where the simulated arm ended up, never where it was told to go, so a
blocked command genuinely reads as "no motion".

Speed note: ``SafetyLayer.move`` floors every duration at
``config.MIN_DURATION_MS`` (400 ms shipped) and sleeps for it. These tests
monkeypatch that floor down and pass matching durations, so a full pick takes
well under a second. Only the *timing* changes -- every bounds check, ramp,
token mint, and unload the shipped routine performs still happens.
"""

from __future__ import annotations

import random

import pytest

from robohand import config, pick, sim
from robohand.arm import Arm
from robohand.locate import (
    NullLocator,
    OscillatingLocator,
    OutOfBoundsLocator,
    SimLocator,
)
from robohand.pick import PickRefused, PickSettings, PickState
from robohand.protocol import MotionToken, SafetyBypassError

GRIP_SERVO_ID = 1
GRIP_ABSOLUTE_FLOOR = 1500


# ---------------------------------------------------------------------------
# fixtures / helpers
# ---------------------------------------------------------------------------


@pytest.fixture
def calibrated(monkeypatch):
    """A calibrated arm, running fast.

    Every joint in the shipped config.json is ``calibrated: false``, and the pick
    routine refuses to run on an uncalibrated joint (proved separately below), so
    every behavioural test here has to promote them first. ``JOINT_LIMITS`` is a
    plain dict of immutable NamedTuples, so this is a ``_replace`` +
    ``monkeypatch.setitem`` per joint -- the same mechanism the rest of the suite
    uses, not a third one.
    """
    for name, limit in config.JOINT_LIMITS.items():
        monkeypatch.setitem(config.JOINT_LIMITS, name, limit._replace(calibrated=True))
    monkeypatch.setattr(config, "MIN_DURATION_MS", 20)
    monkeypatch.setattr(config, "SETTLE_MARGIN_MS", 0)


def fast_settings(**kwargs) -> PickSettings:
    defaults = {"move_duration_ms": 20, "grip_duration_ms": 20}
    defaults.update(kwargs)
    return PickSettings(**defaults)


def make_scene(obj=None, **world_kwargs):
    """An ``Arm`` on a ``SimTransport`` with a scene attached, plus its locator."""
    arm = Arm(sim=True, use_vision=False, install_signal_handlers=False)
    world = sim.attach_world(arm._transport, obj, **world_kwargs)
    return arm, world, SimLocator(world)


def default_object(x=10.0, y=140.0, **kwargs) -> sim.SimObject:
    # grasp_radius 14 mm: the routine converges the *image* error, and a single
    # camera's height parallax leaves a systematic few-mm bias in the plane that
    # no amount of image-space convergence can remove (see docs/pick.md).
    kwargs.setdefault("grasp_radius_mm", 14.0)
    return sim.SimObject(x=x, y=y, **kwargs)


# ---------------------------------------------------------------------------
# safety invariants, asserted after EVERY pick attempt in this file
# ---------------------------------------------------------------------------


def commanded_targets(transport):
    return [
        (servo_id, position)
        for entry in transport.log
        if entry["cmd"] == "MOVE"
        for servo_id, position in entry["targets"]
    ]


def assert_pick_was_safe(transport, *, expect_torque_off=True, expect_moves=True):
    """The invariants that must hold no matter how the attempt ended."""
    # 1. The 2026-07-28 incident, restated. GRIP's absolute floor is 1500 -- its
    #    own electrical limit, stricter than the flat 700 global guard. The
    #    incident commanded 700.
    assert config.ELECTRICAL_LIMITS[GRIP_SERVO_ID][0] == GRIP_ABSOLUTE_FLOOR
    assert config.absolute_bounds("GRIP")[0] == GRIP_ABSOLUTE_FLOOR
    grip_commands = [p for sid, p in commanded_targets(transport) if sid == GRIP_SERVO_ID]
    below = [p for p in grip_commands if p < GRIP_ABSOLUTE_FLOOR]
    assert below == [], f"GRIP commanded below its absolute floor: {below}"

    # 2. Every commanded target on every joint stayed inside that joint's
    #    absolute rail -- the ring nothing in this package may widen.
    for servo_id, position in commanded_targets(transport):
        name = config.ID_TO_NAME[servo_id]
        lo, hi = config.absolute_bounds(name)
        assert lo <= position <= hi, f"{name} commanded {position}, outside [{lo}, {hi}]"

    # 3. Every SERVO_MOVE that reached the transport carried its own live,
    #    single-use MotionToken. This is enforced structurally by
    #    ``protocol.require_motion_token`` (a packet without one never reaches the
    #    log at all), but assert it explicitly so the invariant is visible rather
    #    than merely implied.
    tokens = [entry["token"] for entry in transport.log if entry["cmd"] == "MOVE"]
    if expect_moves:
        assert tokens, "the attempt sent no SERVO_MOVE at all -- nothing was exercised"
    assert all(isinstance(t, MotionToken) for t in tokens)
    assert all(t._spent for t in tokens), "a token reached the transport unredeemed"
    assert len({id(t) for t in tokens}) == len(tokens), "a token was reused across packets"
    # And the door is still shut afterwards.
    with pytest.raises(SafetyBypassError):
        transport.move_servos([(GRIP_SERVO_ID, 2000)], 100)

    # 4. Stall budget: EXACTLY ZERO, and that is not an approximation.
    #
    #    ``SimJoint`` accrues stall_ms only when a commanded target is beyond a
    #    *mechanical* end stop (locked rotor); contact with an obstruction -- the
    #    grasped object -- is tracked separately as a contact_event, because
    #    squeezing something is what a gripper is for. Every target this routine
    #    can emit is bounded by ``config.effective_bounds``, which for these
    #    joints is 1200..1800 (and GRIP 1600..2300), strictly inside
    #    ``sim.DEFAULT_MECHANICAL_LIMITS`` (850..2150, 800..2200, 1500..2500).
    #    So a correct pick cannot command past a mechanical stop even once, and
    #    the only honest budget is zero. A loose "moves x duration_ms" bound would
    #    pass just as well while hiding a genuine bounds bug behind slack.
    assert transport.stall_ms_total() == 0.0, (
        f"a pick attempt accrued {transport.stall_ms_total()}ms of locked-rotor stall; "
        "that means a commanded target left the effective bounds"
    )

    if expect_torque_off:
        assert transport.any_torque_on() is False, "the arm was left under torque"


# ===========================================================================
# 1. The happy path -- verified from simulator ground truth, not self-report
# ===========================================================================


def test_successful_pick_holds_the_object(calibrated):
    arm, world, locator = make_scene(default_object())
    try:
        result = pick.pick(arm, locator, fast_settings())

        assert result.success is True
        assert result.state is PickState.DONE
        assert result.states_visited() == [
            "LOCATE", "APPROACH", "DESCEND", "GRASP", "VERIFY", "LIFT", "VERIFY_HELD", "DONE",
        ]
        # Ground truth, not PickResult.success: the simulator says the object is
        # in the jaws and off the table.
        assert world.held is True
        assert world.object.z > 0.0
        assert world.object.slipped is False
        assert world.object.grasp_count >= 1
        # A successful pick deliberately keeps torque on -- otherwise it drops
        # what it just picked up.
        assert result.holding is True
        assert_pick_was_safe(arm._transport, expect_torque_off=False)
    finally:
        arm.close()


def test_grasp_uses_the_incremental_ladder_and_never_an_absolute_close(calibrated):
    arm, world, locator = make_scene(default_object())
    try:
        result = pick.pick(arm, locator, fast_settings())
        assert result.success is True
        # The ladder's own evidence of a real grasp: it stopped because motion
        # ceased, at a position it can name, before its hard backstop.
        assert result.grip_result["stopped_reason"] == "no_motion_detected"
        assert result.grip_result["contact_position"] is not None
        assert result.grip_result["steps"] > 1, "an absolute close would be a single step"
        # Contact was modelled as an obstruction, never as a locked rotor.
        assert arm._transport.joints[GRIP_SERVO_ID].contact_events > 0
        assert arm._transport.joints[GRIP_SERVO_ID].stall_ms == 0.0
    finally:
        arm.close()


def test_pick_never_touches_the_transport_directly():
    """Structural claim: pick.py has no code path to move_servos or MotionToken.

    Checked on the parsed AST rather than the raw text, because the module
    docstring names both on purpose -- to say it does not use them. Prose that
    mentions the bypass is fine; a call site is not.
    """
    import ast
    import pathlib

    tree = ast.parse(pathlib.Path(pick.__file__).read_text())
    attributes = {n.attr for n in ast.walk(tree) if isinstance(n, ast.Attribute)}
    names = {n.id for n in ast.walk(tree) if isinstance(n, ast.Name)}
    assert "move_servos" not in attributes
    assert "MotionToken" not in names | attributes
    # ``arm.set_positions`` is the only motion entry point it reaches for.
    assert "set_positions" in attributes


# ===========================================================================
# 2. Randomized positions -- a real success rate, not a lucky spot
# ===========================================================================

# Reachable band for the toy sim kinematics, kept clear of the joint limits so a
# failure means the *routine* failed, not that the position was unreachable:
#   x = (SHOULDER_PAN - 1500) * 0.25  -> 1200..1800 gives -75..+75 mm
#   y = 120 + (ELBOW - 1500) * 0.20   -> 1200..1800 gives  60..180 mm
REACHABLE_X = (-60.0, 60.0)
REACHABLE_Y = (80.0, 165.0)
RANDOM_TRIALS = 12


def test_success_rate_across_randomized_reachable_positions(calibrated):
    """Run a full pick at 12 independently drawn positions and report the rate.

    Seeded so it is reproducible, but the positions are genuinely spread across
    the reachable workspace rather than hand-picked. The assertion is a bound the
    observed behaviour actually justifies; if it ever drops, the honest response
    is to report the lower number, not to narrow the sampling.
    """
    rng = random.Random(20260728)
    outcomes = []
    for trial in range(RANDOM_TRIALS):
        x = rng.uniform(*REACHABLE_X)
        y = rng.uniform(*REACHABLE_Y)
        arm, world, locator = make_scene(default_object(x=x, y=y))
        try:
            result = pick.pick(arm, locator, fast_settings())
            outcomes.append({
                "trial": trial,
                "x": round(x, 1),
                "y": round(y, 1),
                "success": bool(result.success and world.held),
                "state": result.state.value,
                "reason": result.reason,
                "error_px": result.approach_error_px,
            })
            assert_pick_was_safe(arm._transport, expect_torque_off=not result.success)
        finally:
            arm.close()

    successes = sum(1 for o in outcomes if o["success"])
    rate = successes / len(outcomes)
    print(f"\nrandomized pick success rate: {successes}/{len(outcomes)} = {rate:.0%}")
    for o in outcomes:
        print(f"  trial {o['trial']:>2} ({o['x']:>6}, {o['y']:>6}) -> "
              f"{'OK ' if o['success'] else 'FAIL'} {o['state']}/{o['reason']}")

    assert len(outcomes) == RANDOM_TRIALS
    assert rate == 1.0, f"success rate dropped to {rate:.0%}: {[o for o in outcomes if not o['success']]}"


# Full joint travel, edges included:
#   x in [-75, +75] mm, y in [60, 180] mm.
FULL_X = (-75.0, 75.0)
FULL_Y = (60.0, 180.0)
FULL_TRIALS = 16


def test_success_rate_at_the_edges_of_joint_travel_degrades_honestly(calibrated):
    """The same sweep over the *whole* reachable band, edges and all.

    This one deliberately does not assert 100%, and the number it prints is
    lower. The reason is structural, not a tuning problem: the routine converges
    an *image* error, and the toy camera -- like any real one -- sees the raised
    gripper through some parallax, so aligning the two pixels puts the jaws about
    6 mm short of the object in the plane. Near the joint's own end stop there is
    no travel left to absorb that offset, and the outermost few millimetres of
    nominal reach are simply not pickable.

    Keeping this test separate from the in-band one is the point: the honest
    claim is "100% away from the limits, degrading at the edges", not a single
    averaged number that hides which half you are in. Every edge failure must
    still be a *clean* failure with a named reason and a safe arm.
    """
    rng = random.Random(4711)
    outcomes = []
    for trial in range(FULL_TRIALS):
        x = rng.uniform(*FULL_X)
        y = rng.uniform(*FULL_Y)
        arm, world, locator = make_scene(default_object(x=x, y=y))
        try:
            result = pick.pick(arm, locator, fast_settings())
            success = bool(result.success and world.held)
            outcomes.append((round(x, 1), round(y, 1), success, result.reason))
            if not success:
                # Failing at the edge is acceptable. Failing *messily* is not.
                assert result.state is PickState.FAILED
                assert result.unloaded is True
                assert world.held is False
            assert_pick_was_safe(arm._transport, expect_torque_off=not result.success)
        finally:
            arm.close()

    successes = sum(1 for o in outcomes if o[2])
    rate = successes / len(outcomes)
    print(f"\nfull-travel pick success rate: {successes}/{len(outcomes)} = {rate:.0%}")
    for x, y, ok, reason in outcomes:
        print(f"  ({x:>6}, {y:>6}) -> {'OK ' if ok else 'FAIL'} {reason}")

    # Every failure was the same, understood, geometric one.
    assert {r for _, _, ok, r in outcomes if not ok} <= {"approach_joint_limits_reached"}
    assert rate >= 0.75, f"edge behaviour got worse than the documented {rate:.0%}"


# ===========================================================================
# 3. Failure modes. Each one must end with a safe, defined arm.
# ===========================================================================


def test_object_out_of_reach_fails_without_thrashing(calibrated):
    """Visible on camera, geometrically unreachable: x=100 mm vs a +-75 mm reach."""
    arm, world, locator = make_scene(default_object(x=100.0, y=120.0))
    try:
        result = pick.pick(arm, locator, fast_settings())
        assert result.success is False
        assert result.state is PickState.FAILED
        # Pinned against the joint's own bound, and it says so rather than
        # burning the whole step budget pretending to make progress.
        assert result.reason == "approach_joint_limits_reached"
        assert world.held is False
        assert result.unloaded is True
        assert_pick_was_safe(arm._transport)
    finally:
        arm.close()


def test_object_not_found_fails_before_any_approach(calibrated):
    """An empty table. Not 'probably over there' -- nothing."""
    arm, world, locator = make_scene(None)  # no object in the scene at all
    try:
        result = pick.pick(arm, locator, fast_settings())
        assert result.success is False
        assert result.reason == "object_not_found"
        assert result.states_visited() == ["LOCATE", "FAILED"]
        assert_pick_was_safe(arm._transport)
    finally:
        arm.close()


def test_blind_locator_fails_cleanly(calibrated):
    arm, world, locator = make_scene(default_object())
    try:
        result = pick.pick(arm, NullLocator(world.frame_shape), fast_settings())
        assert result.success is False
        assert result.reason == "object_not_found"
        assert world.held is False
        assert_pick_was_safe(arm._transport)
    finally:
        arm.close()


def test_hallucinated_out_of_workspace_coordinates_are_rejected_not_acted_on(calibrated):
    """The documented ER failure mode: a confident point at nothing.

    The locator here deliberately does not cooperate -- it hands out (5000, -900).
    The routine must throw it away and report "not found", never servo toward it.
    """
    arm, world, locator = make_scene(default_object())
    garbage = OutOfBoundsLocator(frame_shape=world.frame_shape)
    try:
        result = pick.pick(arm, garbage, fast_settings())
        assert result.success is False
        assert result.reason == "object_not_found"
        rejections = [e for e in result.log if "rejected object location" in e.detail]
        assert rejections, "the bad point should be logged as rejected, not silently dropped"
        assert rejections[0].data["px"] == 5000.0
        # Nothing was ever commanded toward it.
        pan_targets = [p for sid, p in commanded_targets(arm._transport)
                       if sid == config.SERVO_IDS["SHOULDER_PAN"]]
        assert all(config.absolute_bounds("SHOULDER_PAN")[0] <= p <= config.absolute_bounds("SHOULDER_PAN")[1]
                   for p in pan_targets)
        assert_pick_was_safe(arm._transport)
    finally:
        arm.close()


def test_grasp_that_closes_on_nothing_is_not_reported_as_a_grasp(calibrated):
    """A grasp radius smaller than the routine's achievable precision.

    The jaws arrive over the object, converge in the image, close -- and slide
    past it, because a single camera's height parallax leaves a few mm of
    in-plane bias the image loop cannot see. The ladder then runs to its own
    backstop with no contact at all, which is exactly what "the grasp missed"
    looks like.
    """
    arm, world, locator = make_scene(default_object(grasp_radius_mm=0.5))
    try:
        result = pick.pick(arm, locator, fast_settings())
        assert result.success is False
        assert result.reason == "grasp_closed_on_nothing"
        assert result.grip_result["stopped_reason"] == "reached_backstop"
        assert result.grip_result["contact_position"] is None
        assert world.held is False
        assert world.object.grasp_count == 0
        assert result.unloaded is True
        assert_pick_was_safe(arm._transport)
    finally:
        arm.close()


def test_object_slipping_during_the_lift_is_caught_by_verify_held(calibrated):
    """Deterministic slip, forced by the sim object rather than left to chance."""
    arm, world, locator = make_scene(default_object(force_slip_on_lift=True))
    try:
        result = pick.pick(arm, locator, fast_settings())
        assert result.success is False
        assert result.state is PickState.FAILED
        assert result.reason == "verify_held_object_did_not_move"
        assert result.states_visited()[-2] == "VERIFY_HELD"
        # Ground truth: it really did slip, and it is back on the table.
        assert world.object.slipped is True
        assert world.held is False
        assert world.object.z == 0.0
        assert result.unloaded is True
        assert_pick_was_safe(arm._transport)
    finally:
        arm.close()


def test_object_too_heavy_to_hold_also_slips(calibrated):
    """The general 'too heavy' path, independent of the deterministic flag."""
    arm, world, locator = make_scene(default_object(weight_g=500.0), holding_capacity_g=80.0)
    try:
        result = pick.pick(arm, locator, fast_settings())
        assert result.success is False
        assert result.reason.startswith("verify_held_")
        assert world.object.slipped is True
        assert_pick_was_safe(arm._transport)
    finally:
        arm.close()


def test_visual_servoing_that_never_converges_trips_the_guard(calibrated):
    """A detector locked onto a static point: the gripper never appears to move.

    The loop must notice the error is not decreasing and stop, rather than
    spending its whole step budget (or hanging) on a hopeless correction.
    """
    arm, world, locator = make_scene(default_object())
    # Three points, so the Jacobian probe still measures a non-singular (if
    # entirely fictitious) sensitivity and the run actually reaches the servo
    # loop -- a detector that reports one frozen point is caught earlier, as a
    # degenerate Jacobian, which is covered by its own test below.
    flapping = OscillatingLocator(
        object_point=(200.0, 150.0),
        gripper_points=[(430.0, 330.0), (410.0, 300.0), (400.0, 350.0)],
        frame_shape=world.frame_shape,
    )
    settings = fast_settings(max_approach_steps=30, convergence_patience=2)
    try:
        result = pick.pick(arm, flapping, settings)
        assert result.success is False
        assert result.reason == "approach_no_convergence", result.reason
        # It gave up early rather than exhausting 30 steps.
        approach_steps = [e for e in result.log
                          if e.state == "APPROACH" and e.detail == "observed error"]
        assert len(approach_steps) < settings.max_approach_steps
        assert_pick_was_safe(arm._transport)
    finally:
        arm.close()


def test_convergence_guard_distinguishes_stuck_from_merely_slow(calibrated):
    """'Ran out of steps while still improving' must not be reported as divergence."""
    arm, world, locator = make_scene(default_object(x=55.0, y=160.0))
    # A tiny gain makes every step a real but small improvement; with a step
    # budget of 2 it cannot converge, but it never stops improving either.
    settings = fast_settings(gain=0.05, max_approach_steps=2, convergence_patience=3)
    try:
        result = pick.pick(arm, locator, settings)
        assert result.success is False
        assert result.reason == "approach_step_budget_exhausted"
        assert_pick_was_safe(arm._transport)
    finally:
        arm.close()


def test_degenerate_sensitivity_is_detected_rather_than_inverted(calibrated):
    """Two joints that move the gripper the same way cannot resolve a 2D error."""
    arm, world, locator = make_scene(default_object())
    settings = fast_settings(servo_joints=("SHOULDER_PAN", "SHOULDER_PAN"))
    try:
        result = pick.pick(arm, locator, settings)
        assert result.success is False
        assert result.reason == "degenerate_jacobian"
        assert_pick_was_safe(arm._transport)
    finally:
        arm.close()


# ===========================================================================
# 4. Every state is individually abortable, and aborting is always safe
# ===========================================================================


@pytest.mark.parametrize("state", [s.value for s in pick.ABORTABLE_STATES])
def test_every_state_can_be_aborted_cleanly(calibrated, state):
    arm, world, locator = make_scene(default_object())
    try:
        result = pick.pick(arm, locator, fast_settings(abort_states=(state,)))
        assert result.success is False
        assert result.state is PickState.ABORTED
        assert result.reason == f"aborted at {state} (injected)"
        assert result.unloaded is True
        assert world.held is False, "an abort must not leave the object clamped"
        # Aborting at LOCATE happens before the first packet is sent, which is
        # the whole point of putting the gate there.
        assert_pick_was_safe(arm._transport, expect_moves=state != "LOCATE")
    finally:
        arm.close()


def test_aborting_after_the_grasp_releases_the_object(calibrated):
    """Abort at VERIFY: the jaws are loaded, so cleanup must open them first."""
    arm, world, locator = make_scene(default_object())
    try:
        result = pick.pick(arm, locator, fast_settings(abort_states=("VERIFY",)))
        assert result.state is PickState.ABORTED
        assert world.held is False
        assert world.object.release_count >= 1
        # The gripper was walked back open, not just de-energised.
        grip_targets = [p for sid, p in commanded_targets(arm._transport) if sid == GRIP_SERVO_ID]
        assert min(grip_targets) <= config.GRIP_MIN_SAFE
        assert_pick_was_safe(arm._transport)
    finally:
        arm.close()


# ===========================================================================
# 5. The calibration gate
# ===========================================================================


def test_refuses_to_run_on_an_uncalibrated_arm(monkeypatch):
    """The shipped config has every joint uncalibrated. That must be a hard stop."""
    monkeypatch.setattr(config, "MIN_DURATION_MS", 20)
    assert all(not config.JOINT_LIMITS[n].calibrated for n in ("SHOULDER_PAN", "ELBOW", "GRIP"))
    arm, world, locator = make_scene(default_object())
    try:
        with pytest.raises(PickRefused) as excinfo:
            pick.pick(arm, locator, fast_settings())
        message = str(excinfo.value)
        assert "not calibrated" in message
        assert "docs/calibration.md" in message
        # Refused BEFORE anything moved -- not partway through.
        assert [e for e in arm._transport.log if e["cmd"] == "MOVE"] == []
        assert arm._transport.any_torque_on() is False
    finally:
        arm.close()


def test_runs_with_an_explicit_override_and_a_reason(monkeypatch):
    monkeypatch.setattr(config, "MIN_DURATION_MS", 20)
    monkeypatch.setattr(config, "SETTLE_MARGIN_MS", 0)
    arm, world, locator = make_scene(default_object())
    try:
        result = pick.pick(
            arm, locator, fast_settings(),
            allow_uncalibrated_override=True,
            override_reason="supervised bench test, operator present",
        )
        assert result.success is True
        assert world.held is True
        # The override is audited, once per move, with the reason attached.
        assert arm._safety.override_log
        assert all(e["reason"] == "supervised bench test, operator present"
                   for e in arm._safety.override_log)
        assert_pick_was_safe(arm._transport, expect_torque_off=False)
    finally:
        arm.close()


def test_override_without_a_reason_is_rejected(monkeypatch):
    monkeypatch.setattr(config, "MIN_DURATION_MS", 20)
    arm, world, locator = make_scene(default_object())
    try:
        with pytest.raises(ValueError, match="override_reason"):
            pick.pick(arm, locator, fast_settings(), allow_uncalibrated_override=True)
        assert [e for e in arm._transport.log if e["cmd"] == "MOVE"] == []
    finally:
        arm.close()


def test_refuses_without_a_motion_observer(calibrated, monkeypatch):
    """No camera means the ladder cannot tell contact from empty air."""
    arm, world, locator = make_scene(default_object())
    monkeypatch.setattr(arm, "sim", False)  # pretend this is real hardware, no camera
    try:
        with pytest.raises(PickRefused, match="motion observer"):
            pick.pick(arm, locator, fast_settings())
        assert [e for e in arm._transport.log if e["cmd"] == "MOVE"] == []
    finally:
        monkeypatch.undo()
        arm.close()


def test_refuses_an_unavailable_locator(calibrated):
    from robohand.locate import UnavailableLocator

    arm, world, locator = make_scene(default_object())
    try:
        with pytest.raises(PickRefused, match="not available"):
            pick.pick(arm, UnavailableLocator(), fast_settings())
        assert [e for e in arm._transport.log if e["cmd"] == "MOVE"] == []
    finally:
        arm.close()


# ===========================================================================
# 6. Result object is diagnosable on its own
# ===========================================================================


def test_failed_pick_can_be_diagnosed_from_the_result_alone(calibrated):
    arm, world, locator = make_scene(default_object(x=100.0, y=120.0))
    try:
        result = pick.pick(arm, locator, fast_settings())
        payload = result.as_dict()
        assert payload["success"] is False
        assert payload["state"] == "FAILED"
        assert payload["reason"]
        assert payload["locator"]
        assert len(payload["log"]) > 3
        for entry in payload["log"]:
            assert set(entry) == {"state", "detail", "timestamp", "data"}
            assert entry["state"] in {s.value for s in PickState}
        # The measured sensitivities are in the log, so a bad Jacobian is
        # diagnosable without re-running anything.
        probes = [e for e in payload["log"] if e["detail"] == "measured joint sensitivity"]
        assert len(probes) == 2
        assert "dpx_per_unit" in probes[0]["data"]
        # And so is the error trajectory.
        errors = [e["data"]["error_px"] for e in payload["log"] if e["detail"] == "observed error"]
        assert errors and all(isinstance(e, float) for e in errors)
    finally:
        arm.close()


def test_required_joints_are_exactly_what_the_routine_moves(calibrated):
    settings = fast_settings()
    assert pick.required_joints(settings) == ["SHOULDER_PAN", "ELBOW", "SHOULDER_LIFT", "GRIP"]

    arm, world, locator = make_scene(default_object())
    try:
        pick.pick(arm, locator, settings)
        moved = {config.ID_TO_NAME[sid] for sid, _ in commanded_targets(arm._transport)}
        assert moved <= set(pick.required_joints(settings))
    finally:
        arm.close()


def test_descend_and_lift_are_recorded_as_open_loop(calibrated):
    """The docs claim height is open loop; the log has to say the same thing."""
    arm, world, locator = make_scene(default_object())
    try:
        result = pick.pick(arm, locator, fast_settings())
        details = [e.detail for e in result.log]
        assert any("open loop" in d and "descend" in d.lower() for d in details)
        assert any("open loop" in d and "lift" in d.lower() for d in details)
    finally:
        arm.close()
