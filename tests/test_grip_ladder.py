"""Gripper-specific proof: the incremental grip ladder stops the instant a
simulated object blocks it, backs off one increment, and unloads -- with **zero**
recorded stall time, because contact with an obstruction is normal gripper
behaviour, not a destructive locked-rotor stall (see ``sim.SimJoint.command``'s
distinction between ``mech_min/max`` and ``obstruction_min/max``).

Direction note (see docs/research-hardware.md): GRIP is a LeArm gripper servo where
LOWER raw = MORE OPEN. So "close" INCREASES the raw value toward grip_max_safe
(2300, capped by the provisional band to 2250 while uncalibrated) and "open"
DECREASES it toward grip_min_safe (1600, floored by the provisional band to 1650
while uncalibrated). Getting this backwards was the original incident.

The synthetic camera frame source renders a single vertical bar whose x-position
tracks the sim's own GRIP position, so frame differencing genuinely reflects
"did the simulated jaw move" -- once ``SimJoint.command`` clamps to the obstruction
and stops advancing, consecutive frames become identical and the vision guard
correctly reports no motion.
"""

from __future__ import annotations

import numpy as np

from robohand.arm import Arm
from robohand.guard import VisionGuard

GRIP_SERVO_ID = 1
FRAME_SIZE = 120


def _make_frame_source(arm: Arm):
    """A fake camera: renders GRIP's *actual simulated* position as a moving bar."""
    joint = arm._transport.joints[GRIP_SERVO_ID]

    def grab() -> np.ndarray:
        frame = np.zeros((FRAME_SIZE, FRAME_SIZE, 3), dtype=np.uint8)
        span = joint.mech_max - joint.mech_min
        x = int((joint.position - joint.mech_min) / span * (FRAME_SIZE - 20)) + 5
        frame[:, x : x + 10] = 255
        return frame

    return grab


def test_grip_ladder_stops_on_contact():
    with Arm(sim=True, use_vision=False, install_signal_handlers=False) as arm:
        # Simulate an object in the jaws that blocks further CLOSING (increasing
        # raw value) well short of both the ladder's own backstop (2250, while
        # uncalibrated) and the true mechanical end stop (2500) -- e.g. a rigid
        # object making contact at 2000.
        arm._transport.set_obstruction(GRIP_SERVO_ID, max=2000)
        arm.vision_guard = VisionGuard(
            frame_source=_make_frame_source(arm),
            min_changed_pixels=20,
        )
        # Start from a known, safely-open-ish position so the ladder has room to
        # close (increase) before hitting the obstruction.
        arm.set_position("GRIP", 1700, hold=["GRIP"], wait=False)

        result = arm.close_grip(step=40)

    assert result["stopped_reason"] == "no_motion_detected", result
    # The ladder must have actually made progress toward closing (increasing) before
    # stopping.
    assert result["final_position"] > 1700
    # Note: "final_position"/"contact_position" are *commanded* values, not
    # feedback-verified physical positions -- there is no position feedback on this
    # hardware (see protocol.NoResponse), so the sim's obstruction may silently clamp
    # a requested target short of what was asked, exactly as the real servo would.
    # The invariant we can actually assert is direction: backing off from a "close"
    # ladder (which increases) must land at a *less closed* (smaller) commanded
    # position than the one where motion ceased.
    assert result["final_position"] < result["contact_position"]
    # The whole point: contact with an object is NOT a stall.
    assert arm._transport.stall_ms_total() == 0
    assert arm._transport.joints[GRIP_SERVO_ID].torque_on is False


def test_grip_ladder_respects_hard_backstop_without_vision():
    """Even with no vision guard at all (degrade-safely case), the ladder must never
    drive past its hard backstop -- the tighter of ``grip_max_safe`` and GRIP's
    current calibration-aware effective bound (the provisional band while
    uncalibrated: 2250, not the full configured 2300) -- independent of whether a
    camera is attached.
    """
    from robohand import config

    with Arm(sim=True, use_vision=False, install_signal_handlers=False) as arm:
        assert arm.vision_guard is None
        arm.set_position("GRIP", 1700, hold=["GRIP"], wait=False)
        result = arm.close_grip(step=40)

    eff_lo, eff_hi = config.effective_bounds("GRIP")
    expected_backstop = min(config.GRIP_MAX_SAFE, eff_hi)
    assert result["final_position"] == expected_backstop
    assert result["final_position"] < 2500  # strictly inside the mechanical stop
    assert arm._transport.stall_ms_total() == 0


def test_grip_ladder_open_direction_stops_on_contact():
    with Arm(sim=True, use_vision=False, install_signal_handlers=False) as arm:
        # Blocks further OPENING (decreasing raw value) well short of both the
        # ladder's own backstop (1650, while uncalibrated) and the true mechanical
        # open stop (1500).
        arm._transport.set_obstruction(GRIP_SERVO_ID, min=1900)
        arm.vision_guard = VisionGuard(
            frame_source=_make_frame_source(arm),
            min_changed_pixels=20,
        )
        # Start from a known, safely-closed-ish position so the ladder has room to
        # open (decrease) before hitting the obstruction.
        arm.set_position("GRIP", 2200, hold=["GRIP"], wait=False)

        result = arm.open_grip(step=40)

    assert result["stopped_reason"] == "no_motion_detected", result
    assert result["final_position"] < 2200
    # Backing off from an "open" ladder (which decreases) must land at a *less open*
    # (larger) commanded position than the one where motion ceased.
    assert result["final_position"] > result["contact_position"]
    assert arm._transport.stall_ms_total() == 0
