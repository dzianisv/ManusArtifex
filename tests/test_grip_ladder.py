"""Gripper-specific proof: the incremental grip ladder stops the instant a
simulated object blocks it, backs off one increment, and unloads -- with **zero**
recorded stall time, because contact with an obstruction is normal gripper
behaviour, not a destructive locked-rotor stall (see ``sim.SimJoint.command``'s
distinction between ``mech_min/max`` and ``obstruction_min/max``).

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
        # Simulate an object in the jaws that blocks travel well short of the
        # mechanical end stop (1150) -- e.g. a rigid object at 1400.
        arm._transport.set_obstruction(GRIP_SERVO_ID, min=1400)
        arm.vision_guard = VisionGuard(
            frame_source=_make_frame_source(arm),
            min_changed_pixels=20,
        )
        # Start from a known, safely-open position so the ladder has room to close.
        arm.set_position("GRIP", 1700, hold=["GRIP"], wait=False)

        result = arm.close_grip(step=40)

    assert result["stopped_reason"] == "no_motion_detected", result
    # The ladder must have actually made progress toward closing before stopping.
    assert result["final_position"] < 1700
    # Note: "final_position"/"contact_position" are *commanded* values, not
    # feedback-verified physical positions -- there is no position feedback on this
    # hardware (see protocol.NoResponse), so the sim's obstruction may silently clamp
    # a requested target short of what was asked, exactly as the real servo would.
    # The invariant we can actually assert is direction: backing off from a "close"
    # ladder must land at a *less closed* (larger) commanded position than the one
    # where motion ceased.
    assert result["final_position"] > result["contact_position"]
    # The whole point: contact with an object is NOT a stall.
    assert arm._transport.stall_ms_total() == 0
    assert arm._transport.joints[GRIP_SERVO_ID].torque_on is False


def test_grip_ladder_respects_hard_backstop_without_vision():
    """Even with no vision guard at all (degrade-safely case), the ladder must never
    drive past ``GRIP_MIN_SAFE`` / ``GRIP_MAX_SAFE`` -- the hard backstop is
    independent of whether a camera is attached.
    """
    from robohand import config

    with Arm(sim=True, use_vision=False, install_signal_handlers=False) as arm:
        assert arm.vision_guard is None
        arm.set_position("GRIP", 1700, hold=["GRIP"], wait=False)
        result = arm.close_grip(step=40)

    assert result["final_position"] == config.GRIP_MIN_SAFE
    assert result["final_position"] >= 1150  # strictly inside the mechanical stop
    assert arm._transport.stall_ms_total() == 0


def test_grip_ladder_open_direction_stops_on_contact():
    with Arm(sim=True, use_vision=False, install_signal_handlers=False) as arm:
        arm._transport.set_obstruction(GRIP_SERVO_ID, max=1600)
        arm.vision_guard = VisionGuard(
            frame_source=_make_frame_source(arm),
            min_changed_pixels=20,
        )
        arm.set_position("GRIP", 1300, hold=["GRIP"], wait=False)

        result = arm.open_grip(step=40)

    assert result["stopped_reason"] == "no_motion_detected", result
    assert result["final_position"] > 1300
    assert arm._transport.stall_ms_total() == 0
