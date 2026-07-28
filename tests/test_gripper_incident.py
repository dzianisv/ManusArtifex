"""Layer 1 proof: the exact incident (gripper commanded to a raw extreme that lies
beyond the mechanical end stop) cannot happen anymore.

The original failure: commanding the GRIPPER servo to 700 (fully closed) or 2200
(fully open) drove the jaws into their mechanical end stop (~1150/~1850 in the
simulator's model) and held locked-rotor torque for the whole commanded duration.

This test replays those exact raw values directly against ``SafetyLayer`` (bypassing
any pose file) and asserts the simulator records **zero** stall milliseconds --
either because the request was refused outright (current behaviour: GRIP is
uncalibrated and both values are outside its provisional band) or, if a future
change relaxes calibration requirements, because clamping still keeps the target
inside a band that never reaches the mechanical stop. Either way the servo must
never be commanded to a raw value beyond the mechanical stop.
"""

from __future__ import annotations

import pytest

from robohand.safety import CalibrationRequired, SafetyLayer
from robohand.sim import SimTransport

INCIDENT_CLOSE = 700  # raw value that drove the jaws into the fully-closed stop
INCIDENT_OPEN = 2200  # raw value that drove the jaws into the fully-open stop


def test_gripper_cannot_reach_end_stop():
    transport = SimTransport()
    safety = SafetyLayer(transport, install_signal_handlers=False)
    try:
        for incident_target in (INCIDENT_CLOSE, INCIDENT_OPEN):
            with pytest.raises(CalibrationRequired):
                safety.move({"GRIP": incident_target}, duration_ms=600, wait=False)
    finally:
        safety.close()

    # The proof: no matter how the request was handled, the simulated servo never
    # accrued locked-rotor stall time.
    assert transport.stall_ms_total() == 0
    assert transport.joints[1].stall_ms == 0
    assert transport.joints[1].blocked is False


def test_gripper_cannot_reach_end_stop_even_with_override():
    """Even an explicit, logged override cannot exceed the *configured* band, which
    sits strictly inside the mechanical stop -- the override widens calibration
    trust, not the physical safety margin.
    """
    transport = SimTransport()
    safety = SafetyLayer(transport, install_signal_handlers=False)
    try:
        for incident_target in (INCIDENT_CLOSE, INCIDENT_OPEN):
            safety.move(
                {"GRIP": incident_target},
                duration_ms=600,
                wait=False,
                allow_uncalibrated_override=True,
                override_reason="test: prove the override still cannot reach the end stop",
            )
    finally:
        safety.close()

    assert transport.stall_ms_total() == 0
