"""Layer 1 proof: the exact incident (gripper commanded to a raw extreme that lies
beyond the mechanical end stop) cannot happen anymore.

Corrected understanding (2026-07-28 follow-up research, see docs/research-hardware.md):
GRIP (servo id 1) is a LeArm-family gripper servo whose *own* valid range is
1500..2500, not the generic 500..2500 used by every other joint -- confirmed by the
reference xArmServoController firmware
(``case LeArm: if (servo == 1) return clamp(value, 1500, 2500);``) and independently
by a LeArm owner (ccourson/Hiwonder-xArm1S#5). Lower raw = MORE OPEN; 1500 is the true
open mechanical/electrical stop. The original incident command was ``close_grip=700``
-- 800 units below that floor -- which drove the servo hard past the open stop and
held it there at locked-rotor torque; that is exactly what the operator's video shows
(jaws splayed wide open, not closed). ``2200`` was the other raw value flagged in the
original incident report and must independently be proven safe even though it did not
itself cause the damage.

This test replays both raw values directly against ``SafetyLayer`` (bypassing any
pose file) and asserts the simulator records **zero** stall milliseconds -- either
because the request was refused outright (current behaviour: GRIP is uncalibrated and
both values are outside its provisional band) or, if a future change relaxes
calibration requirements, because clamping still keeps the target inside a band that
never reaches the mechanical stop. Either way the servo must never be commanded to a
raw value beyond the mechanical stop.
"""

from __future__ import annotations

import pytest

from robohand import config
from robohand.safety import CalibrationRequired, SafetyLayer
from robohand.sim import SimTransport

INCIDENT_CLOSE = 700  # the value that actually caused the 2026-07-28 damage
INCIDENT_OPEN = 2200  # the other raw value named in the original incident report


def test_gripper_cannot_reach_end_stop():
    """700 (the value that actually caused the incident) is 800 units below GRIP's
    provisional band (1650..2250) and is refused outright. 2200 is now, under the
    corrected 1500..2500 range, a perfectly ordinary in-band request (it sits well
    inside the provisional band) -- proving 2200 alone was never the dangerous
    value is itself part of the point of this correction. Either way, neither value
    may ever cause the simulated servo to accrue locked-rotor stall time.
    """
    transport = SimTransport()
    safety = SafetyLayer(transport, install_signal_handlers=False)
    try:
        with pytest.raises(CalibrationRequired):
            safety.move({"GRIP": INCIDENT_CLOSE}, duration_ms=600, wait=False)
        # Does not raise: 2200 is inside GRIP's provisional band under the
        # corrected 1500..2500 range, so it is legitimately allowed -- and must
        # still produce zero stall (it never approaches either mechanical stop).
        safety.move({"GRIP": INCIDENT_OPEN}, duration_ms=600, wait=False)
    finally:
        safety.close()

    # The proof: no matter how each request was handled, the simulated servo never
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


def test_gripper_floor_enforced():
    """Direct requests for servo 1 (GRIP) at 700, 1000, and 1499 -- all below the
    true 1500 mechanical/electrical floor -- must never reach the transport as
    commanded. Without an override they are refused outright (nothing sent at all).
    With an explicit override they are still clamped up to the configured floor
    (1600), never anywhere near 1500, let alone below it.
    """
    below_floor_values = [700, 1000, 1499]
    assert all(v < 1500 for v in below_floor_values)

    # -- without override: refused, and NOTHING is ever sent to the transport ----
    transport = SimTransport()
    safety = SafetyLayer(transport, install_signal_handlers=False)
    try:
        for v in below_floor_values:
            with pytest.raises(CalibrationRequired):
                safety.move({"GRIP": v}, duration_ms=400, wait=False)
    finally:
        safety.close()
    move_entries = [e for e in transport.log if e.get("cmd") == "MOVE"]
    assert move_entries == [], f"expected zero MOVE packets sent, got {move_entries}"
    assert transport.stall_ms_total() == 0

    # -- with an explicit, logged override: clamped, never below the 1500 floor --
    transport2 = SimTransport()
    safety2 = SafetyLayer(transport2, install_signal_handlers=False)
    try:
        for v in below_floor_values:
            safety2.move(
                {"GRIP": v},
                duration_ms=400,
                wait=False,
                allow_uncalibrated_override=True,
                override_reason="test: prove the absolute floor holds even under override",
            )
    finally:
        safety2.close()
    move_entries2 = [e for e in transport2.log if e.get("cmd") == "MOVE"]
    assert move_entries2, "expected the override to still send *some* clamped move"
    for entry in move_entries2:
        for servo_id, target in entry["targets"]:
            if servo_id == config.SERVO_IDS["GRIP"]:
                assert target >= 1500, f"GRIP was sent {target}, below the absolute 1500 floor"
                assert target >= config.GRIP_MIN_SAFE, (
                    f"GRIP was sent {target}, below the configured safe floor "
                    f"{config.GRIP_MIN_SAFE}"
                )
    assert transport2.stall_ms_total() == 0
