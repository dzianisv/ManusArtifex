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

from robohand.safety import CalibrationRequired, PositionOutOfRange, SafetyLayer
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
    trust, not the physical safety margin. A request that is still outside the
    configured band is refused outright (never silently redirected to a clamped
    value) -- 700 is 900 units below GRIP's configured floor (1600) and must raise
    even with the override, while 2200 sits comfortably inside the configured band
    and must succeed normally.
    """
    transport = SimTransport()
    safety = SafetyLayer(transport, install_signal_handlers=False)
    try:
        with pytest.raises(PositionOutOfRange):
            safety.move(
                {"GRIP": INCIDENT_CLOSE},
                duration_ms=600,
                wait=False,
                allow_uncalibrated_override=True,
                override_reason="test: prove the override still cannot reach the end stop",
            )
        safety.move(
            {"GRIP": INCIDENT_OPEN},
            duration_ms=600,
            wait=False,
            allow_uncalibrated_override=True,
            override_reason="test: prove an in-band override request still succeeds",
        )
    finally:
        safety.close()

    move_entries = [e for e in transport.log if e.get("cmd") == "MOVE"]
    assert len(move_entries) == 1, "the refused 700 request must never have reached the transport"
    assert transport.stall_ms_total() == 0


def test_gripper_floor_enforced():
    """Direct requests for servo 1 (GRIP) at 700, 1000, and 1499 -- all below the
    true 1500 mechanical/electrical floor -- must never reach the transport as
    commanded, with or without an override. Without an override they are refused
    as a calibration violation; with an override they are still refused (never
    silently clamped up to some other value) because they remain below GRIP's own
    configured floor (1600) -- an override widens which policy band applies, it
    never fabricates a legal value out of an illegal one.
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

    # -- with an explicit, logged override: STILL refused, nothing sent -----------
    transport2 = SimTransport()
    safety2 = SafetyLayer(transport2, install_signal_handlers=False)
    try:
        for v in below_floor_values:
            with pytest.raises(PositionOutOfRange):
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
    assert move_entries2 == [], (
        f"an override must never rescue a value below GRIP's own configured floor "
        f"by clamping it to a different value -- expected zero MOVE packets, got {move_entries2}"
    )
    assert transport2.stall_ms_total() == 0
