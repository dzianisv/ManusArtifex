"""Layer 1 proof: a large move is split into a ramp of smaller steps, none of which
exceeds ``max_step_per_move``.

Uses a calibrated joint (constructed directly via a temp config-like limit is not
necessary here -- GRIP's provisional band is 1200..1800, a full 600-unit span, which
is exactly the "600-unit move" scenario called out in the spec) so the request
itself is not refused by the calibration check; the point under test is the ramp
step count and step size, not calibration.
"""

from __future__ import annotations

from robohand.safety import SafetyLayer
from robohand.sim import SimTransport

GRIP_SERVO_ID = 1


def test_large_move_is_ramped():
    transport = SimTransport()
    safety = SafetyLayer(transport, max_step_per_move=100, install_signal_handlers=False)
    try:
        # 1200 -> 1800 is a 600-unit move, entirely within GRIP's configured
        # 1200..1800 band (so no calibration refusal), and larger than the
        # end-to-end mechanical stop margin, so any failure to ramp would be
        # immediately visible as one huge single-step jump.
        safety.move({"GRIP": 1800}, current={"GRIP": 1200}, duration_ms=1000, wait=False)
    finally:
        safety.close()

    move_entries = [entry for entry in transport.log if entry["cmd"] == "MOVE"]
    assert len(move_entries) > 1, "a 600-unit move with max_step_per_move=100 must be split into multiple packets"

    positions = [1200] + [entry["achieved"][GRIP_SERVO_ID] for entry in move_entries]
    deltas = [abs(b - a) for a, b in zip(positions, positions[1:])]
    assert all(d <= 100 for d in deltas), f"a ramp step exceeded max_step_per_move: {deltas}"
    assert positions[-1] == 1800, "the ramp must still reach the final target"
    # No step should ever have touched the mechanical stop (1150/1850) or beyond.
    assert transport.stall_ms_total() == 0


def test_small_move_is_a_single_step():
    """Sanity check: a move smaller than max_step_per_move should not be split."""
    transport = SimTransport()
    safety = SafetyLayer(transport, max_step_per_move=100, install_signal_handlers=False)
    try:
        safety.move({"GRIP": 1250}, current={"GRIP": 1200}, duration_ms=400, wait=False)
    finally:
        safety.close()

    move_entries = [entry for entry in transport.log if entry["cmd"] == "MOVE"]
    assert len(move_entries) == 1
