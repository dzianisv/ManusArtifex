"""Layer 1 proof: a large move is split into a ramp of smaller steps, none of which
exceeds ``max_step_per_move``.

Uses GRIP (still uncalibrated) but picks a start/target pair that stays entirely
inside its *provisional* band (center 1950 +/- 300 = 1650..2250, a 600-unit span --
exactly the "600-unit move" scenario called out in the spec) so the request itself
is not refused by the calibration check; the point under test is the ramp step
count and step size, not calibration. See docs/research-hardware.md for why GRIP's
real valid range is 1600..2300, not the generic 500..2500 used by every other joint.
"""

from __future__ import annotations

from robohand.safety import SafetyLayer
from robohand.sim import SimTransport

GRIP_SERVO_ID = 1


def test_large_move_is_ramped():
    transport = SimTransport()
    safety = SafetyLayer(transport, max_step_per_move=100, install_signal_handlers=False)
    try:
        # 1650 -> 2250 is a 600-unit move, entirely within GRIP's provisional band
        # (1650..2250, so no calibration refusal) and larger than the end-to-end
        # mechanical stop margin (1500..2500), so any failure to ramp would be
        # immediately visible as one huge single-step jump.
        safety.move({"GRIP": 2250}, current={"GRIP": 1650}, duration_ms=1000, wait=False)
    finally:
        safety.close()

    move_entries = [entry for entry in transport.log if entry["cmd"] == "MOVE"]
    assert len(move_entries) > 1, "a 600-unit move with max_step_per_move=100 must be split into multiple packets"

    positions = [1650] + [entry["achieved"][GRIP_SERVO_ID] for entry in move_entries]
    deltas = [abs(b - a) for a, b in zip(positions, positions[1:])]
    assert all(d <= 100 for d in deltas), f"a ramp step exceeded max_step_per_move: {deltas}"
    assert positions[-1] == 2250, "the ramp must still reach the final target"
    # No step should ever have touched the mechanical stop (1500/2500) or beyond.
    assert transport.stall_ms_total() == 0


def test_small_move_is_a_single_step():
    """Sanity check: a move smaller than max_step_per_move should not be split."""
    transport = SimTransport()
    safety = SafetyLayer(transport, max_step_per_move=100, install_signal_handlers=False)
    try:
        safety.move({"GRIP": 1700}, current={"GRIP": 1650}, duration_ms=400, wait=False)
    finally:
        safety.close()

    move_entries = [entry for entry in transport.log if entry["cmd"] == "MOVE"]
    assert len(move_entries) == 1
