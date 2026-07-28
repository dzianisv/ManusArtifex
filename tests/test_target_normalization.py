"""Regression tests for transport target normalisation.

Found during adversarial verification of the range-enforcement pass: ``Arm.set_positions``
speaks dicts, but both transports documented ``Sequence[tuple]``. Passing a dict made the
range backstop blow up with an obscure ``TypeError`` from inside a packet-building loop
instead of a clear ``IllegalServoTarget``. It failed closed (no motion), but the error was
misleading, and a generator would have been consumed by the range check and then written
out as an empty/short packet. ``normalize_targets`` closes both.
"""

import pytest

from robohand.protocol import IllegalServoTarget, normalize_targets
from robohand.arm import Arm


ILLEGAL_GRIP = 700  # the exact value that stalled the real gripper on 2026-07-28
LEGAL_GRIP = 1900


@pytest.fixture
def rig():
    arm = Arm(sim=True, install_signal_handlers=False)
    return arm._safety, arm._transport


def test_normalize_accepts_mapping_and_pairs():
    assert normalize_targets({1: 1900}) == [(1, 1900)]
    assert normalize_targets([(1, 1900), (2, 1500)]) == [(1, 1900), (2, 1500)]


@pytest.mark.parametrize(
    "bad",
    [
        42,                      # not iterable at all
        "1,1900",                # a string is a Sequence but never a target list
        [(1, 1900, 7)],          # wrong pair arity
        [1900],                  # bare ints, not pairs
        ((1, p) for p in [1900]),  # one-shot generator: would be consumed by the range check
    ],
)
def test_normalize_rejects_junk(bad):
    with pytest.raises(IllegalServoTarget):
        normalize_targets(bad)


@pytest.mark.parametrize(
    "targets",
    [
        {1: ILLEGAL_GRIP},
        [(1, ILLEGAL_GRIP)],
        ((1, p) for p in [ILLEGAL_GRIP]),
        42,
        [(1, ILLEGAL_GRIP, 9)],
    ],
)
def test_transport_refuses_illegal_targets_in_any_form(rig, targets):
    """Even with a legitimately minted, single-use token, nothing illegal reaches the wire."""
    safety, transport = rig
    with pytest.raises(IllegalServoTarget):
        transport.move_servos(targets, 500, token=safety._mint_token())
    assert all(j.stall_ms == 0.0 for j in transport.joints.values())


@pytest.mark.parametrize("targets", [{1: LEGAL_GRIP}, [(1, LEGAL_GRIP)]])
def test_transport_accepts_legal_targets_in_either_form(rig, targets):
    safety, transport = rig
    transport.move_servos(targets, 500, token=safety._mint_token())
    assert transport.joints[1].position == LEGAL_GRIP
    assert transport.joints[1].stall_ms == 0.0
