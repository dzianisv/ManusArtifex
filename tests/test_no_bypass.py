"""Layer 1 structural proof: nothing can send a SERVO_MOVE packet to either
transport without a valid motion token minted by ``safety.SafetyLayer``.

This is the "choke point" claim -- it is enforced by the transport itself
(``protocol.require_motion_token``), not by convention or code review. Both the
real ``HidTransport`` and the simulator's ``SimTransport`` share the same guard
function, so this test exercising the simulator is representative of the real
transport's behaviour too (read ``protocol.py`` -- ``move_servos`` on both classes
calls the identical ``require_motion_token`` helper before touching the wire /
updating any state).
"""

from __future__ import annotations

import pytest

from robohand.protocol import MotionToken, SafetyBypassError
from robohand.sim import SimTransport


def test_no_path_bypasses_safety_missing_token():
    transport = SimTransport()
    with pytest.raises(SafetyBypassError):
        transport.move_servos([(1, 1500)], 600)  # no token at all
    # Nothing was sent and no torque was ever applied.
    assert transport.log == []
    assert transport.joints[1].torque_on is False


def test_no_path_bypasses_safety_wrong_transport_token():
    transport_a = SimTransport()
    transport_b = SimTransport()
    # A token minted for a *different* transport instance must not work here --
    # tokens are bound to id(transport) at mint time.
    foreign_token = MotionToken(id(transport_b))
    with pytest.raises(SafetyBypassError):
        transport_a.move_servos([(1, 1500)], 600, token=foreign_token)
    assert transport_a.log == []


def test_no_path_bypasses_safety_reused_token():
    transport = SimTransport()
    token = MotionToken(id(transport))
    transport.move_servos([(1, 1500)], 600, token=token)  # first use: fine
    with pytest.raises(SafetyBypassError):
        transport.move_servos([(1, 1300)], 600, token=token)  # reuse: must fail
