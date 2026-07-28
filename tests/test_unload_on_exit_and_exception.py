"""Layer 2 proof, part 1: ``SafetyLayer`` unloads all servos on every exit path
except process kill signals (covered separately in ``test_unload_on_sigint.py``):
normal ``close()``/context-manager exit, and an unhandled exception propagating out
of a ``with`` block.

We command a move with ``hold=[...]`` (i.e. "keep torque on, don't auto-unload after
``hold_ms``") specifically so that if the exit-path unload logic were missing, the
joint would still show ``torque_on is True`` after the ``with`` block -- making these
tests actually able to fail against a design that forgot this path.
"""

from __future__ import annotations

import pytest

from robohand.safety import SafetyLayer
from robohand.sim import SimTransport

GRIP_SERVO_ID = 1


def test_unload_on_exit():
    transport = SimTransport()
    with SafetyLayer(transport, install_signal_handlers=False) as safety:
        safety.move({"GRIP": 1700}, duration_ms=200, hold=["GRIP"], wait=False)
        assert transport.joints[GRIP_SERVO_ID].torque_on is True, (
            "test setup broken: hold=['GRIP'] should keep torque on inside the block"
        )
    # __exit__ (normal path, no exception) must have unloaded everything.
    assert all(not j.torque_on for j in transport.joints.values())
    unload_entries = [e for e in transport.log if e["cmd"] == "UNLOAD"]
    assert unload_entries, "expected at least one MULT_SERVO_UNLOAD on normal exit"


def test_unload_on_exception():
    transport = SimTransport()
    with pytest.raises(RuntimeError, match="boom"):
        with SafetyLayer(transport, install_signal_handlers=False) as safety:
            safety.move({"GRIP": 1700}, duration_ms=200, hold=["GRIP"], wait=False)
            assert transport.joints[GRIP_SERVO_ID].torque_on is True
            raise RuntimeError("boom")
    # __exit__ must unload even though an exception propagated through the block.
    assert all(not j.torque_on for j in transport.joints.values())


def test_unload_on_exit_via_arm():
    """Same guarantee through the public high-level API, not just SafetyLayer
    directly -- ``Arm.close()``/``Arm.__exit__`` must delegate to it.
    """
    from robohand.arm import Arm

    with Arm(sim=True, use_vision=False, install_signal_handlers=False) as arm:
        arm.set_position("GRIP", 1700, duration_ms=200, hold=["GRIP"], wait=False)
        assert arm._transport.joints[GRIP_SERVO_ID].torque_on is True
    assert all(not j.torque_on for j in arm._transport.joints.values())


def test_unload_on_exception_via_arm():
    from robohand.arm import Arm

    with pytest.raises(RuntimeError, match="boom"):
        with Arm(sim=True, use_vision=False, install_signal_handlers=False) as arm:
            arm.set_position("GRIP", 1700, duration_ms=200, hold=["GRIP"], wait=False)
            raise RuntimeError("boom")
    assert all(not j.torque_on for j in arm._transport.joints.values())
