"""Layer 2 proof, part 3: the background watchdog unloads every servo if no new
command arrives within ``idle_unload_ms`` -- even if nobody ever asked for that
joint to be unloaded and its ``hold_ms`` timer hasn't expired yet.

We set ``hold_ms`` deliberately longer than the test's patience and ``idle_unload_ms``
short, so the only thing that can produce an unload within the observed window is
the idle watchdog, not the per-move hold timer.
"""

from __future__ import annotations

import time

from robohand.safety import SafetyLayer
from robohand.sim import SimTransport

GRIP_SERVO_ID = 1


def test_watchdog_unloads_when_idle():
    transport = SimTransport()
    safety = SafetyLayer(
        transport,
        hold_ms=10_000,       # long enough that the hold timer alone would not fire
        idle_unload_ms=250,   # short so the test runs fast
        watchdog_poll_s=0.05,
        install_signal_handlers=False,
    )
    try:
        safety.move({"GRIP": 1700}, duration_ms=100, hold=["GRIP"], wait=False)
        assert transport.joints[GRIP_SERVO_ID].torque_on is True

        # Do nothing -- no new command -- and wait past idle_unload_ms.
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and transport.joints[GRIP_SERVO_ID].torque_on:
            time.sleep(0.05)

        assert transport.joints[GRIP_SERVO_ID].torque_on is False, (
            "watchdog did not unload an idle servo within the expected window"
        )
    finally:
        safety.close()


def test_watchdog_does_not_unload_while_active():
    """Sanity check for the above: if commands keep arriving faster than
    idle_unload_ms, the watchdog must NOT unload out from under an active sequence.
    """
    transport = SimTransport()
    safety = SafetyLayer(
        transport,
        hold_ms=10_000,
        idle_unload_ms=400,
        watchdog_poll_s=0.05,
        install_signal_handlers=False,
    )
    try:
        end = time.monotonic() + 0.6
        while time.monotonic() < end:
            safety.move({"GRIP": 1700}, duration_ms=50, hold=["GRIP"], wait=False)
            time.sleep(0.05)
        assert transport.joints[GRIP_SERVO_ID].torque_on is True
    finally:
        safety.close()
