"""Standalone harness process for tests/test_unload_on_sigint.py.

Not collected by pytest (no test_ prefix). Spawned as a subprocess so a real SIGINT
can be delivered without touching the pytest process itself. Commands a held move
(torque intentionally left on) against the simulator, announces readiness, then
waits to be interrupted. Whatever unload state exists at process-exit time is
printed as JSON on the final line of stdout so the parent test can assert on it.
"""

from __future__ import annotations

import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from robohand.safety import SafetyLayer  # noqa: E402
from robohand.sim import SimTransport  # noqa: E402


def _dump_state(transport: SimTransport) -> None:
    print(json.dumps({str(sid): joint.torque_on for sid, joint in transport.joints.items()}), flush=True)


def main() -> None:
    transport = SimTransport()
    safety = SafetyLayer(
        transport,
        hold_ms=60_000,
        idle_unload_ms=60_000,
        install_signal_handlers=True,
    )
    safety.move({"GRIP": 1700}, duration_ms=100, hold=["GRIP"], wait=False)
    assert transport.joints[1].torque_on is True

    print("READY", flush=True)
    try:
        time.sleep(15)
    except KeyboardInterrupt:
        # Should not normally be reached: SafetyLayer's own SIGINT handler unloads
        # and exits before this would fire. Kept only as a last-resort safety net so
        # the harness still reports state instead of hanging if signal delivery
        # timing differs across platforms.
        pass
    finally:
        _dump_state(transport)


if __name__ == "__main__":
    main()
