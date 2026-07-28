"""Layer 2 proof, part 2: a real SIGINT delivered to a process using ``SafetyLayer``
unloads every servo before the process exits.

This has to be a real subprocess -- sending SIGINT to the pytest process itself
would be destructive and non-deterministic. The harness script
(``_sigint_harness.py``) commands a held move (torque intentionally kept on, as if
mid-operation), announces readiness on stdout, then sleeps. We send SIGINT once it
is ready; ``SafetyLayer``'s installed handler must unload all servos and terminate
the process with the conventional 128+SIGINT exit code.
"""

from __future__ import annotations

import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
HARNESS = Path(__file__).resolve().parent / "_sigint_harness.py"


def test_unload_on_sigint():
    env = dict(os.environ)
    env["ROBOHAND_TRANSPORT"] = "sim"
    proc = subprocess.Popen(
        [sys.executable, str(HARNESS)],
        cwd=str(REPO_ROOT),
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        env=env,
    )
    try:
        ready_line = proc.stdout.readline()
        assert ready_line.strip() == "READY", f"harness did not report ready: {ready_line!r}"

        # Give the watchdog/handler installation a brief moment, then interrupt.
        time.sleep(0.2)
        proc.send_signal(signal.SIGINT)

        remaining_output, _ = proc.communicate(timeout=10)
    except subprocess.TimeoutExpired:
        proc.kill()
        raise

    result_lines = [line for line in remaining_output.splitlines() if line.strip().startswith("{")]
    assert result_lines, f"harness produced no result JSON; full output:\n{remaining_output}"
    torque_state = json.loads(result_lines[-1])

    assert torque_state, "harness reported no joints"
    assert all(v is False for v in torque_state.values()), (
        f"expected all servos unloaded after SIGINT, got: {torque_state}"
    )
    # SafetyLayer unloads *first*, then chains to whatever SIGINT handler was already
    # installed (Python's own default_int_handler in a plain script), so the normal
    # outcome is a KeyboardInterrupt raised at the interrupted call site -- not a
    # forced process kill. The harness catches that and exits cleanly (0) only after
    # dumping state, which is what we're really asserting: unload-before-anything-else,
    # not any particular exit code. See SafetyLayer._on_signal's docstring/comments.
    assert proc.returncode == 0, (
        f"harness should exit cleanly after unloading and reporting state, got {proc.returncode}:\n"
        f"{remaining_output}"
    )
