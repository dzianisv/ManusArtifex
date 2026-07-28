"""Layer 1 proof: an uncalibrated joint refuses (does not silently clamp-and-move) a
request that falls outside its provisional band, and the CLI surfaces this as a
clear non-zero exit rather than moving.
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import pytest

from robohand import config
from robohand.safety import CalibrationRequired, SafetyLayer
from robohand.sim import SimTransport

REPO_ROOT = Path(__file__).resolve().parent.parent


def test_uncalibrated_joint_refuses_out_of_band_move():
    assert config.JOINT_LIMITS["GRIP"].calibrated is False, (
        "test assumes GRIP ships uncalibrated by default; update this test if that changes"
    )
    lo, hi = config.effective_bounds("GRIP")

    transport = SimTransport()
    safety = SafetyLayer(transport, install_signal_handlers=False)
    try:
        with pytest.raises(CalibrationRequired):
            safety.move({"GRIP": hi + 50}, wait=False)  # just outside the provisional band
        # Nothing should have been sent to the transport at all -- refuse, don't clamp.
        assert transport.log == []
        assert transport.stall_ms_total() == 0
    finally:
        safety.close()


def test_uncalibrated_joint_allows_in_band_move():
    """Sanity complement: the same joint must still be movable within its band --
    calibration status restricts range, it does not block motion outright.
    """
    transport = SimTransport()
    safety = SafetyLayer(transport, install_signal_handlers=False)
    try:
        lo, hi = config.effective_bounds("GRIP")
        mid = (lo + hi) // 2
        safety.move({"GRIP": mid}, wait=False)
    finally:
        safety.close()
    assert any(entry["cmd"] == "MOVE" for entry in transport.log)


def test_cli_move_refuses_and_exits_non_zero(tmp_path):
    """End-to-end: the CLI itself refuses to run, per spec ("robohand must exit
    non-zero with a clear message rather than move").
    """
    import os

    _, hi = config.effective_bounds("GRIP")
    env = dict(os.environ)
    env["ROBOHAND_TRANSPORT"] = "sim"
    env["ROBOHAND_STATE"] = str(tmp_path / "state.json")
    result = subprocess.run(
        [sys.executable, "-m", "robohand.cli", "--sim", "move", "--servo", "GRIP", "--pos", str(hi + 100)],
        cwd=str(REPO_ROOT),
        capture_output=True,
        text=True,
        env=env,
        timeout=15,
    )
    assert result.returncode != 0
    combined = (result.stdout + result.stderr).lower()
    assert "calibrat" in combined, f"expected a clear calibration-related message, got:\n{result.stdout}\n{result.stderr}"
