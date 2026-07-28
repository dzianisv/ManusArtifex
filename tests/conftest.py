"""Shared pytest fixtures for the robohand safety test suite.

Every test in this directory runs against the simulator (``robohand.sim``) --
never real hardware. ``ROBOHAND_TRANSPORT=sim`` is forced for *all* tests as a
belt-and-suspenders guard: even a test that forgets to pass ``sim=True`` explicitly
will still hit ``SimTransport``, never the real HID device (VID 0x0483 / PID 0x5750).
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))


@pytest.fixture(autouse=True)
def isolated_environment(tmp_path, monkeypatch):
    """Force the simulator and redirect shadow-state persistence to a scratch file.

    Without this, ``Arm`` would persist last-commanded positions to
    ``~/.robohand/state.json`` on the real machine running the tests.
    """
    monkeypatch.setenv("ROBOHAND_TRANSPORT", "sim")
    monkeypatch.setenv("ROBOHAND_STATE", str(tmp_path / "state.json"))
    yield
