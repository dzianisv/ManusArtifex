"""Layer 3 proof: ``VisionGuard`` detects "no motion" from two identical synthetic
frames and raises ``StallDetected``, invoking the ``on_stall`` callback first (so a
caller can unload before the exception propagates).

Uses plain synthetic numpy arrays -- no camera, no OpenCV window, fully
deterministic and independent of any real camera being present in the environment.
"""

from __future__ import annotations

import numpy as np
import pytest

from robohand.guard import StallDetected, VisionGuard


def test_vision_guard_detects_no_motion():
    frame = np.zeros((60, 80, 3), dtype=np.uint8)
    unload_calls = []

    guard = VisionGuard(frame_source=lambda: frame, min_changed_pixels=10)
    with pytest.raises(StallDetected):
        guard.check(frame, frame, on_stall=lambda: unload_calls.append(True), context="test")

    assert unload_calls == [True], "on_stall must be invoked before StallDetected is raised"


def test_vision_guard_detects_motion_and_does_not_raise():
    """Sanity complement: two frames that genuinely differ enough must NOT raise."""
    before = np.zeros((60, 80, 3), dtype=np.uint8)
    after = before.copy()
    after[10:50, 10:70] = 255  # large bright block -> plenty of changed pixels

    guard = VisionGuard(min_changed_pixels=10)
    guard.check(before, after)  # must not raise


def test_vision_guard_is_a_noop_without_a_camera():
    """Degrade-safely requirement: if frames are unavailable (None), check() must
    not raise and must not call on_stall -- callers rely on this to keep operating
    on layers 1+2 alone when no camera is present.
    """
    guard = VisionGuard(frame_source=None)
    assert guard.available() is False
    calls = []
    guard.check(None, None, on_stall=lambda: calls.append(True))
    assert calls == []


def test_stall_detected_is_reexported_from_safety():
    """``StallDetected`` is defined in ``guard`` but is naturally caught next to the
    Layer-1 refusals, so ``except safety.StallDetected`` must work and must refer to
    the SAME class object -- otherwise an ``except`` on one path silently fails to
    catch the other. Previously raised AttributeError.
    """
    import robohand.guard
    import robohand.safety

    assert robohand.safety.StallDetected is robohand.guard.StallDetected

    # the `from X import Y` spelling must resolve too
    from robohand.safety import StallDetected as FromSafety

    assert FromSafety is robohand.guard.StallDetected

    # and it must actually catch a real raise
    with pytest.raises(robohand.safety.StallDetected):
        raise robohand.guard.StallDetected("boom")


def test_safety_still_raises_attributeerror_for_unknown_names():
    """The module __getattr__ must not turn every typo into a silent success."""
    import robohand.safety

    with pytest.raises(AttributeError):
        robohand.safety.NoSuchThing
