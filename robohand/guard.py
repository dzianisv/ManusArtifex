"""Layer 3 defence: vision-based stall/obstruction detection.

There is no position feedback on this hardware (see protocol.NoResponse), so a
camera is the only real signal available for "did the joint actually move". This
module never assumes a camera is present -- callers must handle
``VisionGuard.available()`` being False and continue operating on layers 1+2 alone.

Detection method: capture a frame before a move and one after, restrict comparison
to a configurable ROI around the arm, take an absolute difference, blur to suppress
sensor noise, threshold, and count changed pixels. If the changed-pixel count is
below a configurable minimum, we conclude the commanded motion did not visibly
happen -- either the servo is stalled against something, or the arm is obstructed --
and treat that as equivalent to a stall for safety purposes: unload immediately.
"""

from __future__ import annotations

import dataclasses
from typing import Callable, Optional, Tuple

import cv2
import numpy as np

Roi = Tuple[float, float, float, float]  # x, y, w, h; fractional (<=1) or pixel (>1)


class StallDetected(RuntimeError):
    """Raised when a commanded move produced no visible motion.

    By the time this is raised, the servo(s) involved have already been unloaded --
    callers must not assume they still need to react to prevent damage, only that
    the requested motion did not verifiably happen.
    """


def _resolve_roi(frame_shape: Tuple[int, int], roi: Optional[Roi]) -> Tuple[int, int, int, int]:
    h, w = frame_shape[:2]
    if roi is None:
        return 0, 0, w, h
    x, y, rw, rh = roi
    if max(x, y, rw, rh) <= 1.0:
        x, y, rw, rh = x * w, y * h, rw * w, rh * h
    x, y, rw, rh = int(x), int(y), int(rw), int(rh)
    x = max(0, min(x, w - 1))
    y = max(0, min(y, h - 1))
    rw = max(1, min(rw, w - x))
    rh = max(1, min(rh, h - y))
    return x, y, rw, rh


@dataclasses.dataclass
class VisionGuard:
    """Frame-differencing stall/obstruction detector.

    ``frame_source`` is any zero-arg callable returning a BGR ``np.ndarray`` frame
    (typically ``camera.Camera.grab``, but tests inject a synthetic source so this
    class can be exercised with zero hardware/camera dependency).  If
    ``frame_source`` is ``None`` (e.g. no camera could be opened), ``available()``
    returns False and every ``check*`` call is a no-op -- callers must check this and
    warn the operator that layer 3 is not active, per the degrade-safely requirement.
    """

    frame_source: Optional[Callable[[], np.ndarray]] = None
    roi: Optional[Roi] = None
    diff_threshold: int = 25
    min_changed_pixels: int = 150
    blur_ksize: int = 5

    def available(self) -> bool:
        return self.frame_source is not None

    def capture(self) -> Optional[np.ndarray]:
        if not self.available():
            return None
        return self.frame_source()

    def motion_score(self, frame_before: np.ndarray, frame_after: np.ndarray) -> int:
        """Count of changed pixels inside the ROI between two frames."""
        if frame_before.shape != frame_after.shape:
            # A resolution/orientation change is not "no motion" -- treat it as
            # maximal motion so callers do not mistake a bad capture for a stall.
            return self.min_changed_pixels + 1
        x, y, w, h = _resolve_roi(frame_before.shape, self.roi)
        before_roi = frame_before[y : y + h, x : x + w]
        after_roi = frame_after[y : y + h, x : x + w]

        before_gray = cv2.cvtColor(before_roi, cv2.COLOR_BGR2GRAY) if before_roi.ndim == 3 else before_roi
        after_gray = cv2.cvtColor(after_roi, cv2.COLOR_BGR2GRAY) if after_roi.ndim == 3 else after_roi

        diff = cv2.absdiff(before_gray, after_gray)
        ksize = self.blur_ksize if self.blur_ksize % 2 == 1 else self.blur_ksize + 1
        if ksize > 1:
            diff = cv2.GaussianBlur(diff, (ksize, ksize), 0)
        _, thresholded = cv2.threshold(diff, self.diff_threshold, 255, cv2.THRESH_BINARY)
        return int(np.count_nonzero(thresholded))

    def detect_motion(self, frame_before: np.ndarray, frame_after: np.ndarray) -> bool:
        return self.motion_score(frame_before, frame_after) >= self.min_changed_pixels

    def check(
        self,
        frame_before: Optional[np.ndarray],
        frame_after: Optional[np.ndarray],
        on_stall: Optional[Callable[[], None]] = None,
        context: str = "",
    ) -> None:
        """Raise ``StallDetected`` if the two frames show no motion.

        If either frame is ``None`` (camera unavailable/capture failed), this is a
        no-op: layers 1+2 remain the only protection for that move, matching the
        "degrade safely" requirement. ``on_stall`` is called (e.g. to unload the
        implicated servo(s)) *before* the exception is raised.
        """
        if frame_before is None or frame_after is None:
            return
        if not self.detect_motion(frame_before, frame_after):
            if on_stall is not None:
                on_stall()
            score = self.motion_score(frame_before, frame_after)
            raise StallDetected(
                f"no motion observed{f' ({context})' if context else ''}: "
                f"{score} changed px < threshold {self.min_changed_pixels}"
            )
