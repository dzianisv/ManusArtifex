"""Fiducial markers: where the arm actually is, instead of what changed.

Why this module exists
----------------------

Everything vision-related in this project used to run through one question:
*how many pixels changed between these two frames?* (``guard.VisionGuard``). That
question has a fatal property -- **it is not about the arm.** A person shifting in
a chair changes pixels. A cloud changes pixels. The camera's own auto-exposure
hunting changes pixels. Measured here on 2026-07-30, with the arm perfectly still:

===========================================  ==========================
scene                                        no-motion floor (p95)
===========================================  ==========================
nobody in the room                           1,243 changed px
one person seated in frame                   43,335 changed px
same person, cropped out of the ROI           5,869-13,676 changed px
===========================================  ==========================

Against a 3,000-changed-pixel motion threshold. The answer to that is not a better
threshold, a tighter ROI or a brightness-normalised metric -- all three were tried
and all three lose, because the signal and the noise are the same kind of thing.
The answer is to stop asking "what changed" and start asking "where is the thing I
care about", which is what every robotics stack that works actually does.

How real robots answer it
-------------------------

1. **Joint encoders.** Every arm LeRobot supports (SO-100, Koch, OMX, Reachy2...)
   reports its own joint positions, and its policies are trained on that state:
   ``obs = robot.get_observation(); action = model.select_action(obs)``. Our LSC-6
   emits PWM and reads nothing back, so this door is closed by construction -- and
   it is why imitation learning (ACT, diffusion policies) is not available to us
   either: those need readable state on both the recording and the execution side.

2. **Fiducial markers.** A printed pattern with a known ID and known geometry,
   found by a detector that is looking for *that pattern* and nothing else. This is
   the standard substitute for encoders on cheap hardware, it ships inside OpenCV
   (``cv2.aruco``), and it is what this module implements.

3. **A VLM.** Good at "which bottle is the gin", bad at geometry and willing to
   invent. This project's own test on 2026-07-28 (docs/cocktail-robot.md) had
   Gemini Robotics-ER 1.6 point at the gripper ~20-30 px off and **hallucinate a
   graspable object in a frame that contained none.** Google's own guidance is to
   pair ER with a low-level controller you supply. So: semantics from the model,
   geometry from the markers, and never the other way round.

What markers buy, concretely
----------------------------

* **A person in frame is not a marker.** The detector is looking for a specific
  black-and-white pattern; a human, a cat, a TV reflection and an exposure shift
  are all simply absent from the result. No noise floor, no ROI tuning, no health
  gate to fail.
* **"Did this joint move?" becomes arithmetic**, not statistics: the marker's
  centre moved N pixels. ``ArucoMotionObserver`` answers the calibrator's question
  with that, at the same ``MotionObserver`` interface the absdiff observer uses,
  so nothing downstream changes.
* **It is free and instant.** Detection is ~10 ms of local CPU. A VLM answering the
  same question costs a network round trip and real money, and calibration asks it
  hundreds of times per joint.

Deliberate boundaries
---------------------

* This module works in **image space** -- marker IDs and pixel coordinates. It does
  *not* do pose estimation (``solvePnP``), because that needs a calibrated camera
  (intrinsics + distortion) which nobody has measured for this webcam, and a
  confidently wrong 3D pose is worse than an honest 2D one. The seam is marked in
  ``MarkerView``; intrinsics plug in there when someone runs a checkerboard.
* A marker that is **not detected** is reported as absent, never as "did not move".
  Those are different facts and conflating them is how the calibrator would infer a
  mechanical stop from a piece of tape falling off.
"""
from __future__ import annotations

import dataclasses
from typing import Any, Dict, Iterable, Optional, Tuple

import numpy as np

from .observer import MotionObserver, ObserverError

#: OpenCV's 4x4 dictionary with 50 ids. 4x4 is the coarsest (largest cells for a
#: given printed size, so the most robust at distance and in poor light) and 50 ids
#: is far more than this arm will ever need. Chosen for margin, not for capacity.
DEFAULT_DICTIONARY = "DICT_4X4_50"

#: Marker ids this project assigns. Small integers, printed once, taped on.
GRIPPER_MARKER_ID = 0
OBJECT_MARKER_ID = 1
BASE_MARKER_ID = 2

#: How far a marker's centre must move, in pixels, to count as motion.
#:
#: MEASURED, 2026-07-30: a stationary marker's detected centre jitters by well
#: under a pixel; the calibrator's smallest step (30 raw units on ELBOW) moved the
#: arm tens of pixels. 3.0 sits far above detector jitter and far below the
#: smallest real step, and unlike ``vision.min_changed_pixels`` it does not have to
#: be re-tuned when the room changes, because the room is not in the measurement.
DEFAULT_MOTION_PX = 3.0


def _aruco():
    """Import ``cv2.aruco`` or explain what is missing. Never returns None."""
    try:
        import cv2  # noqa: F401
    except Exception as exc:  # pragma: no cover - optional dependency
        raise ObserverError(f"OpenCV is not available, so markers cannot be detected: {exc}")
    import cv2

    aruco = getattr(cv2, "aruco", None)
    if aruco is None:  # pragma: no cover - opencv built without contrib
        raise ObserverError(
            "this OpenCV build has no cv2.aruco module (it lives in opencv-contrib). "
            "Install opencv-contrib-python."
        )
    return aruco


def dictionary(name: str = DEFAULT_DICTIONARY):
    aruco = _aruco()
    key = getattr(aruco, name, None)
    if key is None:
        raise ObserverError(f"unknown ArUco dictionary {name!r}")
    getter = getattr(aruco, "getPredefinedDictionary", None)
    return getter(key)


@dataclasses.dataclass(frozen=True)
class MarkerView:
    """One marker as seen in one frame, in image space.

    ``corners`` is the raw 4x2 array in the detector's order (top-left, top-right,
    bottom-right, bottom-left). ``centre`` is their mean.

    There is deliberately no ``pose`` field. Adding one means ``cv2.solvePnP`` with
    camera intrinsics and the marker's physical edge length; the seam is here, and
    the prerequisite is a checkerboard calibration nobody has run on this webcam.
    """

    id: int
    corners: np.ndarray
    centre: Tuple[float, float]

    @property
    def size_px(self) -> float:
        """Mean edge length. A crude range proxy: bigger means nearer."""
        c = self.corners
        return float(np.mean([np.linalg.norm(c[i] - c[(i + 1) % 4]) for i in range(4)]))

    def as_dict(self) -> Dict[str, Any]:
        return {
            "id": self.id,
            "centre": [round(self.centre[0], 2), round(self.centre[1], 2)],
            "size_px": round(self.size_px, 2),
        }


def detect(frame: np.ndarray, *, dictionary_name: str = DEFAULT_DICTIONARY) -> Dict[int, MarkerView]:
    """Every marker in ``frame``, keyed by id. Empty dict when there are none.

    An empty result means "no marker was found", which is NOT "nothing moved" and
    must never be reported as such -- see this module's docstring.
    """
    import cv2

    aruco = _aruco()
    if frame is None:
        raise ObserverError("cannot detect markers in a frame that was never captured")
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
    d = dictionary(dictionary_name)
    # OpenCV moved this API in 4.7: prefer the modern ArucoDetector, fall back to
    # the legacy free function so this works on either.
    if hasattr(aruco, "ArucoDetector"):
        params = aruco.DetectorParameters()
        corners, ids, _ = aruco.ArucoDetector(d, params).detectMarkers(gray)
    else:  # pragma: no cover - opencv < 4.7
        corners, ids, _ = aruco.detectMarkers(gray, d, parameters=aruco.DetectorParameters_create())
    out: Dict[int, MarkerView] = {}
    if ids is None:
        return out
    for marker_id, quad in zip(ids.flatten().tolist(), corners):
        pts = np.asarray(quad, dtype=float).reshape(4, 2)
        out[int(marker_id)] = MarkerView(
            id=int(marker_id), corners=pts, centre=(float(pts[:, 0].mean()), float(pts[:, 1].mean()))
        )
    return out


def render_sheet(
    ids: Iterable[int],
    *,
    px: int = 600,
    border_px: int = 60,
    dictionary_name: str = DEFAULT_DICTIONARY,
) -> np.ndarray:
    """A printable strip of markers, each with a white quiet zone around it.

    The quiet zone is not decoration: the detector needs unbroken white around the
    pattern to find its border. A marker cut out flush with its black edge, or taped
    over a dark part of the arm, will not be detected.
    """
    import cv2

    aruco = _aruco()
    d = dictionary(dictionary_name)
    tiles = []
    for marker_id in ids:
        img = aruco.generateImageMarker(d, int(marker_id), px)
        tile = np.full((px + 2 * border_px, px + 2 * border_px), 255, dtype=np.uint8)
        tile[border_px:border_px + px, border_px:border_px + px] = img
        cv2.putText(tile, f"id {marker_id}", (border_px, border_px - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, 0, 2, cv2.LINE_AA)
        tiles.append(tile)
    if not tiles:
        raise ObserverError("no marker ids requested")
    return np.hstack(tiles)


class ArucoMotionObserver(MotionObserver):
    """"Did this joint move?" answered by one marker's displacement in pixels.

    Same interface as ``VisionMotionObserver``, so ``calibrate`` cannot tell the
    difference -- but the answer no longer depends on what else is in the room.

    ``marker_id`` must be on the link the joint actually moves. The observer's
    verdict is about that marker and nothing else: if the marker is missing from
    either frame it raises rather than returning False, because "I could not see
    the marker" and "the joint did not move" are the two facts the calibrator must
    never confuse (it infers mechanical stops from repeated no-motion).
    """

    def __init__(
        self,
        frame_source,
        *,
        marker_id: int = GRIPPER_MARKER_ID,
        motion_px: float = DEFAULT_MOTION_PX,
        dictionary_name: str = DEFAULT_DICTIONARY,
    ) -> None:
        self._frames = frame_source
        self._marker_id = int(marker_id)
        self._motion_px = float(motion_px)
        self._dictionary = dictionary_name

    def available(self) -> bool:
        return self._frames is not None

    def capture(self) -> Any:
        if self._frames is None:
            raise ObserverError("no frame source: cannot watch for markers")
        frame = self._frames()
        if frame is None:
            raise ObserverError("camera returned no frame")
        return detect(frame, dictionary_name=self._dictionary)

    def _centre(self, snapshot: Dict[int, MarkerView], which: str) -> Tuple[float, float]:
        view = snapshot.get(self._marker_id)
        if view is None:
            raise ObserverError(
                f"marker {self._marker_id} was not visible in the {which} frame. That is "
                "not evidence the joint did not move -- it means the marker is out of "
                "shot, occluded by the arm itself, too small at this distance, or has "
                "come unstuck. Re-aim or re-tape before trusting any calibration."
            )
        return view.centre

    def score(self, before: Any, after: Any) -> float:
        (x0, y0) = self._centre(before, "before")
        (x1, y1) = self._centre(after, "after")
        return float(np.hypot(x1 - x0, y1 - y0))

    def observe(self, before: Any, after: Any) -> bool:
        return self.score(before, after) >= self._motion_px

    def motion_threshold(self) -> float:
        return self._motion_px

    def describe(self) -> str:
        return (
            f"ArucoMotionObserver(marker={self._marker_id}, "
            f"threshold={self._motion_px:g}px, dict={self._dictionary})"
        )
