"""Object detection and pixel<->workspace calibration for the arm's camera.

Dependency-light on purpose (cv2 + numpy only) so it can run on the same
machine that drives the servo controller without pulling in a full CV/ML
stack. Detection is HSV color-threshold + contour filtering: fast, fully
offline, and good enough to find brightly colored graspable objects
(blocks, bottle caps, fruit, etc.) on a work surface.
"""

from __future__ import annotations

import dataclasses
import json
from pathlib import Path
from typing import Optional, TypedDict

import cv2
import numpy as np

Bbox = tuple[int, int, int, int]  # x, y, w, h


class Detection(TypedDict):
    cx: int
    cy: int
    area: float
    bbox: Bbox
    color: str


# Default HSV ranges (OpenCV hue range 0-179). Red wraps around hue 0/180,
# so it needs two ranges. Tune these for your lighting/objects; see
# docs/camera.md for a calibration tip.
DEFAULT_COLOR_RANGES: dict[str, list[tuple[tuple[int, int, int], tuple[int, int, int]]]] = {
    "red": [
        ((0, 100, 80), (8, 255, 255)),
        ((172, 100, 80), (179, 255, 255)),
    ],
    "orange": [((9, 120, 100), (19, 255, 255))],
    "yellow": [((20, 100, 100), (35, 255, 255))],
    "green": [((36, 60, 60), (85, 255, 255))],
    "blue": [((94, 80, 60), (126, 255, 255))],
}


def find_objects(
    frame: np.ndarray,
    color_ranges: Optional[dict[str, list[tuple[tuple[int, int, int], tuple[int, int, int]]]]] = None,
    min_area: float = 200.0,
    max_objects: Optional[int] = None,
) -> list[Detection]:
    """Detect graspable objects via HSV color thresholding + contours.

    Returns a list of detections (one per matched contour above
    ``min_area``), sorted by area descending, each with the pixel centroid,
    area, bounding box, and matched color name.
    """
    if color_ranges is None:
        color_ranges = DEFAULT_COLOR_RANGES

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    detections: list[Detection] = []

    for color_name, ranges in color_ranges.items():
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lower, upper in ranges:
            mask |= cv2.inRange(hsv, np.array(lower), np.array(upper))

        # Clean up speckle noise / small gaps before contouring.
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < min_area:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            moments = cv2.moments(contour)
            if moments["m00"] == 0:
                continue
            cx = int(moments["m10"] / moments["m00"])
            cy = int(moments["m01"] / moments["m00"])
            detections.append(
                Detection(cx=cx, cy=cy, area=float(area), bbox=(x, y, w, h), color=color_name)
            )

    detections.sort(key=lambda d: d["area"], reverse=True)
    if max_objects is not None:
        detections = detections[:max_objects]
    return detections


def detect_biggest(frame: np.ndarray, color: Optional[str] = None) -> Optional[Detection]:
    """Convenience wrapper: return only the single largest detection.

    If ``color`` is given, restrict detection to that color's HSV range.
    """
    ranges = {color: DEFAULT_COLOR_RANGES[color]} if color else None
    detections = find_objects(frame, color_ranges=ranges)
    return detections[0] if detections else None


_COLOR_BGR = {
    "red": (0, 0, 255),
    "orange": (0, 140, 255),
    "yellow": (0, 220, 220),
    "green": (0, 200, 0),
    "blue": (255, 0, 0),
}


def annotate(frame: np.ndarray, detections: list[Detection]) -> np.ndarray:
    """Draw bounding boxes + centroids + labels for each detection."""
    out = frame.copy()
    for det in detections:
        x, y, w, h = det["bbox"]
        color = _COLOR_BGR.get(det["color"], (255, 255, 255))
        cv2.rectangle(out, (x, y), (x + w, y + h), color, 2)
        cv2.circle(out, (det["cx"], det["cy"]), 5, color, -1)
        label = f"{det['color']} {int(det['area'])}px"
        cv2.putText(out, label, (x, max(0, y - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
    return out


@dataclasses.dataclass
class Calibration:
    """Pixel -> workspace-plane calibration via a 4-point homography.

    Procedure (documented in docs/camera.md):
      1. Place a known reference marker (e.g. a checkerboard corner, taped
         cross) at 4 known workspace (x, y) coordinates on the tabletop
         within the arm's reach (measure with a ruler relative to the
         arm's base, in mm).
      2. Click each marker's location in a captured frame (or otherwise
         determine its pixel coordinates).
      3. Build a Calibration with the 4 (pixel, workspace) point pairs and
         call `.solve()` to compute the homography.
      4. `save_calibration()` to persist it; `load_calibration()` next run.

    This assumes the workspace surface is planar and the camera pose is
    fixed relative to it -- true for a camera clamped above/beside a
    tabletop arm, not true if the camera moves.
    """

    pixel_points: list[tuple[float, float]]
    workspace_points: list[tuple[float, float]]
    _homography: Optional[np.ndarray] = dataclasses.field(default=None, repr=False, compare=False)

    def solve(self) -> np.ndarray:
        if len(self.pixel_points) < 4 or len(self.workspace_points) < 4:
            raise ValueError("Need at least 4 point pairs to solve a homography")
        src = np.array(self.pixel_points, dtype=np.float32)
        dst = np.array(self.workspace_points, dtype=np.float32)
        homography, _ = cv2.findHomography(src, dst, method=0)
        if homography is None:
            raise ValueError("cv2.findHomography failed to converge on these points")
        self._homography = homography
        return homography

    def pixel_to_workspace(self, px: float, py: float) -> tuple[float, float]:
        """Map a pixel coordinate to workspace (x, y) using the homography."""
        if self._homography is None:
            self.solve()
        point = np.array([[[px, py]]], dtype=np.float32)
        mapped = cv2.perspectiveTransform(point, self._homography)
        x, y = mapped[0, 0]
        return float(x), float(y)

    def save_calibration(self, path: str | Path) -> Path:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(
            json.dumps(
                {"pixel_points": self.pixel_points, "workspace_points": self.workspace_points},
                indent=2,
            )
        )
        return path

    @classmethod
    def load_calibration(cls, path: str | Path) -> "Calibration":
        data = json.loads(Path(path).read_text())
        calib = cls(
            pixel_points=[tuple(p) for p in data["pixel_points"]],
            workspace_points=[tuple(p) for p in data["workspace_points"]],
        )
        calib.solve()
        return calib


def pixel_to_workspace(
    px: float, py: float, calibration_path: str | Path
) -> tuple[float, float]:
    """Stand-alone helper: load a saved calibration and map one point.

    Prefer keeping a `Calibration` instance around (via `load_calibration`)
    if you're mapping many points, to avoid re-solving the homography on
    every call.
    """
    calib = Calibration.load_calibration(calibration_path)
    return calib.pixel_to_workspace(px, py)
