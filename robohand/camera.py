"""Camera access layer for the robotic-arm vision system.

Wraps OpenCV's AVFoundation backend (macOS) so the arm's workspace can be
observed by the built-in FaceTime camera, an external USB webcam, or an
iPhone via Continuity Camera. Handles the macOS quirks that trip up naive
``cv2.VideoCapture`` usage: warm-up frames that come back dark/garbage,
permission prompts on first access, and camera indices that shift when
devices are plugged/unplugged.

See docs/camera.md for setup, camera-selection, and calibration notes.
"""

from __future__ import annotations

import dataclasses
import time
from pathlib import Path
from typing import Optional

import cv2
import numpy as np

# AVFoundation is the only capture backend that reliably works on macOS.
BACKEND = cv2.CAP_AVFOUNDATION

# macOS/AVFoundation only enumerates a handful of devices; probing further
# indices just spams "device out of bound" warnings.
MAX_PROBE_INDEX = 6


class CameraError(RuntimeError):
    """Raised when a camera cannot be opened or produces no usable frames."""


@dataclasses.dataclass
class CameraInfo:
    """Result of probing a single capture index."""

    index: int
    opened: bool
    width: int = 0
    height: int = 0
    fps: float = 0.0
    mean_brightness: Optional[float] = None

    @property
    def usable(self) -> bool:
        """True if the index opened and returned a non-trivial frame.

        A near-zero mean brightness after warm-up usually means the OS is
        still showing the camera permission prompt (frame is solid black),
        not that the scene itself is dark.
        """
        return self.opened and self.mean_brightness is not None and self.mean_brightness > 1.0


def list_cameras(max_index: int = MAX_PROBE_INDEX, warmup_frames: int = 5) -> list[CameraInfo]:
    """Probe capture indices 0..max_index-1 and report which ones work.

    Opens each index with the AVFoundation backend, discards ``warmup_frames``
    frames (macOS returns dark frames on the first reads), then reads one
    frame to measure resolution/brightness. Always releases the device
    before moving to the next index.
    """
    results: list[CameraInfo] = []
    for index in range(max_index):
        cap = cv2.VideoCapture(index, BACKEND)
        opened = cap.isOpened()
        info = CameraInfo(index=index, opened=opened)
        if opened:
            for _ in range(warmup_frames):
                cap.read()
            ok, frame = cap.read()
            info.width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            info.height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            info.fps = float(cap.get(cv2.CAP_PROP_FPS))
            if ok and frame is not None:
                info.mean_brightness = float(frame.mean())
        cap.release()
        results.append(info)
    return results


@dataclasses.dataclass
class CameraConfig:
    """Configuration for opening and post-processing a camera stream.

    Transform order applied in :meth:`Camera._transform` is
    ``rotate -> flip -> crop``. Rotate first so flip/crop reason about the
    final "upright" orientation of the frame; crop last so fractional crop
    boxes are expressed in already-rotated/flipped frame coordinates.
    """

    index: int = 0
    width: Optional[int] = None
    height: Optional[int] = None
    fps: Optional[float] = None
    warmup_frames: int = 8
    max_retries: int = 10
    retry_delay_s: float = 0.1
    open_timeout_s: float = 5.0
    # One of: None, 90, 180, 270 (clockwise degrees).
    rotate: Optional[int] = None
    flip_horizontal: bool = False
    flip_vertical: bool = False
    # (x, y, w, h). Values in (0, 1] are treated as fractions of the frame;
    # values > 1 are treated as absolute pixels.
    crop: Optional[tuple[float, float, float, float]] = None

    def rotate_code(self) -> Optional[int]:
        return {
            90: cv2.ROTATE_90_CLOCKWISE,
            180: cv2.ROTATE_180,
            270: cv2.ROTATE_90_COUNTERCLOCKWISE,
        }.get(self.rotate)


class Camera:
    """Context-manager wrapper around ``cv2.VideoCapture``.

    Example::

        with Camera(CameraConfig(index=0, rotate=180)) as cam:
            frame = cam.grab()
            cam.snapshot("docs/media/snapshot.jpg")
    """

    def __init__(self, config: Optional[CameraConfig] = None, index: Optional[int] = None):
        if config is None:
            config = CameraConfig(index=index if index is not None else 0)
        elif index is not None:
            config = dataclasses.replace(config, index=index)
        self.config = config
        self._cap: Optional[cv2.VideoCapture] = None

    def open(self) -> "Camera":
        if self._cap is not None:
            return self
        cfg = self.config
        deadline = time.monotonic() + cfg.open_timeout_s
        cap = cv2.VideoCapture(cfg.index, BACKEND)
        while not cap.isOpened() and time.monotonic() < deadline:
            time.sleep(cfg.retry_delay_s)
        if not cap.isOpened():
            cap.release()
            raise CameraError(
                f"Could not open camera index {cfg.index} within {cfg.open_timeout_s}s"
            )
        if cfg.width:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, cfg.width)
        if cfg.height:
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cfg.height)
        if cfg.fps:
            cap.set(cv2.CAP_PROP_FPS, cfg.fps)
        self._cap = cap
        self._warm_up()
        return self

    def _warm_up(self) -> None:
        """Discard the first N frames.

        macOS/AVFoundation frequently returns solid-black or stale frames
        for the first few reads after opening a device, especially right
        after a permission grant. Discarding them avoids false "no object
        detected" / black-snapshot results downstream.
        """
        assert self._cap is not None
        for _ in range(self.config.warmup_frames):
            self._cap.read()

    def _transform(self, frame: np.ndarray) -> np.ndarray:
        cfg = self.config
        rotate_code = cfg.rotate_code()
        if rotate_code is not None:
            frame = cv2.rotate(frame, rotate_code)
        if cfg.flip_horizontal and cfg.flip_vertical:
            frame = cv2.flip(frame, -1)
        elif cfg.flip_horizontal:
            frame = cv2.flip(frame, 1)
        elif cfg.flip_vertical:
            frame = cv2.flip(frame, 0)
        if cfg.crop is not None:
            frame = self._apply_crop(frame, cfg.crop)
        return frame

    @staticmethod
    def _apply_crop(frame: np.ndarray, crop: tuple[float, float, float, float]) -> np.ndarray:
        h, w = frame.shape[:2]
        x, y, cw, ch = crop
        if max(x, y, cw, ch) <= 1.0:
            x, y, cw, ch = x * w, y * h, cw * w, ch * h
        x, y, cw, ch = int(x), int(y), int(cw), int(ch)
        x = max(0, min(x, w - 1))
        y = max(0, min(y, h - 1))
        cw = max(1, min(cw, w - x))
        ch = max(1, min(ch, h - y))
        return frame[y : y + ch, x : x + cw]

    def grab(self) -> np.ndarray:
        """Read one frame, retrying on transient decode failures.

        Raises ``CameraError`` if no usable frame is produced after
        ``max_retries`` attempts (e.g. camera disconnected, or permission
        was never granted).
        """
        if self._cap is None:
            self.open()
        assert self._cap is not None
        last_err = "unknown error"
        for attempt in range(self.config.max_retries):
            ok, frame = self._cap.read()
            if ok and frame is not None:
                return self._transform(frame)
            last_err = f"read() returned ok={ok} frame={'None' if frame is None else 'empty'}"
            time.sleep(self.config.retry_delay_s)
        raise CameraError(
            f"No frame from camera index {self.config.index} after "
            f"{self.config.max_retries} attempts ({last_err}). If this is the first run, "
            "grant camera permission to this terminal app in System Settings > Privacy & "
            "Security > Camera and try again."
        )

    def snapshot(self, path: str | Path) -> Path:
        """Grab one frame and save it as a JPEG."""
        frame = self.grab()
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        ok = cv2.imwrite(str(path), frame)
        if not ok:
            raise CameraError(f"cv2.imwrite failed for {path}")
        return path

    def release(self) -> None:
        if self._cap is not None:
            self._cap.release()
            self._cap = None

    def __enter__(self) -> "Camera":
        return self.open()

    def __exit__(self, exc_type, exc, tb) -> None:
        self.release()


def find_camera_index_by_name(name: str) -> Optional[int]:
    """Best-effort lookup of a capture index by device name.

    OpenCV's AVFoundation backend does not expose device names directly, so
    this shells out to ``system_profiler SPCameraDataType`` to get macOS's
    camera enumeration order and assumes ``cv2`` indices follow the same
    order as devices that actually open. This is a heuristic: if multiple
    cameras are attached, verify with :func:`list_cameras` before trusting it.
    """
    import re
    import subprocess

    try:
        out = subprocess.run(
            ["system_profiler", "SPCameraDataType"],
            capture_output=True,
            text=True,
            timeout=10,
        ).stdout
    except Exception:
        return None

    device_names = [line.strip().rstrip(":") for line in out.splitlines() if re.match(r"^\s{4}\S.*:$", line)]
    openable = [info.index for info in list_cameras() if info.opened]
    for device_name, index in zip(device_names, openable):
        if name.lower() in device_name.lower():
            return index
    return None
