"""Video recording and evidence-artifact helpers for the arm's camera.

Supports two recording modes:
  * synchronous, fixed-duration clips (``record_video(path, seconds=5)``)
  * "start now, stop whenever" background recording (``BackgroundRecorder``)
    so a caller can kick off filming, run an arm motion sequence, and stop
    the recording once the motion completes -- capturing the whole event.

Also provides ``make_gif`` to turn an mp4 into a small gif for embedding as
evidence (e.g. in verification docs / PRs) without a giant video file.
"""

from __future__ import annotations

import shutil
import subprocess
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Optional

import cv2
import numpy as np

from robohand.camera import Camera, CameraConfig

DEFAULT_FPS = 20.0


def _open_camera(camera: Optional[Camera], config: Optional[CameraConfig]) -> tuple[Camera, bool]:
    """Return (camera, owns_camera). Opens a new Camera if none was given."""
    if camera is not None:
        camera.open()
        return camera, False
    return Camera(config or CameraConfig()).open(), True


def _burn_overlay(frame: np.ndarray, text: str) -> np.ndarray:
    out = frame.copy()
    cv2.putText(out, text, (10, out.shape[0] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4)
    cv2.putText(out, text, (10, out.shape[0] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
    return out


def _make_writer(path: Path, width: int, height: int, fps: float) -> cv2.VideoWriter:
    path.parent.mkdir(parents=True, exist_ok=True)
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(str(path), fourcc, fps, (width, height))
    if not writer.isOpened():
        raise RuntimeError(f"cv2.VideoWriter failed to open for {path}")
    return writer


def record_video(
    path: str | Path,
    seconds: Optional[float] = None,
    fps: float = DEFAULT_FPS,
    camera: Optional[Camera] = None,
    config: Optional[CameraConfig] = None,
    overlay: bool = True,
    label: Optional[str] = None,
    stop_event: Optional[threading.Event] = None,
) -> Path:
    """Record an mp4 clip.

    Records for ``seconds`` if given, otherwise runs until ``stop_event`` is
    set (used internally by :class:`BackgroundRecorder`; at least one of
    ``seconds``/``stop_event`` must be provided).

    ``fps`` is a *target/cap* on the capture rate, not a guarantee: macOS
    AVFoundation capture (and the underlying USB/Continuity link) may not
    sustain it. Frames are buffered in memory with real timestamps while
    recording, then the mp4 is encoded at the *measured* achieved fps
    (``frame_count / elapsed_seconds``). This keeps the saved clip's
    playback duration matching real elapsed time -- if we instead encoded
    at the nominal ``fps`` while actually capturing slower, the clip would
    play back sped-up relative to what really happened, which would be
    actively misleading for verifying an arm motion. Because frames are
    buffered, this is only appropriate for short (few-second) clips.

    If ``camera`` is passed in already-open, it is used as-is and left open
    for the caller to manage (e.g. sharing one camera between a snapshot and
    a recording). Otherwise a new :class:`Camera` is opened and released
    internally.
    """
    if seconds is None and stop_event is None:
        raise ValueError("record_video needs either `seconds` or `stop_event`")

    path = Path(path)
    cam, owns_camera = _open_camera(camera, config)
    try:
        frames: list[np.ndarray] = []
        frame_interval = 1.0 / fps
        start = time.monotonic()
        next_frame_at = start
        while True:
            if seconds is not None and (time.monotonic() - start) >= seconds:
                break
            if stop_event is not None and stop_event.is_set():
                break
            sleep_for = next_frame_at - time.monotonic()
            if sleep_for > 0:
                time.sleep(sleep_for)
            frame = cam.grab()
            if overlay:
                text = label or datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                frame = _burn_overlay(frame, text)
            frames.append(frame)
            next_frame_at += frame_interval
        elapsed = time.monotonic() - start

        if not frames:
            raise RuntimeError(f"No frames captured from camera index {cam.config.index}")

        achieved_fps = len(frames) / elapsed if elapsed > 0 else fps
        # Clamp to a sane range: never below 1fps, never above the
        # requested cap (a burst of unusually fast reads shouldn't produce
        # an absurdly high-fps, super-choppy-looking file).
        achieved_fps = max(1.0, min(achieved_fps, fps))

        height, width = frames[0].shape[:2]
        writer = _make_writer(path, width, height, achieved_fps)
        try:
            for frame in frames:
                writer.write(frame)
        finally:
            writer.release()
    finally:
        if owns_camera:
            cam.release()
    return path


class BackgroundRecorder:
    """Record video on a background thread: start now, stop later.

    ``start()`` opens (and warms up) the camera *synchronously*, on the
    calling thread, before spawning the capture thread and returning. This
    matters: camera open + warm-up can take a second or more on macOS, and
    if that cost were paid *after* start() returns, the caller's "run the
    arm motion, then stop()" window would silently eat into the recording
    (fewer real frames than the elapsed wall time implies). Paying that
    cost up front means the time between start() returning and stop() being
    called maps directly onto the recorded footage.

    Example::

        rec = BackgroundRecorder("docs/media/pick.mp4", config=CameraConfig(rotate=180))
        rec.start()
        run_pick_up_sequence()  # drive the arm
        rec.stop()
    """

    def __init__(
        self,
        path: str | Path,
        camera: Optional[Camera] = None,
        config: Optional[CameraConfig] = None,
        fps: float = DEFAULT_FPS,
        overlay: bool = True,
        label: Optional[str] = None,
    ):
        self.path = Path(path)
        self._camera = camera
        self._owns_camera = camera is None
        self._config = config
        self._fps = fps
        self._overlay = overlay
        self._label = label
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._error: Optional[BaseException] = None

    def start(self) -> "BackgroundRecorder":
        if self._thread is not None:
            raise RuntimeError("BackgroundRecorder already started")
        self._stop_event.clear()
        if self._camera is None:
            self._camera = Camera(self._config or CameraConfig())
        self._camera.open()  # blocking: pays the open/warm-up cost before we return
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        return self

    def _run(self) -> None:
        try:
            record_video(
                self.path,
                camera=self._camera,
                fps=self._fps,
                overlay=self._overlay,
                label=self._label,
                stop_event=self._stop_event,
            )
        except BaseException as exc:  # surfaced to caller in stop()
            self._error = exc

    def stop(self, timeout: float = 15.0) -> Path:
        if self._thread is None:
            raise RuntimeError("BackgroundRecorder was never started")
        self._stop_event.set()
        self._thread.join(timeout)
        if self._thread.is_alive():
            raise TimeoutError(f"Recorder thread did not stop within {timeout}s")
        self._thread = None
        if self._owns_camera and self._camera is not None:
            self._camera.release()
        if self._error is not None:
            raise self._error
        return self.path


def make_gif(mp4_path: str | Path, gif_path: str | Path, fps: int = 10, width: int = 480) -> Path:
    """Convert an mp4 clip to a gif for lightweight evidence artifacts.

    Uses ffmpeg with a two-pass palette (much better quality/size than a
    naive single-pass gif) when ffmpeg is available on PATH. Falls back to
    OpenCV + imageio if ffmpeg is missing.
    """
    mp4_path, gif_path = Path(mp4_path), Path(gif_path)
    gif_path.parent.mkdir(parents=True, exist_ok=True)

    if shutil.which("ffmpeg"):
        filters = f"fps={fps},scale={width}:-1:flags=lanczos"
        palette = gif_path.with_suffix(".palette.png")
        try:
            subprocess.run(
                [
                    "ffmpeg", "-y", "-i", str(mp4_path),
                    "-vf", f"{filters},palettegen",
                    str(palette),
                ],
                check=True, capture_output=True,
            )
            subprocess.run(
                [
                    "ffmpeg", "-y", "-i", str(mp4_path), "-i", str(palette),
                    "-lavfi", f"{filters}[x];[x][1:v]paletteuse",
                    str(gif_path),
                ],
                check=True, capture_output=True,
            )
        finally:
            palette.unlink(missing_ok=True)
        return gif_path

    # Fallback: decode with cv2, encode with imageio (pure-python gif writer).
    import imageio.v2 as imageio

    cap = cv2.VideoCapture(str(mp4_path))
    src_fps = cap.get(cv2.CAP_PROP_FPS) or fps
    step = max(1, round(src_fps / fps))
    frames = []
    i = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if i % step == 0:
            h, w = frame.shape[:2]
            new_h = int(h * (width / w))
            frame = cv2.resize(frame, (width, new_h))
            frames.append(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        i += 1
    cap.release()
    imageio.mimsave(str(gif_path), frames, fps=fps)
    return gif_path
