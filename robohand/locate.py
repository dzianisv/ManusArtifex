"""``Locator``: "where is the object, and where is the gripper?", as a swappable
component.

This is the perception half of the Code-as-Policies split described in
docs/cocktail-robot.md: a VLM or a classical detector says *where things are*,
and ordinary, auditable Python (``safety.SafetyLayer``) decides *how the arm
moves*. Nothing in this module is allowed to command motion, and nothing that
comes out of it is trusted without validation.

It deliberately mirrors the shape of ``observer.py`` (ABC + real implementation +
sim ground-truth stand-in + fault injectors), because that pattern is already the
house idiom for "a signal the arm depends on that can be wrong":

* ``Locator`` -- the ABC. ``available()`` / ``locate_object()`` / ``locate_gripper()``.
* ``ClassicalLocator`` -- real camera, HSV colour threshold + contours. A thin
  wrapper over ``vision.find_objects`` / ``vision.detect_biggest``; it does not
  reimplement any detection.
* ``GeminiLocator`` -- real camera, ``gemini-robotics-er-1.6-preview`` pointing.
  Verified against this exact hardware on 2026-07-28 (docs/cocktail-robot.md),
  including the part where **it hallucinated an object that was not there**.
  Treated accordingly: one signal among several, never an authorisation to move.
* ``SimLocator`` -- reads ground truth out of ``sim.SimWorld``. A **test
  stand-in, not a camera simulation** (same disclaimer as
  ``observer.SimMotionObserver``): no noise, no latency, no lighting, no
  occlusion, no detector failure. It exists so the whole pick pipeline can be
  exercised end to end with zero camera and zero network.
* ``NullLocator`` / ``FixedPointLocator`` / ``OscillatingLocator`` /
  ``OutOfBoundsLocator`` -- deliberate fault injectors, the analogue of
  ``observer.FlakyMotionObserver`` / ``NullMotionObserver``. They make the
  convergence guard and the hallucination-rejection path testable without
  relying on real flakiness.

Coordinates
-----------
Every ``Locator`` reports **pixel** coordinates in the source frame, plus a
confidence in [0, 1] and a ``source`` tag naming the backend that produced it.
Pixels, not workspace millimetres, because a single camera on a known table plane
can only ever give you X/Y (see ``vision.Calibration``); there is no depth
measurement anywhere in this system. Callers that want workspace coordinates ask
for the optional homography themselves and must treat the result as a
table-plane assumption, never a measurement.
"""

from __future__ import annotations

import abc
import dataclasses
import json
import os
import re
import subprocess
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

import numpy as np

from . import vision

# Model id verified against our own hardware photos on 2026-07-28; see
# docs/cocktail-robot.md "Verified on our hardware".
GEMINI_MODEL = "gemini-robotics-er-1.6-preview"

# Gemini Robotics-ER returns points as [y, x] normalised to 0..1000.
_ER_POINT_SCALE = 1000.0

# Bitwarden lookup used only if GEMINI_API_KEY is not already in the environment.
# The key is read into memory and never logged, printed, or written to disk.
_BW_KEY_COMMAND = (
    "source ~/.env.d/bitwarden.env && bw get notes GEMINI_API_KEY "
    "| tr -d '\\n' | grep -oE 'AIza[A-Za-z0-9_-]{20,}' | head -1"
)


class LocatorError(RuntimeError):
    """Base class for locator problems."""


class LocatorUnavailable(LocatorError):
    """Raised when a locator backend cannot be used at all.

    Deliberately raised late (at construction or first use), never at import
    time: ``robohand.pick`` and ``robohand.cli`` must keep working with the other
    backends when, say, no Gemini key is configured or ``google-genai`` is not
    installed.
    """


@dataclasses.dataclass(frozen=True)
class ObjectLocation:
    """Where a locator believes the target object is, in source-frame pixels."""

    px: float
    py: float
    confidence: float
    source: str
    label: Optional[str] = None

    def as_dict(self) -> Dict[str, Any]:
        return dataclasses.asdict(self)


@dataclasses.dataclass(frozen=True)
class GripperLocation:
    """Where a locator believes the gripper is, in source-frame pixels."""

    px: float
    py: float
    confidence: float
    source: str
    label: Optional[str] = None

    def as_dict(self) -> Dict[str, Any]:
        return dataclasses.asdict(self)


@dataclasses.dataclass(frozen=True)
class WorkspaceEnvelope:
    """Rectangular table-plane region the arm is allowed to be pointed at.

    Only usable when a ``vision.Calibration`` homography exists -- without one
    there is no way to turn a pixel into a workspace coordinate, and callers fall
    back to plain frame-bounds rejection. Units are whatever the homography's
    ``workspace_points`` were measured in (millimetres, by convention).
    """

    x_min: float
    x_max: float
    y_min: float
    y_max: float

    def contains(self, x: float, y: float) -> bool:
        return self.x_min <= x <= self.x_max and self.y_min <= y <= self.y_max


def point_in_frame(px: float, py: float, frame_shape: Optional[Tuple[int, int]]) -> bool:
    """True if a pixel lies inside the frame. Unknown frame size => reject.

    "I do not know how big the frame is" must never resolve to "the point is
    fine" -- an unvalidated point is exactly how a hallucinated coordinate would
    reach the safety layer.
    """
    if frame_shape is None:
        return False
    height, width = frame_shape[0], frame_shape[1]
    if height <= 0 or width <= 0:
        return False
    return 0.0 <= px <= float(width - 1) and 0.0 <= py <= float(height - 1)


def point_in_workspace(
    px: float,
    py: float,
    *,
    frame_shape: Optional[Tuple[int, int]],
    calibration: Optional["vision.Calibration"] = None,
    envelope: Optional[WorkspaceEnvelope] = None,
) -> bool:
    """The single acceptance test every locator output must pass.

    Applied identically to every backend -- classical, Gemini, sim, or a test
    double. There is deliberately no per-backend bypass: a point from a large
    multimodal model gets exactly the same scrutiny as a point from a 20-line
    HSV threshold, because the model has already been observed inventing one
    (docs/cocktail-robot.md).

    With a homography and an envelope, the point must map inside the calibrated
    workspace. Without them, the weaker frame-bounds check is the best available
    and is applied instead.
    """
    if not np.isfinite(px) or not np.isfinite(py):
        return False
    if not point_in_frame(px, py, frame_shape):
        return False
    if calibration is None or envelope is None:
        return True
    try:
        x, y = calibration.pixel_to_workspace(float(px), float(py))
    except Exception:
        # A homography that cannot map this point is not a licence to use it.
        return False
    if not (np.isfinite(x) and np.isfinite(y)):
        return False
    return envelope.contains(float(x), float(y))


class Locator(abc.ABC):
    """Answers "where is the object / the gripper, in this frame?"."""

    #: (height, width) of the frames this locator reports pixels in, if known.
    frame_shape: Optional[Tuple[int, int]] = None
    #: Optional pixel -> table-plane homography, for workspace-bounds rejection.
    calibration: Optional["vision.Calibration"] = None
    #: Optional table-plane envelope, only meaningful together with ``calibration``.
    envelope: Optional[WorkspaceEnvelope] = None

    @abc.abstractmethod
    def available(self) -> bool:
        """False when this locator cannot produce any answer at all."""

    @abc.abstractmethod
    def locate_object(self, frame: Optional[np.ndarray] = None) -> Optional[ObjectLocation]:
        """The target object, or ``None`` if it is not confidently visible."""

    @abc.abstractmethod
    def locate_gripper(self, frame: Optional[np.ndarray] = None) -> Optional[GripperLocation]:
        """The gripper, or ``None`` if it is not confidently visible."""

    def capture(self) -> Optional[np.ndarray]:
        """Grab one frame, if this locator has a frame source. May return None."""
        return None

    def accepts(self, px: float, py: float) -> bool:
        """Whether a point passes this locator's workspace/frame validation."""
        return point_in_workspace(
            px,
            py,
            frame_shape=self.frame_shape,
            calibration=self.calibration,
            envelope=self.envelope,
        )

    def describe(self) -> str:
        return type(self).__name__


class ClassicalLocator(Locator):
    """HSV colour-threshold locator. Thin wrapper over ``robohand.vision``.

    The gripper is found the same way the object is: by looking for a distinctly
    coloured blob. On real hardware that means putting a coloured marker on the
    gripper (a strip of tape works); there is no appearance model of the arm
    here and this module does not pretend to have one.

    Both colours must differ from each other and from the background, or the two
    "biggest blob" queries will return the same thing.
    """

    def __init__(
        self,
        frame_source: Optional[Callable[[], Optional[np.ndarray]]] = None,
        *,
        object_color: Optional[str] = None,
        gripper_color: Optional[str] = None,
        min_area: float = 200.0,
        calibration: Optional["vision.Calibration"] = None,
        envelope: Optional[WorkspaceEnvelope] = None,
    ) -> None:
        if object_color is not None and gripper_color is not None and object_color == gripper_color:
            raise ValueError(
                "object_color and gripper_color must differ -- otherwise both queries "
                f"return the same blob (both were {object_color!r})."
            )
        self._frame_source = frame_source
        self._object_color = object_color
        self._gripper_color = gripper_color
        self._min_area = float(min_area)
        self.calibration = calibration
        self.envelope = envelope
        self.frame_shape = None

    def available(self) -> bool:
        return self._frame_source is not None

    def capture(self) -> Optional[np.ndarray]:
        if self._frame_source is None:
            return None
        frame = self._frame_source()
        if frame is not None:
            self.frame_shape = (int(frame.shape[0]), int(frame.shape[1]))
        return frame

    def _frame(self, frame: Optional[np.ndarray]) -> np.ndarray:
        if frame is None:
            frame = self.capture()
        if frame is None:
            raise LocatorUnavailable(
                "classical locator has no frame source (no camera) -- refusing to "
                "guess where anything is."
            )
        self.frame_shape = (int(frame.shape[0]), int(frame.shape[1]))
        return frame

    def _biggest(self, frame: np.ndarray, color: Optional[str]):
        ranges = {color: vision.DEFAULT_COLOR_RANGES[color]} if color else None
        detections = vision.find_objects(frame, color_ranges=ranges, min_area=self._min_area)
        return detections[0] if detections else None

    @staticmethod
    def _confidence(detection) -> float:
        # Area is the only quality signal a contour detector gives us. Map it to a
        # bounded, monotonic score; it is an ordering hint, not a probability, and
        # is documented as such.
        area = float(detection["area"])
        return float(min(1.0, area / (area + 1500.0) + 0.25))

    def locate_object(self, frame: Optional[np.ndarray] = None) -> Optional[ObjectLocation]:
        image = self._frame(frame)
        detection = self._biggest(image, self._object_color)
        if detection is None:
            return None
        if not self.accepts(detection["cx"], detection["cy"]):
            return None
        return ObjectLocation(
            px=float(detection["cx"]),
            py=float(detection["cy"]),
            confidence=self._confidence(detection),
            source="classical",
            label=detection["color"],
        )

    def locate_gripper(self, frame: Optional[np.ndarray] = None) -> Optional[GripperLocation]:
        image = self._frame(frame)
        detection = self._biggest(image, self._gripper_color)
        if detection is None:
            return None
        if not self.accepts(detection["cx"], detection["cy"]):
            return None
        return GripperLocation(
            px=float(detection["cx"]),
            py=float(detection["cy"]),
            confidence=self._confidence(detection),
            source="classical",
            label=detection["color"],
        )

    def describe(self) -> str:
        return (
            f"ClassicalLocator(object_color={self._object_color!r}, "
            f"gripper_color={self._gripper_color!r}, min_area={self._min_area})"
        )


def resolve_gemini_api_key(*, allow_bitwarden: bool = True, timeout_s: float = 20.0) -> Optional[str]:
    """Find a Gemini API key at *runtime*, in priority order. Never logs it.

    1. ``GEMINI_API_KEY`` in the environment.
    2. Bitwarden item ``GEMINI_API_KEY`` (the value lives in the item **notes**,
       not the password field -- see docs/cocktail-robot.md).

    Returns ``None`` when neither yields a key. The key is never printed,
    logged, written to a file, or included in an exception message.
    """
    env_key = os.environ.get("GEMINI_API_KEY", "").strip()
    if env_key:
        return env_key
    if not allow_bitwarden:
        return None
    try:
        completed = subprocess.run(
            ["bash", "-lc", _BW_KEY_COMMAND],
            capture_output=True,
            text=True,
            timeout=timeout_s,
        )
    except (OSError, subprocess.SubprocessError):
        return None
    candidate = (completed.stdout or "").strip()
    # Re-validate the shape here too: never hand an arbitrary blob of stdout
    # (which could be an error message, a prompt, or an unlock nag) onward as a
    # credential.
    match = re.fullmatch(r"AIza[A-Za-z0-9_-]{20,}", candidate)
    return match.group(0) if match else None


_ER_PROMPT_TEMPLATE = (
    "You are looking at a photo of a tabletop robot arm workspace.\n"
    "Point at: {target}.\n"
    "Return ONLY a JSON list, no prose, no markdown fences, in exactly this form:\n"
    '[{{"point": [y, x], "label": "<short label>"}}]\n'
    "y and x are normalized to 0-1000 with the origin at the top-left of the image.\n"
    "If the requested thing is NOT clearly visible in this image, return an empty "
    "list [] and nothing else. Do NOT guess, do NOT substitute a similar or "
    "background object, and do NOT invent a point. An empty list is the correct, "
    "expected answer when the thing is absent."
)


class GeminiLocator(Locator):
    """Gemini Robotics-ER 1.6 pointing, wrapped in the same contract as every
    other locator.

    Verified against a real photo of this arm on 2026-07-28
    (docs/cocktail-robot.md): the gripper point was ~20-30 px off, and asked for
    "a graspable object on the table" in a frame containing none, it **pointed at
    a background box instead of returning ``[]``**. That is the design
    constraint, not a footnote:

    * the prompt explicitly demands ``[]`` when the target is absent;
    * every returned point is shape-validated, range-validated, converted using
      the *actual* frame size, and then run through the same
      ``point_in_workspace`` acceptance test as any other backend;
    * a rejected point is reported as "not found", never as a usable location;
    * ``robohand.pick`` re-validates independently and still requires the
      look-move-look-again loop to converge. **A single Gemini response never
      authorises motion by itself.**

    ``google.genai`` is imported lazily, inside the method that needs it, so the
    rest of the package has no hard dependency on it. The API key is resolved at
    call time and never stored on the instance in a form that gets logged.
    """

    def __init__(
        self,
        frame_source: Optional[Callable[[], Optional[np.ndarray]]] = None,
        *,
        object_prompt: str = "the graspable object on the table",
        gripper_prompt: str = "the tip of the robot arm's gripper/claw",
        model: str = GEMINI_MODEL,
        client: Optional[Any] = None,
        allow_bitwarden: bool = True,
        calibration: Optional["vision.Calibration"] = None,
        envelope: Optional[WorkspaceEnvelope] = None,
    ) -> None:
        self._frame_source = frame_source
        self._object_prompt = object_prompt
        self._gripper_prompt = gripper_prompt
        self._model = model
        self._client = client
        self._allow_bitwarden = allow_bitwarden
        self.calibration = calibration
        self.envelope = envelope
        self.frame_shape = None
        #: Raw rejected candidates from the last call, for diagnostics. Contains
        #: only coordinates and labels -- never the key, never the raw response.
        self.rejected: List[dict] = []

    # -- plumbing ----------------------------------------------------------

    def available(self) -> bool:
        if self._frame_source is None and self._client is None:
            return False
        if self._client is not None:
            return True
        return resolve_gemini_api_key(allow_bitwarden=self._allow_bitwarden) is not None

    def capture(self) -> Optional[np.ndarray]:
        if self._frame_source is None:
            return None
        frame = self._frame_source()
        if frame is not None:
            self.frame_shape = (int(frame.shape[0]), int(frame.shape[1]))
        return frame

    def _get_client(self):
        if self._client is not None:
            return self._client
        api_key = resolve_gemini_api_key(allow_bitwarden=self._allow_bitwarden)
        if not api_key:
            raise LocatorUnavailable(
                "no Gemini API key available (checked GEMINI_API_KEY, then the "
                "Bitwarden 'GEMINI_API_KEY' item notes). Use --locator classical "
                "or --locator sim, or configure the key. The key value is never "
                "printed by this package."
            )
        try:
            from google import genai  # noqa: WPS433 -- deliberately lazy
        except ImportError as e:  # pragma: no cover - depends on the environment
            raise LocatorUnavailable(
                "the 'google-genai' package is not installed, so the Gemini locator "
                "cannot be used. Install it or choose another --locator backend."
            ) from e
        self._client = genai.Client(api_key=api_key)
        return self._client

    @staticmethod
    def _encode(frame: np.ndarray) -> bytes:
        import cv2  # noqa: WPS433 -- already a hard dep of vision/guard, kept local

        ok, buffer = cv2.imencode(".jpg", frame)
        if not ok:
            raise LocatorUnavailable("could not JPEG-encode the frame for the Gemini API")
        return buffer.tobytes()

    def _ask(self, frame: np.ndarray, target: str) -> str:
        client = self._get_client()
        from google.genai import types  # noqa: WPS433 -- deliberately lazy

        response = client.models.generate_content(
            model=self._model,
            contents=[
                types.Part.from_bytes(data=self._encode(frame), mime_type="image/jpeg"),
                _ER_PROMPT_TEMPLATE.format(target=target),
            ],
        )
        return getattr(response, "text", "") or ""

    # -- response parsing (strict on purpose) ------------------------------

    @staticmethod
    def parse_points(text: str) -> List[Tuple[float, float, Optional[str]]]:
        """Parse an ER response into ``[(y_norm, x_norm, label), ...]``.

        Strict by design: anything that is not a JSON list of objects each
        carrying a 2-element numeric ``point`` inside [0, 1000] is dropped.
        Malformed output is treated as "nothing found", never as a partial
        result to be salvaged. Returns ``[]`` for an empty list, for prose, for
        a truncated response, and for a response of the wrong shape alike.
        """
        if not text:
            return []
        stripped = text.strip()
        # Models often wrap JSON in ```json fences despite being told not to.
        fence = re.match(r"^```(?:json)?\s*(.*?)\s*```$", stripped, re.DOTALL)
        if fence:
            stripped = fence.group(1).strip()
        try:
            payload = json.loads(stripped)
        except (ValueError, TypeError):
            return []
        if not isinstance(payload, list):
            return []
        out: List[Tuple[float, float, Optional[str]]] = []
        for item in payload:
            if not isinstance(item, dict):
                continue
            point = item.get("point")
            if not isinstance(point, (list, tuple)) or len(point) != 2:
                continue
            if any(isinstance(v, bool) or not isinstance(v, (int, float)) for v in point):
                continue
            y_norm, x_norm = float(point[0]), float(point[1])
            if not (np.isfinite(y_norm) and np.isfinite(x_norm)):
                continue
            if not (0.0 <= y_norm <= _ER_POINT_SCALE and 0.0 <= x_norm <= _ER_POINT_SCALE):
                continue
            label = item.get("label")
            out.append((y_norm, x_norm, label if isinstance(label, str) else None))
        return out

    def _locate(self, frame: Optional[np.ndarray], target: str):
        if frame is None:
            frame = self.capture()
        if frame is None:
            raise LocatorUnavailable(
                "Gemini locator has no frame source (no camera) -- refusing to ask a "
                "model to point at an image that does not exist."
            )
        self.frame_shape = (int(frame.shape[0]), int(frame.shape[1]))
        height, width = self.frame_shape
        self.rejected = []
        for y_norm, x_norm, label in self.parse_points(self._ask(frame, target)):
            # Convert with the ACTUAL frame size, not an assumed one.
            px = x_norm / _ER_POINT_SCALE * (width - 1)
            py = y_norm / _ER_POINT_SCALE * (height - 1)
            if not self.accepts(px, py):
                self.rejected.append({"px": px, "py": py, "label": label, "reason": "out_of_workspace"})
                continue
            # ER gives no calibrated confidence. 0.5 is an honest "this is a
            # coarse semantic hint", matching the measured ~20-30 px error.
            return px, py, label
        return None

    def locate_object(self, frame: Optional[np.ndarray] = None) -> Optional[ObjectLocation]:
        found = self._locate(frame, self._object_prompt)
        if found is None:
            return None
        px, py, label = found
        return ObjectLocation(px=px, py=py, confidence=0.5, source="gemini-er", label=label)

    def locate_gripper(self, frame: Optional[np.ndarray] = None) -> Optional[GripperLocation]:
        found = self._locate(frame, self._gripper_prompt)
        if found is None:
            return None
        px, py, label = found
        return GripperLocation(px=px, py=py, confidence=0.5, source="gemini-er", label=label)

    def describe(self) -> str:
        return f"GeminiLocator(model={self._model!r}, hallucination-guarded)"


class SimLocator(Locator):
    """Ground-truth locator for ``sim.SimWorld``. **Not** a camera simulation.

    Same disclaimer as ``observer.SimMotionObserver``: this reads the simulator's
    own bookkeeping and projects it with a fixed toy camera model. It has no
    sensor noise, no lighting, no latency, no motion blur, no occlusion, no
    detector false positives, and it never fails to find something that exists.
    Every one of those is a real way the real locators are wrong. Its only job is
    to make ``robohand.pick``'s control flow testable end to end with no camera
    and no network.
    """

    def __init__(self, world) -> None:
        self._world = world
        self.frame_shape = tuple(world.frame_shape)

    def available(self) -> bool:
        return True

    def locate_object(self, frame: Optional[np.ndarray] = None) -> Optional[ObjectLocation]:
        point = self._world.object_pixel()
        if point is None:
            return None
        px, py = point
        if not self.accepts(px, py):
            return None
        return ObjectLocation(px=px, py=py, confidence=1.0, source="sim-ground-truth", label="sim-object")

    def locate_gripper(self, frame: Optional[np.ndarray] = None) -> Optional[GripperLocation]:
        px, py = self._world.gripper_pixel()
        if not self.accepts(px, py):
            return None
        return GripperLocation(px=px, py=py, confidence=1.0, source="sim-ground-truth", label="sim-gripper")

    def describe(self) -> str:
        return f"SimLocator(frame={self.frame_shape}, SYNTHETIC -- ground truth, not a camera)"


# ---------------------------------------------------------------------------
# Fault injectors. Analogues of observer.FlakyMotionObserver / NullMotionObserver.
# ---------------------------------------------------------------------------


class NullLocator(Locator):
    """Available, but never finds anything. A lens cap, or an empty table."""

    def __init__(self, frame_shape: Tuple[int, int] = (480, 640)) -> None:
        self.frame_shape = frame_shape
        self.calls = 0

    def available(self) -> bool:
        return True

    def locate_object(self, frame: Optional[np.ndarray] = None) -> Optional[ObjectLocation]:
        self.calls += 1
        return None

    def locate_gripper(self, frame: Optional[np.ndarray] = None) -> Optional[GripperLocation]:
        self.calls += 1
        return None

    def describe(self) -> str:
        return "NullLocator(finds nothing, ever)"


class UnavailableLocator(Locator):
    """A locator that reports itself unusable -- e.g. no camera was opened."""

    def available(self) -> bool:
        return False

    def locate_object(self, frame: Optional[np.ndarray] = None) -> Optional[ObjectLocation]:
        raise LocatorUnavailable("this locator has no frame source")

    def locate_gripper(self, frame: Optional[np.ndarray] = None) -> Optional[GripperLocation]:
        raise LocatorUnavailable("this locator has no frame source")


class FixedPointLocator(Locator):
    """Always reports the same two points, whatever the arm actually does.

    Models a detector locked onto the wrong thing (a reflection, a static
    background object). The gripper never appears to move, so no amount of
    servoing reduces the error -- exactly the situation the convergence guard
    exists for.
    """

    def __init__(
        self,
        object_point: Tuple[float, float],
        gripper_point: Tuple[float, float],
        frame_shape: Tuple[int, int] = (480, 640),
    ) -> None:
        self._object_point = object_point
        self._gripper_point = gripper_point
        self.frame_shape = frame_shape

    def available(self) -> bool:
        return True

    def locate_object(self, frame: Optional[np.ndarray] = None) -> Optional[ObjectLocation]:
        px, py = self._object_point
        return ObjectLocation(px=px, py=py, confidence=0.9, source="fixed", label="fixed-object")

    def locate_gripper(self, frame: Optional[np.ndarray] = None) -> Optional[GripperLocation]:
        px, py = self._gripper_point
        return GripperLocation(px=px, py=py, confidence=0.9, source="fixed", label="fixed-gripper")

    def describe(self) -> str:
        return "FixedPointLocator(never moves)"


class OscillatingLocator(Locator):
    """Reports a gripper that flips between two points on alternate calls.

    Models a detector that keeps latching onto two similar blobs. The measured
    error never settles, so a naive servo loop would burn its whole step budget;
    the convergence guard must catch it first.
    """

    def __init__(
        self,
        object_point: Tuple[float, float],
        gripper_points: Sequence[Tuple[float, float]],
        frame_shape: Tuple[int, int] = (480, 640),
    ) -> None:
        if len(gripper_points) < 2:
            raise ValueError("OscillatingLocator needs at least two gripper points")
        self._object_point = object_point
        self._gripper_points = list(gripper_points)
        self.frame_shape = frame_shape
        self.calls = 0

    def available(self) -> bool:
        return True

    def locate_object(self, frame: Optional[np.ndarray] = None) -> Optional[ObjectLocation]:
        px, py = self._object_point
        return ObjectLocation(px=px, py=py, confidence=0.9, source="oscillating", label="object")

    def locate_gripper(self, frame: Optional[np.ndarray] = None) -> Optional[GripperLocation]:
        px, py = self._gripper_points[self.calls % len(self._gripper_points)]
        self.calls += 1
        return GripperLocation(px=px, py=py, confidence=0.9, source="oscillating", label="gripper")

    def describe(self) -> str:
        return f"OscillatingLocator(cycles through {len(self._gripper_points)} gripper points)"


class OutOfBoundsLocator(Locator):
    """Reports coordinates outside the frame -- the hallucination stand-in.

    Note the ``accepts()`` check lives in the *base* class and in
    ``robohand.pick``, so this double does not need to cooperate: it hands out
    garbage and the pipeline is required to throw it away. That is the point.
    """

    def __init__(
        self,
        frame_shape: Tuple[int, int] = (480, 640),
        object_point: Tuple[float, float] = (5000.0, -900.0),
        gripper_point: Tuple[float, float] = (320.0, 240.0),
    ) -> None:
        self.frame_shape = frame_shape
        self._object_point = object_point
        self._gripper_point = gripper_point

    def available(self) -> bool:
        return True

    def locate_object(self, frame: Optional[np.ndarray] = None) -> Optional[ObjectLocation]:
        px, py = self._object_point
        return ObjectLocation(px=px, py=py, confidence=0.99, source="out-of-bounds", label="hallucination")

    def locate_gripper(self, frame: Optional[np.ndarray] = None) -> Optional[GripperLocation]:
        px, py = self._gripper_point
        return GripperLocation(px=px, py=py, confidence=0.99, source="out-of-bounds", label="gripper")

    def describe(self) -> str:
        return "OutOfBoundsLocator(returns coordinates off the frame)"


def locator_for_arm(arm, backend: str = "sim", **kwargs) -> Locator:
    """Build the requested locator backend for ``arm``.

    ``sim`` requires a simulated arm carrying a ``sim.SimWorld``; the real
    backends require a camera. Raises ``LocatorUnavailable`` rather than silently
    degrading -- a pick with no perception is not a degraded pick, it is an
    open-loop lunge, which is exactly what this package exists to prevent.
    """
    backend = (backend or "").strip().lower()
    if backend not in ("sim", "classical", "gemini"):
        raise ValueError(f"unknown locator backend {backend!r}; known: sim, classical, gemini")
    if backend == "sim":
        world = getattr(arm, "world", None) or getattr(getattr(arm, "_transport", None), "world", None)
        if world is None:
            raise LocatorUnavailable(
                "the 'sim' locator needs a simulated arm with a sim.SimWorld attached "
                "(see sim.attach_world). It reads simulator ground truth and cannot "
                "run against real hardware."
            )
        return SimLocator(world)

    guard = getattr(arm, "vision_guard", None)
    frame_source = None
    if guard is not None and guard.available():
        frame_source = guard.capture
    if frame_source is None:
        raise LocatorUnavailable(
            f"the {backend!r} locator needs a camera, and none is available. Attach a "
            "camera, or run against the simulator with --sim --locator sim."
        )
    if backend == "classical":
        return ClassicalLocator(frame_source, **kwargs)
    return GeminiLocator(frame_source, **kwargs)
