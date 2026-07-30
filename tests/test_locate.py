"""Proof that the locator layer produces usable answers, refuses to produce
unusable ones, and never lets a hallucinated coordinate through.

Everything here is offline: no camera, no HID device, no network, no API key.
The Gemini backend is exercised against an injected fake client, so CI never
needs a live key and no key value is ever produced, logged, or asserted on.
``tests/conftest.py`` forces ``ROBOHAND_TRANSPORT=sim`` for every test.
"""

from __future__ import annotations

import numpy as np
import pytest

from robohand import locate, sim
from robohand.locate import (
    ClassicalLocator,
    FixedPointLocator,
    GeminiLocator,
    LocatorUnavailable,
    NullLocator,
    ObjectLocation,
    OscillatingLocator,
    OutOfBoundsLocator,
    SimLocator,
    UnavailableLocator,
    WorkspaceEnvelope,
    point_in_frame,
    point_in_workspace,
)

FRAME_SHAPE = (480, 640)


def make_frame(blobs) -> np.ndarray:
    """A black BGR frame with solid colour squares at the given pixel centres."""
    frame = np.zeros((FRAME_SHAPE[0], FRAME_SHAPE[1], 3), dtype=np.uint8)
    for (cx, cy), bgr, half in blobs:
        frame[cy - half : cy + half, cx - half : cx + half] = bgr
    return frame


# ---------------------------------------------------------------------------
# Point validation -- the single acceptance test every backend goes through
# ---------------------------------------------------------------------------


def test_unknown_frame_size_rejects_every_point():
    """'I don't know how big the frame is' must never resolve to 'the point is fine'."""
    assert point_in_frame(10.0, 10.0, None) is False
    assert point_in_workspace(10.0, 10.0, frame_shape=None) is False


@pytest.mark.parametrize(
    "px, py",
    [(-1.0, 10.0), (10.0, -1.0), (640.0, 10.0), (10.0, 480.0), (5000.0, -900.0)],
)
def test_points_outside_the_frame_are_rejected(px, py):
    assert point_in_workspace(px, py, frame_shape=FRAME_SHAPE) is False


def test_non_finite_points_are_rejected():
    assert point_in_workspace(float("nan"), 10.0, frame_shape=FRAME_SHAPE) is False
    assert point_in_workspace(float("inf"), 10.0, frame_shape=FRAME_SHAPE) is False


def test_workspace_envelope_rejects_in_frame_but_off_table_points():
    """A pixel can be perfectly valid and still point somewhere the arm may not go."""
    calibration = _identity_calibration()
    envelope = WorkspaceEnvelope(x_min=100.0, x_max=400.0, y_min=100.0, y_max=400.0)
    assert point_in_workspace(200.0, 200.0, frame_shape=FRAME_SHAPE,
                              calibration=calibration, envelope=envelope) is True
    # In frame, but the homography maps it outside the calibrated table region.
    assert point_in_workspace(600.0, 50.0, frame_shape=FRAME_SHAPE,
                              calibration=calibration, envelope=envelope) is False


def _identity_calibration():
    from robohand.vision import Calibration

    points = [(0.0, 0.0), (640.0, 0.0), (640.0, 480.0), (0.0, 480.0)]
    calib = Calibration(pixel_points=points, workspace_points=list(points))
    calib.solve()
    return calib


# ---------------------------------------------------------------------------
# ClassicalLocator -- delegates to vision.find_objects, never reimplements it
# ---------------------------------------------------------------------------


def test_classical_locator_finds_object_and_gripper_by_colour():
    frame = make_frame([
        ((200, 150), (0, 0, 255), 25),    # red -> the object
        ((420, 300), (255, 0, 0), 25),    # blue -> the gripper marker
    ])
    loc = ClassicalLocator(lambda: frame, object_color="red", gripper_color="blue")
    assert loc.available() is True

    obj = loc.locate_object()
    grip = loc.locate_gripper()
    assert obj is not None and grip is not None
    assert obj.label == "red" and grip.label == "blue"
    assert abs(obj.px - 200) <= 2 and abs(obj.py - 150) <= 2
    assert abs(grip.px - 420) <= 2 and abs(grip.py - 300) <= 2
    assert obj.source == "classical" and 0.0 <= obj.confidence <= 1.0
    assert loc.frame_shape == FRAME_SHAPE


def test_classical_locator_returns_none_when_nothing_matches():
    loc = ClassicalLocator(lambda: make_frame([]), object_color="red", gripper_color="blue")
    assert loc.locate_object() is None
    assert loc.locate_gripper() is None


def test_classical_locator_without_a_camera_refuses_rather_than_guessing():
    loc = ClassicalLocator(None, object_color="red", gripper_color="blue")
    assert loc.available() is False
    with pytest.raises(LocatorUnavailable):
        loc.locate_object()


def test_classical_locator_rejects_identical_object_and_gripper_colours():
    with pytest.raises(ValueError, match="must differ"):
        ClassicalLocator(lambda: make_frame([]), object_color="red", gripper_color="red")


def test_classical_locator_drops_detections_outside_the_workspace():
    """Even a real detection is discarded when it maps off the calibrated table."""
    frame = make_frame([((600, 40), (0, 0, 255), 25)])
    envelope = WorkspaceEnvelope(x_min=100.0, x_max=400.0, y_min=100.0, y_max=400.0)
    loc = ClassicalLocator(
        lambda: frame, object_color="red", gripper_color="blue",
        calibration=_identity_calibration(), envelope=envelope,
    )
    loc.capture()  # establish frame_shape
    assert loc.locate_object() is None


# ---------------------------------------------------------------------------
# GeminiLocator -- strict parsing, hallucination rejection, no key ever leaks
# ---------------------------------------------------------------------------


class FakeResponse:
    def __init__(self, text):
        self.text = text


class FakeGeminiClient:
    """Stands in for ``google.genai.Client``. No network, no key."""

    def __init__(self, replies):
        self._replies = list(replies)
        self.calls = []
        self.models = self

    def generate_content(self, *, model, contents):
        self.calls.append({"model": model, "contents": contents})
        return FakeResponse(self._replies.pop(0) if self._replies else "")


@pytest.mark.parametrize(
    "text, expected",
    [
        ('[{"point": [500, 250], "label": "cup"}]', [(500.0, 250.0, "cup")]),
        ("[]", []),
        ('```json\n[{"point": [10, 20], "label": "x"}]\n```', [(10.0, 20.0, "x")]),
        ("I could not find it, sorry.", []),                      # prose
        ('{"point": [10, 20]}', []),                              # object, not a list
        ('[{"point": [10]}]', []),                                # wrong arity
        ('[{"point": [10, 20, 30]}]', []),                        # wrong arity
        ('[{"point": ["a", "b"]}]', []),                          # non-numeric
        ('[{"point": [true, false]}]', []),                       # bools are not numbers
        ('[{"point": [1200, 20]}]', []),                          # out of the 0..1000 range
        ('[{"point": [-5, 20]}]', []),                            # negative
        ('[{"label": "no point here"}]', []),                     # missing key
        ('[{"point": [10, 20], "label": "x"}', []),               # truncated JSON
        ("", []),
    ],
)
def test_gemini_response_parsing_is_strict(text, expected):
    assert GeminiLocator.parse_points(text) == expected


def test_gemini_locator_converts_normalised_points_with_the_real_frame_size():
    # ER returns [y, x] normalised 0..1000.
    client = FakeGeminiClient(['[{"point": [500, 250], "label": "cup"}]'])
    loc = GeminiLocator(lambda: make_frame([]), client=client, allow_bitwarden=False)
    found = loc.locate_object()
    assert found is not None
    assert found.px == pytest.approx(250 / 1000 * (640 - 1), abs=0.01)
    assert found.py == pytest.approx(500 / 1000 * (480 - 1), abs=0.01)
    assert found.source == "gemini-er"
    assert client.calls[0]["model"] == locate.GEMINI_MODEL


def test_gemini_prompt_demands_an_empty_list_when_nothing_is_there():
    """The 2026-07-28 hallucination, encoded as a prompt requirement."""
    client = FakeGeminiClient(["[]"])
    loc = GeminiLocator(lambda: make_frame([]), client=client, allow_bitwarden=False)
    assert loc.locate_object() is None
    prompt = [c for c in client.calls[0]["contents"] if isinstance(c, str)][0]
    assert "empty list" in prompt
    assert "Do NOT guess" in prompt


def test_gemini_hallucination_outside_the_frame_is_rejected_not_used():
    """A confident point that is not on the table is 'not found', never a target."""
    client = FakeGeminiClient(['[{"point": [999, 999], "label": "invented"}]'])
    envelope = WorkspaceEnvelope(x_min=100.0, x_max=400.0, y_min=100.0, y_max=400.0)
    loc = GeminiLocator(
        lambda: make_frame([]), client=client, allow_bitwarden=False,
        calibration=_identity_calibration(), envelope=envelope,
    )
    assert loc.locate_object() is None
    assert loc.rejected and loc.rejected[0]["reason"] == "out_of_workspace"


def test_gemini_locator_without_a_key_raises_a_catchable_error(monkeypatch):
    monkeypatch.delenv("GEMINI_API_KEY", raising=False)
    loc = GeminiLocator(lambda: make_frame([]), allow_bitwarden=False)
    assert loc.available() is False
    with pytest.raises(LocatorUnavailable) as excinfo:
        loc.locate_object()
    # The refusal must name the problem without ever quoting a credential.
    assert "AIza" not in str(excinfo.value)


def test_importing_locate_does_not_require_google_genai():
    """``google.genai`` must be imported lazily, never at module import time."""
    import ast
    import pathlib

    source = pathlib.Path(locate.__file__).read_text()
    tree = ast.parse(source)
    for node in tree.body:  # module level only
        if isinstance(node, (ast.Import, ast.ImportFrom)):
            name = getattr(node, "module", "") or ""
            names = [a.name for a in node.names]
            assert "genai" not in name and not any("genai" in n for n in names)


def test_api_key_resolution_prefers_the_environment(monkeypatch):
    monkeypatch.setenv("GEMINI_API_KEY", "AIza" + "x" * 30)
    # allow_bitwarden=True, but the env var must short-circuit it: if the
    # Bitwarden shell-out ran, this would be slow and could prompt.
    called = []
    monkeypatch.setattr(locate.subprocess, "run", lambda *a, **k: called.append(a))
    assert locate.resolve_gemini_api_key() == "AIza" + "x" * 30
    assert called == []


def test_api_key_resolution_rejects_junk_from_the_vault(monkeypatch):
    """Never hand arbitrary stdout onward as a credential."""
    monkeypatch.delenv("GEMINI_API_KEY", raising=False)

    class Done:
        stdout = "Vault is locked. Run 'bw unlock'.\n"

    monkeypatch.setattr(locate.subprocess, "run", lambda *a, **k: Done())
    assert locate.resolve_gemini_api_key() is None


def test_api_key_resolution_survives_a_missing_bitwarden(monkeypatch):
    monkeypatch.delenv("GEMINI_API_KEY", raising=False)

    def boom(*a, **k):
        raise OSError("bw: command not found")

    monkeypatch.setattr(locate.subprocess, "run", boom)
    assert locate.resolve_gemini_api_key() is None


# ---------------------------------------------------------------------------
# SimLocator -- ground truth, explicitly not a camera
# ---------------------------------------------------------------------------


def test_sim_locator_reports_simulator_ground_truth():
    transport = sim.SimTransport()
    world = sim.attach_world(transport, sim.SimObject(x=10.0, y=140.0))
    loc = SimLocator(world)
    assert loc.available() is True

    obj = loc.locate_object()
    grip = loc.locate_gripper()
    assert obj is not None and grip is not None
    assert (obj.px, obj.py) == pytest.approx(world.object_pixel())
    assert (grip.px, grip.py) == pytest.approx(world.gripper_pixel())
    assert obj.source == "sim-ground-truth"
    assert "not a camera" in loc.describe()


def test_sim_locator_reports_nothing_on_an_empty_table():
    world = sim.attach_world(sim.SimTransport(), None)
    loc = SimLocator(world)
    assert loc.locate_object() is None
    assert loc.locate_gripper() is not None


# ---------------------------------------------------------------------------
# Fault injectors
# ---------------------------------------------------------------------------


def test_null_locator_is_available_but_blind():
    loc = NullLocator()
    assert loc.available() is True
    assert loc.locate_object() is None
    assert loc.locate_gripper() is None
    assert loc.calls == 2


def test_unavailable_locator_raises_rather_than_returning_none():
    loc = UnavailableLocator()
    assert loc.available() is False
    with pytest.raises(LocatorUnavailable):
        loc.locate_object()


def test_fixed_point_locator_never_moves():
    loc = FixedPointLocator((100.0, 100.0), (300.0, 300.0))
    first = loc.locate_gripper()
    second = loc.locate_gripper()
    assert (first.px, first.py) == (second.px, second.py)


def test_oscillating_locator_alternates():
    loc = OscillatingLocator((320.0, 240.0), [(100.0, 100.0), (500.0, 400.0)])
    seen = [(loc.locate_gripper().px, loc.locate_gripper().px) for _ in range(2)]
    assert seen == [(100.0, 500.0), (100.0, 500.0)]
    with pytest.raises(ValueError):
        OscillatingLocator((0.0, 0.0), [(1.0, 1.0)])


def test_out_of_bounds_locator_output_fails_the_base_class_acceptance_check():
    """The double does not cooperate -- the pipeline is required to reject it."""
    loc = OutOfBoundsLocator()
    hallucination = loc.locate_object()
    assert isinstance(hallucination, ObjectLocation)
    assert loc.accepts(hallucination.px, hallucination.py) is False
    real = loc.locate_gripper()
    assert loc.accepts(real.px, real.py) is True


# ---------------------------------------------------------------------------
# locator_for_arm
# ---------------------------------------------------------------------------


def test_locator_for_arm_sim_backend_needs_a_world():
    from robohand.arm import Arm

    with Arm(sim=True, use_vision=False, install_signal_handlers=False) as arm:
        with pytest.raises(LocatorUnavailable, match="SimWorld"):
            locate.locator_for_arm(arm, "sim")
        sim.attach_world(arm._transport, sim.SimObject(x=0.0, y=130.0))
        assert isinstance(locate.locator_for_arm(arm, "sim"), SimLocator)


def test_locator_for_arm_camera_backends_refuse_without_a_camera():
    from robohand.arm import Arm

    with Arm(sim=True, use_vision=False, install_signal_handlers=False) as arm:
        for backend in ("classical", "gemini"):
            with pytest.raises(LocatorUnavailable, match="camera"):
                locate.locator_for_arm(arm, backend)


def test_locator_for_arm_rejects_unknown_backends():
    from robohand.arm import Arm

    with Arm(sim=True, use_vision=False, install_signal_handlers=False) as arm:
        sim.attach_world(arm._transport, sim.SimObject(x=0.0, y=130.0))
        with pytest.raises(ValueError, match="unknown locator backend"):
            locate.locator_for_arm(arm, "telepathy")


def test_missing_google_genai_degrades_even_when_a_client_is_injected(monkeypatch):
    """A missing optional SDK must surface as LocatorUnavailable, never ModuleNotFoundError.

    Regression for a real drift: ``_get_client`` guarded its ``google.genai``
    import, but a second, later ``from google.genai import types`` inside
    ``_ask`` did not. Whenever the client was injected or already cached,
    ``_get_client`` never ran its guard, so an absent package escaped as a raw
    ``ModuleNotFoundError`` from the middle of a pick -- breaking the contract
    documented on ``LocatorUnavailable`` that the other backends keep working.
    """
    import importlib

    real_import_module = importlib.import_module

    def fake_import_module(name, *args, **kwargs):
        if name.startswith("google.genai"):
            raise ImportError(f"No module named {name!r}")
        return real_import_module(name, *args, **kwargs)

    monkeypatch.setattr(importlib, "import_module", fake_import_module)

    # An injected client means _get_client returns without ever importing the SDK,
    # so this only passes if the *second* import site is guarded too.
    loc = GeminiLocator(
        lambda: make_frame([]),
        client=FakeGeminiClient(["[]"]),
        allow_bitwarden=False,
    )
    with pytest.raises(LocatorUnavailable) as excinfo:
        loc.locate_object()
    assert "google-genai" in str(excinfo.value)
