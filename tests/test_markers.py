"""Fiducial markers as the state estimator, tested against the failure that
motivated them.

The absdiff observer answers "how many pixels changed", which is a question about
the *room*. These tests pin the property that makes markers a different kind of
answer: the detector is looking for one specific printed pattern, so everything
else in the room -- a person, a pet, a reflection, an exposure shift -- is not
merely below a threshold, it is absent from the result entirely.

No camera and no arm: every test either renders a marker itself or reads a real
frame committed to the repo.
"""
from __future__ import annotations

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")

from robohand import markers
from robohand.observer import ObserverError


def _sheet(ids=(0, 1)):
    return cv2.cvtColor(markers.render_sheet(ids, px=200, border_px=40), cv2.COLOR_GRAY2BGR)


def _scene(marker_id=0, at=(300, 200), size=120, noise_seed=None, brightness=0):
    """A 640x480 scene with one marker pasted at ``at``, over optional clutter."""
    frame = np.full((480, 640, 3), 200, dtype=np.uint8)
    if noise_seed is not None:
        rng = np.random.default_rng(noise_seed)
        # Big, high-contrast, moving clutter: a stand-in for a person in shot.
        frame[:, 380:] = rng.integers(0, 255, size=(480, 260, 3), dtype=np.uint8)
    if brightness:
        frame = np.clip(frame.astype(np.int16) + brightness, 0, 255).astype(np.uint8)
    tile = cv2.cvtColor(
        cv2.resize(
            markers.render_sheet([marker_id], px=size, border_px=size // 4),
            (size + size // 2, size + size // 2),
            interpolation=cv2.INTER_NEAREST,
        ),
        cv2.COLOR_GRAY2BGR,
    )
    h, w = tile.shape[:2]
    x, y = at
    frame[y:y + h, x:x + w] = tile
    return frame


# ---------------------------------------------------------------------------
# 1. detection basics
# ---------------------------------------------------------------------------

def test_a_rendered_marker_round_trips_through_the_detector():
    found = markers.detect(_sheet((0, 1, 2)))
    assert sorted(found) == [0, 1, 2]
    assert all(isinstance(v, markers.MarkerView) for v in found.values())


def test_a_marker_reports_its_centre_and_edge_length():
    view = markers.detect(_sheet((0,)))[0]
    assert view.size_px > 0
    assert view.as_dict()["id"] == 0


def test_a_frame_with_no_markers_returns_an_empty_result_not_an_error():
    """Absence must be ordinary. The calibrator has to be able to ask and be told
    'nothing', without that being an exception it might swallow."""
    assert markers.detect(np.full((100, 100, 3), 128, dtype=np.uint8)) == {}


def test_a_real_room_photograph_contains_no_markers():
    """The headline property, on real pixels rather than synthetic ones: a photo of
    a room -- furniture, a screen, a person's arm -- yields nothing. There is no
    threshold to tune here; a person simply is not a marker."""
    frame = cv2.imread("tests/fixtures/room_with_person.jpg")
    assert frame is not None, "fixture missing"
    assert markers.detect(frame) == {}


# ---------------------------------------------------------------------------
# 2. the observer, against the noise that defeated absdiff
# ---------------------------------------------------------------------------

def test_motion_is_the_markers_displacement_in_pixels():
    """The score is a distance, not a statistic. A 40 px move reads as 40."""
    frames = [_scene(at=(300, 200)), _scene(at=(340, 200))]
    obs = markers.ArucoMotionObserver(lambda: frames.pop(0), marker_id=0)
    before, after = obs.capture(), obs.capture()
    assert obs.score(before, after) == pytest.approx(40, abs=1.0)
    assert obs.observe(before, after) is True


def test_a_still_marker_reads_as_no_motion_even_with_a_person_thrashing_in_frame():
    """The exact scenario that made absdiff unusable: the arm is still, something
    large and high-contrast is moving elsewhere in the frame. absdiff sees tens of
    thousands of changed pixels; the marker has not moved, so this says no."""
    frames = [_scene(at=(300, 200), noise_seed=1), _scene(at=(300, 200), noise_seed=2)]
    obs = markers.ArucoMotionObserver(lambda: frames.pop(0), marker_id=0)
    before, after = obs.capture(), obs.capture()

    assert obs.score(before, after) < markers.DEFAULT_MOTION_PX
    assert obs.observe(before, after) is False


def test_a_global_brightness_shift_is_not_motion():
    """Auto-exposure hunting changes every pixel in the frame at once. It does not
    move the marker, so it is not motion -- no normalisation required."""
    frames = [_scene(at=(300, 200)), _scene(at=(300, 200), brightness=40)]
    obs = markers.ArucoMotionObserver(lambda: frames.pop(0), marker_id=0)
    assert obs.observe(obs.capture(), obs.capture()) is False


def test_real_motion_is_still_detected_while_a_person_thrashes():
    """The other half: the guard must not have bought its calm by going deaf."""
    frames = [_scene(at=(300, 200), noise_seed=1), _scene(at=(330, 200), noise_seed=2)]
    obs = markers.ArucoMotionObserver(lambda: frames.pop(0), marker_id=0)
    assert obs.observe(obs.capture(), obs.capture()) is True


# ---------------------------------------------------------------------------
# 3. absence is not stillness
# ---------------------------------------------------------------------------

def test_a_missing_marker_raises_instead_of_reporting_no_motion():
    """The single most dangerous confusion available to this class. `calibrate`
    infers a mechanical stop from repeated no-motion, so a marker that fell off
    would otherwise be written into config.json as a joint limit."""
    frames = [_scene(at=(300, 200)), np.full((480, 640, 3), 200, dtype=np.uint8)]
    obs = markers.ArucoMotionObserver(lambda: frames.pop(0), marker_id=0)
    before, after = obs.capture(), obs.capture()

    with pytest.raises(ObserverError) as excinfo:
        obs.observe(before, after)
    msg = str(excinfo.value)
    assert "not evidence the joint did not move" in msg
    assert "after frame" in msg


def test_watching_the_wrong_marker_id_raises_rather_than_guessing():
    """Two markers in shot, following neither of them silently is not an option."""
    frames = [_scene(marker_id=0), _scene(marker_id=0)]
    obs = markers.ArucoMotionObserver(lambda: frames.pop(0), marker_id=1)
    with pytest.raises(ObserverError):
        obs.observe(obs.capture(), obs.capture())


def test_a_camera_returning_nothing_raises():
    obs = markers.ArucoMotionObserver(lambda: None, marker_id=0)
    with pytest.raises(ObserverError):
        obs.capture()


def test_no_frame_source_is_unavailable_rather_than_silently_blind():
    obs = markers.ArucoMotionObserver(None, marker_id=0)
    assert obs.available() is False
    with pytest.raises(ObserverError):
        obs.capture()


# ---------------------------------------------------------------------------
# 4. interface compatibility -- calibrate must not be able to tell the difference
# ---------------------------------------------------------------------------

def test_it_satisfies_the_motion_observer_interface():
    from robohand.observer import MotionObserver

    obs = markers.ArucoMotionObserver(lambda: _scene(), marker_id=0)
    assert isinstance(obs, MotionObserver)
    assert obs.motion_threshold() == markers.DEFAULT_MOTION_PX
    assert "marker=0" in obs.describe()


def test_the_threshold_is_reported_in_the_same_units_as_the_score():
    """camera_health compares score against motion_threshold; mixing pixels-moved
    with pixels-changed would silently compare two different quantities."""
    frames = [_scene(at=(300, 200)), _scene(at=(310, 200))]
    obs = markers.ArucoMotionObserver(lambda: frames.pop(0), marker_id=0, motion_px=5.0)
    score = obs.score(obs.capture(), obs.capture())
    assert score == pytest.approx(10, abs=1.0)
    assert obs.motion_threshold() == 5.0
    assert score >= obs.motion_threshold()


# ---------------------------------------------------------------------------
# 5. head to head, on identical frames
#
# The claim this whole module rests on, stated as a test rather than as prose:
# given the same two frames -- arm still, something large moving elsewhere --
# absdiff says "motion" and the marker observer says "no motion". If this ever
# fails because absdiff got quiet, the module has lost its reason to exist and
# should be reconsidered, not patched.
# ---------------------------------------------------------------------------

def test_absdiff_calls_a_thrashing_person_motion_and_markers_do_not():
    from robohand.guard import VisionGuard
    from robohand.observer import VisionMotionObserver

    still_a = _scene(at=(300, 200), noise_seed=1)
    still_b = _scene(at=(300, 200), noise_seed=2)

    absdiff = VisionMotionObserver(
        VisionGuard(frame_source=lambda: None, roi=(0, 0, 1.0, 1.0),
                    diff_threshold=25, min_changed_pixels=3000, blur_ksize=5)
    )
    changed = absdiff.guard.motion_score(still_a, still_b)

    frames = [still_a, still_b]
    aruco = markers.ArucoMotionObserver(lambda: frames.pop(0), marker_id=0)
    moved_px = aruco.score(aruco.capture(), aruco.capture())

    assert changed >= 3000, f"the clutter must be big enough to fool absdiff (got {changed})"
    assert moved_px < markers.DEFAULT_MOTION_PX, f"the marker did not move (got {moved_px})"
