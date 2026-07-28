"""Layer 1 proof: every pose ever shipped in data/poses.json stays inside every
targeted joint's *effective* safe band (global guard ∩ configured min/max ∩
provisional band for uncalibrated joints). This is a static data check -- it exists
so a future careless hand-edit of poses.json (e.g. re-adding the incident's raw
700/2200 gripper values) is caught by CI, not by hardware.
"""

from __future__ import annotations

from robohand import config
from robohand.poses import load_poses


def test_all_poses_within_safe_band():
    poses = load_poses()
    assert poses, "expected at least one pose in data/poses.json"

    violations = []
    for pose_name, pose in poses.items():
        for joint_name, requested in pose["positions"].items():
            key = joint_name.strip().upper()
            lo, hi = config.effective_bounds(key)
            if not (lo <= requested <= hi):
                violations.append((pose_name, key, requested, lo, hi))

    assert not violations, (
        "poses.json contains positions outside the effective safe band: "
        f"{violations}"
    )


def test_all_poses_within_global_guard():
    """Belt-and-suspenders: even independent of per-joint configured min/max, nothing
    in poses.json may fall outside each joint's hardcoded *absolute* rail --
    ``config.absolute_bounds``, the 700..2300 global guard tightened per-servo where
    a joint's own hardware demands it (e.g. GRIP's 1500 floor -- this is exactly the
    check that would have caught the 2026-07-28 incident's raw close_grip=700 value
    even if every other layer had been misconfigured).
    """
    poses = load_poses()
    for pose_name, pose in poses.items():
        for joint_name, requested in pose["positions"].items():
            key = joint_name.strip().upper()
            abs_lo, abs_hi = config.absolute_bounds(key)
            assert abs_lo <= requested <= abs_hi, (
                f"pose {pose_name!r} joint {key!r} = {requested} outside "
                f"absolute guard [{abs_lo}, {abs_hi}]"
            )
