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
    """Belt-and-suspenders: even independent of per-joint config, nothing in
    poses.json may fall outside the hardcoded 700..2300 global guard.
    """
    poses = load_poses()
    for pose_name, pose in poses.items():
        for joint_name, requested in pose["positions"].items():
            assert config.GLOBAL_GUARD_MIN <= requested <= config.GLOBAL_GUARD_MAX, (
                f"pose {pose_name!r} joint {joint_name!r} = {requested} outside "
                f"global guard [{config.GLOBAL_GUARD_MIN}, {config.GLOBAL_GUARD_MAX}]"
            )
