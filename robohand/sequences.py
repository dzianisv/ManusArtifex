"""Sequence playback/recording: ordered lists of poses (or raw positions) with durations.

A sequence file is JSON: a list of steps, each either
    {"pose": "ready", "duration_ms": 1200}
or
    {"positions": {"GRIP": 700}, "duration_ms": 600}

``record`` is interactive: it appends steps by referencing already-saved named poses
(see ``robohand pose --save``) or by typing raw joint:position pairs. It does not
attempt to auto-capture a physically-posed arm, because this controller has no
position feedback (see arm.Arm.supports_hardware_feedback / protocol.NoResponse) --
there is nothing to read back. Use ``robohand teach`` to create named poses first
(manual entry there for the same reason), then compose them here.

Every step is validated up front by ``validate_sequence`` -- at load time
(``load_sequence``), again defensively at the start of ``play_sequence`` (defence in
depth), and before writing (``save_sequence``/``record_interactive``). A sequence
with even one illegal step is rejected wholesale before any motion starts: it can
never be half-executed.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import List

from . import config
from . import poses as pose_store
from .arm import Arm


def validate_sequence(steps) -> List[dict]:
    """Validate every step in a sequence up front. Raises ``config.InvalidPosition``
    naming the first offending step -- a sequence with one illegal step is rejected
    wholesale, before any motion starts, never executed partially. Returns a
    normalized copy of ``steps`` (validated/uppercased positions, validated
    duration_ms).
    """
    if not isinstance(steps, list) or not steps:
        raise config.InvalidPosition("sequence must be a non-empty JSON list of steps.")
    known_poses = pose_store.list_poses()
    validated: List[dict] = []
    for i, step in enumerate(steps):
        if not isinstance(step, dict):
            raise config.InvalidPosition(f"sequence step {i}: must be an object, got {step!r}.")
        try:
            duration_ms = config.validate_duration_ms(step.get("duration_ms"), allow_none=True)
        except config.InvalidPosition as e:
            raise config.InvalidPosition(f"sequence step {i}: {e}") from e
        if "pose" in step:
            name = step["pose"]
            if name not in known_poses:
                raise config.InvalidPosition(
                    f"sequence step {i}: unknown pose {name!r}. "
                    f"Known poses: {sorted(known_poses)}."
                )
            validated.append({**step, "duration_ms": duration_ms})
        elif "positions" in step:
            try:
                positions = config.validate_positions(step["positions"])
            except config.InvalidPosition as e:
                raise config.InvalidPosition(f"sequence step {i}: {e}") from e
            validated.append({**step, "positions": positions, "duration_ms": duration_ms})
        else:
            raise config.InvalidPosition(
                f"sequence step {i} has neither 'pose' nor 'positions': {step}"
            )
    return validated


def load_sequence(path: Path) -> List[dict]:
    with open(path) as f:
        steps = json.load(f)
    return validate_sequence(steps)


def save_sequence(path: Path, steps: List[dict]) -> None:
    validate_sequence(steps)
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w") as f:
        json.dump(steps, f, indent=2)
        f.write("\n")


def play_sequence(arm: Arm, steps: List[dict]) -> None:
    # Re-validated here (defence in depth) so that even a caller who bypassed
    # load_sequence/save_sequence (e.g. by constructing steps in-process) still
    # gets the "reject wholesale before any motion starts" guarantee: every step
    # is checked before the first arm.set_positions() call below.
    steps = validate_sequence(steps)
    for i, step in enumerate(steps):
        duration_ms = step.get("duration_ms")
        if "pose" in step:
            pose = pose_store.get_pose(step["pose"])
            positions = pose["positions"]
            duration_ms = duration_ms or pose.get("duration_ms")
        else:
            positions = step["positions"]
        print(f"[{i + 1}/{len(steps)}] -> {positions} ({duration_ms} ms)")
        arm.set_positions(positions, duration_ms=duration_ms, wait=True)


def record_interactive(path: Path) -> List[dict]:
    steps: List[dict] = []
    known = sorted(pose_store.list_poses())
    print(f"Known poses: {known or '(none saved yet)'}")
    print("Add steps. For each step enter a pose name, or 'joint=pos,joint=pos' pairs.")
    print("Type 'done' to finish, 'undo' to remove the last step.")
    while True:
        raw = input(f"step {len(steps) + 1}> ").strip()
        if raw.lower() == "done":
            break
        if raw.lower() == "undo":
            if steps:
                steps.pop()
                print("removed last step")
            continue
        duration_raw = input("  duration_ms [1000]> ").strip()
        try:
            duration_ms = int(duration_raw) if duration_raw else 1000
        except ValueError:
            print(f"  error: duration_ms must be an integer, got {duration_raw!r}. Step discarded.")
            continue
        if "=" in raw:
            positions = {}
            try:
                for pair in raw.split(","):
                    name, value = pair.split("=")
                    positions[name.strip().upper()] = int(value.strip())
                step = {"positions": config.validate_positions(positions), "duration_ms": duration_ms}
            except (ValueError, config.InvalidPosition) as e:
                print(f"  error: {e}. Step discarded, not saved.")
                continue
        else:
            try:
                step = validate_sequence([{"pose": raw, "duration_ms": duration_ms}])[0]
            except config.InvalidPosition as e:
                print(f"  error: {e}. Step discarded, not saved.")
                continue
        steps.append(step)
    save_sequence(path, steps)
    print(f"Saved {len(steps)} step(s) to {path}")
    return steps
