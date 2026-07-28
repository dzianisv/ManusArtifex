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
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import List

from . import poses as pose_store
from .arm import Arm


def load_sequence(path: Path) -> List[dict]:
    with open(path) as f:
        return json.load(f)


def save_sequence(path: Path, steps: List[dict]) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w") as f:
        json.dump(steps, f, indent=2)
        f.write("\n")


def play_sequence(arm: Arm, steps: List[dict]) -> None:
    for i, step in enumerate(steps):
        duration_ms = step.get("duration_ms")
        if "pose" in step:
            pose = pose_store.get_pose(step["pose"])
            positions = pose["positions"]
            duration_ms = duration_ms or pose.get("duration_ms")
        elif "positions" in step:
            positions = step["positions"]
        else:
            raise ValueError(f"step {i} has neither 'pose' nor 'positions': {step}")
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
        duration_ms = int(duration_raw) if duration_raw else 1000
        if "=" in raw:
            positions = {}
            for pair in raw.split(","):
                name, value = pair.split("=")
                positions[name.strip().upper()] = int(value.strip())
            steps.append({"positions": positions, "duration_ms": duration_ms})
        else:
            steps.append({"pose": raw, "duration_ms": duration_ms})
    save_sequence(path, steps)
    print(f"Saved {len(steps)} step(s) to {path}")
    return steps
