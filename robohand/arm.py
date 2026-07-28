"""High-level control of the LeArm/xArm 1S 6-DOF arm over the LSC-6 HID protocol."""

from __future__ import annotations

import json
import os
import time
from pathlib import Path
from typing import Dict, Iterable, Mapping, Optional

from . import config
from .protocol import HidTransport, NoResponse, ProtocolError

# Every CLI invocation is a fresh process, so the in-memory shadow position would
# otherwise reset to "unknown" between commands even though the physical servos
# haven't moved. Persist last-commanded positions here so 'robohand pos' after
# 'robohand move' reports something useful instead of always "unknown".
DEFAULT_STATE_PATH = Path.home() / ".robohand" / "state.json"


def _state_path() -> Path:
    return Path(os.environ.get("ROBOHAND_STATE", DEFAULT_STATE_PATH))


class Arm:
    """Safe, closed-loop-aware controller for the 6-servo LeArm.

    Servo units are raw controller positions in [500, 2500] (center ~1500). Joints
    are addressed by name (GRIP, WRIST_ROLL, WRIST_FLEX, ELBOW, SHOULDER_LIFT,
    SHOULDER_PAN) or by raw numeric servo id (1-6).
    """

    def __init__(self, vendor_id: Optional[int] = None, product_id: Optional[int] = None) -> None:
        self._transport = HidTransport(
            vendor_id or config.VENDOR_ID,
            product_id or config.PRODUCT_ID,
        )
        # Shadow/last-commanded position, per joint name. None = unknown (never
        # commanded, or relaxed/moved by hand since). Loaded from disk so it survives
        # across separate CLI invocations.
        self._shadow: Dict[str, Optional[int]] = {name: None for name in config.JOINT_NAMES}
        self._load_shadow()
        self._feedback_supported: Optional[bool] = None
        self._probe_feedback()

    def _load_shadow(self) -> None:
        path = _state_path()
        if not path.exists():
            return
        try:
            with open(path) as f:
                saved = json.load(f)
            for name in config.JOINT_NAMES:
                if name in saved:
                    self._shadow[name] = saved[name]
        except (json.JSONDecodeError, OSError):
            pass

    def _save_shadow(self) -> None:
        path = _state_path()
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            with open(path, "w") as f:
                json.dump(self._shadow, f)
        except OSError:
            pass

    # -- lifecycle --------------------------------------------------------

    def __enter__(self) -> "Arm":
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def close(self) -> None:
        self._transport.close()

    # -- feedback capability probing --------------------------------------

    def _probe_feedback(self) -> None:
        """Try one real hardware position query to see if this controller supports it.

        On the verified hardware (analog PWM servos, no feedback wire) this always
        times out -- see protocol.NoResponse. We probe once so every later call
        doesn't pay a multi-hundred-ms timeout tax for a query we already know fails.
        """
        try:
            ids = list(config.SERVO_IDS.values())
            self._transport.get_positions(ids, timeout_ms=300)
            self._feedback_supported = True
        except (NoResponse, ProtocolError, OSError):
            self._feedback_supported = False

    @property
    def supports_hardware_feedback(self) -> bool:
        return bool(self._feedback_supported)

    # -- battery ------------------------------------------------------------

    def battery_voltage(self) -> float:
        return self._transport.get_battery_voltage()

    # -- position read --------------------------------------------------------

    def get_positions(self, names: Optional[Iterable[str]] = None) -> Dict[str, dict]:
        """Return {joint_name: {"position": int|None, "source": "hardware"|"estimated"}}.

        Tries a real hardware read first (if this controller has ever answered one).
        Falls back to the software-tracked last-commanded position, clearly labeled,
        when the hardware does not support position feedback (this unit does not).
        """
        names = list(names) if names is not None else config.JOINT_NAMES
        result: Dict[str, dict] = {}

        if self._feedback_supported:
            ids = [config.resolve_servo(n) for n in names]
            try:
                raw = self._transport.get_positions(ids)
                for n in names:
                    sid = config.resolve_servo(n)
                    if sid in raw:
                        result[n] = {"position": raw[sid], "source": "hardware"}
                        self._shadow[n] = raw[sid]
                if len(result) == len(names):
                    self._save_shadow()
                    return result
            except (NoResponse, ProtocolError):
                pass  # fall through to shadow state below

        for n in names:
            if n not in result:
                result[n] = {"position": self._shadow.get(n), "source": "estimated"}
        return result

    def get_position(self, name: str) -> dict:
        return self.get_positions([name])[name]

    # -- motion ---------------------------------------------------------------

    def set_positions(
        self,
        positions: Mapping[str, int],
        duration_ms: Optional[int] = None,
        wait: bool = True,
    ) -> Dict[str, int]:
        """Move one or more named joints, clamped to their configured safe range.

        Large jumps are automatically split into smaller sub-moves
        (config.MAX_STEP_PER_MOVE per step) so no joint is ever slammed in one shot.
        """
        duration_ms = duration_ms or config.DEFAULT_DURATION_MS
        duration_ms = max(duration_ms, config.MIN_DURATION_MS)

        targets: Dict[str, int] = {}
        starts: Dict[str, int] = {}
        for name, requested in positions.items():
            key = name.strip().upper()
            clamped = config.clamp(key, requested)
            targets[key] = clamped
            current = self._shadow.get(key)
            starts[key] = current if current is not None else clamped

        max_delta = max((abs(targets[k] - starts[k]) for k in targets), default=0)
        steps = max(1, -(-max_delta // config.MAX_STEP_PER_MOVE)) if max_delta else 1

        for step in range(1, steps + 1):
            frac = step / steps
            id_positions = []
            for name, target in targets.items():
                start = starts[name]
                pos = int(round(start + (target - start) * frac))
                pos = config.clamp(name, pos)
                id_positions.append((config.resolve_servo(name), pos))
            step_duration = max(config.MIN_DURATION_MS, duration_ms // steps)
            self._transport.move_servos(id_positions, step_duration)
            if wait:
                time.sleep(step_duration / 1000.0 + config.SETTLE_MARGIN_MS / 1000.0)

        for name, target in targets.items():
            self._shadow[name] = target
        self._save_shadow()
        return targets

    def set_position(self, name: str, position: int, duration_ms: Optional[int] = None, wait: bool = True) -> int:
        return self.set_positions({name: position}, duration_ms=duration_ms, wait=wait)[name.strip().upper()]

    def wait(self, duration_ms: int) -> None:
        time.sleep(duration_ms / 1000.0)

    # -- named poses / power ----------------------------------------------------

    def home(self, duration_ms: Optional[int] = None, wait: bool = True) -> Dict[str, int]:
        centers = {name: limit.center for name, limit in config.JOINT_LIMITS.items()}
        return self.set_positions(centers, duration_ms=duration_ms or 1500, wait=wait)

    def relax(self, names: Optional[Iterable[str]] = None) -> None:
        """Power off torque on the given joints (or all) so they can be moved by hand."""
        names = [n.strip().upper() for n in names] if names is not None else config.JOINT_NAMES
        ids = [config.resolve_servo(n) for n in names]
        self._transport.unload_servos(ids)
        for n in names:
            self._shadow[n] = None
        self._save_shadow()

    # Alias used for panic-stop paths (e.g. Ctrl-C handlers in the CLI).
    stop = relax
