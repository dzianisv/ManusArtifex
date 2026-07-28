"""High-level control of the LeArm/xArm 1S 6-DOF arm over the LSC-6 HID protocol.

All motion goes through ``safety.SafetyLayer`` -- see that module's docstring for
why this is a hard requirement, not a convention. ``Arm`` owns the shadow/estimated
position tracking (there is no hardware feedback -- see ``protocol.NoResponse``) and
the optional camera-based ``guard.VisionGuard``; it never calls
``transport.move_servos`` itself.
"""

from __future__ import annotations

import json
import logging
import os
import time
from pathlib import Path
from typing import Dict, Iterable, Mapping, Optional

from . import config
from .protocol import HidTransport, NoResponse, ProtocolError
from .safety import SafetyLayer

logger = logging.getLogger("robohand.arm")

# Every CLI invocation is a fresh process, so the in-memory shadow position would
# otherwise reset to "unknown" between commands even though the physical servos
# haven't moved. Persist last-commanded positions here so 'robohand pos' after
# 'robohand move' reports something useful instead of always "unknown".
DEFAULT_STATE_PATH = Path.home() / ".robohand" / "state.json"


def _state_path() -> Path:
    return Path(os.environ.get("ROBOHAND_STATE", DEFAULT_STATE_PATH))


def _use_sim(sim: bool) -> bool:
    return sim or os.environ.get("ROBOHAND_TRANSPORT", "").strip().lower() == "sim"


def _open_vision_guard() -> Optional["VisionGuard"]:
    """Best-effort: open a camera and wrap it in a VisionGuard. None if unavailable.

    Degrades safely per spec: any failure to open a camera (not attached, OS
    permission not granted, wrong backend, etc.) results in ``None`` -- callers must
    treat that as "layer 3 is not active" and keep going with layers 1+2 alone.
    """
    try:
        from . import camera as camera_module
        from .guard import VisionGuard
    except Exception:  # pragma: no cover - optional dependency (cv2/numpy) missing
        return None
    try:
        cam = camera_module.Camera(camera_module.CameraConfig(index=0))
        cam.open()
    except Exception as exc:  # CameraError or anything else -- no camera, no guard
        logger.info("vision guard unavailable: %s", exc)
        return None
    return VisionGuard(
        frame_source=cam.grab,
        roi=config.VISION_ROI,
        diff_threshold=config.VISION_DIFF_THRESHOLD,
        min_changed_pixels=config.VISION_MIN_CHANGED_PIXELS,
        blur_ksize=config.VISION_BLUR_KSIZE,
    )


class Arm:
    """Safe, closed-loop-aware controller for the 6-servo LeArm.

    Servo units are raw controller positions in [500, 2500] (center ~1500). Joints
    are addressed by name (GRIP, WRIST_ROLL, WRIST_FLEX, ELBOW, SHOULDER_LIFT,
    SHOULDER_PAN) or by raw numeric servo id (1-6).

    Every motion call is routed through a private ``safety.SafetyLayer`` -- see that
    module for the limit/calibration/ramping/token design. Pass ``sim=True`` (or set
    ``ROBOHAND_TRANSPORT=sim``) to run entirely against ``sim.SimTransport`` with no
    real hardware access; ``dry_run=True`` additionally prints every packet instead
    of silently succeeding, and never persists shadow-position state to disk.
    """

    def __init__(
        self,
        vendor_id: Optional[int] = None,
        product_id: Optional[int] = None,
        *,
        sim: bool = False,
        dry_run: bool = False,
        use_vision: Optional[bool] = None,
        install_signal_handlers: bool = True,
    ) -> None:
        self.dry_run = dry_run
        self.sim = _use_sim(sim) or dry_run
        if self.sim:
            from .sim import DryRunTransport, SimTransport

            self._transport = DryRunTransport() if dry_run else SimTransport()
        else:
            self._transport = HidTransport(
                vendor_id or config.VENDOR_ID,
                product_id or config.PRODUCT_ID,
            )

        # A simulated transport has no real arm for a camera to watch, so vision
        # guard defaults to off in sim/dry-run mode (a real camera pointed at an
        # empty desk would just report every simulated move as a stall). Callers can
        # still force it on (e.g. hardware-in-the-loop testing) by passing
        # use_vision=True explicitly, or assign `arm.vision_guard` themselves (e.g.
        # tests inject a synthetic frame source tied to the sim's own state).
        if use_vision is None:
            use_vision = not self.sim
        self.vision_guard = _open_vision_guard() if use_vision else None
        if use_vision and self.vision_guard is None:
            print(
                "WARNING: no camera available -- running WITHOUT visual stall detection "
                "(layer 3). Layers 1 (limits) and 2 (bounded torque) are still active."
            )

        self._safety = SafetyLayer(
            self._transport,
            install_signal_handlers=install_signal_handlers,
            dry_run=dry_run,
        )

        # Shadow/last-commanded position, per joint name. None = unknown (never
        # commanded, or relaxed/moved by hand since). Loaded from disk so it survives
        # across separate CLI invocations. Not persisted in dry-run mode.
        self._shadow: Dict[str, Optional[int]] = {name: None for name in config.JOINT_NAMES}
        if not self.dry_run:
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
        if self.dry_run:
            return
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
        # SafetyLayer.close() unloads all servos unconditionally (including on an
        # unhandled exception propagating out of a `with Arm() as arm:` block) before
        # the transport handle is released.
        self._safety.close()
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

    # Sentinel meaning "use self.vision_guard" -- lets set_positions()/set_position()
    # distinguish "no vision_guard override supplied" from an explicit
    # vision_guard=None (used internally by the grip ladder, see _grip_ladder).
    _AUTO_VISION_GUARD = object()

    def set_positions(
        self,
        positions: Mapping[str, int],
        duration_ms: Optional[int] = None,
        wait: bool = True,
        *,
        hold: Optional[Iterable[str]] = None,
        allow_uncalibrated_override: bool = False,
        override_reason: Optional[str] = None,
        vision_guard: object = _AUTO_VISION_GUARD,
    ) -> Dict[str, int]:
        """Move one or more named joints. The ONLY thing this method does with the
        transport is call ``safety.SafetyLayer.move()`` -- every limit check,
        calibration check, ramp-step split, and motion-token mint lives there (see
        that module's docstring). Raises ``safety.CalibrationRequired`` instead of
        moving if a joint is uncalibrated and the request is outside its provisional
        band (pass ``allow_uncalibrated_override=True`` + ``override_reason`` to
        widen this, which is logged as a WARNING).

        By default this uses whatever ``self.vision_guard`` currently is (an
        automatic, hard-fail check: any move with no observed motion raises
        ``StallDetected`` and unloads). Pass ``vision_guard=None`` explicitly to
        disable that automatic check for this one call -- used internally by the
        grip ladder, which implements its own *graceful* no-motion handling (contact
        with an object is an expected stopping condition there, not an anomaly) and
        would otherwise have every one of its small steps racing against, and
        pre-empted by, this method's own stricter automatic check.
        """
        current = {name.strip().upper(): pos for name, pos in self._shadow.items() if pos is not None}
        guard = self.vision_guard if vision_guard is self._AUTO_VISION_GUARD else vision_guard
        targets = self._safety.move(
            positions,
            current=current,
            duration_ms=duration_ms,
            hold=hold,
            allow_uncalibrated_override=allow_uncalibrated_override,
            override_reason=override_reason,
            vision_guard=guard,
            wait=wait,
        )
        for name, target in targets.items():
            self._shadow[name] = target
        self._save_shadow()
        return targets

    def set_position(
        self,
        name: str,
        position: int,
        duration_ms: Optional[int] = None,
        wait: bool = True,
        **kwargs,
    ) -> int:
        return self.set_positions({name: position}, duration_ms=duration_ms, wait=wait, **kwargs)[name.strip().upper()]

    def wait(self, duration_ms: int) -> None:
        time.sleep(duration_ms / 1000.0)

    # -- named poses / power ----------------------------------------------------

    def home(self, duration_ms: Optional[int] = None, wait: bool = True) -> Dict[str, int]:
        centers = {name: limit.center for name, limit in config.JOINT_LIMITS.items()}
        return self.set_positions(centers, duration_ms=duration_ms or 1500, wait=wait)

    def relax(self, names: Optional[Iterable[str]] = None) -> None:
        """Power off torque on the given joints (or all) so they can be moved by hand."""
        names = [n.strip().upper() for n in names] if names is not None else config.JOINT_NAMES
        self._safety.unload(names)
        for n in names:
            self._shadow[n] = None
        self._save_shadow()

    # Alias used for panic-stop paths (e.g. Ctrl-C handlers in the CLI).
    stop = relax

    # -- gripper-specific: incremental ladder, never an absolute target --------

    def _grip_ladder(self, direction: str, step: Optional[int] = None, duration_ms: Optional[int] = None) -> Dict[str, object]:
        """Step the gripper in small increments, stopping the instant motion ceases.

        ``direction`` is "close" or "open". At every step we ask the vision guard
        whether the commanded increment actually produced visible motion; the first
        step that shows no motion is assumed to be contact with an object OR the
        approach to the mechanical end stop, so we stop, back off one step, and
        unload -- we never rely on reaching an absolute target. ``grip_min_safe`` /
        ``grip_max_safe`` are hard backstops that bound the ladder regardless of what
        the vision guard reports (and regardless of whether a camera is attached at
        all): if there is no camera, the ladder still stops there rather than driving
        into the mechanical end stop.
        """
        if direction not in ("close", "open"):
            raise ValueError("direction must be 'close' or 'open'")
        step = abs(step if step is not None else config.GRIP_STEP)
        duration_ms = duration_ms or config.GRIP_LADDER_DURATION_MS
        sign = -1 if direction == "close" else 1
        hard_stop = config.GRIP_MIN_SAFE if direction == "close" else config.GRIP_MAX_SAFE

        current = self._shadow.get("GRIP")
        if current is None:
            current = config.JOINT_LIMITS["GRIP"].center

        guard = self.vision_guard
        last_frame = guard.capture() if guard is not None and guard.available() else None

        positions_visited = [current]
        stopped_reason = "reached_backstop"
        max_iterations = 200  # generous bound; real bands are far smaller than this
        for _ in range(max_iterations):
            next_pos = current + sign * step
            if direction == "close" and next_pos <= hard_stop:
                next_pos = hard_stop
            elif direction == "open" and next_pos >= hard_stop:
                next_pos = hard_stop

            # vision_guard=None: disable SafetyLayer's own automatic hard-fail vision
            # check for this internal step. The ladder implements its own graceful
            # no-motion handling right below (contact with an object, or reaching
            # the mechanical stop, is an expected way for a ladder step to end, not
            # an anomaly worth raising StallDetected for).
            self.set_position("GRIP", next_pos, duration_ms=duration_ms, hold=["GRIP"], vision_guard=None)
            positions_visited.append(next_pos)

            if guard is not None and guard.available() and last_frame is not None:
                frame_after = guard.capture()
                if not guard.detect_motion(last_frame, frame_after):
                    stopped_reason = "no_motion_detected"
                    # Back off one step from the position that showed no motion, then
                    # unload -- this is the "contact" position minus one increment,
                    # not the position we just tried to reach.
                    back_off = positions_visited[-2] if len(positions_visited) >= 2 else current
                    self.set_position("GRIP", back_off, duration_ms=duration_ms, hold=["GRIP"], vision_guard=None)
                    self._safety.unload(["GRIP"])
                    return {
                        "stopped_reason": stopped_reason,
                        "contact_position": next_pos,
                        "final_position": back_off,
                        "steps": len(positions_visited) - 1,
                    }
                last_frame = frame_after

            current = next_pos
            if next_pos == hard_stop:
                break

        self._safety.unload(["GRIP"])
        return {
            "stopped_reason": stopped_reason,
            "contact_position": None,
            "final_position": current,
            "steps": len(positions_visited) - 1,
        }

    def close_grip(self, step: Optional[int] = None, duration_ms: Optional[int] = None) -> Dict[str, object]:
        """Incrementally close the gripper; stop+back off+unload on contact/end-stop.

        Never commands an absolute "fully closed" target -- see the incident this
        package was redesigned to prevent (docs/safety.md).
        """
        return self._grip_ladder("close", step=step, duration_ms=duration_ms)

    def open_grip(self, step: Optional[int] = None, duration_ms: Optional[int] = None) -> Dict[str, object]:
        """Incrementally open the gripper; stop+back off+unload on contact/end-stop."""
        return self._grip_ladder("open", step=step, duration_ms=duration_ms)

    # -- doctor / introspection --------------------------------------------------

    def calibration_report(self) -> Dict[str, dict]:
        report = {}
        for name, limit in config.JOINT_LIMITS.items():
            lo, hi = config.effective_bounds(name)
            report[name] = {
                "calibrated": limit.calibrated,
                "configured_min": limit.min,
                "configured_max": limit.max,
                "effective_min": lo,
                "effective_max": hi,
                "center": limit.center,
            }
        return report

    def safety_status(self) -> Dict[str, object]:
        return {
            "transport": "sim" if self.sim else "hardware",
            "dry_run": self.dry_run,
            "layer1_limits": True,
            "layer2_bounded_torque": True,
            "layer3_vision_guard": bool(self.vision_guard is not None and self.vision_guard.available()),
            "hold_ms": self._safety.hold_ms,
            "idle_unload_ms": self._safety.idle_unload_ms,
            "max_continuous_torque_ms": self._safety.max_continuous_torque_ms,
        }
