"""Layer 1 (limits) + Layer 2 (time-bounded torque): the single choke point.

``SafetyLayer.move()`` is the *only* supported way to command a SERVO_MOVE packet in
this package. Every joint-limit check, calibration check, ramping decision, and
motion-token mint happens here and nowhere else. ``arm.Arm`` never calls
``transport.move_servos`` directly; ``protocol.HidTransport`` and ``sim.SimTransport``
both refuse to execute a SERVO_MOVE that did not arrive with a valid token from this
class (see ``protocol.require_motion_token`` / ``protocol.SafetyBypassError``).

Layer 2 (bounded torque) is a background watchdog thread plus per-move hold timers:
every joint that completes a move and is not explicitly told to keep holding gets
unloaded ``hold_ms`` after that move finishes; if *no* command arrives for
``idle_unload_ms``, every servo is unloaded; and a hard ``max_continuous_torque_ms``
cap forces an unload (with a logged warning) regardless of hold requests. Unload-all
is registered for normal interpreter exit, SIGINT, SIGTERM, and is also triggered by
``__exit__`` (including on an unhandled exception inside a ``with`` block).
"""

from __future__ import annotations

import atexit
import logging
import signal
import threading
import time
from typing import Callable, Dict, Iterable, Mapping, Optional

from . import config
from .protocol import MotionToken

logger = logging.getLogger("robohand.safety")


class SafetyViolation(RuntimeError):
    """Base class for Layer-1 rejections."""


class CalibrationRequired(SafetyViolation):
    """Raised when an uncalibrated joint is asked to move outside its provisional band.

    This is a *refusal*, not a clamp: silently redirecting an explicit out-of-band
    request could give the operator false confidence that their command succeeded.
    Clamping is still applied deeper in the stack (``config.clamp``) as a pure
    defense-in-depth backstop, but the primary, user-visible behaviour for this case
    is to refuse and explain why.
    """


class OutOfGlobalGuard(SafetyViolation):
    """Raised only if something computed a target outside the 700..2300 rail.

    In normal operation this should be unreachable -- ``config.effective_bounds``
    always intersects with the global guard before we ever get here. It exists as a
    final, loud backstop in case of a future bug in the bounds computation.
    """


class SafetyLayer:
    """Owns a transport and is the only thing allowed to mint motion tokens for it."""

    def __init__(
        self,
        transport,
        *,
        hold_ms: int = config.HOLD_MS,
        idle_unload_ms: int = config.IDLE_UNLOAD_MS,
        max_step_per_move: int = config.MAX_STEP_PER_MOVE,
        max_continuous_torque_ms: int = config.MAX_CONTINUOUS_TORQUE_MS,
        watchdog_poll_s: float = 0.2,
        install_signal_handlers: bool = True,
        vision_guard=None,
        dry_run: bool = False,
    ) -> None:
        self._transport = transport
        self.hold_ms = hold_ms
        self.idle_unload_ms = idle_unload_ms
        self.max_step_per_move = max_step_per_move
        self.max_continuous_torque_ms = max_continuous_torque_ms
        self.vision_guard = vision_guard
        self.dry_run = dry_run

        self._lock = threading.RLock()
        self._last_activity = time.monotonic()
        self._hold_deadline: Dict[int, float] = {}  # servo_id -> monotonic deadline
        self._torque_since: Dict[int, float] = {}  # servo_id -> monotonic start of continuous torque
        self._closed = False
        self.override_log: list = []

        self._watchdog_stop = threading.Event()
        self._watchdog_poll_s = watchdog_poll_s
        self._watchdog = threading.Thread(target=self._watchdog_loop, daemon=True, name="robohand-safety-watchdog")
        self._watchdog.start()

        atexit.register(self._on_exit)
        self._prev_handlers = {}
        if install_signal_handlers:
            for sig in (signal.SIGINT, signal.SIGTERM):
                try:
                    self._prev_handlers[sig] = signal.signal(sig, self._on_signal)
                except (ValueError, OSError):
                    # e.g. not the main thread / not supported on this platform -- the
                    # atexit handler and idle watchdog remain as backstops.
                    pass

    # -- the choke point --------------------------------------------------------

    def _mint_token(self) -> MotionToken:
        return MotionToken(id(self._transport))

    def move(
        self,
        positions: Mapping[str, int],
        *,
        current: Optional[Mapping[str, int]] = None,
        duration_ms: Optional[int] = None,
        hold: Optional[Iterable[str]] = None,
        allow_uncalibrated_override: bool = False,
        override_reason: Optional[str] = None,
        vision_guard=None,
        wait: bool = True,
    ) -> Dict[str, int]:
        """Validate, ramp, and send a move. The ONLY path to a SERVO_MOVE packet.

        ``current`` is the caller's best estimate of each targeted joint's present
        position (there is no hardware feedback to ask for the truth -- see
        ``protocol.NoResponse``); it is only used to decide the ramp step count and
        defaults to the clamped target itself (i.e. no ramp) when unknown.

        Raises ``CalibrationRequired`` if any targeted joint is uncalibrated and the
        requested position is outside its provisional band, unless
        ``allow_uncalibrated_override=True`` is passed together with a non-empty
        ``override_reason`` -- that combination is logged as a WARNING (audit trail)
        and widens the allowed band to the joint's configured min/max (still
        intersected with the global 700..2300 guard, which can never be widened).
        """
        if self._closed:
            raise SafetyViolation("SafetyLayer is closed; all servos have been unloaded")
        if allow_uncalibrated_override and not override_reason:
            raise ValueError("allow_uncalibrated_override requires a non-empty override_reason")

        duration_ms = duration_ms or config.DEFAULT_DURATION_MS
        duration_ms = max(duration_ms, config.MIN_DURATION_MS)
        hold_names = {n.strip().upper() for n in hold} if hold else set()
        guard = vision_guard if vision_guard is not None else self.vision_guard
        current = current or {}

        targets: Dict[str, int] = {}
        starts: Dict[str, int] = {}
        for name, requested in positions.items():
            key = name.strip().upper()
            lo, hi = config.effective_bounds(key, allow_uncalibrated_override=allow_uncalibrated_override)
            requested = int(requested)
            limit = config.JOINT_LIMITS[key]
            if not limit.calibrated and not allow_uncalibrated_override and not (lo <= requested <= hi):
                raise CalibrationRequired(
                    f"{key} is not calibrated (calibrated: false in config.json) and the "
                    f"requested position {requested} is outside its provisional band "
                    f"[{lo}, {hi}] (center {limit.center} +/- {config.PROVISIONAL_BAND}). "
                    "Refusing to move. Calibrate this joint (see docs/safety.md) or pass "
                    "an explicit, logged override if you really mean to widen the band."
                )
            clamped = max(lo, min(hi, requested))
            if not (config.GLOBAL_GUARD_MIN <= clamped <= config.GLOBAL_GUARD_MAX):
                # Unreachable given effective_bounds always intersects the guard --
                # kept as a loud backstop against future regressions.
                raise OutOfGlobalGuard(f"{key} target {clamped} outside global guard "
                                        f"[{config.GLOBAL_GUARD_MIN}, {config.GLOBAL_GUARD_MAX}]")
            targets[key] = clamped
            known_current = current.get(key)
            starts[key] = int(known_current) if known_current is not None else clamped

        if allow_uncalibrated_override:
            entry = {
                "ts": time.time(),
                "joints": sorted(targets),
                "reason": override_reason,
                "targets": dict(targets),
            }
            self.override_log.append(entry)
            logger.warning(
                "UNCALIBRATED OVERRIDE used for %s -> %s | reason: %s",
                sorted(targets), targets, override_reason,
            )


        max_delta = max((abs(targets[k] - starts[k]) for k in targets), default=0)
        steps = max(1, -(-max_delta // self.max_step_per_move)) if max_delta else 1

        frame_before = guard.capture() if guard is not None and guard.available() else None

        with self._lock:
            for step in range(1, steps + 1):
                frac = step / steps
                id_positions = []
                for name, target in targets.items():
                    start = starts[name]
                    pos = int(round(start + (target - start) * frac))
                    lo, hi = config.effective_bounds(name, allow_uncalibrated_override=allow_uncalibrated_override)
                    pos = max(lo, min(hi, pos))
                    pos = max(config.GLOBAL_GUARD_MIN, min(config.GLOBAL_GUARD_MAX, pos))
                    id_positions.append((config.resolve_servo(name), pos))
                step_duration = max(config.MIN_DURATION_MS, duration_ms // steps)
                self._transport.move_servos(id_positions, step_duration, token=self._mint_token())
                self._note_activity(id_positions, step_duration)
                if wait:
                    time.sleep(step_duration / 1000.0 + config.SETTLE_MARGIN_MS / 1000.0)

        if guard is not None and guard.available() and frame_before is not None:
            frame_after = guard.capture()
            moved_names = [n for n, t in targets.items() if t != starts[n]]
            servo_ids = [config.resolve_servo(n) for n in moved_names]
            guard.check(
                frame_before,
                frame_after,
                on_stall=lambda: self.unload(moved_names or list(targets)),
                context=f"move {targets}",
            )

        for name, target in targets.items():
            key = name.strip().upper()
            if key in hold_names:
                self._hold_deadline.pop(config.resolve_servo(key), None)
            else:
                self._schedule_hold_unload(config.resolve_servo(key))

        return targets

    # -- bookkeeping for Layer 2 -------------------------------------------------

    def _note_activity(self, id_positions, duration_ms: int) -> None:
        now = time.monotonic()
        self._last_activity = now
        for servo_id, _pos in id_positions:
            self._torque_since.setdefault(servo_id, now)

    def _schedule_hold_unload(self, servo_id: int) -> None:
        self._hold_deadline[servo_id] = time.monotonic() + self.hold_ms / 1000.0

    # -- unload paths (never require a token; unload is always safe) -----------

    def unload(self, names: Optional[Iterable[str]] = None) -> None:
        names = [n.strip().upper() for n in names] if names is not None else config.JOINT_NAMES
        ids = [config.resolve_servo(n) for n in names]
        self._transport.unload_servos(ids)
        for servo_id in ids:
            self._hold_deadline.pop(servo_id, None)
            self._torque_since.pop(servo_id, None)

    def unload_all(self) -> None:
        self.unload(None)

    # -- watchdog thread -----------------------------------------------------

    def _watchdog_loop(self) -> None:
        while not self._watchdog_stop.is_set():
            time.sleep(self._watchdog_poll_s)
            now = time.monotonic()
            try:
                if now - self._last_activity > self.idle_unload_ms / 1000.0:
                    if self._torque_since or self._hold_deadline:
                        logger.info("idle for >%dms -- unloading all servos", self.idle_unload_ms)
                    self.unload_all()
                    continue  # unload_all cleared both dicts; nothing else to do this tick

                expired = [sid for sid, deadline in list(self._hold_deadline.items()) if now >= deadline]
                if expired:
                    names = [config.ID_TO_NAME.get(sid, str(sid)) for sid in expired]
                    logger.info("hold_ms elapsed for %s -- unloading", names)
                    self._transport.unload_servos(expired)
                    for sid in expired:
                        self._hold_deadline.pop(sid, None)
                        self._torque_since.pop(sid, None)

                overdue = [
                    sid for sid, since in list(self._torque_since.items())
                    if now - since > self.max_continuous_torque_ms / 1000.0
                ]
                if overdue:
                    names = [config.ID_TO_NAME.get(sid, str(sid)) for sid in overdue]
                    logger.warning(
                        "max_continuous_torque_ms (%dms) exceeded for %s -- forcing unload",
                        self.max_continuous_torque_ms, names,
                    )
                    self._transport.unload_servos(overdue)
                    for sid in overdue:
                        self._hold_deadline.pop(sid, None)
                        self._torque_since.pop(sid, None)
            except Exception:  # pragma: no cover - watchdog must never die silently
                logger.exception("safety watchdog tick failed")

    # -- exit paths: atexit, signals, exceptions, context manager --------------

    def _on_exit(self) -> None:
        if not self._closed:
            try:
                self.unload_all()
            except Exception:
                logger.exception("unload_all failed during atexit")

    def _on_signal(self, signum, frame) -> None:
        logger.warning("signal %s received -- unloading all servos", signum)
        try:
            self.unload_all()
        finally:
            prev = self._prev_handlers.get(signum)
            self.close()
            if callable(prev):
                prev(signum, frame)
            else:
                raise SystemExit(128 + signum)

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._watchdog_stop.set()
        try:
            self.unload_all()
        finally:
            for sig, prev in self._prev_handlers.items():
                try:
                    signal.signal(sig, prev)
                except (ValueError, OSError):
                    pass
            try:
                atexit.unregister(self._on_exit)
            except Exception:
                pass

    def __enter__(self) -> "SafetyLayer":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        # Unload unconditionally, including when leaving the `with` block because of
        # an unhandled exception -- never leave the process without unloading.
        self.close()
