"""Low-level HID transport for the Hiwonder/LewanSoul LSC-6 servo controller.

Protocol reference: ``doc/LSC Series Servo Controller Communication Protocol V1.2.pdf``
(command IDs and byte layouts below were cross-checked against that PDF and against
real hardware captures).

This module is a from-scratch, self-contained reimplementation of the wire protocol.
It was written after studying (and fixing a bug found in) the HID transport of the
``xarm`` package from https://github.com/ccourson/xArmServoController (MIT licensed,
Copyright (c) 2021 Chris Courson) via the fork at
https://github.com/dzianisv/xArmServoController. No code was copied verbatim; the
command table (CMD_* values) and packet framing are re-derived here so this repo has
no runtime dependency on that project.

Bug fixed relative to the upstream reference: the upstream ``Controller`` calls
``hid.device.set_nonblocking(1)`` once at connect time and then calls
``read(64, timeout_ms)`` expecting it to block for up to ``timeout_ms``. Per hidapi
semantics, once a device is in non-blocking mode, ``read()`` returns immediately
regardless of the timeout argument -- so any reply that has not already arrived in
the OS buffer is missed and ``read()`` returns an empty list, which the upstream code
then indexes into unconditionally (``self._input_report[0]``), raising
``IndexError: list index out of range``. This module never enables non-blocking mode
for command/response calls: reads are real blocking reads bounded by a wall-clock
deadline, wrapped in a loop that also validates the ``0x55 0x55`` signature and
discards any stray/asynchronous frames that don't match the command we asked for.

CMD_ACTION_GROUP_COMPLETE (cmd 8) handling: the controller sends this frame
unsolicited, on its own schedule, whenever a stored action group finishes (e.g. one
started directly on the device, or by ``pushButtons.py``'s ``CMD_ACTION_GROUP_RUN``
outside this SDK). If left in the input queue, it corrupts the next expected reply --
the reader would parse cmd-8's bytes as if they were an answer to whatever was asked
next (e.g. a battery read), silently returning garbage. ``HidTransport.recv()``
recognises and discards cmd-8 frames (and any other frame that doesn't match the
cmd we're waiting for) without spending down the read-retry budget meant for genuine
non-replies, and logs each occurrence at DEBUG. ``HidTransport.__init__`` also drains
any frames already queued from before this process opened the device.
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from typing import List, Optional, Sequence

import hid

from . import config

logger = logging.getLogger(__name__)

SIGNATURE = 0x55
REPORT_LENGTH = 64

# Command IDs, per the LSC-6 protocol doc.
CMD_SERVO_MOVE = 3
CMD_ACTION_GROUP_RUN = 6
CMD_ACTION_GROUP_STOP = 7
CMD_ACTION_GROUP_COMPLETE = 8  # controller -> host ONLY, always unsolicited -- see recv()
CMD_ACTION_SPEED = 11
CMD_GET_BATTERY_VOLTAGE = 15
CMD_MULT_SERVO_UNLOAD = 20
CMD_MULT_SERVO_POS_READ = 21


class ProtocolError(RuntimeError):
    """Raised when a response frame is malformed."""


class SafetyBypassError(PermissionError):
    """Raised when SERVO_MOVE (cmd 3) is attempted without a valid motion token.

    This is the structural enforcement of the Layer-1 choke point: there is no
    "please remember to call the safety layer" convention to forget. Every transport
    that speaks this protocol (``HidTransport`` and ``sim.SimTransport``) refuses to
    emit a SERVO_MOVE packet unless it receives a ``MotionToken`` that was minted by
    ``safety.SafetyLayer`` for *this exact transport instance* and has not already
    been spent. There is no way to obtain a valid token except by going through
    ``SafetyLayer.move()``, which is where all limit/calibration/ramping checks live.
    """


class MotionToken:
    """Opaque, single-use capability authorizing exactly one SERVO_MOVE packet.

    Minted only by ``safety.SafetyLayer._mint_token()`` for a specific transport
    instance (bound via ``id(transport)``) immediately before that transport is asked
    to send a packet whose contents the safety layer has already validated. Consumed
    (invalidated) the instant it is used, so it cannot be replayed. Constructing one
    of these yourself and handing it to a transport bypasses every limit/calibration/
    ramping check the safety layer performs -- doing so anywhere in shipped code is
    exactly the failure mode ``tests/test_no_bypass.py`` exists to catch.
    """

    __slots__ = ("_transport_id", "_spent")

    def __init__(self, transport_id: int) -> None:
        self._transport_id = transport_id
        self._spent = False

    def _redeem(self, transport: object) -> None:
        if self._spent:
            raise SafetyBypassError("motion token already used (single-use; cannot be replayed)")
        if self._transport_id != id(transport):
            raise SafetyBypassError("motion token was not minted for this transport instance")
        self._spent = True


def require_motion_token(transport: object, token: Optional["MotionToken"]) -> None:
    """Shared choke-point check used by every transport implementation.

    Raises ``SafetyBypassError`` unless ``token`` is a live ``MotionToken`` minted for
    this exact ``transport`` object. Both ``HidTransport`` and ``sim.SimTransport``
    call this before emitting a SERVO_MOVE packet -- it is not implemented twice, it
    is imported from here, so there is exactly one place this rule can drift from.
    """
    if not isinstance(token, MotionToken):
        raise SafetyBypassError(
            "SERVO_MOVE rejected: no valid safety token supplied. All motion must go "
            "through safety.SafetyLayer.move(), which validates joint limits, "
            "calibration status, and step ramping before minting a single-use token "
            "for this transport instance. Calling move_servos()/CMD_SERVO_MOVE "
            "directly is not supported."
        )
    token._redeem(transport)


class IllegalServoTarget(config.InvalidPosition):
    """Last-resort, transport-layer defence in depth (Layer 1's innermost ring).

    Raised by ``move_servos`` on both ``HidTransport`` and ``sim.SimTransport`` if
    any ``(servo_id, position)`` pair is outside that servo's hardware-legal
    electrical range (``config.ELECTRICAL_LIMITS``) -- even if it arrived with a
    valid, single-use ``MotionToken``. Every higher layer (config.json validation,
    pose/sequence loading, CLI parsing, ``safety.SafetyLayer.move``) should already
    have rejected an illegal value long before it reaches here, but nothing in this
    package may assume that and skip this check: this is the check that runs
    immediately before any bytes are written to the wire.
    """


def assert_targets_within_electrical_range(id_positions: Sequence[tuple], duration_ms: Optional[int] = None) -> None:
    """Raise ``IllegalServoTarget`` unless every ``(servo_id, position)`` pair is a
    real integer inside that servo's own electrical range, and ``duration_ms`` (if
    given) is a sane positive integer. Called by both transports' ``move_servos``
    before any SERVO_MOVE bytes are ever written.
    """
    if duration_ms is not None:
        if isinstance(duration_ms, bool) or not isinstance(duration_ms, int) or duration_ms <= 0:
            raise IllegalServoTarget(
                f"refusing to send SERVO_MOVE: duration_ms must be a positive integer, "
                f"got {duration_ms!r}."
            )
    for servo_id, position in id_positions:
        if isinstance(servo_id, bool) or not isinstance(servo_id, int) or servo_id not in config.ELECTRICAL_LIMITS:
            raise IllegalServoTarget(
                f"refusing to send SERVO_MOVE: servo id {servo_id!r} is not a known "
                f"joint (known ids: {sorted(config.ELECTRICAL_LIMITS)})."
            )
        if isinstance(position, bool) or not isinstance(position, int):
            raise IllegalServoTarget(
                f"refusing to send SERVO_MOVE: servo {servo_id} position must be an "
                f"integer, got {position!r} ({type(position).__name__})."
            )
        lo, hi = config.ELECTRICAL_LIMITS[servo_id]
        if not (lo <= position <= hi):
            name = config.ID_TO_NAME.get(servo_id, str(servo_id))
            raise IllegalServoTarget(
                f"refusing to send SERVO_MOVE: servo {servo_id} ({name}) commanded "
                f"position {position} is outside its hardware-legal electrical range "
                f"[{lo}, {hi}]. This is the transport-layer backstop -- it never sends "
                "a packet outside this range, even with a valid MotionToken."
            )


class NoResponse(RuntimeError):
    """Raised when the controller never replies to a query command.

    On this specific controller/servo combination (plain PWM servos with no feedback
    wire), ``CMD_MULT_SERVO_POS_READ`` never elicits a reply -- verified by direct HID
    testing (single and multi-servo, 5s+ timeouts, before and after commanding a
    move). Callers should catch this and fall back to software-tracked state; it does
    not necessarily mean the transport is broken.
    """


@dataclass
class RawFrame:
    cmd: int
    params: List[int]


class HidTransport:
    """Blocking, retrying HID transport implementing the LSC-6 wire protocol."""

    def __init__(
        self,
        vendor_id: int,
        product_id: int,
        read_timeout_ms: int = 300,
        read_retries: int = 3,
        open_retries: int = 5,
        open_retry_delay_s: float = 0.3,
    ) -> None:
        self._dev = hid.device()
        # The OS HID stack briefly refuses re-open right after a previous handle to
        # this device was closed (observed in back-to-back CLI invocations). Retry
        # with a short backoff instead of failing outright.
        last_error: Optional[Exception] = None
        for attempt in range(max(1, open_retries)):
            try:
                self._dev.open(vendor_id, product_id)
                last_error = None
                break
            except OSError as e:
                last_error = e
                time.sleep(open_retry_delay_s)
        if last_error is not None:
            raise OSError(
                f"failed to open HID device {vendor_id:#06x}:{product_id:#06x} "
                f"after {open_retries} attempts: {last_error}"
            ) from last_error
        # Deliberately NOT calling set_nonblocking(1) here -- see module docstring.
        self.read_timeout_ms = read_timeout_ms
        self.read_retries = read_retries
        # Flush any input reports already queued before this process opened the
        # device (e.g. an ACTION_GROUP_COMPLETE left over from a stored action group
        # that finished while nobody was reading) so the first real request() call
        # doesn't have to discard it under time pressure.
        self.drain()

    def close(self) -> None:
        try:
            self._dev.close()
        except Exception:
            pass

    def __enter__(self) -> "HidTransport":
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def drain(self) -> int:
        """Discard any stale queued input reports (non-blocking) before a fresh request."""
        drained = 0
        self._dev.set_nonblocking(1)
        try:
            while self._dev.read(REPORT_LENGTH):
                drained += 1
                if drained > 32:
                    break
        finally:
            self._dev.set_nonblocking(0)
        return drained

    def send(self, cmd: int, params: Sequence[int] = ()) -> None:
        params = list(params)
        report = [0, SIGNATURE, SIGNATURE, len(params) + 2, cmd] + params
        self._dev.write(report)

    def recv(self, cmd: int, timeout_ms: Optional[int] = None, retries: Optional[int] = None) -> Optional[List[int]]:
        """Block for a reply to ``cmd``, retrying on empty/short/mismatched frames.

        Returns the parameter bytes (not including header/length/cmd) on success, or
        ``None`` if no matching reply arrived before the deadline.

        Genuine misses (the read itself timed out with nothing at all in the
        buffer) consume the ``retries`` budget. Frames that DID arrive but don't
        match ``cmd`` -- including CMD_ACTION_GROUP_COMPLETE, which the controller
        sends unsolicited whenever a stored action group finishes -- do not consume
        that budget; they are logged and discarded, and we keep waiting, bounded
        instead by an overall wall-clock deadline so a device that spews stray
        frames forever still cannot hang this call indefinitely.
        """
        timeout_ms = self.read_timeout_ms if timeout_ms is None else timeout_ms
        retries = self.read_retries if retries is None else retries
        deadline = time.monotonic() + (timeout_ms / 1000.0) * max(1, retries)
        misses = 0
        while misses < max(1, retries) and time.monotonic() < deadline:
            try:
                report = self._dev.read(REPORT_LENGTH, timeout_ms)
            except OSError:
                # Transient HID stack hiccup -- treat as a missed reply and retry.
                misses += 1
                continue
            if not report or len(report) < 4:
                misses += 1
                continue
            if report[0] != SIGNATURE or report[1] != SIGNATURE:
                misses += 1
                continue
            length = report[2]
            got_cmd = report[3]
            if got_cmd != cmd:
                if got_cmd == CMD_ACTION_GROUP_COMPLETE:
                    logger.debug(
                        "recv(cmd=%s): discarded unsolicited ACTION_GROUP_COMPLETE frame "
                        "(does not count against retry budget)", cmd,
                    )
                else:
                    logger.debug(
                        "recv(cmd=%s): discarded stray frame cmd=%s (does not count "
                        "against retry budget)", cmd, got_cmd,
                    )
                continue  # a real frame arrived, just not the one we want -- don't count it as a miss
            n_params = max(length - 2, 0)
            return list(report[4 : 4 + n_params])
        return None

    def request(
        self,
        cmd: int,
        params: Sequence[int] = (),
        timeout_ms: Optional[int] = None,
        retries: Optional[int] = None,
    ) -> List[int]:
        self.send(cmd, params)
        data = self.recv(cmd, timeout_ms=timeout_ms, retries=retries)
        if data is None:
            raise NoResponse(f"no reply to cmd={cmd:#x} within timeout")
        return data

    # -- high level operations -------------------------------------------------

    def get_battery_voltage(self) -> float:
        data = self.request(CMD_GET_BATTERY_VOLTAGE)
        if len(data) < 2:
            raise ProtocolError(f"short battery reply: {data}")
        return (data[1] * 256 + data[0]) / 1000.0

    def move_servos(
        self,
        id_positions: Sequence[tuple],
        duration_ms: int,
        *,
        token: Optional[MotionToken] = None,
    ) -> None:
        """id_positions: sequence of (servo_id, position) pairs.

        Requires a ``MotionToken`` minted by ``safety.SafetyLayer`` for this exact
        transport instance -- see ``require_motion_token`` / ``SafetyBypassError``.
        Also asserts every (servo_id, position) pair is within that servo's
        electrical range before a single byte is written to the wire -- see
        ``assert_targets_within_electrical_range`` / ``IllegalServoTarget``.
        """
        require_motion_token(self, token)
        assert_targets_within_electrical_range(id_positions, duration_ms)
        params = [len(id_positions), duration_ms & 0xFF, (duration_ms >> 8) & 0xFF]
        for servo_id, position in id_positions:
            params.extend([servo_id, position & 0xFF, (position >> 8) & 0xFF])
        self.send(CMD_SERVO_MOVE, params)

    def unload_servos(self, servo_ids: Sequence[int]) -> None:
        params = [len(servo_ids), *servo_ids]
        self.send(CMD_MULT_SERVO_UNLOAD, params)

    def get_positions(self, servo_ids: Sequence[int], timeout_ms: Optional[int] = None) -> dict:
        """Query hardware position feedback for the given servo ids.

        Raises ``NoResponse`` if the controller does not answer -- on this hardware
        (analog PWM servos) that is the expected outcome; see ``NoResponse`` docstring.
        """
        params = [len(servo_ids), *servo_ids]
        data = self.request(CMD_MULT_SERVO_POS_READ, params, timeout_ms=timeout_ms)
        if not data:
            raise ProtocolError("empty position reply")
        count = data[0]
        positions = {}
        for i in range(count):
            base = 1 + i * 3
            if base + 2 >= len(data):
                break
            servo_id = data[base]
            lo, hi = data[base + 1], data[base + 2]
            positions[servo_id] = hi * 256 + lo
        return positions
