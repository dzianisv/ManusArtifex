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
for command/response calls: reads are real blocking reads bounded by ``timeout_ms``,
wrapped in a retry loop that also validates the ``0x55 0x55`` signature and discards
any stray/asynchronous frames (e.g. action-group notifications) that don't match the
command we asked for.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import List, Optional, Sequence

import hid

SIGNATURE = 0x55
REPORT_LENGTH = 64

# Command IDs, per the LSC-6 protocol doc.
CMD_SERVO_MOVE = 3
CMD_ACTION_GROUP_RUN = 6
CMD_ACTION_GROUP_STOP = 7
CMD_ACTION_SPEED = 11
CMD_GET_BATTERY_VOLTAGE = 15
CMD_MULT_SERVO_UNLOAD = 20
CMD_MULT_SERVO_POS_READ = 21


class ProtocolError(RuntimeError):
    """Raised when a response frame is malformed."""


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
        ``None`` if no matching reply arrived within the retry budget.
        """
        timeout_ms = self.read_timeout_ms if timeout_ms is None else timeout_ms
        retries = self.read_retries if retries is None else retries
        for _ in range(max(1, retries)):
            try:
                report = self._dev.read(REPORT_LENGTH, timeout_ms)
            except OSError:
                # Transient HID stack hiccup -- treat as a missed reply and retry.
                continue
            if not report or len(report) < 4:
                continue
            if report[0] != SIGNATURE or report[1] != SIGNATURE:
                continue
            length = report[2]
            if report[3] != cmd:
                # Stray/async frame (e.g. an action-group event) -- discard, keep waiting.
                continue
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

    def move_servos(self, id_positions: Sequence[tuple], duration_ms: int) -> None:
        """id_positions: sequence of (servo_id, position) pairs."""
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
