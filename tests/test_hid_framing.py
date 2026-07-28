"""Proves HidTransport.recv() correctly discards unsolicited CMD_ACTION_GROUP_COMPLETE
(cmd 8) frames instead of either (a) misparsing them as the reply it asked for, or
(b) burning its retry budget on them and timing out a request that would otherwise
have succeeded.

This does NOT open the real HID device. ``hid.device`` is replaced with an in-memory
fake that hands back a scripted sequence of raw report bytes -- there is no VID
0x0483 / PID 0x5750 anywhere in this file, and no real ``hid.device().open()`` call
ever happens. This is exactly the kind of protocol-framing bug that cannot be
exercised through the simulator (``sim.SimTransport`` does not model raw HID byte
framing at all, only the higher-level move/unload/read surface), so it has to be
tested at this layer with a fake transport instead.
"""

from __future__ import annotations

from typing import List, Optional

from robohand import protocol

SIGNATURE = protocol.SIGNATURE


def _frame(cmd: int, params: List[int]) -> List[int]:
    """Build a raw input-report byte list as hidapi's read() would return it."""
    return [SIGNATURE, SIGNATURE, len(params) + 2, cmd, *params]


class FakeHidDevice:
    """Stands in for ``hid.device()``. Never touches real hardware.

    Models two independent queues, matching real HID timing:
    - ``blocking_reads``: frames that "arrive" while HidTransport.recv() is doing a
      real blocking read (i.e. after a ``send()``). This is what real request/reply
      traffic (and any cmd-8 that happens to interleave with it) looks like.
    - ``preexisting_reads``: frames already sitting in the OS input queue before
      this process even opened the device -- only visible to the *non-blocking*
      poll that ``HidTransport.drain()`` performs at connect time. Real hidapi
      would return these instantly in non-blocking mode; anything not already
      queued returns ``[]`` immediately rather than waiting.
    """

    def __init__(
        self,
        blocking_reads: List[Optional[List[int]]],
        preexisting_reads: Optional[List[Optional[List[int]]]] = None,
    ):
        self._blocking_reads = list(blocking_reads)
        self._preexisting_reads = list(preexisting_reads or [])
        self._nonblocking = False
        self.opened_with = None
        self.writes: List[List[int]] = []

    # -- hid.device surface used by HidTransport --------------------------------

    def open(self, vendor_id: int, product_id: int) -> None:
        self.opened_with = (vendor_id, product_id)

    def close(self) -> None:
        pass

    def set_nonblocking(self, flag: int) -> None:
        self._nonblocking = bool(flag)

    def write(self, report: List[int]) -> None:
        self.writes.append(report)

    def read(self, length: int, timeout_ms: int = 0) -> List[int]:
        if self._nonblocking:
            if not self._preexisting_reads:
                return []
            return self._preexisting_reads.pop(0) or []
        if not self._blocking_reads:
            return []
        return self._blocking_reads.pop(0) or []


def _make_transport(monkeypatch, scripted_reads: List[Optional[List[int]]]) -> "protocol.HidTransport":
    fake = FakeHidDevice(blocking_reads=scripted_reads)
    monkeypatch.setattr(protocol.hid, "device", lambda: fake)
    # read_retries=3 so a test can prove stray frames don't eat into this budget.
    return protocol.HidTransport(0x0483, 0x5750, read_timeout_ms=50, read_retries=3, open_retries=1)


def test_unsolicited_action_group_complete_is_drained_not_misparsed(monkeypatch):
    """A cmd-8 frame arrives before the real battery reply; recv() must skip it and
    return the real reply, not misinterpret the cmd-8 bytes as battery data.
    """
    battery_frame = _frame(protocol.CMD_GET_BATTERY_VOLTAGE, [0x24, 0x1C])  # 7204 mV
    scripted = [
        _frame(protocol.CMD_ACTION_GROUP_COMPLETE, [5, 0, 0]),  # unsolicited, arrives first
        battery_frame,
    ]
    transport = _make_transport(monkeypatch, scripted)
    voltage = transport.get_battery_voltage()
    assert voltage == 7.204


def test_stray_frames_do_not_consume_the_retry_budget(monkeypatch):
    """Feed MORE stray cmd-8 frames than read_retries allows, followed by the real
    reply. If stray frames incorrectly consumed the retry budget, this would time
    out (return None / raise NoResponse) even though a valid reply was always
    coming right behind them.
    """
    scripted = [
        _frame(protocol.CMD_ACTION_GROUP_COMPLETE, [1, 0, 0]),
        _frame(protocol.CMD_ACTION_GROUP_COMPLETE, [2, 0, 0]),
        _frame(protocol.CMD_ACTION_GROUP_COMPLETE, [3, 0, 0]),
        _frame(protocol.CMD_ACTION_GROUP_COMPLETE, [4, 0, 0]),
        _frame(protocol.CMD_GET_BATTERY_VOLTAGE, [0x10, 0x1C]),  # 7184 mV
    ]
    transport = _make_transport(monkeypatch, scripted)
    voltage = transport.get_battery_voltage()
    assert voltage == 7.184


def test_genuine_non_reply_still_times_out(monkeypatch):
    """Sanity check the fix didn't accidentally make recv() wait forever: if the
    device truly never replies (only empty reads, no frames at all), NoResponse is
    still raised -- exactly the verified real-hardware behaviour for cmd 21.
    """
    scripted = [[], [], [], [], []]
    transport = _make_transport(monkeypatch, scripted)
    try:
        transport.get_positions([1])
        assert False, "expected NoResponse"
    except protocol.NoResponse:
        pass


def test_init_drains_stale_queue_before_first_request(monkeypatch):
    """A stale ACTION_GROUP_COMPLETE left over from before this process opened the
    device must be flushed by drain() at connect time, not allowed to sit in the
    queue and confuse the very first request this transport makes.
    """
    stale = _frame(protocol.CMD_ACTION_GROUP_COMPLETE, [9, 0, 0])
    battery_frame = _frame(protocol.CMD_GET_BATTERY_VOLTAGE, [0x00, 0x1D])  # 7424 mV
    fake = FakeHidDevice(blocking_reads=[battery_frame], preexisting_reads=[stale])
    monkeypatch.setattr(protocol.hid, "device", lambda: fake)
    transport = protocol.HidTransport(0x0483, 0x5750, read_timeout_ms=50, read_retries=3, open_retries=1)
    # drain() (non-blocking, __init__ time) should have consumed `stale` via the
    # preexisting-reads queue and left the blocking-reads queue (the real request
    # traffic) completely untouched.
    assert fake._preexisting_reads == []
    assert fake._blocking_reads == [battery_frame]
    assert transport.get_battery_voltage() == 7.424
