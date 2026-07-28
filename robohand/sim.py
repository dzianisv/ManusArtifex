"""Fake transport for validating robohand's safety layer without real hardware.

``SimTransport`` implements the same surface as ``protocol.HidTransport``
(``move_servos``, ``unload_servos``, ``get_positions``, ``get_battery_voltage``,
``close``, context-manager protocol) so ``arm.Arm`` and ``safety.SafetyLayer`` cannot
tell the difference at the call-site. Unlike the real controller it:

* Models each joint's *mechanical* end stop (narrower than the electrical
  500..2500 range) and the servo's locked-rotor behaviour when a commanded
  target is beyond it -- accumulating ``stall_ms`` for exactly as long as the
  controller would have held torque trying (and failing) to reach that target.
* Optionally models a rigid *obstruction* (e.g. an object in the gripper) that
  blocks travel short of the mechanical stop. Contact with an obstruction is
  not counted as a stall: gripping an object and holding torque against it is
  the intended behaviour of a gripper, not a destructive locked-rotor event.
* Never answers ``get_positions`` -- this mirrors the verified real hardware
  (``CMD_MULT_SERVO_POS_READ`` times out on plain PWM servos with no feedback
  wire) so tests cannot accidentally rely on position feedback that does not
  exist on the real device.
* Keeps a full command log for test assertions.

Selected via ``ROBOHAND_TRANSPORT=sim`` env var or ``--sim`` CLI flag (see
``arm.Arm.__init__`` / ``cli.py``). The real ``HidTransport`` remains the default so
nobody accidentally ships simulator behaviour.
"""

from __future__ import annotations

import dataclasses
import time
from typing import Dict, List, Optional, Sequence, Tuple

from .protocol import MotionToken, NoResponse, require_motion_token

# Plausible mechanical end stops per servo id, narrower than the 500..2500
# electrical range. These are *not* the configured safe operating band (which is
# narrower still) -- they represent where the physical jaws/links actually jam.
#
# GRIP (servo id 1) is the one exception where "mechanical stop" and "electrical
# range" are almost the same thing: it is a LeArm gripper servo whose *own*
# reference firmware clamps it to 1500..2500 (not the generic 500..2500), and
# 1500 is confirmed to be its open mechanical stop (lower raw = more open). We
# model 2500 as its closed-on-itself stop (no confirmed measurement beyond "it is
# the electrical ceiling"; see docs/research-hardware.md open questions).
DEFAULT_MECHANICAL_LIMITS: Dict[int, Tuple[int, int]] = {
    1: (1500, 2500),  # GRIP -- LeArm gripper servo, confirmed 1500..2500 (not 500..2500)
    2: (900, 2100),  # WRIST_ROLL
    3: (900, 2100),  # WRIST_FLEX
    4: (850, 2150),  # ELBOW
    5: (850, 2150),  # SHOULDER_LIFT
    6: (800, 2200),  # SHOULDER_PAN
}

DEFAULT_BATTERY_VOLTAGE = 7.4


@dataclasses.dataclass
class SimJoint:
    servo_id: int
    position: int
    mech_min: int
    mech_max: int
    torque_on: bool = False
    blocked: bool = False
    obstruction_min: Optional[int] = None
    obstruction_max: Optional[int] = None
    stall_ms: float = 0.0
    contact_events: int = 0

    def _reachable_bounds(self) -> Tuple[int, int]:
        lo, hi = self.mech_min, self.mech_max
        if self.obstruction_min is not None:
            lo = max(lo, self.obstruction_min)
        if self.obstruction_max is not None:
            hi = min(hi, self.obstruction_max)
        return lo, hi

    def command(self, target: int, duration_ms: int) -> int:
        """Apply one commanded move; return the position actually achieved.

        Mirrors the real controller: it is told "go to X over N ms" and has no idea
        whether X is reachable. If the mechanical end stop blocks it, the servo
        pushes at locked-rotor torque for the *entire* commanded duration -- that is
        exactly ``duration_ms`` of destructive stall time. If only an obstruction
        (e.g. a grasped object) blocks it, that is normal gripper operation, not a
        stall, and is tracked separately.
        """
        mech_lo, mech_hi = self.mech_min, self.mech_max
        reach_lo, reach_hi = self._reachable_bounds()
        achievable = max(reach_lo, min(reach_hi, target))

        hit_mechanical_stop = target < mech_lo or target > mech_hi
        hit_obstruction_only = (not hit_mechanical_stop) and achievable != target

        self.torque_on = True
        if hit_mechanical_stop:
            self.stall_ms += duration_ms
            self.blocked = True
        elif hit_obstruction_only:
            self.contact_events += 1
            self.blocked = True
        else:
            self.blocked = False
        self.position = achievable
        return achievable

    def unload(self) -> None:
        self.torque_on = False
        self.blocked = False


class SimTransport:
    """Drop-in fake for ``protocol.HidTransport``. See module docstring."""

    def __init__(
        self,
        mechanical_limits: Optional[Dict[int, Tuple[int, int]]] = None,
        initial_positions: Optional[Dict[int, int]] = None,
        battery_voltage: float = DEFAULT_BATTERY_VOLTAGE,
    ) -> None:
        limits = dict(DEFAULT_MECHANICAL_LIMITS)
        if mechanical_limits:
            limits.update(mechanical_limits)
        self.joints: Dict[int, SimJoint] = {}
        for servo_id, (lo, hi) in limits.items():
            start = (initial_positions or {}).get(servo_id, (lo + hi) // 2)
            self.joints[servo_id] = SimJoint(servo_id=servo_id, position=start, mech_min=lo, mech_max=hi)
        self._battery_voltage = battery_voltage
        self.log: List[dict] = []
        self.closed = False

    # -- lifecycle, matches HidTransport -----------------------------------

    def close(self) -> None:
        self.closed = True

    def __enter__(self) -> "SimTransport":
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    # -- test helpers --------------------------------------------------------

    def set_obstruction(self, servo_id: int, *, min: Optional[int] = None, max: Optional[int] = None) -> None:
        """Simulate a rigid object blocking travel for one servo (test/demo use)."""
        joint = self.joints[servo_id]
        joint.obstruction_min = min
        joint.obstruction_max = max

    def stall_ms_total(self) -> float:
        return sum(j.stall_ms for j in self.joints.values())

    def any_torque_on(self) -> bool:
        return any(j.torque_on for j in self.joints.values())

    # -- protocol surface ------------------------------------------------------

    def get_battery_voltage(self) -> float:
        self.log.append({"cmd": "BATTERY"})
        return self._battery_voltage

    def move_servos(
        self,
        id_positions: Sequence[tuple],
        duration_ms: int,
        *,
        token: Optional[MotionToken] = None,
    ) -> None:
        require_motion_token(self, token)
        entry = {
            "cmd": "MOVE",
            "targets": list(id_positions),
            "duration_ms": duration_ms,
            "ts": time.monotonic(),
            "achieved": {},
        }
        for servo_id, target in id_positions:
            joint = self.joints.get(servo_id)
            if joint is None:
                raise KeyError(f"sim has no joint for servo id {servo_id}")
            achieved = joint.command(target, duration_ms)
            entry["achieved"][servo_id] = achieved
        self.log.append(entry)

    def unload_servos(self, servo_ids: Sequence[int]) -> None:
        self.log.append({"cmd": "UNLOAD", "servo_ids": list(servo_ids), "ts": time.monotonic()})
        for servo_id in servo_ids:
            joint = self.joints.get(servo_id)
            if joint is not None:
                joint.unload()

    def get_positions(self, servo_ids: Sequence[int], timeout_ms: Optional[int] = None) -> dict:
        """Always times out -- mirrors the verified real hardware (no feedback wire)."""
        self.log.append({"cmd": "POS_READ", "servo_ids": list(servo_ids)})
        raise NoResponse("sim: CMD_MULT_SERVO_POS_READ never replies (matches real hardware)")


class DryRunTransport(SimTransport):
    """Like ``SimTransport`` but also prints every packet it would have sent.

    Used by ``--dry-run``: validated through the exact same safety pipeline (so
    limit/calibration/ramping bugs are still caught), but nothing is persisted and
    the output makes clear this is a rehearsal, not a real move.
    """

    def move_servos(self, id_positions, duration_ms, *, token: Optional[MotionToken] = None) -> None:
        super().move_servos(id_positions, duration_ms, token=token)
        print(f"[dry-run] SERVO_MOVE {list(id_positions)} over {duration_ms}ms")

    def unload_servos(self, servo_ids) -> None:
        super().unload_servos(servo_ids)
        print(f"[dry-run] MULT_SERVO_UNLOAD {list(servo_ids)}")
