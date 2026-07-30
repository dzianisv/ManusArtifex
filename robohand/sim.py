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

``SimWorld`` (bottom of this module) adds an optional *scene* on top of the
transport -- a graspable object, a toy joints->workspace mapping, and a toy
camera projection -- so ``robohand.pick`` can be exercised end to end. Read its
docstring before believing anything it says about the real arm: it is a test
fixture, not a model of this hardware.
"""

from __future__ import annotations

import dataclasses
import math
import time
from typing import Dict, List, Optional, Sequence, Tuple

from .protocol import (
    MotionToken,
    NoResponse,
    assert_targets_within_electrical_range,
    normalize_targets,
    require_motion_token,
)

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

# How far the simulated pack sags per raw unit of *real* joint travel, and the
# floor it will not droop below. This is a deliberately crude stand-in for the
# pack's internal-resistance drop under load: its only job is to make "a joint
# that actually moved drew current" observable through ``get_battery_voltage`` the
# way it is on the real arm, so calibration can tell a disconnected servo
# (no motion, flat pack) from a live-but-unseen one (moved, pack sagged). Sized so
# one observer-reliability burst (>= ~150 units of oscillation) drops the pack past
# calibrate.py's detection epsilon (0.05 V) without a whole run drooping absurdly.
# NOTE: this models current as proportional to TRAVEL only. It deliberately does not
# model locked-rotor draw, so a joint pinned by a soft obstruction reads flat here,
# identical to a failed-open one. On the real arm a jam sags the pack hard. See
# calibrate._raise_unseen_motion for what that limitation does and does not affect.
_BATTERY_SAG_PER_UNIT = 0.0004
_BATTERY_SAG_FLOOR_V = 3.0


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
    dead: bool = False
    # Cumulative raw units this joint has *actually* moved. Stands in for charge
    # drawn from the pack: a servo only pulls current when it turns a load, so a
    # completely dead (disconnected/failed-open) joint never adds to this, and its
    # battery therefore never sags -- exactly the servo-5 signature.
    travel: float = 0.0

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
        if self.dead:
            # A completely dead servo (disconnected / failed-open -- the
            # 2026-07-28 SHOULDER_LIFT / servo-5 fault, since cleared). The controller still energises the bus
            # for the commanded window because it has no feedback and cannot know
            # the servo is gone, so ``torque_on`` flips exactly as it would for a
            # live joint (torque/packet accounting is identical). But nothing turns:
            # the position never changes, so this is NOT a locked-rotor jam (no
            # stall accrues) and NO current is drawn (``travel`` does not grow, so
            # the battery does not sag). That flat-battery-plus-no-motion signature
            # is precisely what tells a disconnected servo apart from a jam.
            self.torque_on = True
            self.blocked = False
            return self.position
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
        self.travel += abs(achievable - self.position)
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
        # Optional scene (a graspable object + toy kinematics). None by default:
        # every existing test and every safety-layer behaviour works without it.
        self.world: Optional["SimWorld"] = None

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

    def set_dead(self, servo_id: int, dead: bool = True) -> None:
        """Simulate a completely dead servo (disconnected / failed-open).

        The joint stops responding to every commanded move: it never changes
        position, never stalls, and draws no current (the pack stays flat) -- the
        real SHOULDER_LIFT / servo-5 signature. Torque still flips on for the
        commanded window, because the controller has no feedback and cannot tell.
        Test/demo use only, mirroring ``set_obstruction``.
        """
        self.joints[servo_id].dead = dead

    def stall_ms_total(self) -> float:
        return sum(j.stall_ms for j in self.joints.values())

    def any_torque_on(self) -> bool:
        return any(j.torque_on for j in self.joints.values())

    # -- protocol surface ------------------------------------------------------

    def get_battery_voltage(self) -> float:
        self.log.append({"cmd": "BATTERY"})
        drawn = sum(j.travel for j in self.joints.values())
        return max(_BATTERY_SAG_FLOOR_V, self._battery_voltage - _BATTERY_SAG_PER_UNIT * drawn)

    def move_servos(
        self,
        id_positions: Sequence[tuple],
        duration_ms: int,
        *,
        token: Optional[MotionToken] = None,
    ) -> None:
        require_motion_token(self, token)
        targets = normalize_targets(id_positions)
        assert_targets_within_electrical_range(targets, duration_ms)
        if self.world is not None:
            # Before the joints move: decide what is physically in the way. Doing
            # this first is what makes "the jaws closed onto an object" a real
            # blocked command rather than a retroactive annotation.
            self.world.before_move(targets)
        entry = {
            "cmd": "MOVE",
            "targets": list(targets),
            "duration_ms": duration_ms,
            "ts": time.monotonic(),
            "achieved": {},
            # Recorded so the "every SERVO_MOVE carried its own single-use token"
            # invariant is *inspectable* in a test, not merely true by
            # construction. ``require_motion_token`` above has already redeemed
            # it, so what is kept here is a spent token: it proves this packet was
            # separately authorised and can never be replayed. Keeping the object
            # (rather than its id) is deliberate -- ids of short-lived objects get
            # recycled, which would make a uniqueness assertion meaningless.
            "token": token,
        }
        for servo_id, target in targets:
            joint = self.joints.get(servo_id)
            if joint is None:
                raise KeyError(f"sim has no joint for servo id {servo_id}")
            achieved = joint.command(target, duration_ms)
            entry["achieved"][servo_id] = achieved
        self.log.append(entry)
        if self.world is not None:
            self.world.after_move()

    def unload_servos(self, servo_ids: Sequence[int]) -> None:
        self.log.append({"cmd": "UNLOAD", "servo_ids": list(servo_ids), "ts": time.monotonic()})
        for servo_id in servo_ids:
            joint = self.joints.get(servo_id)
            if joint is not None:
                joint.unload()
        if self.world is not None:
            self.world.after_unload(servo_ids)

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


# ===========================================================================
# SimWorld -- a scene for end-to-end pick tests. TEST FIXTURE ONLY.
# ===========================================================================
#
# READ THIS BEFORE USING ANY NUMBER BELOW.
#
# The constants and the joints->workspace mapping in this section are a TOY. They
# are NOT a kinematic model of the LeArm, they were never measured against it, and
# they are not even the right *shape* of model (a real arm is a chain of
# revolute joints, i.e. trigonometric and coupled; this is three independent
# linear axes). They exist for exactly one reason: so that ``robohand.pick``'s
# control flow -- look, move, look again, converge or give up -- can be exercised
# end to end with no camera, no network, and no hardware.
#
# ``robohand.pick`` MUST NEVER import, assume, or hardcode this mapping. It
# discovers whatever local sensitivity it needs empirically, by nudging one joint
# a small amount and measuring the resulting image-space displacement through the
# locator -- the same "command a small increment, look, decide" idiom
# ``calibrate.py`` uses. If pick.py ever needs to know these numbers, the design
# is wrong and the routine will not survive contact with the real arm.
#
# The camera is equally toy: a fixed linear projection with a small height-parallax
# term, standing in for an obliquely-mounted camera. It has no lens distortion, no
# noise, no exposure, no latency, and no perspective divide.

#: mm of workspace X per raw unit of SHOULDER_PAN.
X_MM_PER_PAN_UNIT = 0.25
#: mm of workspace Y (reach) per raw unit of ELBOW, plus the reach at ELBOW=1500.
Y_MM_PER_ELBOW_UNIT = 0.20
Y_MM_AT_ELBOW_CENTER = 120.0
#: mm of height per raw unit of SHOULDER_LIFT, plus the height at LIFT=1500.
Z_MM_PER_LIFT_UNIT = 0.25
Z_MM_AT_LIFT_CENTER = 100.0

#: Toy camera: pixels per mm in the table plane, image centre, and the
#: height-parallax terms that make a raised object shift in the image (the reason
#: a single camera cannot be trusted for the last centimetres of a grasp).
CAM_PX_PER_MM = 3.0
CAM_ORIGIN_PX = (320.0, 420.0)
CAM_PARALLAX_X_PER_MM = 0.35
CAM_PARALLAX_Y_PER_MM = 0.30
CAM_FRAME_SHAPE = (480, 640)

#: Height window (mm) in which closed jaws actually reach a table-standing object.
DEFAULT_GRASP_Z_MIN = 25.0
DEFAULT_GRASP_Z_MAX = 65.0


@dataclasses.dataclass
class SimObject:
    """A graspable thing sitting on the simulated table.

    ``x``/``y`` are table-plane millimetres in the same frame as ``SimWorld``'s
    toy forward kinematics; ``z`` is height above the table (0 = resting on it).

    ``grip_contact_position`` is the raw GRIP value at which the jaws close onto
    this object -- i.e. how wide it is, expressed in the only unit the controller
    understands. It becomes ``SimJoint.obstruction_max`` while the gripper is over
    the object, which is why closing onto it registers as a ``contact_event`` and
    never as ``stall_ms``: squeezing a held object is what a gripper is for.

    ``force_slip_on_lift`` makes a slip deterministic for tests. The
    weight-vs-capacity path (``weight_g`` > ``SimWorld.holding_capacity_g``) covers
    the general "too heavy" case; the flag covers "reproduce the failure exactly,
    every time, with no RNG".
    """

    x: float
    y: float
    z: float = 0.0
    grasp_radius_mm: float = 12.0
    grip_contact_position: int = 2050
    weight_g: float = 30.0
    force_slip_on_lift: bool = False
    # -- state, owned by SimWorld ------------------------------------------
    held: bool = False
    dropped: bool = False
    slipped: bool = False
    grasp_count: int = 0
    release_count: int = 0

    def as_dict(self) -> Dict[str, object]:
        return dataclasses.asdict(self)


class SimWorld:
    """A table, an object, a toy gripper pose, and toy grasp physics.

    Attach one with :func:`attach_world`; ``SimTransport`` calls into it around
    every move so the scene stays consistent with the joints. Everything it
    exposes (``gripper_pose``, ``object_pixel``, ``gripper_pixel``, ``held``) is
    **ground truth**, which is exactly what makes it useful for verifying a pick
    independently of what the routine itself believes happened.

    What is modelled:

    * where the gripper is, from three joints, via the toy mapping above;
    * whether the jaws are over the object and low enough to reach it;
    * closing onto the object as a rigid obstruction on servo 1 (reusing
      ``SimJoint.obstruction_max`` -- there is deliberately no second,
      parallel contact system);
    * closing onto empty air, which reaches the ladder's own backstop with no
      contact at all, so "the grasp missed" is genuinely representable;
    * a held object moving rigidly with the gripper;
    * slipping during a lift, either because it is too heavy or because a test
      demanded it.

    What is NOT modelled, and would matter on the real arm: gravity/torque
    coupling, arm compliance, backlash, jaw friction and squeeze force, the
    object toppling or rolling, collisions with anything other than the object,
    and the entire real relationship between commanded PWM value and geometry.
    """

    def __init__(
        self,
        transport: "SimTransport",
        obj: Optional[SimObject] = None,
        *,
        holding_capacity_g: float = 80.0,
        grasp_z_min: float = DEFAULT_GRASP_Z_MIN,
        grasp_z_max: float = DEFAULT_GRASP_Z_MAX,
        frame_shape: Tuple[int, int] = CAM_FRAME_SHAPE,
    ) -> None:
        self.transport = transport
        self.object = obj
        self.holding_capacity_g = float(holding_capacity_g)
        self.grasp_z_min = float(grasp_z_min)
        self.grasp_z_max = float(grasp_z_max)
        self.frame_shape = tuple(frame_shape)
        self._grasp_offset: Optional[Tuple[float, float, float]] = None
        #: Diagnostic trail for tests: every state transition the scene made.
        self.events: List[dict] = []

    # -- toy forward kinematics (TEST FIXTURE ONLY, see module note) --------

    def _joint(self, servo_id: int) -> int:
        return self.transport.joints[servo_id].position

    def gripper_pose(self, overrides: Optional[Dict[int, int]] = None) -> Tuple[float, float, float]:
        """Toy gripper (x, y, z) in mm from SHOULDER_PAN / ELBOW / SHOULDER_LIFT.

        ``overrides`` lets ``before_move`` ask "where will the gripper be when
        this packet lands", which is what makes obstruction placement causal
        rather than one packet late.
        """
        overrides = overrides or {}
        pan = overrides.get(6, self._joint(6))
        elbow = overrides.get(4, self._joint(4))
        lift = overrides.get(5, self._joint(5))
        x = (pan - 1500) * X_MM_PER_PAN_UNIT
        y = Y_MM_AT_ELBOW_CENTER + (elbow - 1500) * Y_MM_PER_ELBOW_UNIT
        z = Z_MM_AT_LIFT_CENTER + (lift - 1500) * Z_MM_PER_LIFT_UNIT
        return x, y, z

    # -- toy camera --------------------------------------------------------

    def project(self, x: float, y: float, z: float) -> Tuple[float, float]:
        """Toy oblique-camera projection of a workspace point to pixels."""
        cx, cy = CAM_ORIGIN_PX
        px = cx + x * CAM_PX_PER_MM + z * CAM_PARALLAX_X_PER_MM
        py = cy - (y - 60.0) * CAM_PX_PER_MM - z * CAM_PARALLAX_Y_PER_MM
        return px, py

    def gripper_pixel(self) -> Tuple[float, float]:
        return self.project(*self.gripper_pose())

    def object_pixel(self) -> Optional[Tuple[float, float]]:
        if self.object is None:
            return None
        return self.project(self.object.x, self.object.y, self.object.z)

    # -- queries used by tests and by locate.SimLocator --------------------

    @property
    def held(self) -> bool:
        return bool(self.object is not None and self.object.held)

    def object_position(self) -> Optional[Tuple[float, float, float]]:
        if self.object is None:
            return None
        return self.object.x, self.object.y, self.object.z

    def distance_to_object(self, overrides: Optional[Dict[int, int]] = None) -> Optional[float]:
        """Planar mm between the gripper and the object; None if there is no object."""
        if self.object is None:
            return None
        gx, gy, _gz = self.gripper_pose(overrides)
        return math.hypot(gx - self.object.x, gy - self.object.y)

    def can_grasp(self, overrides: Optional[Dict[int, int]] = None) -> bool:
        """True when the jaws are over the object AND low enough to reach it."""
        if self.object is None:
            return False
        gx, gy, gz = self.gripper_pose(overrides)
        planar = math.hypot(gx - self.object.x, gy - self.object.y)
        return planar <= self.object.grasp_radius_mm and self.grasp_z_min <= gz <= self.grasp_z_max

    # -- transport hooks ---------------------------------------------------

    def before_move(self, targets: Sequence[Tuple[int, int]]) -> None:
        """Place/remove the GRIP obstruction for the pose this packet will produce."""
        if self.object is None:
            return
        overrides = {servo_id: pos for servo_id, pos in targets}
        grip = self.transport.joints.get(1)
        if grip is None:
            return
        if self.object.held or self.can_grasp(overrides):
            grip.obstruction_max = int(self.object.grip_contact_position)
        else:
            grip.obstruction_max = None

    def after_move(self) -> None:
        """Update grasp/hold/slip state from the pose the joints actually reached."""
        if self.object is None:
            return
        grip = self.transport.joints.get(1)
        gx, gy, gz = self.gripper_pose()
        squeezing = bool(
            grip is not None
            and grip.torque_on
            and grip.position >= self.object.grip_contact_position
        )

        if not self.object.held:
            if squeezing and self.can_grasp():
                self.object.held = True
                self.object.dropped = False
                self.object.slipped = False
                self.object.grasp_count += 1
                self._grasp_offset = (
                    self.object.x - gx,
                    self.object.y - gy,
                    self.object.z - gz,
                )
                self.events.append({"event": "grasped", "gripper": (gx, gy, gz)})
            return

        # Held: the object rides with the gripper, keeping the offset it had at
        # the moment of the grasp.
        if not squeezing:
            self._release("jaws_opened", gx, gy, gz)
            return

        ox, oy, oz = self._grasp_offset or (0.0, 0.0, 0.0)
        self.object.x, self.object.y, self.object.z = gx + ox, gy + oy, gz + oz

        lifting = gz > self.grasp_z_max
        if lifting and (self.object.force_slip_on_lift or self.object.weight_g > self.holding_capacity_g):
            self.object.slipped = True
            self._release("slipped_during_lift", gx, gy, gz)

    def after_unload(self, servo_ids: Sequence[int]) -> None:
        if self.object is None or not self.object.held:
            return
        if 1 in servo_ids:
            gx, gy, gz = self.gripper_pose()
            self._release("grip_unloaded", gx, gy, gz)

    def _release(self, reason: str, gx: float, gy: float, gz: float) -> None:
        self.object.held = False
        self.object.release_count += 1
        # Whatever it was doing, it is now on the table wherever it was let go.
        # "dropped" means it came out involuntarily; a deliberate open-and-place
        # is a release, not a drop.
        self.object.z = 0.0
        self.object.dropped = bool(self.object.slipped)
        self._grasp_offset = None
        self.events.append({"event": "released", "reason": reason, "gripper": (gx, gy, gz)})

    def describe(self) -> str:
        return (
            f"SimWorld(object={None if self.object is None else (round(self.object.x, 1), round(self.object.y, 1))}, "
            f"held={self.held}, TOY KINEMATICS -- test fixture only)"
        )


def attach_world(transport: "SimTransport", obj: Optional[SimObject] = None, **kwargs) -> SimWorld:
    """Attach a :class:`SimWorld` scene to a ``SimTransport`` and return it."""
    world = SimWorld(transport, obj, **kwargs)
    transport.world = world
    return world
