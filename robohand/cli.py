"""Command-line interface for robohand.

Only one process may hold the arm's HID handle at a time -- every command opens the
device, does its work, and closes it again before exiting. Ctrl-C, unhandled
exceptions, and normal exit are all handled by ``safety.SafetyLayer`` (installed
inside ``Arm``), which unloads every servo before the process actually goes away --
see docs/safety.md.

Global ``--sim`` (or ``ROBOHAND_TRANSPORT=sim``) runs entirely against
``sim.SimTransport`` with no real hardware access. ``--dry-run`` additionally prints
every packet instead of executing it and never persists shadow-position state.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path
from typing import Optional

from . import config, locate, poses as pose_store, sequences
from .arm import Arm
from .protocol import NoResponse
from .safety import CalibrationRequired, SafetyViolation
from .guard import StallDetected


def _is_simulated(args: argparse.Namespace) -> bool:
    """True when this invocation will run against a simulated transport.

    Deliberately mirrors ``arm._use_sim`` (flag OR ``ROBOHAND_TRANSPORT=sim``) so
    the operator-presence gate below can never disagree with which transport
    ``_make_arm`` is actually about to open.
    """
    if getattr(args, "sim", False) or getattr(args, "dry_run", False):
        return True
    return os.environ.get("ROBOHAND_TRANSPORT", "").strip().lower() == "sim"


def _make_arm(args: argparse.Namespace) -> Arm:
    return Arm(sim=getattr(args, "sim", False), dry_run=getattr(args, "dry_run", False))


def _print_positions(positions: dict, as_json: bool) -> None:
    if as_json:
        print(json.dumps(positions, indent=2))
        return
    width = max(len(n) for n in positions)
    for name, info in positions.items():
        pos = info["position"]
        pos_str = str(pos) if pos is not None else "unknown"
        print(f"  {name:<{width}}  {pos_str:>6}  ({info['source']})")


def cmd_info(args: argparse.Namespace) -> int:
    with _make_arm(args) as arm:
        voltage = arm.battery_voltage()
        positions = arm.get_positions()
        payload = {
            "device": {"vendor_id": hex(config.VENDOR_ID), "product_id": hex(config.PRODUCT_ID)},
            "battery_voltage": voltage,
            "hardware_position_feedback": arm.supports_hardware_feedback,
            "positions": {n: p["position"] for n, p in positions.items()},
        }
        if args.json:
            print(json.dumps(payload, indent=2))
        else:
            print(f"Device: VID={hex(config.VENDOR_ID)} PID={hex(config.PRODUCT_ID)}")
            print(f"Battery: {voltage:.3f} V")
            print(f"Hardware position feedback: {'yes' if arm.supports_hardware_feedback else 'no (using last-commanded estimate)'}")
            print("Positions:")
            _print_positions(positions, as_json=False)
    return 0


def cmd_pos(args: argparse.Namespace) -> int:
    with _make_arm(args) as arm:
        positions = arm.get_positions()
        _print_positions(positions, as_json=args.json)
    return 0


def _parse_joints(raw: str) -> dict:
    try:
        data = json.loads(raw)
    except json.JSONDecodeError as e:
        raise config.InvalidPosition(
            f"--joints must be valid JSON, e.g. '{{\"ELBOW\":2100,\"GRIP\":1950}}'. "
            f"Parse error: {e}"
        ) from e
    if not isinstance(data, dict):
        raise config.InvalidPosition(
            f"--joints must be a JSON object of {{JOINT: position}}, got {data!r}."
        )
    return config.validate_positions(data)


def cmd_move(args: argparse.Namespace) -> int:
    if args.joints:
        targets = _parse_joints(args.joints)
    elif args.servo and args.pos is not None:
        targets = config.validate_positions({args.servo.upper(): args.pos})
    else:
        print("error: provide either --joints '{...}' or --servo NAME --pos N", file=sys.stderr)
        return 2
    with _make_arm(args) as arm:
        try:
            result = arm.set_positions(
                targets,
                duration_ms=args.duration,
                wait=True,
                allow_uncalibrated_override=bool(args.override_reason),
                override_reason=args.override_reason,
            )
        except KeyboardInterrupt:
            arm.relax()
            print("\nInterrupted -- servos relaxed.", file=sys.stderr)
            return 130
        print(json.dumps(result, indent=2))
    return 0


def cmd_pose(args: argparse.Namespace) -> int:
    if args.list:
        for name, spec in sorted(pose_store.list_poses().items()):
            print(f"{name:<16} {spec.get('positions')} ({spec.get('duration_ms')} ms) -- {spec.get('comment', '')}")
        return 0
    if args.save:
        with _make_arm(args) as arm:
            current = arm.get_positions()
            positions = {n: p["position"] for n, p in current.items() if p["position"] is not None}
            if not positions:
                print("error: no known positions to save (hardware feedback unavailable and no prior moves).", file=sys.stderr)
                print("Move the arm with 'robohand move' or 'robohand home' first so a state is known.", file=sys.stderr)
                return 1
        pose_store.save_pose(args.save, positions, duration_ms=args.duration)
        print(f"Saved pose {args.save!r}: {positions}")
        return 0
    if args.name:
        pose = pose_store.get_pose(args.name)
        with _make_arm(args) as arm:
            result = arm.set_positions(pose["positions"], duration_ms=args.duration or pose.get("duration_ms"))
        print(json.dumps(result, indent=2))
        return 0
    print("error: specify a pose name, --list, or --save NAME", file=sys.stderr)
    return 2


def cmd_grip(args: argparse.Namespace) -> int:
    """Incrementally open/close the gripper via the vision-checked grip ladder.

    This intentionally never accepts an absolute target position -- see
    docs/safety.md for the incident this design prevents. Use --step to change the
    increment size (default from config.json gripper.step).
    """
    with _make_arm(args) as arm:
        if args.action == "open":
            result = arm.open_grip(step=args.step, duration_ms=args.duration)
        else:
            result = arm.close_grip(step=args.step, duration_ms=args.duration)
    print(json.dumps(result, indent=2))
    if result["stopped_reason"] == "no_motion_detected":
        print(f"Stopped on contact at {result['contact_position']}; backed off to {result['final_position']} and unloaded.")
    return 0


def cmd_home(args: argparse.Namespace) -> int:
    with _make_arm(args) as arm:
        result = arm.home()
    print(json.dumps(result, indent=2))
    return 0


def cmd_relax(args: argparse.Namespace) -> int:
    with _make_arm(args) as arm:
        names = [s.upper() for s in args.servo] if args.servo else None
        arm.relax(names)
    print(f"Relaxed: {args.servo or 'ALL'}")
    return 0


def cmd_teach(args: argparse.Namespace) -> int:
    names = [s.upper() for s in args.servo] if args.servo else config.JOINT_NAMES
    with _make_arm(args) as arm:
        arm.relax(names)
        print(f"Servos relaxed: {names}. Move the arm by hand into the desired pose.")
        if not arm.supports_hardware_feedback:
            print(
                "NOTE: this controller has no position feedback (verified: "
                "CMD_MULT_SERVO_POS_READ never replies on this hardware -- plain PWM "
                "servos have no sense wire). The pose cannot be auto-captured; enter "
                "each joint's position by hand below (or press Enter to skip a joint)."
            )
            positions = {}
            for name in names:
                raw = input(f"  {name} [skip]> ").strip()
                if raw:
                    try:
                        value = int(raw)
                    except ValueError:
                        print(
                            f"error: {name}: position must be an integer, got {raw!r}. "
                            "Nothing saved.",
                            file=sys.stderr,
                        )
                        return 1
                    try:
                        positions[name] = config.validate_position_value(name, value)
                    except config.InvalidPosition as e:
                        print(f"error: {e} Nothing saved.", file=sys.stderr)
                        return 1
            if not positions:
                print("No positions entered; nothing saved.", file=sys.stderr)
                return 1
        else:
            input("Press Enter once positioned...")
            current = arm.get_positions(names)
            positions = {n: p["position"] for n, p in current.items() if p["position"] is not None}
        pose_store.save_pose(args.name, positions, duration_ms=args.duration)
    print(f"Saved pose {args.name!r}: {positions}")
    return 0


def cmd_play(args: argparse.Namespace) -> int:
    steps = sequences.load_sequence(Path(args.file))
    with _make_arm(args) as arm:
        try:
            sequences.play_sequence(arm, steps)
        except KeyboardInterrupt:
            arm.relax()
            print("\nInterrupted -- servos relaxed.", file=sys.stderr)
            return 130
    return 0


def cmd_record(args: argparse.Namespace) -> int:
    sequences.record_interactive(Path(args.file))
    return 0


def cmd_pick(args: argparse.Namespace) -> int:
    """Vision-guided pick: locate, servo, descend, grasp on the ladder, verify, lift.

    Replaces an earlier placeholder that played back three ``pick_*`` poses which
    were never present in poses.json -- it raised KeyError the moment anyone ran
    it, and it was an open-loop lunge at a precomputed target besides. See
    docs/pick.md.
    """
    from . import pick as pick_module
    from .locate import LocatorError

    simulated = _is_simulated(args)

    # Checked BEFORE _make_arm(), so the real HID device is never even opened
    # without an operator. Same gate, same reasoning, as cmd_calibrate.
    if not simulated and not args.i_am_present_and_watching:
        print(
            "REFUSING TO RUN.\n"
            "\n"
            "A pick drives the arm to a position derived from a camera, then closes\n"
            "the jaws on whatever is there. Every layer below it is bounded, but the\n"
            "target itself comes from a detector that can be wrong -- and the descend\n"
            "and lift heights are open loop, because one camera on a table plane\n"
            "measures X and Y and never Z.\n"
            "\n"
            "Run it only with a human physically present, watching the arm, with the\n"
            "power switch within reach and the workspace clear.\n"
            "\n"
            "Re-run with --i-am-present-and-watching to confirm, or add --sim to\n"
            "rehearse the whole routine against the simulator with no hardware access.",
            file=sys.stderr,
        )
        return 2

    if args.locator == "sim" and not simulated:
        print(
            "error: --locator sim reads simulator ground truth and means nothing on "
            "real hardware. Use --locator classical or --locator gemini, or add --sim.",
            file=sys.stderr,
        )
        return 2

    settings_kwargs = {}
    for attr, key in (
        ("step", "grip_step"),
        ("descend_units", "descend_units"),
        ("lift_units", "lift_units"),
        ("max_steps", "max_approach_steps"),
        ("tolerance_px", "approach_tolerance_px"),
    ):
        value = getattr(args, attr, None)
        if value is not None:
            settings_kwargs[key] = value
    if args.release:
        settings_kwargs["hold_on_success"] = False
    settings = pick_module.PickSettings(**settings_kwargs)

    with _make_arm(args) as arm:
        if args.locator == "sim":
            # The simulated locator needs something to look at. Placed here, in
            # the CLI, because it is scene setup for a rehearsal -- pick.py knows
            # nothing about the simulator beyond what the Locator interface says.
            from .sim import SimObject, attach_world

            attach_world(arm._transport, SimObject(x=args.object_x, y=args.object_y))

        locator_kwargs = {}
        if args.target:
            # The two real backends name their target differently because they
            # look for different things: a colour band vs a description.
            key = "object_prompt" if args.locator == "gemini" else "object_color"
            locator_kwargs[key] = args.target
        try:
            locator = locate.locator_for_arm(arm, args.locator, **locator_kwargs)
        except LocatorError as e:
            print(f"error: locator unavailable -- {e}", file=sys.stderr)
            return 2

        try:
            result = pick_module.pick(
                arm,
                locator,
                settings,
                allow_uncalibrated_override=args.allow_uncalibrated,
                override_reason=args.override_reason,
            )
        except KeyboardInterrupt:
            arm.relax()
            print("\nInterrupted -- servos relaxed.", file=sys.stderr)
            return 130

    if args.json:
        print(json.dumps(result.as_dict(), indent=2))
    else:
        print(f"Locator: {result.locator}")
        for entry in result.log:
            data = " ".join(f"{k}={v}" for k, v in entry.data.items())
            print(f"  [{entry.state:<11}] {entry.detail}{'  ' + data if data else ''}")
        print(
            f"\n{'PICKED' if result.success else result.state.value}: {result.reason} "
            f"({result.elapsed_s:.1f}s)"
        )
        if result.holding:
            print(
                "Servos left under torque so the object is not dropped. The safety "
                "watchdog will force-unload them shortly -- this arm cannot hold "
                "anything indefinitely (docs/pick.md)."
            )
        elif result.unloaded:
            print("Arm unloaded.")

    return 0 if result.success else 1


def cmd_calibrate(args: argparse.Namespace) -> int:
    """Vision-guided auto-calibration, or promotion of an earlier result.

    Three modes, deliberately not overlapping: ``--apply FILE`` (no motion at all,
    just merges a previous result into config.json), ``--joint NAME`` (calibrate
    one), ``--all`` (calibrate every joint).
    """
    from . import calibrate as calibration

    if args.apply:
        applied = calibration.apply_calibration(
            Path(args.apply),
            joints=args.joint or None,
            allow_simulated=args.allow_simulated,
        )
        print(f"Promoted calibration from {args.apply} into config.json:")
        for name, spec in sorted(applied.items()):
            print(f"  {name:<14} [{spec['min']}, {spec['max']}] center={spec['center']} calibrated=true")
        print(
            "These joints are no longer restricted to the provisional band. Re-run "
            "'robohand doctor' to confirm, and re-verify by hand before trusting the "
            "new range with a payload."
        )
        return 0

    if not args.joint and not args.all:
        print("error: provide --joint NAME (repeatable), --all, or --apply FILE", file=sys.stderr)
        return 2
    if args.joint and args.all:
        print("error: --joint and --all are mutually exclusive", file=sys.stderr)
        return 2

    # Operator-presence gate. Checked BEFORE _make_arm(), so a missing flag can
    # never open the real HID device even for a moment.
    if not _is_simulated(args) and not args.i_am_present_and_watching:
        print(
            "REFUSING TO RUN.\n"
            "\n"
            "Auto-calibration drives each joint toward its unknown mechanical stops. It\n"
            "is bounded (small steps, sub-500ms torque bursts, unload after every step,\n"
            "hard torque/time budgets), but it is still deliberate motion into the\n"
            "unknown on an arm with no position feedback.\n"
            "\n"
            "Run it only with a human physically present, watching the arm, with the\n"
            "power switch within reach, the workspace clear, and (for GRIP) the jaws\n"
            "empty and unobstructed.\n"
            "\n"
            "Re-run with --i-am-present-and-watching to confirm, or add --sim to\n"
            "rehearse the whole routine against the simulator with no hardware access.",
            file=sys.stderr,
        )
        return 2

    settings_kwargs = {}
    for attr, key in (
        ("step", "step"),
        ("margin", "margin"),
        ("confirmations", "confirmations"),
        ("step_duration", "step_duration_ms"),
    ):
        value = getattr(args, attr, None)
        if value is not None:
            settings_kwargs[key] = value
    settings = calibration.CalibrationSettings(**settings_kwargs)

    joints = args.joint if args.joint else None
    with _make_arm(args) as arm:
        try:
            run = calibration.calibrate(
                arm,
                joints,
                settings=settings,
                output_dir=Path(args.out_dir) if args.out_dir else None,
            )
        except KeyboardInterrupt:
            arm.relax()
            print("\nInterrupted -- servos relaxed, no calibration written.", file=sys.stderr)
            return 130

    if args.json:
        print(json.dumps(run.as_dict(), indent=2))
    else:
        print(f"Calibration complete ({run.elapsed_s:.1f}s, {run.torque_ms}ms commanded torque)")
        print(f"Observer: {run.observer}")
        for line in calibration.summarize(run):
            print(line)
        print(f"Written to: {run.output_path}")
        print(
            "config.json was NOT modified. Review the file above, then promote it with:\n"
            f"  robohand calibrate --apply {run.output_path}"
        )
    return 0


def cmd_doctor(args: argparse.Namespace) -> int:
    """Report device presence, per-joint calibration status, camera availability,
    and which safety layers are currently active. Never commands any motion."""
    report: dict = {}

    if getattr(args, "sim", False) or getattr(args, "dry_run", False):
        report["device"] = {"checked": False, "reason": "running with --sim/--dry-run; hardware check skipped"}
    else:
        try:
            from .protocol import HidTransport

            transport = HidTransport(config.VENDOR_ID, config.PRODUCT_ID, open_retries=1, open_retry_delay_s=0.1)
            try:
                voltage = transport.get_battery_voltage()
                report["device"] = {"present": True, "battery_voltage": voltage}
            finally:
                transport.close()
        except Exception as e:  # OSError (no device) or NoResponse (battery query failed)
            report["device"] = {"present": False, "error": str(e)}

    report["joints"] = {}
    for name, limit in config.JOINT_LIMITS.items():
        lo, hi = config.effective_bounds(name)
        report["joints"][name] = {
            "calibrated": limit.calibrated,
            "configured_range": [limit.min, limit.max],
            "effective_range": [lo, hi],
        }

    camera_available = False
    camera_error = None
    try:
        from . import camera as camera_module

        cam = camera_module.Camera(camera_module.CameraConfig(index=0))
        cam.open()
        camera_available = True
        cam.release()
    except Exception as e:
        camera_error = str(e)
    report["camera"] = {"available": camera_available}
    if camera_error:
        report["camera"]["error"] = camera_error

    report["safety_layers"] = {
        "layer1_limits_and_calibration": True,
        "layer2_bounded_torque_watchdog": True,
        "layer3_vision_guard": camera_available,
    }

    if args.json:
        print(json.dumps(report, indent=2))
    else:
        dev = report["device"]
        print("Device:", "present" if dev.get("present") else dev.get("reason", "NOT present"),
              f"({dev.get('battery_voltage')} V)" if dev.get("battery_voltage") is not None else "")
        print("Joints:")
        for name, info in report["joints"].items():
            cal = "calibrated" if info["calibrated"] else "UNCALIBRATED (provisional band)"
            print(f"  {name:<14} {cal:<32} effective={info['effective_range']}")
        print(f"Camera available: {camera_available}" + (f" ({camera_error})" if camera_error else ""))
        print("Safety layers:")
        for k, v in report["safety_layers"].items():
            print(f"  {k}: {'ACTIVE' if v else 'inactive (no camera -- layer 3 skipped)'}")
        if not camera_available:
            print("WARNING: no camera detected -- layer 3 (vision stall detection) will not run.")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="robohand", description="Control the LeArm/xArm 1S 6-DOF arm.")
    parser.add_argument(
        "--sim", action="store_true",
        help="Use the simulated transport (sim.SimTransport) instead of real hardware. "
        "Also enabled by ROBOHAND_TRANSPORT=sim.",
    )
    parser.add_argument(
        "--dry-run", action="store_true", dest="dry_run",
        help="Validate and print every packet through the simulator; write nothing to disk.",
    )
    sub = parser.add_subparsers(dest="command", required=True)

    p = sub.add_parser("info", help="Show device, battery, and all servo positions")
    p.add_argument("--json", action="store_true")
    p.set_defaults(func=cmd_info)

    p = sub.add_parser("pos", help="Print current positions")
    p.add_argument("--json", action="store_true")
    p.set_defaults(func=cmd_pos)

    p = sub.add_parser("move", help="Move one servo or several named joints")
    p.add_argument("--servo", help="Joint name, e.g. GRIP")
    p.add_argument("--pos", type=int, help="Target position, raw servo units (each joint has its own valid range -- see 'robohand doctor'; GRIP is 1500-2500, others are 500-2500)")
    p.add_argument("--joints", help='JSON object, e.g. \'{"ELBOW":2100,"GRIP":1950}\'')
    p.add_argument("--duration", type=int, default=None, help="Move duration in ms (default 1000)")
    p.add_argument(
        "--override-reason", metavar="REASON", default=None,
        help="Explicitly widen an uncalibrated joint's band to its configured min/max "
        "for this move (still capped at the 700-2300 global guard, and at each servo's "
        "own hardcoded absolute floor/ceiling -- e.g. GRIP can never go below 1500 "
        "regardless of override). Logged as a WARNING.",
    )
    p.set_defaults(func=cmd_move)

    p = sub.add_parser("pose", help="Move to / list / save named poses")
    p.add_argument("name", nargs="?", help="Pose name to move to")
    p.add_argument("--list", action="store_true", help="List known poses")
    p.add_argument("--save", metavar="NAME", help="Save current positions as a named pose")
    p.add_argument("--duration", type=int, default=None)
    p.set_defaults(func=cmd_pose)

    p = sub.add_parser("grip", help="Open or close the grip via the incremental, vision-checked ladder")
    p.add_argument("action", choices=["open", "close"])
    p.add_argument("--step", type=int, default=None, help="Ladder step size in raw units (default from config.json)")
    p.add_argument("--duration", type=int, default=None, help="Per-step duration in ms")
    p.set_defaults(func=cmd_grip)

    p = sub.add_parser("home", help="Move all joints to their configured center")
    p.set_defaults(func=cmd_home)

    p = sub.add_parser("relax", help="Power off torque (all servos, or --servo NAME ...)")
    p.add_argument("--servo", action="append", help="Joint name; repeat for multiple")
    p.set_defaults(func=cmd_relax)

    p = sub.add_parser("teach", help="Relax servos, let a human pose the arm, save the result")
    p.add_argument("name", help="Name to save the taught pose as")
    p.add_argument("--servo", action="append", help="Limit to these joints; default all")
    p.add_argument("--duration", type=int, default=config.DEFAULT_DURATION_MS)
    p.set_defaults(func=cmd_teach)

    p = sub.add_parser("play", help="Play a saved sequence of poses/positions")
    p.add_argument("file")
    p.set_defaults(func=cmd_play)

    p = sub.add_parser("record", help="Interactively build and save a sequence")
    p.add_argument("file")
    p.set_defaults(func=cmd_record)

    p = sub.add_parser(
        "pick",
        help="Vision-guided pick: locate, visually servo, descend, grasp, verify, lift",
        description=(
            "Runs the LOCATE/APPROACH/DESCEND/GRASP/VERIFY/LIFT/VERIFY_HELD state "
            "machine. Refuses to run on uncalibrated joints without an explicit "
            "reason, and refuses on real hardware without an operator present. "
            "See docs/pick.md."
        ),
    )
    p.add_argument(
        "--locator",
        choices=("classical", "gemini", "sim"),
        default="sim",
        help="Perception backend: classical HSV blob detection, Gemini Robotics-ER "
             "1.6, or simulator ground truth (--sim only). Default: sim.",
    )
    p.add_argument("--target", default=None, help="What to look for (a colour name, or a description for --locator gemini)")
    p.add_argument("--step", type=int, default=None, help="Grip ladder step size in raw units")
    p.add_argument("--descend-units", type=int, default=None, help="OPEN LOOP raw-unit offset to grasp height")
    p.add_argument("--lift-units", type=int, default=None, help="OPEN LOOP raw-unit offset to lift by")
    p.add_argument("--max-steps", type=int, default=None, help="Visual servo step budget for APPROACH")
    p.add_argument("--tolerance-px", type=float, default=None, help="APPROACH convergence tolerance, pixels")
    p.add_argument("--object-x", type=float, default=10.0, help="[--locator sim] object X in the simulated workspace, mm")
    p.add_argument("--object-y", type=float, default=140.0, help="[--locator sim] object Y in the simulated workspace, mm")
    p.add_argument("--release", action="store_true", help="Unload on success instead of holding the object")
    p.add_argument("--allow-uncalibrated", action="store_true", help="Run on uncalibrated joints (requires --override-reason)")
    p.add_argument("--override-reason", default=None, help="Logged justification for --allow-uncalibrated")
    p.add_argument(
        "--i-am-present-and-watching",
        action="store_true",
        help="Required on real hardware: confirms an operator is watching with the power switch in reach",
    )
    p.add_argument("--json", action="store_true")
    p.set_defaults(func=cmd_pick)

    p = sub.add_parser("doctor", help="Report device/camera/calibration status and active safety layers")
    p.add_argument("--json", action="store_true")
    p.set_defaults(func=cmd_doctor)

    p = sub.add_parser(
        "calibrate",
        help="Vision-guided auto-calibration of a joint's real mechanical limits, "
        "or promotion of a previous result into config.json",
        description="Steps a joint in small, individually-unloaded torque bursts and uses a "
        "camera (or, under --sim, a synthetic stand-in) to tell whether it actually moved. "
        "Repeated no-motion is a candidate mechanical stop. Writes a timestamped result "
        "file; never edits config.json unless you separately run --apply. "
        "See docs/calibration.md for what this can and cannot detect.",
    )
    p.add_argument("--joint", action="append", metavar="NAME",
                   help="Joint to calibrate (repeatable). With --apply, restricts which joints are promoted.")
    p.add_argument("--all", action="store_true", help="Calibrate every joint")
    p.add_argument("--apply", metavar="FILE",
                   help="Promote a previous calibration-*.json into config.json (commands no motion)")
    p.add_argument("--allow-simulated", action="store_true", dest="allow_simulated",
                   help="With --apply, permit promoting a result produced against the simulator. "
                   "Simulated limits are assumptions about this arm, not measurements of it.")
    p.add_argument("--step", type=int, default=None, help="Probe increment in raw units (default 30)")
    p.add_argument("--margin", type=int, default=None,
                   help="Back-off from the last confirmed-moving position, in raw units (default 60)")
    p.add_argument("--confirmations", type=int, default=None,
                   help="Consecutive no-motion observations required before believing a stop (default 2, minimum 2)")
    p.add_argument("--step-duration", type=int, default=None, dest="step_duration",
                   help="Torque burst per step in ms (default config.MIN_DURATION_MS; must stay under 500)")
    p.add_argument("--out-dir", default=None, dest="out_dir",
                   help="Directory for the result file (default robohand/data/)")
    p.add_argument("--i-am-present-and-watching", action="store_true", dest="i_am_present_and_watching",
                   help="Required on real hardware: confirms a human is physically present, watching the "
                   "arm, with the power switch in reach and the workspace (and gripper jaws) clear.")
    p.add_argument("--json", action="store_true")
    p.set_defaults(func=cmd_calibrate)

    return parser


def main(argv: Optional[list] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    # Imported lazily-but-eagerly here (not inside the handler) so the except
    # clauses below are bound before any command runs.
    from .calibrate import CalibrationAborted, CalibrationRefused
    from .observer import ObserverError
    from .pick import PickRefused

    try:
        return args.func(args)
    except PickRefused as e:
        print(f"error: refusing to pick -- {e}", file=sys.stderr)
        return 2
    except CalibrationRefused as e:
        print(f"error: refusing to calibrate -- {e}", file=sys.stderr)
        return 2
    except CalibrationAborted as e:
        print(f"error: calibration aborted, nothing written -- {e}", file=sys.stderr)
        return 1
    except ObserverError as e:
        print(f"error: motion observer unusable -- {e}", file=sys.stderr)
        return 1
    except CalibrationRequired as e:
        print(f"error: refusing to move -- {e}", file=sys.stderr)
        return 1
    except StallDetected as e:
        print(f"error: stall/obstruction detected -- {e}", file=sys.stderr)
        return 1
    except SafetyViolation as e:
        print(f"error: safety violation -- {e}", file=sys.stderr)
        return 1
    except (config.InvalidPosition, config.ConfigError) as e:
        print(f"error: {e}", file=sys.stderr)
        return 2
    except json.JSONDecodeError as e:
        print(f"error: invalid JSON -- {e}", file=sys.stderr)
        return 2
    except KeyError as e:
        print(f"error: {e}", file=sys.stderr)
        return 1
    except ValueError as e:
        print(f"error: {e}", file=sys.stderr)
        return 2
    except NoResponse as e:
        print(f"error: {e}", file=sys.stderr)
        return 1
    except OSError as e:
        print(f"error: HID device problem: {e}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("\nInterrupted.", file=sys.stderr)
        return 130


if __name__ == "__main__":
    sys.exit(main())
