"""Command-line interface for robohand.

Only one process may hold the arm's HID handle at a time -- every command opens the
device, does its work, and closes it again before exiting. Ctrl-C is caught during any
motion command and relaxes the servos before exiting, rather than leaving the arm
mid-move with a wedged handle.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Optional

from . import config, poses as pose_store, sequences
from .arm import Arm
from .protocol import NoResponse


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
    with Arm() as arm:
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
    with Arm() as arm:
        positions = arm.get_positions()
        _print_positions(positions, as_json=args.json)
    return 0


def _parse_joints(raw: str) -> dict:
    data = json.loads(raw)
    return {k.upper(): int(v) for k, v in data.items()}


def cmd_move(args: argparse.Namespace) -> int:
    if args.joints:
        targets = _parse_joints(args.joints)
    elif args.servo and args.pos is not None:
        targets = {args.servo.upper(): args.pos}
    else:
        print("error: provide either --joints '{...}' or --servo NAME --pos N", file=sys.stderr)
        return 2
    with Arm() as arm:
        try:
            result = arm.set_positions(targets, duration_ms=args.duration, wait=True)
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
        with Arm() as arm:
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
        with Arm() as arm:
            result = arm.set_positions(pose["positions"], duration_ms=args.duration or pose.get("duration_ms"))
        print(json.dumps(result, indent=2))
        return 0
    print("error: specify a pose name, --list, or --save NAME", file=sys.stderr)
    return 2


def cmd_grip(args: argparse.Namespace) -> int:
    limit = config.JOINT_LIMITS["GRIP"]
    if args.action == "open":
        target = limit.max if args.strength is None else config.clamp("GRIP", limit.center + args.strength)
    else:
        target = limit.min if args.strength is None else config.clamp("GRIP", limit.center - args.strength)
    with Arm() as arm:
        pos = arm.set_position("GRIP", target, duration_ms=args.duration)
    print(f"GRIP -> {pos}")
    return 0


def cmd_home(args: argparse.Namespace) -> int:
    with Arm() as arm:
        result = arm.home()
    print(json.dumps(result, indent=2))
    return 0


def cmd_relax(args: argparse.Namespace) -> int:
    with Arm() as arm:
        names = [s.upper() for s in args.servo] if args.servo else None
        arm.relax(names)
    print(f"Relaxed: {args.servo or 'ALL'}")
    return 0


def cmd_teach(args: argparse.Namespace) -> int:
    names = [s.upper() for s in args.servo] if args.servo else config.JOINT_NAMES
    with Arm() as arm:
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
                    positions[name] = config.clamp(name, int(raw))
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
    with Arm() as arm:
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
    """First-cut pick routine: open -> approach -> descend -> close -> lift -> verify.

    Relies on the pick_approach/pick_descend/pick_lift poses in poses.json, which ship
    as placeholders -- run 'robohand teach pick_approach' / 'pick_descend' to
    calibrate them for a real target before trusting this for anything but a dry run.
    """
    with Arm() as arm:
        try:
            print("1/5 open grip")
            arm.set_positions(pose_store.get_pose("open_grip")["positions"], duration_ms=600)
            print("2/5 approach")
            approach = pose_store.get_pose("pick_approach")
            arm.set_positions(approach["positions"], duration_ms=approach.get("duration_ms"))
            print("3/5 descend")
            descend = pose_store.get_pose("pick_descend")
            arm.set_positions(descend["positions"], duration_ms=descend.get("duration_ms"))
            print("4/5 close grip")
            close_target = config.clamp("GRIP", config.JOINT_LIMITS["GRIP"].min + (args.strength or 0))
            arm.set_position("GRIP", close_target, duration_ms=600)
            print("5/5 lift")
            lift = pose_store.get_pose("pick_lift")
            arm.set_positions(lift["positions"], duration_ms=lift.get("duration_ms"))
            grip_state = arm.get_position("GRIP")
            print(f"Done. GRIP now at {grip_state}. Verify visually/with camera that an object was grasped.")
        except KeyboardInterrupt:
            arm.relax()
            print("\nInterrupted -- servos relaxed.", file=sys.stderr)
            return 130
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="robohand", description="Control the LeArm/xArm 1S 6-DOF arm.")
    sub = parser.add_subparsers(dest="command", required=True)

    p = sub.add_parser("info", help="Show device, battery, and all servo positions")
    p.add_argument("--json", action="store_true")
    p.set_defaults(func=cmd_info)

    p = sub.add_parser("pos", help="Print current positions")
    p.add_argument("--json", action="store_true")
    p.set_defaults(func=cmd_pos)

    p = sub.add_parser("move", help="Move one servo or several named joints")
    p.add_argument("--servo", help="Joint name, e.g. GRIP")
    p.add_argument("--pos", type=int, help="Target position (500-2500)")
    p.add_argument("--joints", help='JSON object, e.g. \'{"ELBOW":2100,"GRIP":1200}\'')
    p.add_argument("--duration", type=int, default=None, help="Move duration in ms (default 1000)")
    p.set_defaults(func=cmd_move)

    p = sub.add_parser("pose", help="Move to / list / save named poses")
    p.add_argument("name", nargs="?", help="Pose name to move to")
    p.add_argument("--list", action="store_true", help="List known poses")
    p.add_argument("--save", metavar="NAME", help="Save current positions as a named pose")
    p.add_argument("--duration", type=int, default=None)
    p.set_defaults(func=cmd_pose)

    p = sub.add_parser("grip", help="Open or close the grip")
    p.add_argument("action", choices=["open", "close"])
    p.add_argument("--strength", type=int, default=None, help="Offset from center (raw units)")
    p.add_argument("--duration", type=int, default=800)
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

    p = sub.add_parser("pick", help="Run open->approach->descend->close->lift pick routine")
    p.add_argument("--strength", type=int, default=200, help="Grip closing offset from fully-open min")
    p.set_defaults(func=cmd_pick)

    return parser


def main(argv: Optional[list] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        return args.func(args)
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
