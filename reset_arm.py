#!/usr/bin/env python3
"""Emergency reset for the LeArm/xArm LSC-6 controller.

Sends ONLY a multi-servo unload (torque off). It never commands motion, so it is
safe to run at any time -- including immediately after a stall. After unload the
joints are back-drivable by hand.

Usage:
    python3 reset_arm.py
"""
from __future__ import annotations

import sys
import time

VENDOR_ID = 0x0483
PRODUCT_ID = 0x5750
SIGNATURE = 0x55
CMD_MULT_SERVO_UNLOAD = 20
CMD_ACTION_GROUP_STOP = 7
CMD_GET_BATTERY_VOLTAGE = 15
SERVO_IDS = (1, 2, 3, 4, 5, 6)


def _packet(cmd: int, params: tuple[int, ...] = ()) -> list[int]:
    length = len(params) + 2
    return [0x00, SIGNATURE, SIGNATURE, length, cmd, *params]


def main() -> int:
    try:
        import hid
    except ImportError:
        print("hidapi not installed: pip install hidapi", file=sys.stderr)
        return 2

    try:
        dev = hid.device()
        dev.open(VENDOR_ID, PRODUCT_ID)
    except Exception as exc:
        print(f"Cannot open arm (is it powered on and plugged in?): {exc}", file=sys.stderr)
        return 1

    try:
        dev.set_nonblocking(1)

        dev.write(_packet(CMD_ACTION_GROUP_STOP))
        time.sleep(0.05)
        print("action group stopped")

        # Unload all servos: torque off, joints become back-drivable.
        dev.write(_packet(CMD_MULT_SERVO_UNLOAD, (len(SERVO_IDS), *SERVO_IDS)))
        time.sleep(0.05)
        print(f"servos unloaded (torque OFF): {list(SERVO_IDS)}")

        dev.write(_packet(CMD_GET_BATTERY_VOLTAGE))
        deadline = time.time() + 1.0
        while time.time() < deadline:
            report = dev.read(64, timeout_ms=100)
            if len(report) >= 6 and report[0] == SIGNATURE and report[1] == SIGNATURE:
                if report[3] == CMD_GET_BATTERY_VOLTAGE:
                    mv = report[4] | (report[5] << 8)
                    print(f"battery: {mv / 1000:.3f} V")
                    break
            time.sleep(0.02)
        else:
            print("battery: no reply (not fatal)")

        print("\nARM IS RELAXED. Move the joints gently by hand to a neutral pose.")
        return 0
    finally:
        dev.close()


if __name__ == "__main__":
    raise SystemExit(main())
