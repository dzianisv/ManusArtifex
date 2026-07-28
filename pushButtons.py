#!/usr/bin/env python3
"""DISABLED legacy script -- kept only as a record of the original servo map.

This script drove the arm through the raw `xarm` library, bypassing every
safety layer in the `robohand` package. It commands ELBOW to 2600, which is
past the servo's 2500 electrical maximum and well past its mechanical stop --
exactly the class of command that stalled the gripper on 2026-07-28 and forced
the operator to cut power.

Use the `robohand` CLI instead. See docs/safety.md.
"""

import sys

sys.exit(
    "pushButtons.py is DISABLED: it bypasses all safety limits and commands "
    "ELBOW to 2600 (past the 2500 maximum). Use the `robohand` CLI. "
    "See docs/safety.md."
)

# ---------------------------------------------------------------------------
# Historical reference only -- the verified servo map. Do not re-enable.
#
#   1=GRIP  2=WRIST_ROLL  3=WRIST_FLEX  4=ELBOW  5=SHOULDER_LIFT  6=SHOULDER_PAN
#
# Note GRIP is special: its valid range is 1500..2500, not 500..2500.
# ---------------------------------------------------------------------------

# https://www.youtube.com/watch?v=Qh7lPC_Qf74&list=PLFbzd0m6AcmIRtTkAc3stBz77wRqlpMFD&index=11
import xarm
import enum

limit=1600

class Servo(enum.Enum):
    GRIP=1
    WRIST_ROLL=2
    WRIST_FLEX=3
    ELBOW=4
    SHOULDER_LIFT=5
    SHOULDER_PAN=6


arm = xarm.Controller('USB', debug=True)
arm.actionGroupRun(0)

def move(states):
    for state in states:
        print(state)
        arm.setPosition([[servo_id.value, angle] for servo_id, angle in state.items()], wait=True)

def push_enter():
    cmds = [
        {
            Servo.ELBOW: 2100
        },{
            Servo.ELBOW: 2600,
        }
    ]
    move(cmds)

def push_power_button():
    cmds = [
        {
            Servo.SHOULDER_PAN: 700,
        },{
            Servo.ELBOW: 2100
        },{
            Servo.ELBOW: 2600
        },{
            Servo.SHOULDER_PAN: 1100
        }
    ]

    move(cmds)

def shoulder_test():
    cmds = [
        {
            Servo.SHOULDER_LIFT: 700,
        },         {
            Servo.SHOULDER_LIFT: 1500,
        },         {
            Servo.SHOULDER_LIFT: 2500,
        }, 
    ]

    move(cmds)


shoulder_test()