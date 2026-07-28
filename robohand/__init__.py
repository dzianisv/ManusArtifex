"""robohand: control library and CLI for the Hiwonder/LewanSoul LeArm (xArm 1S, LSC-6)."""

from .arm import Arm
from .config import JOINT_LIMITS, JOINT_NAMES, SERVO_IDS

__version__ = "0.1.0"

__all__ = ["Arm", "JOINT_LIMITS", "JOINT_NAMES", "SERVO_IDS", "__version__"]
