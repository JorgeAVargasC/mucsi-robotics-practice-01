"""Shared constants between nodes."""

from math import pi

JOINT_STATES_TOPIC: str = "/joint_states"
QUEUE_DEPTH: int = 10

ARM_JOINTS: tuple[str, ...] = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)

# Home position, same as in the course example (rad).
RAD_SHOULDER_PAN: float = 0.0
RAD_SHOULDER_LIFT: float = -pi / 2
RAD_ELBOW: float = -pi / 2
RAD_WRIST_1: float = -pi / 2
RAD_WRIST_2: float = pi / 2
RAD_WRIST_3: float = 0.0

# Same order as ARM_JOINTS
HOME_JOINTS: tuple[float, ...] = (
    RAD_SHOULDER_PAN,
    RAD_SHOULDER_LIFT,
    RAD_ELBOW,
    RAD_WRIST_1,
    RAD_WRIST_2,
    RAD_WRIST_3,
)
