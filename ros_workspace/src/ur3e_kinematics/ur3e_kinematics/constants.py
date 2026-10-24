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
HOME_SHOULDER_PAN_RAD: float = 0.0
HOME_SHOULDER_LIFT_RAD: float = -pi / 2
HOME_ELBOW_RAD: float = -pi / 2
HOME_WRIST_1_RAD: float = -pi / 2
HOME_WRIST_2_RAD: float = pi / 2
HOME_WRIST_3_RAD: float = 0.0

# Same order as ARM_JOINTS
HOME_JOINTS: tuple[float, ...] = (
    HOME_SHOULDER_PAN_RAD,
    HOME_SHOULDER_LIFT_RAD,
    HOME_ELBOW_RAD,
    HOME_WRIST_1_RAD,
    HOME_WRIST_2_RAD,
    HOME_WRIST_3_RAD,
)

# Joint trajectory target (rad): the base turns 90 degrees and the elbow opens.
# Shoulder lift + elbow + wrist 1 add up to the same value as in home,
# so the tool keeps pointing down.
JOINT_TARGET_SHOULDER_PAN_RAD: float = pi / 2  #  0 -> 90 degress
JOINT_TARGET_SHOULDER_LIFT_RAD: float = HOME_SHOULDER_LIFT_RAD
JOINT_TARGET_ELBOW_RAD: float = -pi / 4  # -90 degress -> -45 degress
JOINT_TARGET_WRIST_1_RAD: float = HOME_WRIST_1_RAD
JOINT_TARGET_WRIST_2_RAD: float = HOME_WRIST_2_RAD
JOINT_TARGET_WRIST_3_RAD: float = HOME_WRIST_3_RAD

# Same order as ARM_JOINTS
JOINT_TARGET_JOINTS: tuple[float, ...] = (
    JOINT_TARGET_SHOULDER_PAN_RAD,
    JOINT_TARGET_SHOULDER_LIFT_RAD,
    JOINT_TARGET_ELBOW_RAD,
    JOINT_TARGET_WRIST_1_RAD,
    JOINT_TARGET_WRIST_2_RAD,
    JOINT_TARGET_WRIST_3_RAD,
)
