"""Shared constants between nodes."""

JOINT_STATES_TOPIC: str = '/joint_states'
QUEUE_DEPTH: int = 10

ARM_JOINTS: tuple[str, ...] = (
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
)
