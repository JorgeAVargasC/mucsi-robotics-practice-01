"""ROS 2 node  that records the UR3e joint states."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from ur3e_kinematics.constants import ARM_JOINTS, JOINT_STATES_TOPIC, QUEUE_DEPTH

# * One log per second
LOG_THROTTLE_S = 1.0


class KinematicsRecorder(Node):
    """Suscribe to /joint_states and keep only the ARM_JOINTS."""

    def __init__(self) -> None:
        super().__init__("kinematics_recorder")
        self.create_subscription(
            msg_type=JointState,
            topic=JOINT_STATES_TOPIC,
            callback=self._on_joint_states,
            qos_profile=QUEUE_DEPTH,
        )

    def _on_joint_states(self, msg: JointState) -> None:
        """Log arm positions, indexed by name beacause name[] has no fixed order."""
        positions = dict(zip(msg.name, msg.position))
        # ? msg.name     = ["elbow_joint", "shoulder_lift_joint", "shoulder_pan_joint", ...]
        # ? msg.position = [-1.6         , -1.6                 , 0.0                 , ...]
        # ? ("elbow_joint", -1.6), ("shoulder_lift_joint", -1.6), ...
        # ? positions = {"elbow_joint": -1.6, "shoulder_lift_joint": -1.6

        #! skip gripper messages: they do not include the arm joints
        if not all(joint in positions for joint in ARM_JOINTS):
            return

        arm_positions = {joint: positions[joint] for joint in ARM_JOINTS}
        self.get_logger().info(str(arm_positions), throttle_duration_sec=LOG_THROTTLE_S)


def main() -> None:
    print("\033[32m")
    print("=" * 45)
    print("kinematics_recorder.py")
    print("=" * 45)
    print("\033[0m")

    rclpy.init()
    node = KinematicsRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
