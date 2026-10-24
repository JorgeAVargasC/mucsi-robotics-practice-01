"""ROS 2 node that records the UR3e joint states."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from ur3e_kinematics.constants import ARM_JOINTS, JOINT_STATES_TOPIC, QUEUE_DEPTH

# * One log per second
LOG_THROTTLE_S = 1.0


class KinematicsRecorder(Node):
    """Subscribe to joint states and keep only the arm joints."""

    def __init__(self) -> None:
        """Initialize the node and its joint-state subscription."""
        super().__init__('kinematics_recorder')
        self.create_subscription(
            msg_type=JointState,
            topic=JOINT_STATES_TOPIC,
            callback=self._on_joint_states,
            qos_profile=QUEUE_DEPTH,
        )

    def _on_joint_states(self, msg: JointState) -> None:
        """Log arm positions, using joint names because message order can vary."""
        # The name and position arrays share indices, but joint ordering can vary.
        positions = dict(zip(msg.name, msg.position))

        # Ignore messages that do not contain every arm joint.
        if not all(joint in positions for joint in ARM_JOINTS):
            return

        arm_positions = {joint: positions[joint] for joint in ARM_JOINTS}
        self.get_logger().info(str(arm_positions), throttle_duration_sec=LOG_THROTTLE_S)


def main() -> None:
    """Initialize ROS 2 and spin the recorder node."""
    print('\033[32m')
    print('=' * 45)
    print('kinematics_recorder.py')
    print('=' * 45)
    print('\033[0m')

    rclpy.init()
    node = KinematicsRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
