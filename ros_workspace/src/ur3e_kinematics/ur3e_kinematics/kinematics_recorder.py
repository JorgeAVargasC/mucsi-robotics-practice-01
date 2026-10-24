"""ROS 2 node that records the UR3e joint states."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from ur3e_kinematics.constants import ARM_JOINTS, JOINT_STATES_TOPIC, QUEUE_DEPTH
from ur3e_kinematics.joint_state_processor import JointStateProcessor

# * One log per second
LOG_THROTTLE_S = 1.0


class KinematicsRecorder(Node):
    """Subscribe to joint states and log arm kinematics."""

    def __init__(self) -> None:
        """Initialize the node and its joint-state subscription."""
        super().__init__('kinematics_recorder')
        self._processor = JointStateProcessor(joint_names=ARM_JOINTS)
        self.create_subscription(
            msg_type=JointState,
            topic=JOINT_STATES_TOPIC,
            callback=self._on_joint_states,
            qos_profile=QUEUE_DEPTH,
        )

    def _on_joint_states(self, msg: JointState) -> None:
        """Process and log arm positions, velocities, and accelerations."""
        timestamp = (
            msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        )
        try:
            sample = self._processor.process(
                timestamp=timestamp,
                joint_names=msg.name,
                positions=msg.position,
                velocities=msg.velocity,
            )
        except ValueError as error:
            self.get_logger().warning(
                f'Ignoring invalid joint-state message: {error}',
                throttle_duration_sec=LOG_THROTTLE_S,
            )
            return

        if sample is None:
            return

        sample_data = {
            'timestamp_s': sample.timestamp,
            'positions_rad': sample.positions,
            'velocities_rad_s': sample.velocities,
            'accelerations_rad_s2': sample.accelerations,
        }
        self.get_logger().info(
            f'Arm kinematics: {sample_data}',
            throttle_duration_sec=LOG_THROTTLE_S,
        )


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
