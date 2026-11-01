"""ROS 2 node that records the UR3e joint states."""

from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from ur3e_kinematics.constants import (
    ARM_JOINTS, JOINT_STATES_TOPIC, QUEUE_DEPTH, TRAJECTORIES, TRAJECTORY_TOPIC,
)
from ur3e_kinematics.joint_state_processor import JointStateProcessor
from ur3e_kinematics.kinematics_csv_writer import KinematicsCsvWriter

# * One log per second
LOG_THROTTLE_S = 1.0


class KinematicsRecorder(Node):
    """Subscribe to joint states and log arm kinematics."""

    def __init__(self) -> None:
        """Initialize the node and its joint-state subscription."""
        super().__init__('kinematics_recorder')
        self._processor = JointStateProcessor(joint_names=ARM_JOINTS)
        self._trajectory = ''
        self.declare_parameter(
            'output_directory', str(Path.home() / 'ros_workspace')
        )
        output_directory = Path(
            self.get_parameter('output_directory').get_parameter_value().string_value
        )
        output_directory.mkdir(parents=True, exist_ok=True)
        run_id = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        self._output_path = output_directory / f'kinematics_{run_id}.csv'
        self._csv_writer = KinematicsCsvWriter(self._output_path, ARM_JOINTS)
        self.get_logger().info(
            f'Saving joint kinematics to {self._output_path.resolve()}'
        )
        self.create_subscription(
            String, TRAJECTORY_TOPIC, self._on_trajectory,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.create_subscription(
            msg_type=JointState,
            topic=JOINT_STATES_TOPIC,
            callback=self._on_joint_states,
            qos_profile=QUEUE_DEPTH,
        )

    def _on_trajectory(self, msg: String) -> None:
        """Record only the trajectories announced by the executor."""
        self._trajectory = msg.data if msg.data in TRAJECTORIES else ''

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

        if sample is None or not self._trajectory:
            return

        self._csv_writer.write_sample(sample, self._trajectory)
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

    def close(self) -> None:
        """Close the output file after the recorder stops."""
        self._csv_writer.close()


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
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
