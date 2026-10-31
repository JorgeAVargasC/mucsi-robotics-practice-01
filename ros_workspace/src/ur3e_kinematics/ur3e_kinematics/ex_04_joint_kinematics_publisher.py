"""ROS 2 node that publishes the UR3e kinematics in one custom message."""

from math import nan

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from ur3e_kinematics.constants import (
    ARM_JOINTS,
    JOINT_KINEMATICS_TOPIC,
    JOINT_STATES_TOPIC,
    QUEUE_DEPTH,
)
from ur3e_kinematics.joint_state_processor import JointStateProcessor
from ur3e_kinematics_interfaces.msg import JointKinematics

# * One log per second
LOG_THROTTLE_S = 1.0


def _none_to_nan(value: float | None) -> float:
    """Replace a missing value because float64 fields cannot hold None."""

    return nan if value is None else value
    # ? None -> nan, 0.5 -> 0.5


class Ex04JointKinematicsPublisher(Node):
    """Subscribe to /joint_states and publish /joint_kinematics."""

    def __init__(self) -> None:

        super().__init__("ex_04_joint_kinematics_publisher")

        self._processor = JointStateProcessor(joint_names=ARM_JOINTS)

        # write
        self._publisher = self.create_publisher(
            msg_type=JointKinematics,
            topic=JOINT_KINEMATICS_TOPIC,
            qos_profile=QUEUE_DEPTH,
        )

        # read
        self.create_subscription(
            msg_type=JointState,
            topic=JOINT_STATES_TOPIC,
            callback=self._on_joint_states,
            qos_profile=QUEUE_DEPTH,
        )

    def _on_joint_states(self, msg: JointState) -> None:
        """Calculate the arm kinematics and publish them in one message."""

        timestamp = msg.header.stamp.sec + (msg.header.stamp.nanosec * 1e-9)
        # ? stamp.sec = 1761645600, stamp.nanosec = 123456789 -> timestamp = 1761645600.123

        try:
            sample = self._processor.process(
                timestamp=timestamp,
                joint_names=msg.name,
                positions=msg.position,
                velocities=msg.velocity,
            )
        except ValueError as error:
            self.get_logger().warning(
                f"Ignoring invalid joint-state message: {error}",
                throttle_duration_sec=LOG_THROTTLE_S,
            )
            return

        #! skip gripper messages: they do not contain the ARM_JOINTS
        if sample is None:
            return

        #! all lists follow the ARM_JOINTS order, not the msg.name order
        # ? joint_names[0] = "shoulder_pan_joint" -> positions[0], velocities[0], accelerations[0]
        kinematics = JointKinematics()
        kinematics.header = msg.header
        kinematics.joint_names = list(ARM_JOINTS)
        kinematics.positions = [sample.positions[name] for name in ARM_JOINTS]
        kinematics.velocities = [
            _none_to_nan(sample.velocities[name]) for name in ARM_JOINTS
        ]
        kinematics.accelerations = [
            _none_to_nan(sample.accelerations[name]) for name in ARM_JOINTS
        ]

        self._publisher.publish(kinematics)

        self.get_logger().info(
            f"Published kinematics at t={timestamp:.3f}",
            throttle_duration_sec=LOG_THROTTLE_S,
        )


def main() -> None:
    print("\033[32m")
    print("=" * 45)
    print("ex_04_joint_kinematics_publisher.py")
    print("=" * 45)
    print("\033[0m")

    rclpy.init()
    node = Ex04JointKinematicsPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
