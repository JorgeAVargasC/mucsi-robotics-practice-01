"""ROS 2 node that records the UR3e joint states."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from ur3e_kinematics.constants import ARM_JOINTS, JOINT_STATES_TOPIC, QUEUE_DEPTH

# * One log per second
LOG_THROTTLE_S = 1.0


class Ex01JointStatesSubscriber(Node):
    """Subscribe to /joint_states and keep only the ARM_JOINTS."""

    def __init__(self) -> None:

        super().__init__("ex_01_joint_states_subscriber")

        self.create_subscription(
            msg_type=JointState,
            topic=JOINT_STATES_TOPIC,
            callback=self._on_joint_states,
            qos_profile=QUEUE_DEPTH,  # number of messages kept in the queue
        )

    def _on_joint_states(self, msg: JointState) -> None:
        """Log arm positions, indexed by name because name[] has no fixed order."""

        #! skip gripper messages:
        #! ['finger_joint', 'left_inner_knuckle_joint', 'left_inner_finger_joint', 'right_outer_knuckle_joint', 'right_inner_knuckle_joint', 'right_inner_finger_joint']

        if set(msg.name) == set(ARM_JOINTS):
            # ? msg.name     = ["elbow_joint", "shoulder_lift_joint", "shoulder_pan_joint", ...]
            # ? msg.position = [-1.6         , -1.6                 , 0.0                 , ...]

            positions = dict(zip(msg.name, msg.position))
            # ? zip -> ("elbow_joint", -1.6), ("shoulder_lift_joint", -1.6), ...
            # ? dict -> positions = {"elbow_joint": -1.6, "shoulder_lift_joint": -1.6 ...

            stamp = msg.header.stamp
            sec = stamp.sec
            nanosec = stamp.nanosec
            timestamp = sec + (nanosec * 1e-9)
            # ? stamp.sec = 1761645600, stamp.nanosec = 123456789 -> timestamp = 1761645600.123

            self.get_logger().info(
                f"t={timestamp:.3f} {positions}", throttle_duration_sec=LOG_THROTTLE_S
            )


def main() -> None:
    print("\033[32m")
    print("=" * 45)
    print("ex_01_joint_states_subscriber.py")
    print("=" * 45)
    print("\033[0m")

    rclpy.init()
    node = Ex01JointStatesSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
