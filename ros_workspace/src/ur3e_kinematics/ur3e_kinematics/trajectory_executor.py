"""Run the practice trajectories on the UR3e."""

import rclpy

from ur3e_kinematics.constants import HOME_JOINTS
from ur3e_kinematics.robot_controller import RobotController


class TrajectoryExecutor(RobotController):
    """Move the robot through the Joint, Linear and Circular trajectories."""

    def __init__(self) -> None:
        """Connect to MoveIt."""
        super().__init__('trajectory_executor')

    def run(self) -> None:
        """Move to home and run the trajectories in order."""
        self._move_home()

    def _move_home(self) -> None:
        """Move to the start position."""
        if not self.move_joints(HOME_JOINTS):
            self.get_logger().error('Could not reach the home position')


def main() -> None:
    """Run the trajectories once and exit."""
    rclpy.init()
    node = TrajectoryExecutor()
    try:
        node.run()
    finally:
        node.shutdown()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
