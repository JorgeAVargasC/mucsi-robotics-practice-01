"""Run the practice trajectories on the UR3e."""

import copy
from math import cos, pi, sin

from geometry_msgs.msg import Pose
import rclpy

from ur3e_kinematics.constants import (
    CIRCLE_RADIUS_M,
    CIRCLE_WAYPOINTS,
    HOME_JOINTS,
    JOINT_TARGET_JOINTS,
    LINEAR_DISTANCE_M,
)
from ur3e_kinematics.robot_controller import RobotController


class TrajectoryExecutor(RobotController):
    """Move the robot through the Joint, Linear and Circular trajectories."""

    def __init__(self) -> None:
        """Connect to MoveIt."""
        super().__init__('trajectory_executor')

    def run(self) -> None:
        """Move to home and run the trajectories in order."""
        self._move_home()
        self._run_joint_trajectory()
        self._move_home()
        self._run_linear_trajectory()
        self._move_home()
        self._run_circular_trajectory()
        self._move_home()

    def _move_home(self) -> None:
        """Move to the start position."""
        if not self.move_joints(HOME_JOINTS):
            self.get_logger().error('Could not reach the home position')

    def _run_joint_trajectory(self) -> None:
        """Move all joints at once from home to the target (the tool path is a curve)."""
        self.get_logger().info('Joint trajectory: start')
        if not self.move_joints(JOINT_TARGET_JOINTS):
            self.get_logger().error('Joint trajectory failed')

    def _run_linear_trajectory(self) -> None:
        """Move the tool in a straight line, keeping its orientation."""
        self.get_logger().info('Linear trajectory: start')
        target = self.current_pose()
        target.position.y += LINEAR_DISTANCE_M
        if not self.move_cartesian_path([target]):
            self.get_logger().error('Linear trajectory failed')

    def _run_circular_trajectory(self) -> None:
        """Move the tool around a full circle, keeping its orientation."""
        self.get_logger().info('Circular trajectory: start')
        if not self.move_cartesian_path(self._circle_waypoints()):
            self.get_logger().error('Circular trajectory failed')

    def _circle_waypoints(self) -> list[Pose]:
        """Return points on a circle that starts and ends at the current tool position."""
        start = self.current_pose()
        # The center is one radius away, so the circle passes through the start point
        center_y = start.position.y - CIRCLE_RADIUS_M
        center_z = start.position.z
        waypoints = []
        for step in range(1, CIRCLE_WAYPOINTS + 1):
            angle = 2 * pi * step / CIRCLE_WAYPOINTS
            waypoint = copy.deepcopy(start)
            waypoint.position.y = center_y + CIRCLE_RADIUS_M * cos(angle)
            waypoint.position.z = center_z + CIRCLE_RADIUS_M * sin(angle)
            waypoints.append(waypoint)
        return waypoints


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
