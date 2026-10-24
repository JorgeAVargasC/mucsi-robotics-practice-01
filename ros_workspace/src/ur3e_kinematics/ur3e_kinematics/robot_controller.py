"""Move the UR3e with MoveIt 2. Based on the course example control_robot.py."""

from collections.abc import Sequence
import threading
import time
from typing import Any

from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    CollisionObject,
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    PlanningScene,
    RobotState,
)
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future
from rclpy.time import Time
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformListener

from ur3e_kinematics.constants import ARM_JOINTS, JOINT_STATES_TOPIC, QUEUE_DEPTH

# MoveIt configuration of the lab robot
REFERENCE_FRAME = 'world'
PLANNING_GROUP = 'robot'
END_EFFECTOR_LINK = 'wrist_3_link'

# Robot speed: 10% of its maximum velocity and acceleration
SPEED_SCALING = 0.1

# Joint-space planning
JOINT_TOLERANCE_RAD = 0.001
CONSTRAINT_WEIGHT = 1.0
PLANNING_ATTEMPTS = 10
PLANNING_TIME_S = 5.0

# Straight-line moves: run them only if MoveIt can do the whole path
CARTESIAN_STEP_M = 0.01
JUMP_THRESHOLD_DISABLED = 0.0
FULL_PATH_FRACTION = 1.0

# Waiting times
TF_TIMEOUT_S = 5.0
FUTURE_POLL_S = 0.01
JOINT_STATE_POLL_S = 0.05

# Floor box, so the robot never plans a move through the floor
FLOOR_NAME = 'floor'
FLOOR_SIZE_M = (2.0, 2.0, 0.05)
FLOOR_HEIGHT_M = -0.026

MoveItGoal = MoveGroup.Goal | ExecuteTrajectory.Goal
MoveItResult = MoveGroup.Result | ExecuteTrajectory.Result


class RobotController(Node):
    """Send movement requests to MoveIt and wait until they finish."""

    def __init__(self, node_name: str = 'robot_controller') -> None:
        """Connect to MoveIt, wait for the robot state and add the floor."""
        super().__init__(node_name)
        self._joint_positions: dict[str, float] = {}
        self.create_subscription(
            JointState, JOINT_STATES_TOPIC, self._on_joint_states, QUEUE_DEPTH)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._move_group_client = ActionClient(self, MoveGroup, 'move_action')
        self._execute_client = ActionClient(self, ExecuteTrajectory, 'execute_trajectory')
        self._cartesian_client = self.create_client(GetCartesianPath, 'compute_cartesian_path')
        self._scene_client = self.create_client(ApplyPlanningScene, 'apply_planning_scene')

        # Handle ROS messages in a background thread, so our methods can wait
        # for MoveIt. Do not call rclpy.spin() on this node.
        self._callback_executor = MultiThreadedExecutor()
        self._callback_executor.add_node(self)
        threading.Thread(target=self._callback_executor.spin, daemon=True).start()

        self._wait_for_moveit()
        self._add_floor()

    def move_joints(self, joint_positions: Sequence[float]) -> bool:
        """Move each joint to the given angle (rad). The tool does not move in a line."""
        constraints = Constraints(joint_constraints=self._joint_constraints(joint_positions))
        goal = self._move_group_goal(constraints)
        return self._succeeded(self._send_goal_and_wait(self._move_group_client, goal))

    def move_cartesian_path(self, waypoints: Sequence[Pose]) -> bool:
        """Move the tool in straight lines from where it is through each waypoint."""
        request = self._cartesian_path_request([self.current_pose(), *waypoints])
        response = self._wait_for(self._cartesian_client.call_async(request))
        if response.fraction < FULL_PATH_FRACTION:
            self.get_logger().error(
                f'Only {response.fraction:.0%} of the Cartesian path is reachable')
            return False
        goal = ExecuteTrajectory.Goal(trajectory=response.solution)
        return self._succeeded(self._send_goal_and_wait(self._execute_client, goal))

    def current_pose(self) -> Pose:
        """Return where the tool is now (position and orientation)."""
        transform = self._tf_buffer.lookup_transform(
            REFERENCE_FRAME, END_EFFECTOR_LINK, Time(),
            timeout=Duration(seconds=TF_TIMEOUT_S)).transform
        pose = Pose()
        pose.position.x = transform.translation.x
        pose.position.y = transform.translation.y
        pose.position.z = transform.translation.z
        pose.orientation = transform.rotation
        return pose

    def shutdown(self) -> None:
        """Stop the background executor and destroy the node."""
        self._callback_executor.shutdown()
        self.destroy_node()

    def _on_joint_states(self, msg: JointState) -> None:
        """Save the latest angle of each joint."""
        self._joint_positions.update(zip(msg.name, msg.position))

    def _wait_for_moveit(self) -> None:
        """Wait until MoveIt is ready and all arm joint angles have arrived."""
        self.get_logger().info('Waiting for MoveIt (move_group)...')
        self._move_group_client.wait_for_server()
        self._execute_client.wait_for_server()
        self._cartesian_client.wait_for_service()
        self._scene_client.wait_for_service()
        while not all(joint in self._joint_positions for joint in ARM_JOINTS):
            time.sleep(JOINT_STATE_POLL_S)

    def _joint_constraints(self, joint_positions: Sequence[float]) -> list[JointConstraint]:
        """Create one target angle per arm joint, in ARM_JOINTS order."""
        return [
            JointConstraint(
                joint_name=joint,
                position=float(position),
                tolerance_above=JOINT_TOLERANCE_RAD,
                tolerance_below=JOINT_TOLERANCE_RAD,
                weight=CONSTRAINT_WEIGHT,
            )
            for joint, position in zip(ARM_JOINTS, joint_positions)
        ]

    def _move_group_goal(self, constraints: Constraints) -> MoveGroup.Goal:
        """Create a request that plans and runs a move from the current state."""
        goal = MoveGroup.Goal()
        goal.request.group_name = PLANNING_GROUP
        goal.request.num_planning_attempts = PLANNING_ATTEMPTS
        goal.request.allowed_planning_time = PLANNING_TIME_S
        goal.request.max_velocity_scaling_factor = SPEED_SCALING
        goal.request.max_acceleration_scaling_factor = SPEED_SCALING
        goal.request.goal_constraints.append(constraints)
        # is_diff = True: start from the current robot state and scene
        goal.request.start_state.is_diff = True
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        goal.planning_options.plan_only = False
        return goal

    def _cartesian_path_request(self, waypoints: list[Pose]) -> GetCartesianPath.Request:
        """Create a request for a straight-line path through the waypoints."""
        request = GetCartesianPath.Request()
        request.header.frame_id = REFERENCE_FRAME
        request.start_state.is_diff = True
        request.group_name = PLANNING_GROUP
        request.link_name = END_EFFECTOR_LINK
        request.waypoints = waypoints
        request.max_step = CARTESIAN_STEP_M
        request.jump_threshold = JUMP_THRESHOLD_DISABLED
        request.avoid_collisions = True
        request.max_velocity_scaling_factor = SPEED_SCALING
        request.max_acceleration_scaling_factor = SPEED_SCALING
        return request

    def _send_goal_and_wait(self, client: ActionClient, goal: MoveItGoal) -> MoveItResult | None:
        """Send a goal and wait for the result. Return None if MoveIt rejects it."""
        goal_handle = self._wait_for(client.send_goal_async(goal))
        if not goal_handle.accepted:
            self.get_logger().error('MoveIt rejected the goal')
            return None
        return self._wait_for(goal_handle.get_result_async()).result

    @staticmethod
    def _succeeded(result: MoveItResult | None) -> bool:
        """Return True if MoveIt finished the move without errors."""
        return result is not None and result.error_code.val == MoveItErrorCodes.SUCCESS

    def _add_floor(self) -> None:
        """Add the floor as an obstacle."""
        floor_pose = Pose()
        floor_pose.position.z = FLOOR_HEIGHT_M
        floor_pose.orientation.w = 1.0  # w = 1: the box is not rotated
        self._add_collision_box(FLOOR_NAME, floor_pose, FLOOR_SIZE_M)

    def _add_collision_box(
            self, name: str, pose: Pose, size_m: tuple[float, float, float]) -> None:
        """Add a box that the robot must not hit."""
        box = CollisionObject(id=name, operation=CollisionObject.ADD)
        box.header.frame_id = REFERENCE_FRAME
        box.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(size_m)))
        box.primitive_poses.append(pose)

        request = ApplyPlanningScene.Request()
        request.scene = PlanningScene(is_diff=True, robot_state=RobotState(is_diff=True))
        request.scene.world.collision_objects.append(box)
        self._wait_for(self._scene_client.call_async(request))

    @staticmethod
    def _wait_for(future: Future) -> Any:
        """Wait until a request finishes and return its result."""
        while not future.done():
            time.sleep(FUTURE_POLL_S)
        return future.result()
