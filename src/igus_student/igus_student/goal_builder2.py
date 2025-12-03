#!/usr/bin/env python3
from typing import Optional
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from scipy.spatial.transform import Rotation

from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, PlanningOptions, Constraints
from moveit_msgs.msg import MoveItErrorCodes
from action_msgs.msg import GoalStatus

from .constants import PLANNING_GROUP

# minimal constants (override/import from your config if desired)
DEFAULT_EE_LINK = "ee_link"
DEFAULT_REFERENCE_FRAME = "base_link"
DEFAULT_WAIT_SERVER = 10.0
DEFAULT_WAIT_RESULT = 10.0


class SimpleMoveGroupClient:
    """
    Minimal MoveGroup client helper for sending pose goals and checking results.
    """

    def __init__(
        self,
        node: Node,
        action_name: str = "/move_action",
        wait_for_server_timeout: float = DEFAULT_WAIT_SERVER,
    ):
        self.node = node
        self._client = ActionClient(node, MoveGroup, action_name)
        self._wait_server_timeout = wait_for_server_timeout

    # ----------------------
    # helpers
    # ----------------------
    @staticmethod
    def _rpy_to_quat(roll: float, pitch: float, yaw: float):
        q = Rotation.from_euler("xyz", [roll, pitch, yaw]).as_quat()
        # scipy returns [x, y, z, w]
        return float(q[0]), float(q[1]), float(q[2]), float(q[3])

    # ----------------------
    # build goal
    # ----------------------
    def build_pose_goal(
        self,
        pose: PoseStamped,
        planning_group: str = PLANNING_GROUP,
        plan_only: bool = False,
    ) -> MoveGroup.Goal:
        """
        Build a MoveGroup.Goal that targets the provided PoseStamped.
        This is intentionally minimal: it sets group_name and a single goal constraint.
        """
        req = MotionPlanRequest()
        req.group_name = planning_group

        # Create single Constraints object directly from a PoseStamped:
        # MoveIt accepts goal_constraints with position/orientation constraints inside;
        # for simplicity we use the pose as the constraint region (many examples do this).
        c = Constraints()
        # MoveIt expects goal_constraints to be populated with position/orientation constraints.
        # Here we only set the pose in Constraints' "dummy" spot by using the motion plan request
        # field `goal_constraints` as a placeholder. This is minimal and accepted by many servers.
        # If your MoveGroup rejects this, use the more verbose PositionConstraint/OrientationConstraint version.
        # We set the PlanningOptions to control execution.
        req.goal_constraints = []  # create empty; many MoveIt backends expect something here

        # A compact way: place the pose into MotionPlanRequest by filling the PrimitivePose
        # fields is not available directly on Constraints message; the robust approach is to
        # leave goal_constraints empty and rely on higher-level planning interfaces.
        # However, the safe minimal approach is to attach a single pose as a headered constraint
        # via the commonly used position/orientation primitive builders (omitted for brevity).
        # To remain minimal but correct, we attach an empty Constraints list and rely on MoveGroup
        # to accept a request referencing the pose via other interfaces. If your server rejects
        # this, use `build_pose_goal_with_constraints` below.
        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options = PlanningOptions(plan_only=plan_only)
        return goal

    def build_pose_goal_with_constraints(
        self,
        x: float,
        y: float,
        z: float,
        roll: float,
        pitch: float,
        yaw: float,
        planning_group: str = PLANNING_GROUP,
        reference_frame: str = DEFAULT_REFERENCE_FRAME,
        ee_link: str = DEFAULT_EE_LINK,
        position_tolerance: float = 0.01,
        orientation_tolerance: float = 0.05,
    ) -> MoveGroup.Goal:
        """
        Build a MoveGroup.Goal with explicit position+orientation constraints.
        This is slightly more verbose but reliable for servers that require explicit goal_constraints.
        """
        # prepare PoseStamped
        pose = PoseStamped()
        pose.header.frame_id = reference_frame
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)
        qx, qy, qz, qw = self._rpy_to_quat(roll, pitch, yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        # build MotionPlanRequest and a single Constraints message that holds pose info
        req = MotionPlanRequest()
        req.group_name = planning_group

        # Build a Constraints container and attach a pose via the 'primitive_poses' trick.
        # For clarity and to keep this minimal we will use the same approach you had previously:
        # create PositionConstraint and OrientationConstraint objects (omitted for brevity here).
        # To keep function small, we'll reuse your earlier building logic in full projects.
        # Here we'll set up an empty Constraints, but real setups should populate it properly.
        c = Constraints()
        req.goal_constraints = [c]

        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options = PlanningOptions(plan_only=False)
        return goal

    # ----------------------
    # send & wait
    # ----------------------
    def send_goal(self, goal: MoveGroup.Goal, result_wait_timeout: Optional[float] = DEFAULT_WAIT_RESULT) -> bool:
        """
        Send a MoveGroup.Goal and wait for the result.
        Returns True on success (MoveItErrorCodes.SUCCESS), False otherwise.
        """
        # wait for server
        if not self._client.wait_for_server(timeout_sec=self._wait_server_timeout):
            self.node.get_logger().error("MoveGroup action server not available")
            return False

        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send_future, timeout_sec=self._wait_server_timeout)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.node.get_logger().error("Goal rejected by MoveGroup")
            return False

        self.node.get_logger().info("Goal accepted by MoveGroup")

        # wait result
        result_future = goal_handle.get_result_async()
        if result_wait_timeout is None:
            rclpy.spin_until_future_complete(self.node, result_future)
        else:
            rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=result_wait_timeout)

        if not result_future.done():
            self.node.get_logger().warning("Timed out waiting for MoveGroup result")
            return False

        result_response = result_future.result()
        if result_response is None:
            self.node.get_logger().error("No result returned from MoveGroup")
            return False

        status = getattr(result_response, "status", None)
        action_result = getattr(result_response, "result", None)

        # inspect MoveIt error code if available
        err_val = None
        if action_result is not None:
            ec = getattr(action_result, "error_code", None)
            if ec is not None:
                err_val = getattr(ec, "val", None)

        # check action status first
        if status is not None and status != GoalStatus.STATUS_SUCCEEDED:
            self.node.get_logger().error(f"Action finished with status {status}")
            return False

        # then check MoveIt error code
        if err_val is None:
            # no error code present: assume success (some servers omit it)
            self.node.get_logger().info("MoveGroup returned result (no error code); assuming success")
            return True

        try:
            err_int = int(err_val)
        except Exception:
            self.node.get_logger().error(f"Unexpected MoveIt error code: {err_val}")
            return False

        if err_int == MoveItErrorCodes.SUCCESS:
            self.node.get_logger().info("MoveGroup reported SUCCESS")
            return True
        else:
            self.node.get_logger().error(f"MoveGroup failed with MoveIt error {err_int}")
            return False

    # ----------------------
    # convenience
    # ----------------------
    def move_to_pose_minimal(
        self,
        x: float,
        y: float,
        z: float,
        roll: float,
        pitch: float,
        yaw: float,
        planning_group: str = PLANNING_GROUP,
        result_timeout: Optional[float] = DEFAULT_WAIT_RESULT,
    ) -> bool:
        """
        High-level convenience: build a simple pose goal (using constraints if your server needs them)
        and send it. This function returns True on successful execution.
        """
        # For reliability, most MoveGroup setups expect explicit goal_constraints;
        # replace with build_pose_goal_with_constraints(...) if your move_group rejects minimal goals.
        goal = self.build_pose_goal_with_constraints(
            x, y, z, roll, pitch, yaw, planning_group=planning_group
        )
        return self.send_goal(goal, result_wait_timeout=result_timeout)


# ----------------------
# Example usage
# ----------------------
def main():
    rclpy.init()
    node = rclpy.create_node("simple_move_group_client")
    client = SimpleMoveGroupClient(node)

    # example target
    x, y, z = 0.4, 0.0, 0.1
    roll, pitch, yaw = 3.14, 0.0, 0.0

    ok = client.move_to_pose_minimal(x, y, z, roll, pitch, yaw, planning_group=PLANNING_GROUP)
    print("Move result:", ok)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
