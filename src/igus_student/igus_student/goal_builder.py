"""
safe_move_to: 
    Sends a MoveGroup goal and waits for 
    the action server result (planning+execution result).

safe_move_and_wait: 
    Calls safe_move_to then additionally waits (polls) 
    until the robot has stopped moving for a stable period.
"""

from typing import Optional, Iterable, Sequence, Tuple
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from scipy.spatial.transform import Rotation

from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    PlanningOptions,
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
)
from moveit_msgs.msg import MoveItErrorCodes
from action_msgs.msg import GoalStatus

# import defaults from your constants
from .constants import (
    PLANNING_GROUP,
    END_EFFECTOR_LINK,
    REFERENCE_FRAME,
    POSITION_TOLERANCE,
    ORIENTATION_TOLERANCE,
    MAX_VEL_FACTOR,
    MAX_ACCEL_FACTOR,
    VELOCITY_THRESHOLD,
)

def build_move_group_goal(
    x: float,
    y: float,
    z: float,
    roll: float,
    pitch: float,
    yaw: float,
    planning_group: str = PLANNING_GROUP,
    ee_link: str = END_EFFECTOR_LINK,
    reference_frame: str = REFERENCE_FRAME,
    position_tolerance: float = POSITION_TOLERANCE,
    orientation_tolerance: float = ORIENTATION_TOLERANCE,
    max_vel_scaling: float = MAX_VEL_FACTOR,
    max_acc_scaling: float = MAX_ACCEL_FACTOR,
) -> MoveGroup.Goal:
    """Build and return a MoveGroup.Goal for a single pose (xyz + rpy)."""
    rot = Rotation.from_euler("xyz", [roll, pitch, yaw])
    qx, qy, qz, qw = rot.as_quat()

    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = reference_frame
    pose_stamped.pose.position.x = float(x)
    pose_stamped.pose.position.y = float(y)
    pose_stamped.pose.position.z = float(z)
    pose_stamped.pose.orientation.x = float(qx)
    pose_stamped.pose.orientation.y = float(qy)
    pose_stamped.pose.orientation.z = float(qz)
    pose_stamped.pose.orientation.w = float(qw)

    sphere = SolidPrimitive()
    sphere.type = SolidPrimitive.SPHERE
    sphere.dimensions = [float(position_tolerance)]

    pos_con = PositionConstraint()
    pos_con.header = pose_stamped.header
    pos_con.link_name = ee_link
    pos_con.constraint_region = BoundingVolume(
        primitives=[sphere], primitive_poses=[pose_stamped.pose]
    )
    pos_con.weight = 1.0

    ori_con = OrientationConstraint()
    ori_con.header = pose_stamped.header
    ori_con.link_name = ee_link
    ori_con.orientation = pose_stamped.pose.orientation
    ori_con.absolute_x_axis_tolerance = float(orientation_tolerance)
    ori_con.absolute_y_axis_tolerance = float(orientation_tolerance)
    ori_con.absolute_z_axis_tolerance = float(orientation_tolerance)
    ori_con.weight = 1.0

    req = MotionPlanRequest()
    req.group_name = planning_group
    req.goal_constraints = [
        Constraints(position_constraints=[pos_con], orientation_constraints=[ori_con])
    ]
    req.max_velocity_scaling_factor = float(max_vel_scaling)
    req.max_acceleration_scaling_factor = float(max_acc_scaling)

    goal = MoveGroup.Goal()
    goal.request = req
    goal.planning_options = PlanningOptions(plan_only=False)
    return goal


def send_move_group_goal(
    node: Node,
    action_client: ActionClient,
    goal: MoveGroup.Goal,
    wait_for_server_timeout: float = 10.0,
    result_wait_timeout: Optional[float] = None,
) -> bool:
    """
    Send a MoveGroup.Goal using the provided ActionClient.
    Returns True on success, False otherwise. Logs detailed status/error info.
    """
    if not action_client.wait_for_server(timeout_sec=wait_for_server_timeout):
        node.get_logger().error("MoveGroup server not available")
        return False

    send_future = action_client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send_future)
    goal_handle = send_future.result()
    if not goal_handle or not goal_handle.accepted:
        node.get_logger().error("Goal rejected by MoveGroup")
        return False

    node.get_logger().info("Goal accepted, waiting for result...")
    result_future = goal_handle.get_result_async()

    if result_wait_timeout is None:
        rclpy.spin_until_future_complete(node, result_future)
    else:
        start = time.time()
        while time.time() - start < result_wait_timeout:
            rclpy.spin_once(node, timeout_sec=0.05)
            if result_future.done():
                break
        if not result_future.done():
            node.get_logger().warning("Timed out waiting for goal result")
            return False

    result_response = result_future.result()
    if result_response is None:
        node.get_logger().error("No result returned from MoveGroup action")
        return False

    # action status (action_msgs/GoalStatus numeric)
    status = getattr(result_response, "status", None)

    # extract MoveIt error code & message if available
    action_result = getattr(result_response, "result", None)
    err_val = None
    err_msg = None
    if action_result is not None:
        ec = getattr(action_result, "error_code", None)
        if ec is not None:
            # MoveIt error code message type has .val and may have .message
            err_val = getattr(ec, "val", None) if hasattr(ec, "val") else ec
            err_msg = getattr(ec, "message", None)
        # fallback: maybe result has 'message' field
        if err_msg is None:
            err_msg = getattr(action_result, "message", None)

    # If the action status indicates failure, log details and return False
    if status is not None and status != GoalStatus.STATUS_SUCCEEDED:
        sname = goal_status_name(status)
        ename = moveit_error_name(err_val) if err_val is not None else "UNKNOWN"
        node.get_logger().error(
            f"MoveGroup action finished with status {status} ({sname}), "
            f"MoveIt error: {err_val} ({ename}), msg: {err_msg}"
        )
        return False

    # If the action status succeeded (or absent), validate MoveIt error code
    if err_val is None:
        node.get_logger().debug("MoveGroup result contains no error_code; assuming success")
        return True

    try:
        err_int = int(err_val)
    except Exception:
        node.get_logger().error(f"Unexpected error_code value: {err_val}")
        return False

    if err_int == MoveItErrorCodes.SUCCESS:
        node.get_logger().info("MoveGroup returned SUCCESS")
        return True
    else:
        name = moveit_error_name(err_int)
        node.get_logger().error(
            f"MoveGroup failed with MoveIt error code {err_int} ({name}), msg: {err_msg}"
        )
        return False


def safe_move_to(
    node_or_client,
    x: float,
    y: float,
    z: float,
    roll: float,
    pitch: float,
    yaw: float,
    *,
    action_client: Optional[ActionClient] = None,
    **build_kwargs,
) -> bool:
    """
    Convenience function: build a goal and send it.

    node_or_client: either an rclpy.Node (preferred) or an ActionClient.
      - If you pass a Node, the function will look for `node.move_client` unless you pass
        `action_client=` explicitly.
      - If you pass an ActionClient as the first arg, `action_client` param is ignored.

    Returns True on success.
    """
    # determine node and action_client
    node = None
    client = None
    if isinstance(node_or_client, ActionClient):
        client = node_or_client
        # no node available for logging/spinning, require node in that case
        raise RuntimeError(
            "safe_move_to: pass (node, action_client=...) or pass a Node with attribute move_client"
        )
    else:
        node = node_or_client
        client = action_client if action_client is not None else getattr(node, "move_client", None)
        if client is None:
            raise RuntimeError("safe_move_to: could not locate ActionClient (pass action_client=...)")

    goal = build_move_group_goal(x, y, z, roll, pitch, yaw, **build_kwargs)
    success = send_move_group_goal(node, client, goal)
    return success


def is_moving_from_velocities(velocities, threshold: float) -> bool:
    """Utility: check if velocities array indicates motion."""
    import numpy as _np
    if velocities is None:
        return False
    arr = _np.asarray(velocities)
    if arr.size == 0:
        return False
    return float(_np.max(_np.abs(arr))) > float(threshold)


def safe_move_and_wait(
    node,
    x: float,
    y: float,
    z: float,
    roll: float,
    pitch: float,
    yaw: float,
    *,
    timeout: float = 30.0,
    poll: float = 0.05,
    settle_time: float = 1,
    action_client=None,
    **safe_move_kwargs,
) -> bool:
    """
    Send a move goal and wait until the robot finishes moving.

    - Calls `safe_move_to(node, ...)` which sends the goal and waits for MoveGroup result.
    - Then polls robot motion (via node.is_moving() if present, else checks node.joint_velocities)
      until motion is below VELOCITY_THRESHOLD for `settle_time`, or until `timeout`.

    Returns True on success, False on failure or timeout.
    """
    # send the goal (safe_move_to already waits for planning/execution result)
    success = safe_move_to(node, x, y, z, roll, pitch, yaw, action_client=action_client, **safe_move_kwargs)
    if not success:
        return False

    # fallback: poll joint velocities (node.joint_velocities) or node.is_moving()
    start = time.time()
    stable_since = None
    last_vel = None
    while time.time() - start < timeout:
        rclpy.spin_once(node, timeout_sec=poll)
        # prefer node.is_moving()
        moving = False
        if hasattr(node, "is_moving"):
            try:
                moving = node.is_moving()
            except Exception:
                moving = False
        else:
            velocities = getattr(node, "joint_velocities", None)
            moving = is_moving_from_velocities(velocities, VELOCITY_THRESHOLD)

        if not moving:
            if stable_since is None:
                stable_since = time.time()
            if time.time() - stable_since >= settle_time:
                return True
        else:
            stable_since = None
        last_vel = getattr(node, "joint_velocities", None)
    node.get_logger().warning(f"Timeout waiting for robot to settle (last_vel={last_vel})")
    return False

# name -> int (only include integer constants)
MOVEIT_ERROR_CODES = {}
for name in dir(MoveItErrorCodes):
    if not name.isupper():
        continue
    try:
        val = getattr(MoveItErrorCodes, name)
    except Exception:
        continue
    if isinstance(val, int):
        MOVEIT_ERROR_CODES[name] = val

# int -> name
MOVEIT_ERROR_CODES_BY_VALUE = {v: k for k, v in MOVEIT_ERROR_CODES.items()}

def moveit_error_name(code: int) -> str:
    try:
        return MOVEIT_ERROR_CODES_BY_VALUE.get(int(code), f"UNKNOWN({code})")
    except Exception:
        return f"UNKNOWN({code})"

# Build a mapping of GoalStatus numeric -> name for clearer logs
GOAL_STATUS_NAMES = {}
for a in dir(GoalStatus):
    if a.startswith("STATUS_"):
        try:
            v = getattr(GoalStatus, a)
        except Exception:
            continue
        if isinstance(v, int):
            GOAL_STATUS_NAMES[v] = a

def goal_status_name(code: int) -> str:
    return GOAL_STATUS_NAMES.get(int(code), f"STATUS_{code}")
