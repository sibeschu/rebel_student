from typing import Optional, Tuple
import time
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener
from tf_transformations import euler_from_quaternion

from .constants import REFERENCE_FRAME, END_EFFECTOR_LINK
from .goal_builder import safe_move_and_wait

def _ensure_tf(node: Node) -> Buffer:
    """Create and attach a tf Buffer+Listener on the node if not present."""
    if not hasattr(node, "_tf_buffer"):
        node._tf_buffer = Buffer()
        node._tf_listener = TransformListener(node._tf_buffer, node)
    return node._tf_buffer


def get_current_ee_pose(
    node: Node,
    reference_frame: str,
    end_effector_frame: str,
    timeout_sec: float = 1.0,
) -> Optional[Tuple[float, float, float, float, float, float]]:
    """
    Lookup EE pose and return (x,y,z, roll,pitch,yaw) in `reference_frame`.
    Will spin the node briefly and retry up to `timeout_sec` before failing.
    """
    buf = _ensure_tf(node)
    tf_time = rclpy.time.Time()
    deadline = time.time() + float(timeout_sec)
    poll = 0.05

    while time.time() < deadline:
        # Allow the node to process incoming TF messages so Buffer gets populated
        rclpy.spin_once(node, timeout_sec=poll)

        try:
            # quick can_transform check with tiny internal timeout
            if not buf.can_transform(reference_frame, end_effector_frame, tf_time, timeout=Duration(seconds=0.01)):
                # not yet available, retry
                continue

            trans = buf.lookup_transform(reference_frame, end_effector_frame, tf_time)
            t = trans.transform.translation
            q = trans.transform.rotation
            roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            return (float(t.x), float(t.y), float(t.z), float(roll), float(pitch), float(yaw))
        except Exception:
            # small sleep handled by spin_once loop; continue retrying until deadline
            continue

    node.get_logger().warning(
        f"TF not available: could not transform {end_effector_frame} -> {reference_frame} (waited {timeout_sec}s)."
    )
    return None


def move_ee_vertical(
    node: Node,
    delta_z: float = 0.1,
    reference_frame: Optional[str] = None,
    end_effector_frame: Optional[str] = None,
    action_client=None,
    timeout_sec: float = 1.0,
) -> bool:
    """
    Move the end-effector straight up (positive z in `reference_frame`) by `delta_z`.
    - Reads current EE pose with `get_current_ee_pose`.
    - Keeps current orientation (roll,pitch,yaw).
    - Calls `safe_move_and_wait(node, x, y, z + delta_z, roll, pitch, yaw, action_client=...)`.

    Returns True on success, False on failure.
    """
    reference_frame = reference_frame or REFERENCE_FRAME
    end_effector_frame = end_effector_frame or END_EFFECTOR_LINK

    pos = get_current_ee_pose(node, reference_frame, end_effector_frame, timeout_sec=timeout_sec)
    if pos is None:
        node.get_logger().warning("move_ee_vertical: could not read current EE pose")
        return False

    x, y, z, roll, pitch, yaw = pos
    target_z = float(z) + float(delta_z)

    # choose action client
    client = action_client if action_client is not None else getattr(node, "move_client", None)
    if client is None:
        node.get_logger().error("move_ee_vertical: no ActionClient available (pass action_client= or set node.move_client)")
        return False

    node.get_logger().info(f"move_ee_vertical: moving EE from z={z:.3f} to z={target_z:.3f}")
    return safe_move_and_wait(node, float(x), float(y), target_z, float(roll), float(pitch), float(yaw), action_client=client)