from typing import List, Optional
from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


def build_planning_scene(collision_objects: List[dict], reference_frame: str) -> PlanningScene:
    """
    Build and return a moveit_msgs/PlanningScene message containing the
    provided collision_objects.

    collision_objects: list of dicts with keys:
        - name (str)
        - shape (box|sphere|cylinder)
        - size (list of floats)  # box: [x,y,z], sphere: [radius], cylinder: [height, radius]
        - position (x,y,z)
        - orientation (x,y,z,w)
    reference_frame: frame id for the collision objects (string)
    """
    scene = PlanningScene()
    scene.is_diff = True
    scene.world.collision_objects = []

    for obj in collision_objects:
        collision_obj = CollisionObject()
        collision_obj.header.frame_id = reference_frame
        collision_obj.id = obj.get("name", "object")

        primitive = SolidPrimitive()
        shape = obj.get("shape", "box").lower()
        if shape == "box":
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = [float(d) for d in obj.get("size", [0.0, 0.0, 0.0])]
        elif shape == "sphere":
            primitive.type = SolidPrimitive.SPHERE
            primitive.dimensions = [float(d) for d in obj.get("size", [0.0])]
        elif shape == "cylinder":
            primitive.type = SolidPrimitive.CYLINDER
            primitive.dimensions = [float(d) for d in obj.get("size", [0.0, 0.0])]
        else:
            # unknown shape — skip this object
            continue

        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = reference_frame
        px, py, pz = obj.get("position", (0.0, 0.0, 0.0))
        ox, oy, oz, ow = obj.get("orientation", (0.0, 0.0, 0.0, 1.0))
        pose_stamped.pose.position.x = float(px)
        pose_stamped.pose.position.y = float(py)
        pose_stamped.pose.position.z = float(pz)
        pose_stamped.pose.orientation.x = float(ox)
        pose_stamped.pose.orientation.y = float(oy)
        pose_stamped.pose.orientation.z = float(oz)
        pose_stamped.pose.orientation.w = float(ow)

        collision_obj.primitives = [primitive]
        collision_obj.primitive_poses = [pose_stamped.pose]
        collision_obj.operation = CollisionObject.ADD

        scene.world.collision_objects.append(collision_obj)

    return scene