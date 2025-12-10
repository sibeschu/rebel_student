#!/usr/bin/env python3
import time
import rclpy
import numpy as np
from math import pi

from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from tf_transformations import euler_from_quaternion, quaternion_from_euler
from sensor_msgs.msg import JointState
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import PlanningScene

from igus_student import imports as igus

class StudentRobotController(Node):
    def __init__(self):
        super().__init__("student_robot_controller")
        self.callback_group = ReentrantCallbackGroup()
        
        # clients
        self.move_client = ActionClient(
            self, MoveGroup, "/move_action", 
            callback_group=self.callback_group
        )

        self.gripper_client = igus.DigitalOutputClient()
        
        # pubs
        self.scene_publisher = self.create_publisher(
            PlanningScene, "/planning_scene", 10
        )
        
        # subs
        self.joint_velocities = None
        self.create_subscription(
            JointState, '/joint_states',
            self._joint_state_callback, 10
        )
        
        # wait MoveGroup server
        self.get_logger().info("Connecting to MoveGroup...")
        if not self.move_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("MoveGroup server not available!")
        self.get_logger().info("Connected to MoveGroup")
        
        # build planning scene
        scene = igus.build_planning_scene(igus.COLLISION_OBJECTS, igus.REFERENCE_FRAME)
        self.scene_publisher.publish(scene)
        time.sleep(0.5)
        self.get_logger().info(f"Loaded {len(igus.COLLISION_OBJECTS)} collision objects")
    
    def _joint_state_callback(self, msg):
        """Update joint velocities for motion detection"""
        if msg.velocity and len(msg.velocity) > 0:
            self.joint_velocities = np.array(msg.velocity)
    
    def is_moving(self):
        return igus.is_moving_from_velocities(self.joint_velocities, igus.VELOCITY_THRESHOLD)

    def move_to_pose(self, x, y, z, roll, pitch, yaw):
        return igus.safe_move_to(self, x, y, z, roll, pitch, yaw)

    def move_and_wait(self, x, y, z, roll, pitch, yaw):
        return igus.safe_move_and_wait(
            self, x, y, z, roll, pitch, yaw, action_client=self.move_client
        )
    
    def close_gripper(self):
        print("Closing gripper...")
        self.gripper_client.set_output(30, True)

    def open_gripper(self):
        print("Opening gripper...")
        self.gripper_client.set_output(30, False)

_robot = None

def main():
    global _robot
    
    print("STUDENT CONTROL STARTED")
  
    rclpy.init()
    
    try:
        _robot = StudentRobotController()

        print("Robot controller initialized")
        print("Example program started")

        """ !!! Z KOORDINATE CA +0.12 FÜR STIFTHALTER !!! """
        # Roboter zu Position (0.4, 0.0, 0.15) mit Orientierung (~π, 0, 0) bewegen

        # Hier dein Code

        print("PROGRAM COMPLETED")
        
    except KeyboardInterrupt:
        print("\nStopped by user (Ctrl+C)")
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if _robot is not None:
            _robot.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()