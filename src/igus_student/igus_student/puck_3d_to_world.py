import rclpy
from rclpy.node import Node
from rclpy.time import Time

from igus_student_msgs.msg import Puck3DArray, Puck3D
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point

class PuckTransformer(Node):

    def __init__(self):
        super().__init__('PuckTransformer')

        self.target_frame = 'base_link'

        self.subscription = self.create_subscription(
            Puck3DArray,
            '/puck_3d_points',
            self.listener_callback,
            10
        )

        self.puck_3d_points_world = self.create_publisher(
            Puck3DArray,
            '/puck_3d_points_world',
            10
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info('PuckTransformer started..')

    def listener_callback(self, camera_frame_points: Puck3DArray):
        if not camera_frame_points.header.frame_id:
            self.get_logger().warn("Incoming message has empty header.frame_id, cannot transform.")
            return

        camera_frame_name = camera_frame_points.header.frame_id

        try:
            transform_world_from_camera = self.tf_buffer.lookup_transform(
                self.target_frame,
                camera_frame_name,
                Time()
            )
        except Exception as error:
            self.get_logger().warn(
                f"No TF available: {self.target_frame} <- {camera_frame_name}: {error}"
            )
            return

        world_frame_points = Puck3DArray()
        world_frame_points.header.stamp = camera_frame_points.header.stamp
        world_frame_points.header.frame_id = self.target_frame

        for puck in camera_frame_points.pucks:
            puck_point_in_camera = PointStamped()
            puck_point_in_camera.header.stamp = camera_frame_points.header.stamp
            puck_point_in_camera.header.frame_id = camera_frame_name
            puck_point_in_camera.point = puck.point

            try:
                puck_point_in_world = do_transform_point(puck_point_in_camera, transform_world_from_camera)
            except Exception as error:
                self.get_logger().warn(f"Point transform failed for '{puck.label}': {error}")
                continue

            transformed_puck = Puck3D()
            transformed_puck.label = puck.label
            transformed_puck.point = puck_point_in_world.point
            world_frame_points.pucks.append(transformed_puck)

        self.puck_3d_points_world.publish(world_frame_points)


def main(args=None):
    rclpy.init(args=args)
    node = PuckTransformer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
