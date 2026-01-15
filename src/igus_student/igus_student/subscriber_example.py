import rclpy
from rclpy.node import Node

from igus_student_msgs.msg import Puck3DArray


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')

        self.subscription = self.create_subscription(
            Puck3DArray,
            '/puck_3d_points',
            self.listener_callback,
            10
        )

        self.get_logger().info('minimal_subscriber started..')

    def listener_callback(self, msg: Puck3DArray):
        if not msg.pucks:
            self.get_logger().info('Received Puck3DArray: empty')
            return

        self.get_logger().info(
            f'Received {len(msg.pucks)} puck(s) in frame "{msg.header.frame_id}"'
        )

        for i, puck in enumerate(msg.pucks):
            p = puck.point
            self.get_logger().info(
                f'[{i}] {puck.label}: '
                f'x={p.x:.3f}, y={p.y:.3f}, z={p.z:.3f}'
            )


""" 
# Example for accessing data from the message
# msg is of type Puck3DArray
# ros2 interface show igus_student_msgs/msg/Puck3DArray

    # 1) access header info
    frame = msg.header.frame_id
    stamp = msg.header.stamp

    # 2) access the array of pucks
    for puck in msg.pucks:
        # label string ("red", "blue", ...)
        label = puck.label

        # geometry_msgs/Point
        x = puck.point.x
        y = puck.point.y
        z = puck.point.z

        # example debug usage
        self.get_logger().info(
            f'[{frame}] {label}: x={x:.3f}, y={y:.3f}, z={z:.3f}'
        )

# ATTENTION! 
# to execute this Node it has to be added in the setup.py
"""

def main(args=None):
    rclpy.init(args=args)
    node = MinimalSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
