#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
from igus_student_msgs.msg import Puck2DArray, Puck3D, Puck3DArray
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration


class Puck2DTo3D(Node):
    def __init__(self):
        super().__init__('puck_2d_to_3d')

        # Topics (override via params if you want later)
        self.sub_2d = self.create_subscription(Puck2DArray, '/puck_2d_coords', self.cb_2d, 10)
        self.sub_depth = self.create_subscription(Image, '/rebel/camera/aligned_depth_to_color/image_raw', self.cb_depth, 10)
        self.sub_info = self.create_subscription(CameraInfo, '/rebel/camera/color/camera_info', self.cb_info, 10)

        self.pub_3d = self.create_publisher(Puck3DArray, '/puck_3d_points', 10)

        self.pub_markers = self.create_publisher(MarkerArray, '/puck_3d_markers', 10)

        self.bridge = CvBridge()

        self.depth_img = None
        self.depth_encoding = None
        self.depth_stamp = None

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.frame_id = None

        self.get_logger().info('2d_to_3d node started')

    def cb_info(self, msg: CameraInfo):
        # K = [fx 0 cx; 0 fy cy; 0 0 1]
        self.fx = float(msg.k[0])
        self.fy = float(msg.k[4])
        self.cx = float(msg.k[2])
        self.cy = float(msg.k[5])
        self.frame_id = msg.header.frame_id

    def cb_depth(self, msg: Image):
        # Convert once per frame
        self.depth_encoding = msg.encoding
        self.depth_stamp = msg.header.stamp

        # Depth is commonly 16UC1 (mm) or 32FC1 (m)
        if msg.encoding == '16UC1':
            self.depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        elif msg.encoding == '32FC1':
            self.depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        else:
            self.get_logger().warn(f'Unsupported depth encoding: {msg.encoding}')
            self.depth_img = None

    def cb_2d(self, msg: Puck2DArray):
        if self.depth_img is None:
            return
        if self.fx is None or self.fy is None or self.cx is None or self.cy is None:
            return

        h, w = self.depth_img.shape[:2]

        out = Puck3DArray()
        # publish in camera optical frame (use camera_info frame_id if available)
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.frame_id if self.frame_id else msg.header.frame_id

        for puck in msg.pucks:
            u = int(round(puck.u))
            v = int(round(puck.v))

            if u < 0 or v < 0 or u >= w or v >= h:
                continue

            z = self._depth_at(u, v)
            if z is None:
                continue

            # Pinhole projection (camera optical frame)
            x = (float(u) - self.cx) * z / self.fx
            y = (float(v) - self.cy) * z / self.fy

            p3 = Puck3D()
            p3.label = puck.label
            p3.point = Point(x=float(x), y=float(y), z=float(z))
            out.pucks.append(p3)

        self.pub_3d.publish(out)

        markers = MarkerArray()

        now = msg.header.stamp

        frame = out.header.frame_id

        # delete old markers (ids 0..N from previous frame)
        delete_all = Marker()
        delete_all.header.stamp = now
        delete_all.header.frame_id = frame
        delete_all.ns = "pucks"
        delete_all.id = 0
        delete_all.action = Marker.DELETEALL
        markers.markers.append(delete_all)

        for i, puck3 in enumerate(out.pucks):
            is_red = ("red" in puck3.label.lower())
            # sphere
            m = Marker()
            m.header.stamp = now
            m.header.frame_id = frame
            m.ns = "pucks_spheres"
            m.id = i
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position = puck3.point
            m.pose.orientation.w = 1.0

            # puck size guess: set diameter ~ 0.075m or whatever is correct
            m.scale.x = 0.025
            m.scale.y = 0.025
            m.scale.z = 0.015

            if is_red:
                m.color.r = 1.0; m.color.g = 0.1; m.color.b = 0.1; m.color.a = 0.9
            else:
                m.color.r = 0.1; m.color.g = 0.2; m.color.b = 1.0; m.color.a = 0.9

            m.lifetime = Duration(sec=0, nanosec=int(5e8))  # 0.2s
            markers.markers.append(m)

            # text label
            t = Marker()
            t.header.stamp = now
            t.header.frame_id = frame
            t.ns = "pucks_text"
            t.id = i
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position.x = puck3.point.x
            t.pose.position.y = puck3.point.y
            t.pose.position.z = puck3.point.z + 0.01
            t.pose.orientation.w = 1.0
            t.scale.z = 0.01
            t.color.r = 1.0; t.color.g = 1.0; t.color.b = 1.0; t.color.a = 1.0
            t.text = "R" if is_red else "B"
            t.lifetime = Duration(sec=0, nanosec=int(2e8))
            markers.markers.append(t)

            self.pub_markers.publish(markers)

    def _depth_at(self, u: int, v: int):
        # Use a small median window for robustness
        win = 2  # radius -> (2*win+1)^2
        vals = []
        h, w = self.depth_img.shape[:2]
        for dv in range(-win, win + 1):
            for du in range(-win, win + 1):
                uu = u + du
                vv = v + dv
                if uu < 0 or vv < 0 or uu >= w or vv >= h:
                    continue
                d = self.depth_img[vv, uu]
                z = self._to_meters(d)
                if z is None:
                    continue
                vals.append(z)

        if not vals:
            return None

        vals.sort()
        return vals[len(vals) // 2]

    def _to_meters(self, d):
        # Handle 16UC1 (mm) and 32FC1 (m)
        if self.depth_encoding == '16UC1':
            # d is uint16 millimeters, 0 means invalid
            if int(d) == 0:
                return None
            return float(d) / 1000.0
        if self.depth_encoding == '32FC1':
            z = float(d)
            if not math.isfinite(z) or z <= 0.0:
                return None
            return z
        return None


def main():
    rclpy.init()
    node = Puck2DTo3D()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
