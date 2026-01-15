#!/usr/bin/env python3

import rclpy
import cv2
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data
from igus_student_msgs.msg import Puck2D, Puck2DArray

def make_detector(min_area=500.0, max_area=30000.0):
    params = cv2.SimpleBlobDetector_Params()

    # binary mask settings
    params.minThreshold = 254
    params.maxThreshold = 255
    params.thresholdStep = 255
    params.minRepeatability = 1

    params.filterByColor = True
    params.blobColor = 255

    params.filterByArea = True
    params.minArea = float(min_area)
    params.maxArea = float(max_area)

    params.minDistBetweenBlobs = 2.0

    params.filterByCircularity = True
    params.minCircularity = 0.75

    params.filterByConvexity = False
    params.filterByInertia = False

    return cv2.SimpleBlobDetector_create(params)

class ColorBlobs(Node):
    def __init__(self):
        super().__init__('color_blobs')
        self.bridge = CvBridge()
        self.detector = make_detector(min_area=120.0, max_area=30000.0)
        self.kernel = np.ones((2, 2), np.uint8)

        self.sub = self.create_subscription(
            Image, '/rebel/camera/color/image_raw', self.callback, qos_profile_sensor_data
        )

        self.debug_image_pub = self.create_publisher(Image, '/puck_debug_image', 10)

        self.puck2d_pub = self.create_publisher(Puck2DArray, '/puck_2d_coords', 10)


    def detect_blobs(self, mask, label):
        keypoints = self.detector.detect(mask)
        return [{"x": float(kp.pt[0]), "y": float(kp.pt[1]), "label": label} for kp in keypoints]

    def callback(self, msg):
        bgr_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)

        # color ranges for the masks
        red_mask1 = cv2.inRange(hsv_image, (0, 120, 40), (8, 255, 255))
        red_mask2 = cv2.inRange(hsv_image, (160, 100, 100), (180, 255, 255))
        blue_mask = cv2.inRange(hsv_image, (104, 180, 40), (114, 255, 220))
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)

        # fill holes
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2, 2))
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)
        blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_CLOSE, self.kernel)

        red_blobs = self.detect_blobs(red_mask, "red puck")
        blue_blobs = self.detect_blobs(blue_mask, "blue puck")

        # uncomment for masks debug
        # self.show_debug("red", red_mask, red_blobs, (0, 0, 255))
        # self.show_debug("blue", blue_mask, blue_blobs, (255, 0, 0))
        # cv2.waitKey(1)

        blobs = red_blobs + blue_blobs

        self.publish_debug_image(bgr_image, blobs)

        # build Puck2DArray message and publish
        msg = Puck2DArray()
        msg.header = msg.header  # stamp + frame_id from the incoming color image

        for b in blobs:
            p = Puck2D()
            p.label = b["label"]
            p.u = float(b["x"])
            p.v = float(b["y"])
            msg.pucks.append(p)

        self.puck2d_pub.publish(msg)

        # uncomment for debug
        # if blobs:
        #     self.get_logger().info(str(blobs), throttle_duration_sec=0.5)


    def show_debug(self, window_name, mask, blobs, color_bgr):
        debug_image = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        for b in blobs:
            x, y = int(b["x"]), int(b["y"])
            cv2.drawMarker(debug_image, (x, y), color_bgr, markerType=cv2.MARKER_CROSS, markerSize=12, thickness=2)
        cv2.imshow(window_name, debug_image)
            
    def publish_debug_image(self, bgr_image, blobs):
        debug_image = bgr_image.copy()
        for b in blobs:
            x, y = int(b["x"]), int(b["y"])
            color = (0, 0, 255) if b["label"] == "red puck" else (255, 0, 0)
            cv2.drawMarker(debug_image, (x, y), color, markerType=cv2.MARKER_CROSS, markerSize=20, thickness=2)
            cv2.putText(debug_image, b["label"], (x + 6, y - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)

        msg = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
        self.debug_image_pub.publish(msg)


    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()

def main():
    rclpy.init()
    node = ColorBlobs()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
