import sys
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rclpy.utilities import remove_ros_args

from yolov5_ros2.detector import Detector, parse_opt


class ObjectDetection(Node):

    def __init__(self, **args):
        super().__init__('object_detection')

        self.detector = Detector(**args)

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            'image_raw',
            self.image_callback,
            qos_profile_sensor_data)

    def image_callback(self, msg):
        try:
            img0 = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().warn(str(e))
            return

        img0, result = self.detector.detect(img0)

        cv2.imshow('result', img0)
        cv2.waitKey(1)
        for i, r in enumerate(result):
            self.get_logger().info(
                f'{i}: ({r.u1}, {r.v1}) ({r.u2}, {r.v2})' +
                f' {r.name}, {r.conf:.3f}')


def main():
    rclpy.init()
    opt = parse_opt(remove_ros_args(args=sys.argv))
    node = ObjectDetection(**vars(opt))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
