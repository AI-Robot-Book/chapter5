import sys
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from cv_bridge import CvBridge, CvBridgeError
from rclpy.utilities import remove_ros_args
from message_filters import Subscriber, ApproximateTimeSynchronizer
from tf2_ros import TransformBroadcaster

from yolov5_ros2.detector import Detector, parse_opt


class ObjectDetection(Node):

    def __init__(self, **args):
        super().__init__('object_detection')

        self.target_name = 'cup'
        self.frame_id = 'target'

        self.detector = Detector(**args)

        self.bridge = CvBridge()

        self.sub_info = Subscriber(
            self, CameraInfo, 'camera/aligned_depth_to_color/camera_info')
        self.sub_color = Subscriber(
            self, Image, 'camera/color/image_raw')
        self.sub_depth = Subscriber(
            self, Image, 'camera/aligned_depth_to_color/image_raw')
        self.ts = ApproximateTimeSynchronizer(
            [self.sub_info, self.sub_color, self.sub_depth], 10, 0.1)
        self.ts.registerCallback(self.images_callback)
        self.broadcaster = TransformBroadcaster(self)

    def images_callback(self, msg_info, msg_color, msg_depth):
        try:
            img_color = CvBridge().imgmsg_to_cv2(msg_color, 'bgr8')
            img_depth = CvBridge().imgmsg_to_cv2(msg_depth, 'passthrough')
        except CvBridgeError as e:
            self.get_logger().warn(str(e))
            return

        if img_color.shape[0:2] != img_depth.shape[0:2]:
            self.get_logger().warn('カラーと深度の画像サイズが異なる')
            return

        img_color, result = self.detector.detect(img_color)

        cv2.imshow('color', img_color)

        target = None
        for r in result:
            if r.name == self.target_name:
                target = r
                break

        if target is not None:
            u1 = round(target.u1)
            u2 = round(target.u2)
            v1 = round(target.v1)
            v2 = round(target.v2)
            u = round((target.u1 + target.u2) / 2)
            v = round((target.v1 + target.v2) / 2)
            depth = np.median(img_depth[v1:v2+1, u1:u2+1])
            if depth != 0:
                z = depth * 1e-3
                fx = msg_info.k[0]
                fy = msg_info.k[4]
                cx = msg_info.k[2]
                cy = msg_info.k[5]
                x = z / fx * (u - cx)
                y = z / fy * (v - cy)
                self.get_logger().info(
                    f'{target.name} ({x:.3f}, {y:.3f}, {z:.3f})')
                ts = TransformStamped()
                ts.header = msg_depth.header
                ts.child_frame_id = self.frame_id
                ts.transform.translation.x = x
                ts.transform.translation.y = y
                ts.transform.translation.z = z
                self.broadcaster.sendTransform(ts)

        img_depth *= 16
        if target is not None:
            pt1 = (int(target.u1), int(target.v1))
            pt2 = (int(target.u2), int(target.v2))
            cv2.rectangle(img_depth, pt1=pt1, pt2=pt2, color=0xffff)

        cv2.imshow('depth', img_depth)
        cv2.waitKey(1)


def main():
    rclpy.init()
    opt = parse_opt(remove_ros_args(args=sys.argv))
    node = ObjectDetection(**vars(opt))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
