import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge


class CannyEdgeDetection(Node):

    def __init__(self):
        super().__init__('canny_edge_detection')
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            qos_profile_sensor_data)

        self.publisher = self.create_publisher(
            Image,
            'edges_result', 10)

        self.br = CvBridge()

    def image_callback(self, data):
        frame = self.br.imgmsg_to_cv2(data, 'bgr8')
        frame_grayscale = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        edges = cv2.Canny(frame_grayscale, threshold1=100, threshold2=200)

        edges_result = self.br.cv2_to_imgmsg(edges, 'passthrough')
        self.publisher.publish(edges_result)

        cv2.imshow('Original Image', frame)
        cv2.imshow('Canny Edge Detection', edges)

        cv2.waitKey(1)


def main():
    rclpy.init()
    canny_edge_detection = CannyEdgeDetection()
    try:
        rclpy.spin(canny_edge_detection)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
