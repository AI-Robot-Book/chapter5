import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge


class VideoSubscriber(Node):

    def __init__(self):
        super().__init__('video_subscriber')
        self.subscription = self.create_subscription(
            Image,
            'image_raw', # usb_cam topic
            self.image_callback,
            qos_profile_sensor_data)

        self.publisher = self.create_publisher(
            Image,
            'result', 10)
        
        self.br = CvBridge()

    def image_callback(self, data):
        frame = self.br.imgmsg_to_cv2(data, "bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, result = cv2.threshold(gray, 128, 255, cv2.THRESH_BINARY)
        result_msg = self.br.cv2_to_imgmsg(result, 'passthrough')
        self.publisher.publish(result_msg)


def main():
    rclpy.init()
    video_subscriber = VideoSubscriber()
    try:
        rclpy.spin(video_subscriber)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
