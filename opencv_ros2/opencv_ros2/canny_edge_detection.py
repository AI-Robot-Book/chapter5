import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge
import numpy as np


class CannyEdgeDetection(Node):

    def __init__(self):
        super().__init__('canny_edge_detection')
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.listener_callback,
            qos_profile_sensor_data)
        
        self.publisher = self.create_publisher(Image, 'edges_result', 10)

        # ROSとOpenCVの画像間の変換に利用
        self.br = CvBridge()

    def listener_callback(self, data):
        
        # コンソールにメッセージを表示する
        #self.get_logger().info('Receiving image')
        
        # ROS画像メッセージをOpenCV画像に変換する
        frame = self.br.imgmsg_to_cv2(data, "bgr8")
        frame_grayscale = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Cannyエッジ検出を適用する
        edges = cv2.Canny(frame_grayscale,100,200)

        # ROSで結果をパブリッシュする
        edges_result = self.br.cv2_to_imgmsg(edges, "passthrough")
        self.publisher.publish(edges_result)

        # 画像を表示する
        cv2.imshow("Original Image", frame)
        cv2.imshow("Canny Edge Detection", edges)

        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    canny_edge_detection = CannyEdgeDetection()

    rclpy.spin(canny_edge_detection)
    
    canny_edge_detection.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
