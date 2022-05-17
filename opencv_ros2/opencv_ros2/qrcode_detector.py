import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

import cv2
from cv_bridge import CvBridge
import numpy as np


class QRCodeDetector(Node):

    def __init__(self):
        super().__init__('qrcode_detector')
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            qos_profile_sensor_data)

        self.publisher_result = self.create_publisher(
            Image, 
            'qrcode_detector_result', 10)
        
        self.publisher_data = self.create_publisher(
            String, 
            'qrcode_detector_data', 10)
        
        self.br = CvBridge()


    def image_callback(self, data):
        frame = self.br.imgmsg_to_cv2(data, "bgr8")
        
        detector = cv2.QRCodeDetector()

        qrdata, bbox, rectImg = detector.detectAndDecode(frame)

        if qrdata:
            print("QR Code detected, data:", qrdata)
            qrdata_string = String()
            qrdata_string.data = qrdata
            self.publisher_data.publish(qrdata_string)

            qrcode_rectimg = self.br.cv2_to_imgmsg(rectImg, "8UC1")
            self.publisher_result.publish(qrcode_rectimg)
            
            cv2.imshow("QRCode", rectImg)
            
            cv2.waitKey(1)


def main():
    rclpy.init()
    qrcode_detector = QRCodeDetector()
    try:
        rclpy.spin(qrcode_detector)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
