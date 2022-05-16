import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image # Image is the message type

import cv2 # OpenCV library
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images


class DepthSubscriber(Node):

    def __init__(self):
        super().__init__('depth_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/camera/depth/image_rect_raw', # realsense depth topic
            self.listener_callback,
            qos_profile_sensor_data)
        
        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

    def listener_callback(self, data):
        
        # Display the message on the console
        self.get_logger().info('Receiving image')
        
        # Convert ROS Image message to OpenCV image
        frame = self.br.imgmsg_to_cv2(data, "passthrough")
        
        # Display image
        cv2.imshow("camera", frame)
        
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    depth_subscriber = DepthSubscriber()

    rclpy.spin(depth_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    depth_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()