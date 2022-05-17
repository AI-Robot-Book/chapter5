import rclpy
import rclpy.node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import numpy as np
import cv2
from ros2_aruco import transformations

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from scipy.spatial.transform import Rotation as R

from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose
from ros2_aruco_interfaces.msg import ArucoMarkers


class ArucoNodeTF(rclpy.node.Node):

    def __init__(self):
        super().__init__('aruco_node_tf')

        self.declare_parameter("marker_size", .0625)
        self.declare_parameter("aruco_dictionary_id", "DICT_5X5_250")
        self.declare_parameter("image_topic", "/image_raw")
        self.declare_parameter("camera_info_topic", "/camera_info")
        self.declare_parameter("camera_frame", None)
        self.declare_parameter("aruco_marker_name", "aruco_marker")
     
        self.marker_size = self.get_parameter(
            "marker_size").get_parameter_value().double_value
        dictionary_id_name = self.get_parameter(
            "aruco_dictionary_id").get_parameter_value().string_value
        image_topic = self.get_parameter(
            "image_topic").get_parameter_value().string_value
        info_topic = self.get_parameter(
            "camera_info_topic").get_parameter_value().string_value
        self.camera_frame = self.get_parameter(
            "camera_frame").get_parameter_value().string_value
        self.aruco_marker_name = self.get_parameter(
            "aruco_marker_name").get_parameter_value().string_value

        try:
            dictionary_id = cv2.aruco.__getattribute__(dictionary_id_name)
            if type(dictionary_id) != type(cv2.aruco.DICT_5X5_100):
                raise AttributeError
        except AttributeError:
            self.get_logger().error(
                "bad aruco_dictionary_id: {}".format(dictionary_id_name))
            options = "\n".join([s for s in dir(cv2.aruco) if s.startswith("DICT")])
            self.get_logger().error("valid options: {}".format(options))

        self.info_sub = self.create_subscription(CameraInfo,
                                                 info_topic,
                                                 self.info_callback,
                                                 qos_profile_sensor_data)

        self.create_subscription(Image, image_topic,
                                 self.image_callback, qos_profile_sensor_data)

        self.poses_pub = self.create_publisher(PoseArray, 'aruco_poses', 10)
        self.markers_pub = self.create_publisher(ArucoMarkers, 'aruco_markers', 10)

        self.tfbroadcaster = TransformBroadcaster(self)

        self.info_msg = None
        self.intrinsic_mat = None
        self.distortion = None

        self.aruco_dictionary = cv2.aruco.Dictionary_get(dictionary_id)
        self.aruco_parameters = cv2.aruco.DetectorParameters_create()
        self.bridge = CvBridge()

    def info_callback(self, info_msg):
        self.info_msg = info_msg
        self.intrinsic_mat = np.reshape(np.array(self.info_msg.k), (3, 3))
        self.distortion = np.array(self.info_msg.d)
        self.destroy_subscription(self.info_sub)

    def image_callback(self, img_msg):

        if self.info_msg is None:
            self.get_logger().warn("No camera info has been received!")
            return

        cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        markers = ArucoMarkers()
        pose_array = PoseArray()
        if self.camera_frame == '':
            markers.header.frame_id = self.info_msg.header.frame_id
            pose_array.header.frame_id = self.info_msg.header.frame_id
        else:
            markers.header.frame_id = self.camera_frame
            pose_array.header.frame_id = self.camera_frame
            
        markers.header.stamp = img_msg.header.stamp
        pose_array.header.stamp = img_msg.header.stamp

        corners, marker_ids, rejected = cv2.aruco.detectMarkers(
            cv_image,
            self.aruco_dictionary,
            parameters=self.aruco_parameters)

        if marker_ids is not None:
            
            cv2.aruco.drawDetectedMarkers(cv_image, corners, marker_ids)
 
            if cv2.__version__ > '4.0.0':
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners,
                    self.marker_size,
                    self.intrinsic_mat,
                    self.distortion)
            else:
                rvecs, tvecs = cv2.aruco.estimatePoseSingleMarkers(
                    corners,
                    self.marker_size,
                    self.intrinsic_mat,
                    self.distortion)
            for i, marker_id in enumerate(marker_ids):
                t = TransformStamped()
                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id = markers.header.frame_id
                t.child_frame_id = self.aruco_marker_name + str(marker_id[0])

                t.transform.translation.x = tvecs[i][0][0]
                t.transform.translation.y = tvecs[i][0][1]
                t.transform.translation.z = tvecs[i][0][2]
                 
                pose = Pose()
                pose.position.x = tvecs[i][0][0]
                pose.position.y = tvecs[i][0][1]
                pose.position.z = tvecs[i][0][2]

                rot_matrix = np.eye(4)
                rot_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[i][0]))[0]
                quat = transformations.quaternion_from_matrix(rot_matrix)

                t.transform.rotation.x = quat[0]
                t.transform.rotation.y = quat[1]
                t.transform.rotation.z = quat[2]
                t.transform.rotation.w = quat[3]
                
                self.tfbroadcaster.sendTransform(t)
                
                cv2.aruco.drawAxis(
                    cv_image, self.intrinsic_mat,
                    self.distortion, rvecs[i],
                    tvecs[i], 0.05)

                pose.orientation.x = quat[0]
                pose.orientation.y = quat[1]
                pose.orientation.z = quat[2]
                pose.orientation.w = quat[3]

                pose_array.poses.append(pose)
                markers.poses.append(pose)
                markers.marker_ids.append(marker_id[0])

            self.poses_pub.publish(pose_array)
            self.markers_pub.publish(markers)

        cv2.imshow("camera", cv_image)
        cv2.waitKey(1)


def main():
    rclpy.init()
    node = ArucoNodeTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
