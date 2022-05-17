import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge

face_cascade = cv2.CascadeClassifier(
  cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
eye_cascade = cv2.CascadeClassifier(
  cv2.data.haarcascades + 'haarcascade_eye.xml')


class FaceDetection(Node):

  def __init__(self):
    super().__init__('face_detection')
    self.subscription = self.create_subscription(
      Image, 
      '/image_raw',
      self.image_callback, 
      qos_profile_sensor_data)

    self.publisher = self.create_publisher(
      Image, 
      'face_detection_result', 10)

    self.br = CvBridge()
   
  def image_callback(self, data):
    frame = self.br.imgmsg_to_cv2(data, "bgr8")
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    faces = face_cascade.detectMultiScale(gray, 1.3, 5)
    
    for (x,y,w,h) in faces:
        cv2.rectangle(frame,(x,y),(x+w,y+h),(255,0,0),2)
        roi_gray = gray[y:y+h, x:x+w]
        roi_color = frame[y:y+h, x:x+w]
        eyes = eye_cascade.detectMultiScale(roi_gray, 1.1, 9)
        for (ex,ey,ew,eh) in eyes:
            cv2.rectangle(roi_color,(ex,ey),(ex+ew,ey+eh),(0,255,0),2)
    
    face_detection_result = self.br.cv2_to_imgmsg(frame, "bgr8")
    self.publisher.publish(face_detection_result)

    cv2.imshow("Camera", frame)
    
    cv2.waitKey(1)


def main():
    rclpy.init()
    face_detection = FaceDetection()
    try:
        rclpy.spin(face_detection)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
