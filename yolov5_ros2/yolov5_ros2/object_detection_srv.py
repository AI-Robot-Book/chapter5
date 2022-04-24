import sys
import time
import threading
import queue
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
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from yolov5_ros2.detector import Detector, parse_opt
from airobot_interfaces.srv import StringCommand


class ObjectDetection(Node):

    def __init__(self, **args):
        super().__init__('object_detection')

        self.running = False  # 物体認識の処理のフラグ
        self.target_name = 'cup'  # 探す物体名
        self.counter = 0  # 物体の検出回数を数えるカウンタ
        self.frame_id = 'target'  # ブロードキャストするtfの名前
        self.detector = Detector(**args)  # yolov5を使って物体検出を行うクラス
        self.bridge = CvBridge()  # ROSのメッセージとOpenCVのデータ変換
        self.q_color = queue.Queue()  # メインスレッドへカラー画像を送るキュー
        self.q_depth = queue.Queue()  # メインスレッドへ深度画像を送るキュー
        self.lock = threading.Lock()  # 2つのコールバックメソッド間の排他制御用

        # message_filtersを使って3個のトピックのサブスクライブをまとめて処理する．
        self.callback_group = ReentrantCallbackGroup()   # コールバックの並行処理のため
        self.sub_info = Subscriber(
            self, CameraInfo, 'camera/aligned_depth_to_color/camera_info',
            callback_group=self.callback_group)
        self.sub_color = Subscriber(
            self, Image, 'camera/color/image_raw',
            callback_group=self.callback_group)
        self.sub_depth = Subscriber(
            self, Image, 'camera/aligned_depth_to_color/image_raw',
            callback_group=self.callback_group)
        self.ts = ApproximateTimeSynchronizer(
            [self.sub_info, self.sub_color, self.sub_depth], 10, 0.1)
        self.ts.registerCallback(self.images_callback)

        # 認識した物体の位置をtfとして出力するためのブロードキャスタ
        self.broadcaster = TransformBroadcaster(self)

        # 他のノードからの司令を受け付けるサービスサーバ
        self.service = self.create_service(
            StringCommand, 'vision/command', self.command_callback,
            callback_group=self.callback_group)

    # StringCommandのリクエストが届いたときに呼び出されるメソッド
    def command_callback(self, request, response):
        self.get_logger().info(f'command: {request.command}')
        if request.command.startswith('find'):
            name = request.command[4:].strip()
            if len(name) == 0:
                response.answer = 'NG name required'
            elif name not in self.detector.names:
                response.answer = 'NG unknown name'
            else:
                with self.lock:  # 物体認識を開始させる
                    self.target_name = name
                    self.running = True
                    self.counter = 0
                time.sleep(3)   # しばらく停止
                with self.lock:  # 物体認識の結果を得る
                    self.running = False
                    counter = self.counter
                if counter >= 10:  # 十分な回数で認識されているか？
                    response.answer = 'OK'
                else:
                    response.answer = 'NG not found'
        elif request.command.startswith('track'):
            name = request.command[5:].strip()
            if len(name) == 0:
                response.answer = 'NG name required'
            elif name not in self.detector.names:
                response.answer = 'NG unknown name'
            else:
                with self.lock:  # 物体認識を開始させる
                    self.target_name = name
                    self.running = True
                response.answer = 'OK'
        elif request.command.startswith('stop'):
            with self.lock:
                self.running = False
            response.answer = 'OK'
        else:
            response.answer = f'NG {request.command} not supported'
        self.get_logger().info(f'answer: {response.answer}')
        return response

    # カメラ情報・カラー画像・深度画像が届いたときに呼び出されるメソッド
    def images_callback(self, msg_info, msg_color, msg_depth):
        # サブスクライブした画像をOpenCVのデータ形式に変換
        try:
            img_color = CvBridge().imgmsg_to_cv2(msg_color, 'bgr8')
            img_depth = CvBridge().imgmsg_to_cv2(msg_depth, 'passthrough')
        except CvBridgeError as e:
            self.get_logger().warn(str(e))
            return

        if img_color.shape[0:2] != img_depth.shape[0:2]:
            self.get_logger().warn('カラーと深度の画像サイズが異なる')
            return

        # サービスコールバックメソッドとの共有
        with self.lock:
            target_name = self.target_name
            running = self.running

        # 物体認識
        result = []
        if running:
            img_color, result = self.detector.detect(img_color)

        self.q_color.put(img_color)  # メインスレッドへカラー画像を送る．

        # 物体に認識の結果に指定された名前があるか調べる．
        target = None
        for r in result:
            if r.name == target_name:
                target = r
                break

        # カラー画像内で検出された場合は，深度画像から3次元位置を算出．
        depth = 0
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
                # tfの創出
                ts = TransformStamped()
                ts.header = msg_depth.header
                ts.child_frame_id = self.frame_id
                ts.transform.translation.x = x
                ts.transform.translation.y = y
                ts.transform.translation.z = z
                self.broadcaster.sendTransform(ts)
                # サービスコールバックメソッドとの共有
                with self.lock:
                    self.counter += 1

        # 深度画像の加工
        img_depth *= 16
        if depth != 0:  # 認識していて，かつ，距離が得られた場合
            pt1 = (int(target.u1), int(target.v1))
            pt2 = (int(target.u2), int(target.v2))
            cv2.rectangle(img_depth, pt1=pt1, pt2=pt2, color=0xffff)
        self.q_depth.put(img_depth)  # メインスレッドへ深度画像を送る．


def main():
    rclpy.init()
    opt = parse_opt(remove_ros_args(args=sys.argv))
    node = ObjectDetection(**vars(opt))

    # 別のスレッドでrclpy.spin()を実行する
    executor = MultiThreadedExecutor()
    thread = threading.Thread(
        target=rclpy.spin, args=(node, executor), daemon=True)
    thread.start()

    try:
        # imshow()は仕様上メインスレッド使う必要があるのでここで実行．
        while True:
            if not node.q_color.empty():
                cv2.imshow('color', node.q_color.get())
            if not node.q_depth.empty():
                cv2.imshow('depth', node.q_depth.get())
            cv2.waitKey(1)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
