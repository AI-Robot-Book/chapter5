# YOLOとROS2のサンプルプログラム

## 概要

- YOLOとROS2による深層学習の物体検出プログラム
- Ubuntu 20.04, ROS Foxyで作成・確認

## インストール

- [opencv_ros2](opencv_ros2)のインストール作業

- YOLOv5ソフトウェアをインストール
  ```
  cd ˜/airobot_ws/
  git clone https://github.com/ultralytics/yolov5
  cd yolov5
  pip3 install -r requirements.txt
  ```

## 実行

5.7.2節：YOLOの物体検出
- 端末1：USBカメラのusb_camパッケージを起動
  ```
  ros2 run usb_cam usb_cam_node_exe
  ```
- 端末2：プログラムを実行
  ```
  ros2 run yolov5_ros2 object_detection
  ```
- プログラムが正常に実行されると，新たなウィンドウが現れ，検出物体に色付きの枠が描きたされたカメラ画像が表示されます．

5.7.2節：検出物体の位置推定
- 端末1：深度カメラIntel RealSense D415用のROSノードを起動
  ```
  ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true
  ```
- 端末2：プログラムを実行
  ```
  ros2 run yolov5_ros2 object_detection_tf
  ```
- プログラムが正常に実行されると，新たなウィンドウが現れ，深度画像に対象物体のバウンディングボックスが表示
- /tfトピックにはカメラ座標系における物体の3次元位置が出力

5.7.2節：物体検出のサービスサーバ
- 端末1：深度カメラIntel RealSense D415用のROSノードを起動
  ```
  ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true
  ```
- 端末2：プログラムを実行
  ```
  ros2 run yolov5_ros2 object_detection_srv
  ```
- 端末3：ROSサービスを呼び出し（対象物体’cup’）
  - 対象物体を探す
    ```
    ros2 service call /vision/command airobot_interfaces/srv/StringCommand "{command: ’find cup’}"
    ```
  - 対象物体を連続的に追跡
    ```
    ros2 service call /vision/command airobot_interfaces/srv/StringCommand "{command: ’track cup’}"
    ```
  - 物体検出の処理を停止
    ```
    ros2 service call /vision/command airobot_interfaces/srv/StringCommand "{command: ’stop’}"
    ```

## ヘルプ

- 今のところありません．

## 著者

タン　ジェフリー　トゥ　チュアン TAN Jeffrey Too Chuan

## 履歴

- 2022-08-27: ライセンス・ドキュメントの整備

## ライセンス

Copyright (c) 2022, TAN Jeffrey Too Chuan
All rights reserved.  
This project is licensed under the Apache License 2.0 license found in the LICENSE file in the root directory of this project.

## 参考文献

- https://github.com/ultralytics/yolov5