# OpenCVとROS2のサンプルプログラム

## 概要

- OpenCVとROS2によるロボットビジョンのアプリケーション
- Ubuntu 20.04, ROS Foxyで作成・確認

## インストール

- OpenCV関連のパッケージをインストール
  ```
  pip3 install opencv-contrib-python==4.5.5.64
  ```

- ROS2とOpenCVのインタフェースとなるパッケージをインストール
  ```
  sudo apt install ros-foxy-vision-opencv
  ```

- USBカメラ用ノードのパッケージをインストール
  ```
  sudo apt install ros-foxy-usb-cam
  ```

- Intel RealSense RGB-Dカメラ用のROSラッパをインストール
  ```
  sudo apt install ros-foxy-realsense2-camera
  ```

- ROSのワークスペースを`~/airobot_ws`とする．
  ```
  cd ~/airobot_ws/src
  ```

- JMU-ROBOTICS-VIVAのros2_arucoパッケージを入手
  ```
  git clone https://github.com/JMU-ROBOTICS-VIVA/ros2_aruco
  ```

- このパッケージを含むリポジトリを入手
  ```
  git clone https://github.com/AI-Robot-Book/chapter5
  ```

- サービスのインタフェースを定義しているパッケージを含むリポジトリを入手
  ```
  git clone https://github.com/AI-Robot-Book/chapter2
  ```

- パッケージをビルド
  ```
  cd ~/airobot_ws/
  colcon build
  ```

- オーバレイを設定
  ```
  source install/setup.bash
  ```

## 実行

5.3.1節：OpenCVによる画像処理
- プログラムを実行
  ```
  python3 ~/airobot_ws/src/chapter5/opencv_ros2/opencv_ros2/imgproc_opencv.py
  ```
- 結果を確認

5.3.2節：ROSにおけるOpenCVの画像処理
- 端末1：USBカメラのusb_camパッケージを起動
  ```
  ros2 run usb_cam usb_cam_node_exe
  ```
- 端末2：プログラムを実行
  ```
  ros2 run opencv_ros2 imgproc_opencv_ros
  ```
- 端末3：結果を確認
  - RQtを実行
    ```
    rqt
    ```
  - RQtのメニューのPlugins→Visualization→Image Viewを2回選び，Image Viewプラグインで2つ追加
  - プラグインのトピックを選択するメニューでそれぞれ/image_rawと/resultを選ぶ
- 端末4：ノードやトピックの間のつながりを調べる
  - rqt_graphを実行
    ```
    rqt_graph
    ```

5.3.3節：深度データのサブスクライブ
- 端末1：深度カメラIntel RealSense D415用のROSノードを起動
  ```
  ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true
  ```
- 端末2：トピックを確認
  ```
  ros2 topic list
  ```
- 端末3：カラー画像と深度画像を表示
  - RQtを実行
    ```
    rqt
    ```
  - Image Viewプラグインでカラー画像と深度画像のトピックを表示

5.5.1節：Cannyエッジ検出
- 端末1：USBカメラのusb_camパッケージを起動
  ```
  ros2 run usb_cam usb_cam_node_exe
  ```
- 端末2：プログラムを実行
  ```
  ros2 run opencv_ros2 canny_edge_detection
  ```
- 端末3：結果を確認
  - RQtを実行
    ```
    rqt
    ```
  - Image Viewプラグインで結果のトピック/edges_resultを表示

5.5.2節：Haar特徴量ベースのカスケード分類器による顔検出
- 端末1：USBカメラのusb_camパッケージを起動
  ```
  ros2 run usb_cam usb_cam_node_exe
  ```
- 端末2：プログラムを実行
  ```
  ros2 run opencv_ros2 face_detection
  ```
- 端末3：結果を確認
  - RQtを実行
    ```
    rqt
    ```
  - Image Viewプラグインで結果のトピック/face_detection_resultを表示

5.6.1節：QRコードの検出
- 端末1：USBカメラのusb_camパッケージを起動
  ```
  ros2 run usb_cam usb_cam_node_exe
  ```
- 端末2：プログラムを実行
  ```
  ros2 run opencv_ros2 qrcode_detector
  ```
- 端末3：結果の文字トピックを確認
  ```
  ros2 topic echo /qrcode_detector_data
  ```
- 端末4：結果の画像トピックを確認
  - RQtを実行
    ```
    rqt
    ```
  - Image Viewプラグインで結果の画像トピック/qrcode_detector_resultを表示

5.6.2節：ArUcoマーカによる位置・姿勢推定
- 端末1：サンプルマーカ画像を生成
  ```
  ros2 run ros2_aruco aruco_generate_marker
  ```
- 端末2：USBカメラのusb_camパッケージを起動
  ```
  ros2 run usb_cam usb_cam_node_exe
  ```
- 端末3：プログラムを実行
  ```
  ros2 run opencv_ros2 aruco_node_tf
  ```
- 端末4：結果の/tfトピックを確認
  ```
  ros2 topic echo /tf
  ```
- 端末5：マーカとカメラ座標系の可視化
  - RVizを実行
    ```
    rviz2
    ```
  - RVizのウィンドウに，Fixed Frameをdefault_camに変更し，AddをクリックしてTFを追加

## ヘルプ

- このサンプルプログラムは，Ubuntu上でしか動作が確認できていません．Windowsで開発されている方は，VirtualBox、VMwareなどのバーチャルマシンにUbuntuをインストールしてサンプルプログラムを実行する事ができます．

- Pythonのopencv-contrib-pythonパッケージのバージョンを指定せず最新版をインストールした場合は，aruco_node_tfノードの実行時に以下のようなエラーが発生することを確認しています．
  ```
  AttributeError: module 'cv2' has no attribute 'aruco'
  ```
  または
  ```
  AttributeError: module 'cv2.aruco' has no attribute 'drawAxis'
  ```
  これを回避するために，以下のコマンドを実行してください．
  ```
  pip3 uninstall opencv-python
  pip3 uninstall opencv-contrib-python
  pip3 install opencv-contrib-python==4.5.5.64
  ```

## 著者

タン ジェフリー トゥ チュアン　TAN Jeffrey Too Chuan

## 履歴

- 2022-08-27: ライセンス・ドキュメントの整備

## ライセンス

Copyright (c) 2022, TAN Jeffrey Too Chuan  
All rights reserved.  
This project is licensed under the Apache License 2.0 found in the LICENSE file in the root directory of this project.

## 参考文献

- https://opencv.org/
- https://github.com/JMU-ROBOTICS-VIVA/ros2_aruco
