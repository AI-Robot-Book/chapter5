# 第5章 ビジョン

## 概要

ROS2とPythonで作って学ぶAIロボット入門（出村・萩原・升谷・タン著，講談社）

第5章のサンプルプログラムと補足情報などを掲載しています．

## ディレクトリ構成

- [opencv_ros2](opencv_ros2)： OpenCVとROS2のサンプルプログラム

- [yolov5_ros2](yolov5_ros2)： YOLOv5とROS2のサンプルプログラム

## 補足情報

- 実行するUbuntuの環境でカメラからの入力が出来ていることを事前に確認してください．

- カメラを起動の時，下記のエラーが発生の場合
  ```
  Cannot open '/dev/video0': 13, Permission denied
  ```
  対策：
  ```
  sudo chmod 666 /dev/video0
  ```
