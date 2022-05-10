from setuptools import setup

package_name = 'opencv_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jupiter',
    maintainer_email='i@jeffreytan.org',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
          'face_detection = opencv_ros2.face_detection:main',
          'quiz5_2_smile_detection = opencv_ros2.quiz5_2_smile_detection:main',
          'challenge5_1_say_hello = opencv_ros2.challenge5_1_say_hello:main',
          'challenge5_2_cup_direction = opencv_ros2.challenge5_2_cup_direction:main',
          'qrcode_detector = opencv_ros2.qrcode_detector:main',
          'video_subscriber = opencv_ros2.video_subscriber:main',
          'depth_subscriber = opencv_ros2.depth_subscriber:main',
          'canny_edge_detection = opencv_ros2.canny_edge_detection:main',
          'aruco_node_tf = opencv_ros2.aruco_node_tf:main',
        ],
    },
)
