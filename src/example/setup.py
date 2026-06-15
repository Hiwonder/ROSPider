import os
from glob import glob
from setuptools import setup

package_name = 'example'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('example', '**/*.launch.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'resource'), glob(os.path.join('resource', '*.dae'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='1270161395@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'color_detect = example.opencv_example.include.color_detect_node:main',
            'color_recognition = example.opencv_example.include.color_recognition_node:main',
            'apriltag_recognition = example.opencv_example.include.apriltag_recognition:main',
            'ar = example.opencv_example.include.ar:main',
            'color_position = example.opencv_example.include.color_position:main',
            'apriltag_position = example.opencv_example.include.apriltag_position:main',
            'kcf = example.opencv_example.include.kcf:main',
            'apriltag_track = example.opencv_example.include.apriltag_track:main',

            'face_track = example.mediapipe_example.include.face_track:main',
            'finger_trajectory = example.mediapipe_example.include.finger_trajectory:main',
            'hand_detect = example.mediapipe_example.include.hand_detect:main',
            'pose_control = example.mediapipe_example.include.pose_control:main',
            'hand_gesture = example.mediapipe_example.include.hand_gesture:main',

            'cross_bridge = example.rgbd_example.include.cross_bridge_node:main',
            'prevent_falling = example.rgbd_example.include.prevent_falling_node:main',
            'track_and_grab = example.rgbd_example.include.track_and_grab:main',
            'object_classification = example.rgbd_example.include.object_classification:main',
            'intelligent_kick = example.intelligent_kick.intelligent_kick:main',
            'object_volume_measurement = example.rgbd_example.include.object_volume_measurement:main',

            'garbage_classification = example.garbage_classification.garbage_classification:main',
            'yolo_node = example.yolo_detect.yolo_node:main',
            'yolo_detect_demo = example.yolo_detect.yolo_detect_demo:main',

            'color_track = example.color_track.color_track_node:main',
            'automatic_pick = example.navigation_transport.automatic_pick:main',
            'navigation_transport = example.navigation_transport.navigation_transport:main',
            'intelligent_pick = example.intelligent_transport.intelligent_pick:main',
            'intelligent_transport = example.intelligent_transport.intelligent_transport:main',
            'narrow_slit_traversal = example.competition.narrow_slit_traversal:main',

 
            'tripod_gait = example.body_control.include.tripod_gait:main',
            'ripple_gait = example.body_control.include.ripple_gait:main',
            'forward_and_rorate = example.body_control.include.forward_and_rorate:main',
            'left_and_right = example.body_control.include.left_and_right:main',
            'diagonally = example.body_control.include.diagonally:main',
            'speed_control = example.body_control.include.speed_control:main',
            'broken_line_walk = example.body_control.include.broken_line_walk:main',
            'square_walk = example.body_control.include.square_walk:main',
            'oled = example.body_control.include.oled:main',
            'body_ik = example.body_control.include.body_ik:main',
            'height_adjustment = example.body_control.include.height_adjustment:main',
            'posture_adjustment = example.body_control.include.posture_adjustment:main',
            'body_wave = example.body_control.include.body_wave:main',
            'body_circle = example.body_control.include.body_circle:main',
            'selfbalancing = example.body_control.include.selfbalancing:main',
            'forward_back = example.body_control.include.forward_back:main',
            'across_the_steps = example.body_control.include.across_the_steps:main',

        ],
    },
)
