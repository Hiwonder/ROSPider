#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port', default='/dev/lidar')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='230400') 
    frame_id = LaunchConfiguration('frame_id', default='lidar_frame')
    version = LaunchConfiguration('version', default=4)

    return LaunchDescription([

        DeclareLaunchArgument(
            'serial_port',
            default_value=serial_port,
            description='Specifying usb port to connected lidar'),

        DeclareLaunchArgument(
            'serial_baudrate',
            default_value=serial_baudrate,
            description='Specifying usb port baudrate to connected lidar'),
        
        DeclareLaunchArgument(
            'frame_id',
            default_value=frame_id,
            description='Specifying frame_id of lidar'),

        DeclareLaunchArgument(
            'version',
            default_value=version,
            description='Specifying version of lidar'),

        Node(
            package='sclidar_ros2',
            executable='sclidar',
            name='sclidar_scan_publisher',
            parameters=[{'port': serial_port, 
                         'baudrate': serial_baudrate, 
                         'version': version,
                         'frame_id': frame_id}],
            output='screen',
            remappings=[('scan', 'scan_raw')]
            ),
    ])

