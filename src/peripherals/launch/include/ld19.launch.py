from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration

import os
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():
    compiled = os.environ['need_compile']
    # 声明参数
    lidar_frame = LaunchConfiguration('lidar_frame', default='lidar_frame')
    scan_raw = LaunchConfiguration('scan_raw', default='scan_raw')
    lidar_frame_arg = DeclareLaunchArgument('lidar_frame', default_value=lidar_frame)
    scan_raw_arg = DeclareLaunchArgument('scan_raw', default_value=scan_raw)

    if compiled == 'True':
        peripherals_package_path = get_package_share_directory('peripherals')
    else:
        peripherals_package_path = '/home/ubuntu/ros2_ws/src/peripherals'
        

    ms200_node = Node(
      package='ldlidar_stl_ros2',
      executable='ldlidar_stl_ros2_node',
      name='LD19',
      output='screen',
      parameters=[
        {'product_name': 'LDLiDAR_LD19'},
        {'topic_name': 'scan'},
        {'frame_id': 'lidar_frame'},
        {'port_name': '/dev/lidar'},
        {'port_baudrate': 230400},
        {'laser_scan_dir': True},
        {'enable_angle_crop_func': True},
        {'angle_crop_min': 120.0},
        {'angle_crop_max': 240.0},
      ],
        remappings=[('scan', scan_raw)]
  )

    return LaunchDescription([
        lidar_frame_arg,
        scan_raw_arg,
        ms200_node,

    ])

if __name__ == '__main__':
    # 创建一个LaunchDescription对象(create a LaunchDescription object)
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()

