import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch import LaunchDescription, LaunchService

def generate_launch_description():
    compiled = os.environ['need_compile']
    if compiled == 'True':
        peripherals_package_path = get_package_share_directory('peripherals')
    else:
        peripherals_package_path = '/home/ubuntu/ros2_ws/src/peripherals'
    imu_calib_node = Node(
        package='imu_calib',
        executable='apply_calib',
        name='imu_calib',
        output='screen',
        parameters=[{"calib_file": os.path.join(peripherals_package_path, 'config/imu_calib.yaml')
                     }],
        remappings=[
            ('raw', 'ros_robot_controller/imu_raw'),
            ('corrected', 'imu_corrected')
            ]
        )

    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        output='screen',
        parameters=[
            {'fixed_frame': "imu_link",
            'use_mag': False,
            'publish_tf': False,
            'world_frame': "enu",
            'orientation_stddev': 0.05}
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/imu/data_raw', 'imu_corrected'),
            ('imu/data', 'imu')
            ]
    )

    return LaunchDescription([
        imu_calib_node,
        imu_filter_node
    ])

if __name__ == '__main__':
    # 创建一个LaunchDescription对象(create a LaunchDescription object)
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
