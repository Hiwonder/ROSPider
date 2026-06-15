import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch import LaunchDescription, LaunchService
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource

def launch_setup(context):
    compiled = os.environ['need_compile']
    if compiled == 'True':
        app_package_path = get_package_share_directory('app')
    else:
        app_package_path = '/home/ubuntu/ros2_ws/src/app'

    lidar_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(app_package_path, 'launch/lidar_node.launch.py')),
    )

    line_following_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(app_package_path, 'launch/line_following_node.launch.py')),
    )

    intelligent_kick_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(app_package_path, 'launch/intelligent_kick_node.launch.py')),
    )

    self_balancing_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(app_package_path, 'launch/self_balancing_node.launch.py')),
    )

    hand_gesture_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(app_package_path, 'launch/hand_gesture.launch.py')),
    )

    perform_actions_node = Node(
        package='app',
        executable='perform_actions',
        output='screen',
    )

    return [
            perform_actions_node,
            lidar_node_launch,
            line_following_node_launch,
            hand_gesture_node_launch,
            intelligent_kick_node_launch,
            self_balancing_node_launch,
            ]

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function = launch_setup)
    ])

if __name__ == '__main__':
    # 创建一个LaunchDescription对象(create a LaunchDescription object)
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
