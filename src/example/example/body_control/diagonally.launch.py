import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription, LaunchService
from launch.actions import  OpaqueFunction, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import  PythonLaunchDescriptionSource

def launch_setup(context):
    compiled = os.environ['need_compile']
    direction = LaunchConfiguration('direction', default='45')
    direction_arg = DeclareLaunchArgument('direction', default_value=direction)
    if compiled == 'True':
        controller_package_path = get_package_share_directory('controller')
    else:
        controller_package_path = '/home/ubuntu/ros2_ws/src/driver/controller'
        

    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(controller_package_path, 'launch/controller.launch.py')),
    )

    diagonally_node = Node(
        package='example',
        executable='diagonally',
        output='screen',
        parameters=[{'direction': direction}]
    )

    return [
            direction_arg,
            controller_launch,
            diagonally_node,
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
