import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription, LaunchService
from launch.launch_description_sources import  PythonLaunchDescriptionSource
from launch.actions import  OpaqueFunction, DeclareLaunchArgument, IncludeLaunchDescription

def launch_setup(context):
    compiled = os.environ['need_compile']
    period = LaunchConfiguration('period', default='1')
    stride = LaunchConfiguration('stride', default='15')
    stride_arg = DeclareLaunchArgument('stride', default_value=stride)
    period_arg = DeclareLaunchArgument('period', default_value=period)
    if compiled == 'True':
        controller_package_path = get_package_share_directory('controller')
    else:
        controller_package_path = '/home/ubuntu/ros2_ws/src/driver/controller'
    
    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(controller_package_path, 'launch/controller.launch.py')),
    )

   
    speed_control_node = Node(
        package='example',
        executable='speed_control',
        output='screen',
        parameters=[{'period': period, 'stride': stride}]
    )

    return [
            stride_arg,
            period_arg,
            controller_launch,
            speed_control_node,
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
