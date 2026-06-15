import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, OpaqueFunction, TimerAction, ExecuteProcess

def launch_setup(context):
    compiled = os.environ['need_compile']
    if compiled == 'True':
        slam_package_path = get_package_share_directory('slam')
        large_models_examples_package_path = get_package_share_directory('large_models_examples')
        large_models_package_path = get_package_share_directory('large_models')
    else:
        slam_package_path = '/home/ubuntu/ros2_ws/src/slam'
        large_models_examples_package_path = '/home/ubuntu/ros2_ws/src/large_models_examples'
        large_models_package_path = '/home/ubuntu/ros2_ws/src/large_models/large_models'

    mode = LaunchConfiguration('mode', default=1)
    mode_arg = DeclareLaunchArgument('mode', default_value=mode)
    map_name = LaunchConfiguration('map', default='map_01').perform(context)
    robot_name = LaunchConfiguration('robot_name', default=os.environ['HOST'])
    master_name = LaunchConfiguration('master_name', default=os.environ['MASTER'])

    map_name_arg = DeclareLaunchArgument('map', default_value=map_name)
    master_name_arg = DeclareLaunchArgument('master_name', default_value=master_name)
    robot_name_arg = DeclareLaunchArgument('robot_name', default_value=robot_name)

    transport_dietitianl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(large_models_examples_package_path, 'large_models_examples/transport_dietitianl/transport_dietitianl.launch.py')),
        launch_arguments={
            'map': map_name,
            'debug': 'false',
            'robot_name': robot_name,
            'master_name': master_name,
        }.items(),
    )

    large_models_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(large_models_package_path, 'launch/start.launch.py')),
        launch_arguments={'mode': mode}.items(),
    )

    vllm_transport_dietitianl_node = Node(
        package='large_models_examples',
        executable='vllm_transport_dietitianl',
        output='screen',
    )

    return [
            mode_arg,
            map_name_arg, 
            master_name_arg,
            robot_name_arg,
            transport_dietitianl_launch,
            large_models_launch,
            vllm_transport_dietitianl_node
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
