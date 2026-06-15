import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction

def launch_setup(context):
    compiled = os.environ['need_compile']
    if compiled == 'True':
        slam_package_path = get_package_share_directory('slam')
        example_package_path = get_package_share_directory('large_models_examples')
    else:
        slam_package_path = '/home/ubuntu/ros2_ws/src/slam'
        example_package_path = '/home/ubuntu/ros2_ws/src/large_models_examples'

    debug = LaunchConfiguration('debug', default='false')
    enable_display = LaunchConfiguration('enable_display', default='true')
    robot_name = LaunchConfiguration('robot_name', default=os.environ['HOST'])
    master_name = LaunchConfiguration('master_name', default=os.environ['MASTER'])

    debug_arg = DeclareLaunchArgument('debug', default_value=debug)
    enable_display_arg = DeclareLaunchArgument('enable_display', default_value=enable_display)
    master_name_arg = DeclareLaunchArgument('master_name', default_value=master_name)
    robot_name_arg = DeclareLaunchArgument('robot_name', default_value=robot_name)

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(slam_package_path, 'launch/include/robot.launch.py')),
        launch_arguments={
            'master_name': master_name,
            'robot_name': robot_name
        }.items(),
    )

    automatic_transport_node = Node(
        package='large_models_examples',
        executable='automatic_transport',
        output='screen',
        parameters=[os.path.join(example_package_path, 'config/automatic_transport_roi.yaml'), {'debug': debug, 'enable_display': enable_display}]
    )

    return [debug_arg, 
            enable_display_arg,
            master_name_arg, 
            robot_name_arg, 
            base_launch, 
            automatic_transport_node
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
