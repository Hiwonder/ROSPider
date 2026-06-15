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
        competition_package_path = get_package_share_directory('competition')
    else:
        slam_package_path = '/home/ubuntu/ros2_ws/src/slam'
        competition_package_path = '/home/ubuntu/ros2_ws/src/competition'

    debug = LaunchConfiguration('debug', default='false')
    broadcast = LaunchConfiguration('broadcast', default='false')
    enable_display = LaunchConfiguration('enable_display', default='true')
    robot_name = LaunchConfiguration('robot_name', default=os.environ['HOST'])
    master_name = LaunchConfiguration('master_name', default=os.environ['MASTER'])

    debug_arg = DeclareLaunchArgument('debug', default_value=debug)
    broadcast_arg = DeclareLaunchArgument('broadcast', default_value=broadcast)
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

    yolo_node = Node(
        package='competition',
        executable='yolo_node',
        output='screen',
        parameters=[{'task': 'obb', 'engine': 'competition.engine', 'conf': 0.7, 'start': False, 'display': False}]
    )

    pick_and_place_node = Node(
        package='competition',
        executable='pick_and_place',
        output='screen',
        parameters=[os.path.join(competition_package_path, 'config/automatic_pick_roi.yaml'), {'debug': debug, 'broadcast': broadcast, 'enable_display': enable_display}]
    )

    return [debug_arg, 
            broadcast_arg, 
            enable_display_arg,
            master_name_arg, 
            robot_name_arg, 
            base_launch, 
            yolo_node,
            pick_and_place_node
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
