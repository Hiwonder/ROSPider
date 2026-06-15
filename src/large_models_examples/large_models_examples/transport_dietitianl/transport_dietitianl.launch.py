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
        navigation_package_path = get_package_share_directory('navigation')
        example_package_path = get_package_share_directory('large_models_examples')
    else:
        slam_package_path = '/home/ubuntu/ros2_ws/src/slam'
        navigation_package_path = '/home/ubuntu/ros2_ws/src/navigation'
        example_package_path = '/home/ubuntu/ros2_ws/src/large_models_examples'

    map_name = LaunchConfiguration('map', default='map_01').perform(context)
    debug = LaunchConfiguration('debug', default='false')
    robot_name = LaunchConfiguration('robot_name', default=os.environ['HOST'])
    master_name = LaunchConfiguration('master_name', default=os.environ['MASTER'])

    map_name_arg = DeclareLaunchArgument('map', default_value=map_name)
    debug_arg = DeclareLaunchArgument('debug', default_value=debug)
    master_name_arg = DeclareLaunchArgument('master_name', default_value=master_name)
    robot_name_arg = DeclareLaunchArgument('robot_name', default_value=robot_name)

    automatic_transport_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(example_package_path, 'large_models_examples/transport_dietitianl/automatic_transport.launch.py')),
        launch_arguments={
            'debug': debug,
            'master_name': master_name,
            'robot_name': robot_name,
            'enable_display': 'false',
        }.items(),
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(navigation_package_path, 'launch/include/bringup.launch.py')),
        launch_arguments={
            'use_sim_time': 'false',
            'map': os.path.join(slam_package_path, 'maps', map_name + '.yaml'),
            'params_file': os.path.join(navigation_package_path, 'config', 'nav2_params.yaml'),
            'namespace': robot_name,
            'use_namespace': 'false',
            'autostart': 'true',
            'use_teb': 'false',
        }.items(),
    )

    navigation_controller_node = Node(
        package='large_models_examples',
        executable='navigation_controller',
        output='screen',
        parameters=[{'map_frame': 'map', 'nav_goal': '/nav_goal'}]
    )

    rviz_node = ExecuteProcess(
            cmd=['rviz2', 'rviz2', '-d', os.path.join(navigation_package_path, 'rviz/transport_dietitianl.rviz')],
            output='screen'
        )

    bringup_launch = GroupAction(
     actions=[
         PushRosNamespace(robot_name),
         automatic_transport_launch,
         TimerAction(
             period=15.0,  # 延时等待其它节点启动好(delay for enabling other nodes)
             actions=[navigation_launch],
         ),
      ]
    )

    return [map_name_arg, 
            debug_arg, 
            master_name_arg,
            robot_name_arg,
            bringup_launch, 
            navigation_controller_node,
            rviz_node
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
