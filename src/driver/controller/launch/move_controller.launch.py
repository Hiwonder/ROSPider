import os
from ament_index_python.packages import get_package_share_directory

from nav2_common.launch import RewrittenYaml
from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction

def launch_setup(context):
    namespace = LaunchConfiguration('namespace', default='')
    use_namespace = LaunchConfiguration('use_namespace', default='false')

    namespace_arg = DeclareLaunchArgument('namespace', default_value=namespace)
    use_namespace_arg = DeclareLaunchArgument('use_namespace', default_value=use_namespace)

    move_controller_node = Node(
        package='controller',
        executable='move_controller',
        output='screen', 
    )

    return [
        namespace_arg,
        use_namespace_arg,
        move_controller_node
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
