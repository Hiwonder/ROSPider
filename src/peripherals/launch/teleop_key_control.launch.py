import os
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():

    # teleop_key_control节点(teleop_key_control节点 node)
    teleop_key_control_node = Node(
        package='peripherals',
        executable='teleop_key_control',
        name='teleop_key_control',
        output='screen',
        prefix='xterm -e',

    )

    return LaunchDescription([
        teleop_key_control_node
    ])

if __name__ == '__main__':
    # 创建一个LaunchDescription对象(create a LaunchDescription object)
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
