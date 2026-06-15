from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription, LaunchService
from launch.actions import OpaqueFunction, DeclareLaunchArgument

def launch_setup(context):
    launch_list =  [
            ]

    tts_node = Node(
        package='large_models',
        executable='tts_node',
        output='screen',
    )
    launch_list.append(tts_node)
    return launch_list

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function = launch_setup)
    ])

if __name__ == '__main__':
    # Create a LaunchDescription object. (创建一个LaunchDescription对象)
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
