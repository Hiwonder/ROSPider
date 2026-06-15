import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription, LaunchService
from launch.actions import IncludeLaunchDescription, OpaqueFunction, GroupAction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

def launch_setup(context):
    mode = LaunchConfiguration('mode', default=1)
    mode_arg = DeclareLaunchArgument('mode', default_value=mode)

    camera_topic = LaunchConfiguration('camera_topic', default='/depth_cam/rgb/image_raw')
    camera_topic_arg = DeclareLaunchArgument('camera_topic', default_value=camera_topic)

    vocal_detect_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('large_models'), 'launch/vocal_detect.launch.py')),
        launch_arguments={'mode': mode}.items(),
    )

    agent_process_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('large_models'), 'launch/agent_process.launch.py')),
        launch_arguments={'camera_topic': camera_topic}.items(),
    )

    tts_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('large_models'), 'launch/tts_node.launch.py')),
    )

    bringup_launch = GroupAction(
     actions=[
         agent_process_launch,
         TimerAction(
             period=10.0,  # 延时等待其它节点启动好(delay and wait for other nodes to start up properly)
             actions=[
                 vocal_detect_launch,
                 tts_node_launch
             ],
         ),
      ]
    )

    return [
        mode_arg,
        camera_topic_arg,
        bringup_launch
    ]

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

