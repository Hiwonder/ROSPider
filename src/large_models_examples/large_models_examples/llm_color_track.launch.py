import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription, LaunchService
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, GroupAction, TimerAction
from launch.conditions import IfCondition, UnlessCondition

def launch_setup(context):
    mode = LaunchConfiguration('mode', default=1)
    mode_arg = DeclareLaunchArgument('mode', default_value=mode)
    camera_topic = LaunchConfiguration('camera_topic', default='depth_cam/rgb/image_raw')
    camera_topic_arg = DeclareLaunchArgument('camera_topic', default_value=camera_topic)
    interruption = LaunchConfiguration('interruption', default=False)
    interruption_arg = DeclareLaunchArgument('interruption', default_value=interruption)

    app_package_path = get_package_share_directory('app')
    large_models_package_path = get_package_share_directory('large_models')

    asr_mode = os.environ.get("ASR_MODE", "online").lower()

    object_tracking_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(app_package_path, 'launch/object_tracking_node.launch.py')),
        launch_arguments={
            'debug': 'true',
        }.items(),
    )

    vocal_detect_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('large_models'), 'launch/vocal_detect.launch.py')),
        launch_arguments={'mode': mode,}.items(),
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

    time_interval = 30.0 if asr_mode == 'offline' else 1.0
    large_models_launch = GroupAction(
     actions=[
        tts_node_launch,
        TimerAction(
                period=time_interval, actions=[
                vocal_detect_launch,
            ],
         ),
        TimerAction(period=time_interval, actions=[
            agent_process_launch,
            ],
         ),
      ]
    )

    llm_color_track_node = Node(
        package='large_models_examples',
        executable='llm_color_track',
        output='screen',
        parameters=[{
            'interruption': interruption,
        }],
    )

    bringup_launch = GroupAction(
    actions=[
        large_models_launch,
        TimerAction(
             period=time_interval * 2,  # 延时等待其它节点启动好(delay and wait for other nodes to start up properly)
             actions=[
            object_tracking_node_launch,
            llm_color_track_node,     
            ],
         ),
      ]
    )


    return [mode_arg,
            interruption_arg,
            bringup_launch,
            ]

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function = launch_setup)
    ])

if __name__ == '__main__':
    # 创建一个LaunchDescription对象
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
