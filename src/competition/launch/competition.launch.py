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
        competition_package_path = get_package_share_directory('competition')
        xf_mic_asr_offline_package_path = '/home/ubuntu/ros2_ws/src/xf_mic_asr_offline'
    else:
        competition_package_path = '/home/ubuntu/ros2_ws/src/competition'
        xf_mic_asr_offline_package_path = '/home/ubuntu/ros2_ws/src/xf_mic_asr_offline'
    debug = LaunchConfiguration('debug', default='false')
    debug_arg = DeclareLaunchArgument('debug', default_value=debug)
  
    enable_display = LaunchConfiguration('enable_display', default='true')
    enable_display_arg = DeclareLaunchArgument('enable_display', default_value=enable_display)
        
    #窄缝穿越
    narrow_slit_traversal_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(competition_package_path, 'launch/narrow_slit_traversal.launch.py')),
    )

    #过独木桥
    cross_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(competition_package_path, 'launch/cross_bridge.launch.py')),
    )

    #抓取与放置
    pick_and_place_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(competition_package_path, 'launch/pick_and_place.launch.py')),
        launch_arguments={
            'debug': debug,
            'enable_display': enable_display,
        }.items(),
    )

    #麦克风
    mic_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(xf_mic_asr_offline_package_path, 'launch/mic_init.launch.py')),
    )
    #标签码识别
    apriltag_recognition_node = Node(
        package='example',
        executable='apriltag_recognition',
        output='screen',
        parameters=[{'enable_display': False,}]

    )

    #主节点
    competition_node = Node(
        package='competition',
        executable='competition',
        output='screen',

    )

    return [ 
            debug_arg, 
            mic_launch,
            pick_and_place_launch, 
            cross_bridge_launch,
            narrow_slit_traversal_launch,
            apriltag_recognition_node,
            competition_node,
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
