import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription, LaunchService
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction,ExecuteProcess

def launch_setup(context, *args, **kwargs):
    function = LaunchConfiguration('function').perform(context)
    function_mode_arg = DeclareLaunchArgument('function', default_value=function)

    conf = LaunchConfiguration('conf', default=0.45)
    conf_arg = DeclareLaunchArgument('conf', default_value=conf)

    mode = LaunchConfiguration('mode', default=1)
    mode_arg = DeclareLaunchArgument('mode', default_value=mode)

    interruption = LaunchConfiguration('interruption', default=False)
    interruption_arg = DeclareLaunchArgument('interruption', default_value=interruption)

    camera_topic = LaunchConfiguration('camera_topic', default='/depth_cam/rgb/image_raw')
    camera_topic_arg = DeclareLaunchArgument('camera_topic', default_value=camera_topic)
    

    use_depth_cam = LaunchConfiguration('use_depth_cam', default=False)
    use_depth_cam_arg = DeclareLaunchArgument('use_depth_cam', default_value=use_depth_cam)

    transport_debug = LaunchConfiguration('transport_debug', default='none')
    transport_debug_arg = DeclareLaunchArgument('transport_debug', default_value=transport_debug)

    large_models_package_path = get_package_share_directory('large_models')
    
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

    large_models_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(large_models_package_path, 'launch/start.launch.py')),
        launch_arguments={'mode': mode,'camera_topic': camera_topic}.items(),
    )


    if function == 'smart_factory':
        llm_control_node = Node(
            package='large_models_examples',
            executable='smart_factory',
            output='screen',
    )
    elif function == 'road_network':
        llm_control_node = Node(
            package='large_models_examples',
            executable='road_network_tool',
            output='screen',
        )
    else:
        llm_control_node = Node(
            package='large_models_examples',
            executable='llm_control',
            output='screen',
            parameters=[{
                'use_depth_cam': use_depth_cam,
                'transport_debug' : transport_debug,
            }]
    )        


    base_launch_groups = [
        mode_arg,
        interruption_arg,
        camera_topic_arg,
        use_depth_cam_arg,
        transport_debug_arg,
        tts_node_launch,
        vocal_detect_launch,
        agent_process_launch,
        llm_control_node,

    ]

    return base_launch_groups
        

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'function',
            default_value='default',
            description='The function to execute'),
        OpaqueFunction(function = launch_setup)
    ])

if __name__ == '__main__':
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
