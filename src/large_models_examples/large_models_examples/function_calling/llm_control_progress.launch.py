import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch import LaunchDescription, LaunchService
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction, ExecuteProcess


def launch_setup(context, *args, **kwargs):
    function_mode = LaunchConfiguration('function').perform(context)

    conf = LaunchConfiguration('conf', default=0.45)
    conf_arg = DeclareLaunchArgument('conf', default_value=conf)

    mode = LaunchConfiguration('mode', default=1)
    mode_arg = DeclareLaunchArgument('mode', default_value=mode)

    interruption = LaunchConfiguration('interruption', default=False)
    interruption_arg = DeclareLaunchArgument('interruption', default_value=interruption)

    use_depth_cam = LaunchConfiguration('use_depth_cam', default=True)
    use_depth_cam_arg = DeclareLaunchArgument('use_depth_cam', default_value=use_depth_cam)

    # machine_type = os.environ.get('MACHINE_TYPE', '')
    use_depth_cam_val = use_depth_cam.perform(context).lower() in ['true', '1', 'yes']


    # if use_depth_cam_val:
    camera_topic_value = '/depth_cam/rgb/image_raw'
    # else:
    #     if 'Pro' in machine_type:
    #         camera_topic_value = '/usb_cam/image'
    #     else:
    #         camera_topic_value = '/depth_cam/rgb/image_raw'

    camera_topic = LaunchConfiguration('camera_topic', default=camera_topic_value)
    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value=TextSubstitution(text=camera_topic_value)
    )

    controller_package_path = get_package_share_directory('controller')
    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(controller_package_path, 'launch/controller.launch.py')),
    )

    peripherals_package_path = get_package_share_directory('peripherals')
    depth_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(peripherals_package_path, 'launch/depth_camera.launch.py')),
    )

    lidar_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(peripherals_package_path, 'launch/lidar.launch.py')),
    )

    line_following_node = Node(
        package='app',
        executable='line_following',
        output='screen',
        parameters=[{'debug': False}],
    )

    object_tracking_node = Node(
        package='app',
        executable='object_tracking',
        output='screen',
        parameters=[{'debug': True}],
    )

    kinematics_package_path = get_package_share_directory('kinematics')
    kinematics_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(kinematics_package_path, 'launch/kinematics_node.launch.py')),
    )

    tf_transform_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='usb_to_color_frame_link',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
            '--frame-id', 'usb_link',
            '--child-frame-id', 'depth_cam_color_frame'
        ]
    )

    navigation_package_path = get_package_share_directory('navigation')
    map_name = LaunchConfiguration('map', default='map_01').perform(context)
    robot_name = LaunchConfiguration('robot_name', default=os.environ.get('HOST', 'robot'))
    master_name = LaunchConfiguration('master_name', default=os.environ.get('MASTER', 'master'))

    map_name_arg = DeclareLaunchArgument('map', default_value=map_name)
    master_name_arg = DeclareLaunchArgument('master_name', default_value=master_name)
    robot_name_arg = DeclareLaunchArgument('robot_name', default_value=robot_name)

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_package_path, 'launch/navigation.launch.py')),
        launch_arguments={
            'sim': 'false',
            'map': map_name,
            'robot_name': robot_name,
            'master_name': master_name,
            'use_teb': 'true',
        }.items(),
    )

    navigation_controller_node = Node(
        package='large_models_examples',
        executable='navigation_controller',
        output='screen',
        parameters=[{'map_frame': 'map', 'nav_goal': '/nav_goal'}]
    )

    rviz_node = ExecuteProcess(
        cmd=['rviz2', '-d', os.path.join(navigation_package_path, 'rviz/navigation_controller.rviz')],
        output='screen'
    )

    llm_agent_progress_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('large_models_examples'),
                'large_models_examples/function_calling/llm_agent_progress.launch.py'
            )
        ),
        launch_arguments= {
            'camera_topic': camera_topic,
            'use_depth_cam': use_depth_cam,
        }.items(),
    )

    if function_mode == 'navigation':
        return [
            map_name_arg,
            master_name_arg,
            robot_name_arg,
            camera_topic_arg,
            use_depth_cam_arg,
            rviz_node,
            navigation_launch,
            navigation_controller_node,
            llm_agent_progress_launch,
            
        ]
    else:
        return [
            camera_topic_arg,
            use_depth_cam_arg,
            depth_camera_launch,
            controller_launch,
            lidar_node_launch,
            line_following_node,
            llm_agent_progress_launch,
        ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'function',
            default_value='default',
            description='The function to execute'
        ),
        DeclareLaunchArgument(
            'use_depth_cam',
            default_value='True',
            description='Force using depth camera'
        ),
        OpaqueFunction(function=launch_setup)
    ])


if __name__ == '__main__':
    ld = generate_launch_description()
    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
