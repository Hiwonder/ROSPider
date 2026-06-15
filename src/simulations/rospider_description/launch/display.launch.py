import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch import LaunchDescription, LaunchService
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription, ExecuteProcess

def generate_launch_description():
    compiled = os.environ['need_compile']
    namespace = LaunchConfiguration('namespace', default='')
    use_namespace = LaunchConfiguration('use_namespace', default='false')
    frame_prefix = LaunchConfiguration('frame_prefix', default='')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    frame_prefix_arg = DeclareLaunchArgument('frame_prefix', default_value=frame_prefix)
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value=use_sim_time)
    namespace_arg = DeclareLaunchArgument('namespace', default_value=namespace)
    use_namespace_arg = DeclareLaunchArgument('use_namespace', default_value=use_namespace)

    if compiled == 'True':
        rospider_description_package_path = get_package_share_directory('rospider_description')
    else:
        rospider_description_package_path = '/home/ubuntu/ros2_ws/src/simulations/rospider_description'
    urdf_path = os.path.join(rospider_description_package_path, 'urdf/rospider.xacro')
    rviz_config_file = os.path.join(rospider_description_package_path, 'rviz/view.rviz')

    # robot_description = Command(['xacro ', urdf_path])
    robot_description = ParameterValue(
        Command(['xacro ', urdf_path]),
        value_type=str  # Explicitly set the value type to string
    )
    # 动态TF转换(dynamic TF Transformation)
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
    )
    
    # 静态TF(static TF)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'frame_prefix': frame_prefix, 'use_sim_time': use_sim_time}],
        arguments=[urdf_path],
    )

    rviz_node = ExecuteProcess(
            cmd=['rviz2', 'rviz2', '-d', os.path.join(rospider_description_package_path, 'rviz/view.rviz')],
            output='screen'
        )

    # Timer action to delay rviz_node for 5 seconds
    delay_rviz_node = TimerAction(
        period=5.0,
        actions=[rviz_node],
    )

    return LaunchDescription([
        frame_prefix_arg,
        use_sim_time_arg,
        namespace_arg,
        use_namespace_arg,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        delay_rviz_node,
    ])

if __name__ == '__main__':
    # 创建一个LaunchDescription对象(create a LaunchDescription object)
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
