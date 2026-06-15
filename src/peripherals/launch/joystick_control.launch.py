from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    init = LaunchConfiguration('init', default='true')
    init_arg = DeclareLaunchArgument('init', default_value=init)
    slam = LaunchConfiguration('slam', default='false')
    slam_arg = DeclareLaunchArgument('slam', default_value=slam)
    rtabmap = LaunchConfiguration('rtabmap', default= 'true')
    rtabmap_arg = DeclareLaunchArgument('rtabmap', default_value=rtabmap)
    max_linear = LaunchConfiguration('max_linear', default='0.04')
    max_angular = LaunchConfiguration('max_angular', default='0.3')
    max_linear_arg = DeclareLaunchArgument('max_linear', default_value=max_linear)
    max_angular_arg = DeclareLaunchArgument('max_angular', default_value=max_angular)
    remap_cmd_vel = LaunchConfiguration('remap_cmd_vel', default='controller/cmd_vel')
    remap_cmd_vel_arg = DeclareLaunchArgument('remap_cmd_vel', default_value=remap_cmd_vel)

    joystick_control_node = Node(
        package='peripherals',
        executable='joystick_control',
        output='screen',
        parameters=[{'init': init, 'slam': slam, 'rtabmap': rtabmap}],
        remappings=[('controller/cmd_vel', remap_cmd_vel)]
    )

    return LaunchDescription([
        init_arg,
        slam_arg,
        rtabmap_arg,
        remap_cmd_vel_arg,
        joystick_control_node
    ])

if __name__ == '__main__':
    # 创建一个LaunchDescription对象(create a LaunchDescription object)
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
