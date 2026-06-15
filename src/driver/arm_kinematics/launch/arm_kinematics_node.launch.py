from launch_ros.actions import Node
from launch import LaunchDescription, LaunchService

def generate_launch_description():
    arm_kinematics_node = Node(
        package='arm_kinematics',
        executable='search_kinematics_solutions',
        output='screen',
    )

    return LaunchDescription([
        arm_kinematics_node
    ])

if __name__ == '__main__':
    # 创建一个LaunchDescription对象(create a LaunchDescription object)
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
