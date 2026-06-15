import os
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration
from launch.actions import OpaqueFunction, DeclareLaunchArgument

def launch_setup(context):
    camera_topic = LaunchConfiguration('camera_topic', default='/depth_cam/rgb/image_raw')
    camera_topic_arg = DeclareLaunchArgument('camera_topic', default_value=camera_topic)

    launch_list =  [
            camera_topic_arg,
            ]

    if os.environ["ASR_MODE"] == 'offline':
        ollama_server = ExecuteProcess(
            cmd=['bash', '-c', 'exec ollama serve > /dev/null 2>&1'],
            # cmd=['bash', '-c', 'exec ollama serve'],
            name='ollama_server',
            shell=False,
            output='screen',
            additional_env={
                'PATH': '/usr/local/cuda/bin:' + os.environ.get('PATH', ''),
                'LD_LIBRARY_PATH': '/usr/local/cuda/lib64:' + os.environ.get('LD_LIBRARY_PATH', ''),
                'CUDA_HOME': '/usr/local/cuda',
            }
        )
        launch_list.append(ollama_server)

    agent_process_node = Node(
        package='large_models',
        executable='agent_process',
        output='screen',
        parameters=[{"camera_topic": camera_topic}],
    )

    launch_list.append(agent_process_node)
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
