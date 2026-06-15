import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction

def launch_setup(context):
    compiled = os.environ['need_compile']
    start = LaunchConfiguration('start', default='true')
    start_arg = DeclareLaunchArgument('start', default_value=start)
    debug = LaunchConfiguration('debug', default='false')
    debug_arg = DeclareLaunchArgument('debug', default_value=debug)
    broadcast = LaunchConfiguration('broadcast', default='false')
    broadcast_arg = DeclareLaunchArgument('broadcast', default_value=broadcast)
    enable_display = LaunchConfiguration('enable_display', default='false')
    enable_display_arg = DeclareLaunchArgument('enable_display', default_value=enable_display)
    yolo_model = LaunchConfiguration('yolo', default='yolov8').perform(context)
    yolo_model_arg = DeclareLaunchArgument('yolo', default_value=yolo_model)

    garbage_names = ['BananaPeel','BrokenBones','CigaretteEnd','DisposableChopsticks','Ketchup','Marker','OralLiquidBottle','Plate','PlasticBottle','StorageBattery','Toothbrush', 'Umbrella']

    if compiled == 'True':
        peripherals_package_path = get_package_share_directory('peripherals')
        controller_package_path = get_package_share_directory('controller')
        example_package_path = get_package_share_directory('example')
    else:
        peripherals_package_path = '/home/ubuntu/ros2_ws/src/peripherals'
        controller_package_path = '/home/ubuntu/ros2_ws/src/driver/controller'
        example_package_path = '/home/ubuntu/ros2_ws/src/example'

    if yolo_model == "yolov8":
        engine_file = 'best_garbage_v8.engine'
    elif yolo_model == "yolo11":
        engine_file = 'best_garbage_11.engine'
    else :
        engine_file = 'best_garbage_26.engine'

    depth_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(peripherals_package_path, 'launch/depth_camera.launch.py')),
    )
    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(controller_package_path, 'launch/controller.launch.py')),
    )

    init_pose_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(controller_package_path, 'launch/init_pose.launch.py')),
        launch_arguments={
            'namespace': '',  
            'use_namespace': 'false',
            'action_name': 'garbage_pick_init',
        }.items(),
    )

    yolo_node = Node(
        package='example',
        executable='yolo_node',
        output='screen',
        parameters=[{ 'classes': garbage_names, 'task': 'obb', 'engine': engine_file, 'conf': 0.7, 'display': enable_display, 'model_size': 640 }]
    )

    garbage_classification_node = Node(
        package='example',
        executable='garbage_classification',
        output='screen',
        parameters=[os.path.join(example_package_path, 'config/garbage_classification_roi.yaml'), {'start': start}, {'debug': debug}, {'broadcast': broadcast}],
    )

    return [start_arg,
            debug_arg,
            broadcast_arg,
            yolo_model_arg,
            enable_display_arg,
            depth_camera_launch,
            controller_launch,
            init_pose_launch,
            yolo_node,
            garbage_classification_node,
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
