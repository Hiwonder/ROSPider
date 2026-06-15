import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction

def launch_setup(context):
    compiled = os.environ.get('need_compile', 'False')
    
    if compiled == 'True':
        peripherals_package_path = get_package_share_directory('peripherals')
    else:
        peripherals_package_path = '/home/ubuntu/ros2_ws/src/peripherals'

    depth_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(peripherals_package_path, 'launch/depth_camera.launch.py')),
    )

    camera_topic = LaunchConfiguration('camera_topic', default='depth_cam/rgb/image_raw')
    camera_topic_arg = DeclareLaunchArgument('camera_topic', default_value=camera_topic)
    
    conf = LaunchConfiguration('conf', default=0.6).perform(context)
    conf_arg = DeclareLaunchArgument('conf', default_value=conf)

    model_name = LaunchConfiguration('model_name', default='yolo').perform(context)
    model_name_arg = DeclareLaunchArgument('model_name', default_value=model_name)

    garbage_names = ['BananaPeel','BrokenBones','CigaretteEnd','DisposableChopsticks','Ketchup','Marker','OralLiquidBottle','Plate','PlasticBottle','StorageBattery','Toothbrush', 'Umbrella']
    traffic_names = ['go', 'right', 'park', 'red', 'green', 'crosswalk']
    yolo_names = [
        "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train",
        "truck", "boat", "traffic light", "fire hydrant", "stop sign", "parking meter",
        "bench", "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear",
        "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase",
        "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat",
        "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle",
        "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
        "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut",
        "cake", "chair", "couch", "potted plant", "bed", "dining table", "toilet",
        "tv", "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave",
        "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase",
        "scissors", "teddy bear", "hair drier", "toothbrush"
    ]

    if 'garbage' in model_name:
        model_classes_names = garbage_names
        model_engine = 'best_garbage_26.engine'
        model_detect_method = 'obb'
    elif 'traffic' in  model_name:
        model_classes_names = traffic_names
        model_engine = 'best_traffic_26.engine'
        model_detect_method = 'detect'
    else:
        model_classes_names = yolo_names
        model_engine = 'yolo26n.engine' 
        model_detect_method = 'detect'


    yolo_node = Node(
        package='example',
        executable='yolo_node',
        output='screen',
        parameters=[{
            'image_topic': camera_topic,
            'classes':  model_classes_names,
            'engine': model_engine,
            'conf': conf,
            'task': model_detect_method,
            'display': True}]
    )


    return [
        conf_arg,
        model_name_arg,
        camera_topic_arg,
        depth_camera_launch,
        yolo_node,
    ]

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function = launch_setup)
    ])

if __name__ == '__main__':
    ld = generate_launch_description()
    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
