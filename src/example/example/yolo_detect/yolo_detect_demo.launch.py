import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction

def launch_setup(context):
    compiled = os.environ.get('need_compile', 'False')
    
    # Declare parameters (声明参数)
    camera_topic = LaunchConfiguration('camera_topic', default='depth_cam/rgb/image_raw')
    camera_topic_arg = DeclareLaunchArgument('camera_topic', default_value=camera_topic)

    model_name = LaunchConfiguration('model_name', default='yolo26n')
    model_arg = DeclareLaunchArgument('model_name', default_value=model_name)

    conf = LaunchConfiguration('conf', default=0.6).perform(context)
    conf_arg = DeclareLaunchArgument('conf', default_value=conf)

    # YOLO COCO dataset standard classes (YOLO 标准 COCO 数据集类别)
    classes_yolo = [
        "person", "bicycle", "car", "motorcycle", "airplane", 
        "bus", "train", "truck", "boat", "traffic light", 
        "fire hydrant", "stop sign", "parking meter", "bench", "bird", 
        "cat", "dog", "horse", "sheep", "cow", 
        "elephant", "bear", "zebra", "giraffe", "backpack", 
        "umbrella", "handbag", "tie", "suitcase", "frisbee", 
        "skis", "snowboard", "sports ball", "kite", "baseball bat", 
        "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", 
        "wine glass", "cup", "fork", "knife", "spoon", 
        "bowl", "banana", "apple", "sandwich", "orange", 
        "broccoli", "carrot", "hot dog", "pizza", "donut", 
        "cake", "chair", "couch", "potted plant", "bed", 
        "dining table", "toilet", "tv", "laptop", "mouse", 
        "remote", "keyboard", "cell phone", "microwave", "oven", 
        "toaster", "sink", "refrigerator", "book", "clock", 
        "vase", "scissors", "teddy bear", "hair drier", "toothbrush"
    ]

    # Garbage classification classes (垃圾分类类别)
    classes_garbage = [
        'BananaPeel', 'BrokenBones', 'CigaretteEnd', 'DisposableChopsticks', 'Ketchup',
        'Marker', 'OralLiquidBottle', 'Plate', 'PlasticBottle', 'StorageBattery', 
        'Toothbrush', 'Umbrella'
    ]

    # Traffic light and sign classes (交通标志与信号灯类别)
    classes_traffic = [
        'go', 'right', 'park', 'red', 'green', 
        'crosswalk'
    ]

    # 2. Logic to select classes (选择类别的逻辑)
    model_name_str = model_name.perform(context)
    if 'garbage' in model_name_str:
        current_classes = classes_garbage
    elif 'traffic' in model_name_str:
        current_classes = classes_traffic
    else:
        current_classes = classes_yolo

    if compiled == 'True':
        peripherals_package_path = get_package_share_directory('peripherals')
    else:
        peripherals_package_path = '/home/ubuntu/ros2_ws/src/peripherals'

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(peripherals_package_path, 'launch/depth_camera.launch.py')),
    )

    yolo_detect_demo_node = Node(
        package='example',
        executable='yolo_detect_demo',
        name='yolo_detect_demo',
        parameters=[{
            'start': True,
            'image_topic': camera_topic,
            'model_name': model_name,
            'model_size': 640,
            'conf_threshold': conf,
            'classes': current_classes, # Pass the selected list (传递选中的列表)
        }],
        output='screen'
    )

    return [
        conf_arg,
        model_arg,
        camera_topic_arg,
        camera_launch,
        yolo_detect_demo_node,
    ]

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function = launch_setup)
    ])