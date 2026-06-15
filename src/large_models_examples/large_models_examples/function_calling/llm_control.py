#!/usr/bin/env python3
# encoding: utf-8
# @Author: Gcusms
# @Date: 2025/10/21
import os
import re
import cv2
import textwrap
import ast
import time
import json
import math
import yaml
import rclpy
import threading
from speech import speech
from rclpy.node import Node

import numpy as np
import queue
import message_filters
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo

from geometry_msgs.msg import Twist

from std_msgs.msg import String, Bool, Float32
from std_srvs.srv import Trigger, SetBool, Empty
from large_models_msgs.msg import Tools
from large_models.config import *
from large_models_msgs.srv import SetModel, SetString, SetTools,SetBox,SetContent

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from interfaces.srv import SetPose2D,SetPoint
from interfaces.srv import SetString as SetColor
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

# 机械臂
from servo_controller_msgs.msg import ServosPosition
from servo_controller.bus_servo_control import set_servo_position
from servo_controller.action_group_controller import ActionGroupController
from controller.controller_client import ControllerClient
from arm_kinematics.kinematics_control import set_pose_target
from arm_kinematics_msgs.srv import GetRobotPose, SetRobotPose
from ros_robot_controller_msgs.msg import BuzzerState

# sdk 相关工具
from sdk import common
from sdk.pid import PID


import pycuda.driver as cuda
cuda.init()  # 确保CUDA已经初始化
from large_models_examples.tracker import Tracker

# 物体追踪
from large_models_examples.track_anything import ObjectTracker

def depth_pixel_to_camera(pixel_coords, depth, intrinsics):
    fx, fy, cx, cy = intrinsics
    px, py = pixel_coords
    x = (px - cx) * depth / fx
    y = (py - cy) * depth / fy
    z = depth
    return np.array([x, y, z])


tools = [
    {
        "type": "function",
        "function": {
            "name": "get_object_box_distance",
            "description": "获取一个或多个特定目标物体的深度信息或者距离信息，比如物体距离有多远，可以同时检测多个物体(Get the depth information of one or multiple specific target objects, e.g., how far away are objects)",
            "parameters": {
                "type": "object",
                "properties": {
                    "user_query": {
                        "type": "string",
                        "description": "用户的问题，比如：前面的圆柱体距离你有多远？或者：请检测图片中的苹果和香蕉(User's question, e.g., How far is the cylinder in front of me? Or: Please detect the apple and banana in the picture)",
                    }
                },
                "required": ["user_query"]
            }
        }
    },
    {
        "type": "function",
        "function": {
            "name": "get_available_locations",
            "description": "查询并列出机器人可以导航前往的所有预定义地点的位置列表。(Query and list the location of all pre-defined places that the robot can navigate to.)",
            "parameters": {
                "type": "object",
                "properties": {}
            }
        }
    },
    {
        "type": "function",
        "function": {
            "name": "get_current_location",
            "description": "查询并获取机器人当前在地图中的精确位置坐标和朝向。(Query and get the exact position coordinates and orientation of the robot in the map.)",
            "parameters": {
                "type": "object",
                "properties": {}
            }
        }
    },
    {   "type": "function",
        "function": {
            "name": "move_to_location",
            "description": "将机器人移动到指定位置。这是一个耗时操作，当函数成功返回时，表示机器人已经到达目的地,当去到动物园之后，应该观看有什么动物在，并且播报出来，如果要寻找足球首先应该去足球场，如果没有找到的话就去家里找找，如果说好累好渴，就需要买饮料，去到超市找到饮料之后需要夹取饮料。(Move the robot to the designated location. This is a time-consuming operation. When the function returns successfully, it means the robot has reached its destination. Once at the zoo, observe what animals are there and announce them. To find a soccer ball, first go to the soccer field; if you can't find it there, look around your house. If you're tired and thirsty, buy a drink. After finding a drink at the supermarket, pick it up with your chopsticks.)",
            "parameters": {
                "type": "object",
                "properties": {
                    "destination": {
                        "type": "string",
                        "description": "要去的目标地点名称。必须是预定义地点之一或者临时记录的位置(The name of the destination. Must be one of the predefined locations, or a temporary record of a location)",
                        "enum": [
                            "动物园(Zoo)",
                            "足球场(Football Field)",
                            "超市(Store)",
                            "家(Home)",
                            "起点(Starting Point)",
                        ]
                    }
                },
                "required": ["destination"],
            },
        },
    },

    {
        "type": "function",
        "function": {
            "name": "describe_current_view",
            "description": "详细描述机器人当前看到的画面内容，以回答用户提出的具体问题,比如说观察物体的形状和颜色。比如说看看仓库缺哪些物料,涉及到夹取和放置的功能时不进行调用。如果是要寻找足球的话，找不到足球时需要去家里找(Describe the detailed contents of the scene that the robot sees currently, to answer the specific question that the user asks.)",
            "parameters": {
                "type": "object",
                "properties": {
                    "question": {
                        "type": "string",
                        "description": "关于当前画面的具体问题，例如'前面的大门有没有关'？(A specific question about the current scene, for example 'Is the door closed?', or 'What is there?')'"
                    }
                },
                "required": ["question"]
            }
        }
    },
    {
        "type": "function",
        "function": {
            "name": "line_following",
            "description": "命令机器人沿着指定颜色的线进行巡线。(Command the robot to follow a line of a specified color.)",
            "parameters": {
                "type": "object",
                "properties": {
                    "color": {
                        "type": "string",
                        "description": "要巡线的颜色。(red, green, blue)",
                        "enum": ["red", "green", "blue", "black", "yellow"]
                    }
                },
                "required": ["color"]
            }
        }
    },
    {
        "type": "function",
        "function": {
            "name": "lidar_scan_detect",
            "description": "巡线时默认必须调用的函数，辅助机器人避开障碍物(The function must be called by default when robot is following a line, it helps the robot avoid obstacles)",
            "parameters": {
                "type": "object",
                "properties": {
                    "scan_detect": {
                        "type": "string",
                        "description": "障碍物检测结果。(There is an obstacle, No obstacle)",
                    }
                },
                "required": ["scan_detect"],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "robot_move_control",
            "description": "可以通过 direction 控制移动方向，单位为rad/s，通过 rotation 控制旋转方向,单位rad/s，通过 step 控制移动的步数。(Move the robot with specified line speed and angular speed for a specific time. This is a basic movement instruction.)",
            "parameters": {
                "type": "object",
                "properties": {
                    "direction": {
                        "type": "number",
                        "description": "direction ∈ [0, 3.14]，（0为前进，逆时针方向递增）(The direction ∈ [0, 3.14], (0 is forward, counterclockwise direction increases))",
                    },
                    "rotation": {
                        "type": "number",
                        "description": "rotation ∈ [-0.2, 0.2]（逆时针为正, 顺时针为负）(The rotation ∈ [-0.2, 0.2] (counterclockwise is positive, clockwise is negative))",
                    },
                    "step": {
                        "type": "number",
                        "description": "旋转时step默认为4(The number of steps to move. When rotating, the default step is 4)",
                    },
                },
                "required": ["direction", "rotation", "step"]
            }
        }
    },
    {
        "type": "function",
        "function": {
            "name": "obj_box_detect",
            "description": "获取目标对应在图像上的像素位置，并且用方框进行框选和定位，常用于辅助机器人进行追踪进行物体定位(Get the target corresponding pixels in the image, and use the box to select and locate it. It is often used for robotic tracking and grasping positioning to help locate objects.)",
            "parameters": {
                "type": "object",
                "properties": {
                    "user_query": {
                        "type": "string",
                        "description": "需要追踪非指定颜色的物体的需求(The demand for tracking objects that are not specified color)",
                    }
                },
                "required": ["user_query"]
            }
        }
    },
    {
        "type": "function",
        "function": {
            "name": "object_track",
            "description": "命令机器人追踪目标物体，注意：只有是确定非指定颜色的物体追踪时才进行调用。需要获取目标在图像上的像素方框位置(Order the robot to track the target object, note: only when it is sure to track the object to call. Need to get the pixel square position of the target on the graph line.)",
            "parameters": {
                "type": "object",
                "properties": {
                    "box": {
                        "type": "string",
                        "description": "目标物体的方框(The square of the target object)",
                    }
                },
                "required": ["box"]
            }
        }
    },
    {
        "type": "function",
        "function": {
            "name": "arm_transport_function",
            "description": "常用于机械臂对物品进行夹取或者将夹取的东西进行放置的功能函数，需要仔细揣摩客户的用途，调用此函数时，机器人会先通过视觉模型找到指定物体，然后进行追踪并夹取,如果需要夹取饮料,默认夹取体积最小的或者红色的饮料,不要夹黑色的饮料（Commonly used for the robotic arm to pick up items or place the picked items. When calling this function, the robot will first find the specified object through the visual model, then track and pick it up.)",
            "parameters": {
                "type": "object",
                "properties": {
                    "object_name": {
                        "type": "string",
                        "description": "需要夹取的物体名称(The name of the object to be picked)",
                    },
                    "action": {
                        "type": "string",
                        "description": "进行夹取还是放置,夹取的话就是pick,放置的话就是place",
                        "enum": ["pick", "place"]
                    }
                },
                "required": ["object_name","action"]
            }
        }
    },
]


#xy,rpy,m,deg

position_dict = {
    "动物园(Zoo)": [-0.782467, 1.1869, 0.0, 0.0, 0.00402482],
    "足球场(Football Field)": [0.822368, 0.756738, 0.0, 0.0, -0.061],
    "超市(Store)": [-0.540225, -0.89297, 0.0, 0.0, 0.066667],
    "家(Home)": [0.943655, -0.534877, 0.0, 0.0, -0.006],
    "起点(Starting Point)": [-0.129831, 0.0669761, 0.0, 0.0, 0.0],
}


if os.environ.get('ASR_LANGUAGE') == 'English':
    content_string = textwrap.dedent("""
        # Role setting
        You are a real interactive robot, need to execute tasks according to user instructions,
        and interact with users in a friendly way, just like chatting with friends.
        ## Workflow
        1. **Task Planning:** Before starting a task, you need to break it down and plan it. 
        The steps will be presented in a numbered format, with each number representing an independent step.
        2. **Tool Usage:** Before each tool is used, you need to provide an explanation, no more than 20 words, 
        describing the feedback in a humorous and varied way to make the communication process more engaging.
        3. **Feedback Processing:** After the tool is used, you need to follow up with a commentary on the feedback results, 
        no more than 20 words, describing the feedback in a humorous and varied way to make the communication process more engaging.
        4. **Task Completion:** After all task steps have been completed, provide a summary explanation, no more than 40 words.
        5. ** Answer in english
    """) 

    PROMPT = '''
    As an image recognition expert, your capability is to accurately locate objects in images sent by users through object detection, and output the final results according to the "Output Format".
    ## 1. Understand User Instructions
    I will give you a sentence. You need to make the best decision based on my words and extract the "object name" from the decision. **The name corresponding to the object must be in English**, **do not output objects that are not mentioned**.
    ## 2. Understand the Image
    I will give you an image. Analyze the image and identify all recognizable objects within it.
    ## 3.  Answer in english
    For each identified object, calculate the center point coordinates of the object. **Do not output objects that are not mentioned**.
    【Special Note】: Deeply understand the positional relationships of objects.
    ## Output Format (Please only output the following content, do not say any extra words)
    [
    {
    "object": name_1,
    "center_xy": [center_x_1, center_y_1]
    },
    {
    "object": name_2,
    "center_xy": [center_x_2, center_y_2]
    }
    ]
    '''

    OBJ_TRACK_PROMPT = '''
    As an intelligent vehicle, skilled in image recognition, your capability is to accurately locate objects in images sent by users through object detection, output the final results according to the "Output Format", and then perform tracking.
    ## 1. Understand User Instructions
    I will give you a sentence. You need to extract the "object name" from my words. **The name corresponding to the object must be in English**, **do not output objects that are not mentioned**.
    ## 2. Understand the Image
    ## 3. Answer in english
    I will give you an image. From this image, find the pixel coordinates of the top-left and bottom-right corners of the object corresponding to the "object name". If not found, then xyxy should be []. **Do not output objects that are not mentioned**.
    【Special Note】: Deeply understand the positional relationships of objects. The response needs to combine the user's instruction and the detection results.
    ## Output Format (Please only output the following content, do not say any extra words)
    {
        "object": "name", 
        "xyxy": [xmin, ymin, xmax, ymax]
    }
    '''


    OBJ_DISTANCE_DETECT = '''
    You are an intelligent vehicle skilled in image recognition. Your capability is to detect and localize objects in images provided by users, and output the final results according to the "Output Format".
    ## 1. Understand User Instructions
    I will give you a sentence, and you need to extract the "object names" from my words. **The object names should be in English**, **do not output objects not mentioned**
    ## 2. Understand the Image
    I will give you an image. From this image, find the pixel coordinates of the top-left and bottom-right corners of the object(s) corresponding to the "object names"; if not found, set xyxy to []. **Do not output objects not mentioned**
    【Special Attention】: Deeply understand the positional relationships of objects. The response should combine user instructions with detection results.
    ## Output Format (Please output only the following content, do not say anything extra)
    [
        {
            "object": "name1", 
            "xyxy": [xmin, ymin, xmax, ymax]
        },
        {
            "object": "name2", 
            "xyxy": [xmin, ymin, xmax, ymax]
        },
        ... # Can output multiple targets
    ]
    If no targets are detected, output: []
    '''


else:
    content_string = textwrap.dedent("""
        # 角色设定
        你是一个风趣幽默的机器人助手，用第一人称与用户亲切交流，就像和朋友聊天一样自然。
        
        # 核心规则（必须遵守）
        1. 在执行任何工具调用前，必须先给我一段简短风趣的提示文字
        2. 工具调用完成后，必须给我一段简短风趣的结果说明
        3. 所有给我的文字回复都要保持轻松有趣的风格
        4. 如果任务没有需要执行工具，同样的也要用一小段文字返回
        
        # 工作流程
	        1. **任务规划**：先简要说明你的行动计划（10-20字），风格要风趣幽默
	        2. **调用工具**：每次调用工具前必须给我提示（10-20字），说明你要做什么
	        3. **处理反馈**：工具执行后必须给我结果说明（10-20字），分享进展或趣事
	        4. **任务总结**：完成后进行风趣总结（10-15字）
        
        # 重要提醒
        - 每次工具调用前后都必须给我文字回复
        - 保持对话的连贯性和趣味性
    """)

    PROMPT = '''
    你作为图像识别专家，你的能力是将用户发来的图片进行目标检测精准定位，并按「输出格式」进行最后结果的输出。
    ## 1. 理解用户指令
    我会给你一句话，你需要根据我的话做出最佳决策，从做出的决策中提取「物体名称」, **object对应的name要用英文表示**, **不要输出没有提及到的物体**
    ## 2. 理解图片
    我会给你一张图, 分析图片，找出其中所有可识别的物体。
    对于每一个被识别出的物体，并计算出物体的中心点坐标,**不要输出没有提及到的物体**
    【特别注意】： 要深刻理解物体的方位关系
    ## 输出格式（请仅输出以下内容，不要说任何多余的话)
    [
    {
    "object": name_1,
    "center_xy": [center_x_1, center_y_1]
    },
    {
    "object": name_2,
    "center_xy": [center_x_2, center_y_2]
    }
    ]
    '''

    OBJ_TRACK_PROMPT = '''
    你作为智能机器人，善于图像识别，你的能力是将用户发来的图片进行目标检测精准定位，并按「输出格式」进行最后结果的输出，然后进行跟随。
    ## 1. 理解用户指令
    我会给你一句话，你需要根据我的话中提取「物体名称」。 **object对应的name要用英文表示**, **不要输出没有提及到的物体**
    ## 2. 理解图片
    我会给你一张图, 从这张图中找到「物体名称」对应物体的左上角和右下角的像素坐标; 如果没有找到，那xyxy为[]。**不要输出没有提及到的物体**
    【特别注意】： 要深刻理解物体的方位关系, response需要结合用户指令和检测的结果进行回答
    ## 输出格式（请仅输出以下内容，不要说任何多余的话)
    {
        "object": "name", 
        "xyxy": [xmin, ymin, xmax, ymax]
    }
    '''


    OBJ_DISTANCE_DETECT = '''
    你作为智能机器人，善于图像识别，你的能力是将用户发来的图片进行框选和定位，并按「输出格式」进行最后结果的输出。
    ## 1. 理解用户指令
    我会给你一句话，你需要根据我的话中提取「物体名称」。 **object对应的name要用英文表示**, **不要输出没有提及到的物体**
    ## 2. 理解图片
    我会给你一张图, 从这张图中找到「物体名称」对应物体的左上角和右下角的像素坐标; 如果没有找到，那xyxy为[]。**不要输出没有提及到的物体**
    【特别注意】： 要深刻理解物体的方位关系, response需要结合用户指令和检测的结果进行回答
    ## 输出格式（请仅输出以下内容，不要说任何多余的话)
    [
        {
            "object": "name1", 
            "xyxy": [xmin, ymin, xmax, ymax]
        },
        {
            "object": "name2", 
            "xyxy": [xmin, ymin, xmax, ymax]
        },
        ... # 可以输出多个目标
    ]
    如果未检测到任何目标，输出：[]
    '''


class LogColors:
    RESET = '\033[0m'
    BOLD = '\033[1m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    GREEN = '\033[92m'
    CYAN = '\033[96m'

class LLMControlMove(Node):
    hand2cam_tf_matrix = [
        [0.0, 0.0, 1.0, -0.101],
        [-1.0, 0.0, 0.0, 0.018],
        [0.0, -1.0, 0.0, 0.045],
        [0.0, 0.0, 0.0, 1.0]
    ]

    def __init__(self, name):
        super().__init__(name)

        self.initialize_variables() # 初始化变量(init variables)
        self.setup_ros_components() # 设置ROS节点(init node)
        self.setup_services_and_clients() # 设置ROS服务客户端(init services and clients)
        self.setup_subs_and_pubs() # 设置ROS订阅者与发布者(init subs and pubs)
        self.setup_timers() # 设置ROS定时器(init timers)

    def initialize_variables(self):
        """初始化所有类变量(init all class variables)"""
        self.language = os.environ.get('ASR_LANGUAGE')
        self.tools = []
        self.vllm_result = ''
        self.current_pose = None
        self.saved_pose = None
        self.obstacle_detected = False
        self.bridge = CvBridge()

        self.action = []
        self.llm_result = ''
        self.running = True
        self.interrupt = False
        self.action_finish = False
        self.play_audio_finish = False
        self.is_task_running = False
        self.reach_goal = False

        # self.machine_type = os.environ.get('MACHINE_TYPE', '')
        self.camera_type = os.environ.get('DEPTH_CAMERA_TYPE', '')

        self.cb_group = ReentrantCallbackGroup()

        # 巡线相关变量(line_following)
        self.line_following_count = 0
        self.line_following_start = False
        self.current_linear_x = 0.0
        self.current_angular_z = 0.0

        # 普通物体分类相关变量(nomal_object_classification)
        self.obj_move_finish = False
        self.obj_pick_start = False
        self.obj_place_start = False
        self.obj_place_finish = False

        # 颜色拾取追踪相关变量(color_tracking)
        self.lab_data = common.get_yaml_data("/home/ubuntu/software/lab_tool/lab_config.yaml")
        self.image_proc_size = (320, 240)
        self.start_color_pick = False
        self.start_place = False


        self.yaw_pid = PID(P=0.005, I=0, D=0.000)
        self.linear_pid = PID(P=0.0028, I=0, D=0)
        self.angular_pid = PID(P=0.005, I=0, D=0)

        self.color_transport_stop = False
        self.linear_base_speed = 0.007
        self.angular_base_speed = 0.03
        self.stop_single = False
        self.pick = False
        self.place = False
        self.status = "approach"
        self.endpoint = None

        # 线程安全锁(lock)
        self.draw_lock = threading.Lock()
        self.color_detect_box = None
        self.color_detect_center = None
        self.draw_flag = False
        
        # 物体追踪相关变量(track)
        self.start_track = False
        self.track_box_p1 = None
        self.track_box_p2 = None
        self.object_detect_box = False
        self.object_mode = ''
        self.get_box_flag = False
        self.box = None
        self.box_count = 0

        # 夹取类别
        self.arm_transport_box = []

        # 图像队列(queue)
        self.image_pair_queue = queue.Queue(maxsize=2)
        self.image_queue = queue.Queue(maxsize=2)


        self.config_path = '/home/ubuntu/ros2_ws/src/large_models_examples/config/automatic_pick_roi.yaml'

        self.declare_parameter('transport_debug', 'none')
        self.transport_debug = self.get_parameter('transport_debug').value
        self.arm_transport_param_init_function()
        self.detect_count = 0
        self.start_debug = False
        self.start_pick = False
        self.start_place = False
        self.place_finish = False
        self.last_transprot_action = ''

        self.declare_parameter('use_depth_cam', True)
        self.use_depth_cam = self.get_parameter('use_depth_cam').value
        if self.use_depth_cam:
            self.camera = 'depth_cam'
            self.image_topic = '/%s/rgb/image_raw' % self.camera
            self.depth_topic = '/%s/depth/image_raw' % self.camera
            self.info_topic = '/%s/depth/camera_info' % self.camera

        self.bridge_box = CvBridge()
        self.destination = ''
        self.interface_box = []
        self.interface_box_list = []

        if self.language == 'Chinese':
            self.vllm_model_name = 'qwen-vl-max-latest'
        elif self.language == 'English':
            self.vllm_model_name = vllm_model 

    def transport_finished_callback(self, msg):
        self.transport_finished = msg.data

    def setup_ros_components(self):
        """设置ROS2组件(Setup ROS2 components)"""
        # 物体追踪器(ObjectTracker)
        self.track = ObjectTracker(use_mouse=False, automatic=True, log=self.get_logger())
        # PID参数(PID parameters)
        if self.camera != 'usb_cam' :
            self.pid_params = {
                'kp1': 0.01, 'ki1': 0.0, 'kd1': 0.00,
                'kp2': 0.002, 'ki2': 0.0, 'kd2': 0.0,
            }

        else:
            self.pid_params = {
                'kp1': 0.03, 'ki1': 0.0, 'kd1': 0.00,
                'kp2': 0.003, 'ki2': 0.0, 'kd2': 0.0,
        }      
        for param_name, default_value in self.pid_params.items():
            self.declare_parameter(param_name, default_value)
            self.pid_params[param_name] = self.get_parameter(param_name).value

        self.track.update_pid(
            [self.pid_params['kp1'], self.pid_params['ki1'], self.pid_params['kd1']],
            [self.pid_params['kp2'], self.pid_params['ki2'], self.pid_params['kd2']]
        )

        # 物体夹取(Object Picking)
        self.obj_pick_center_x = 320
        self.obj_pick_center_y = 300

    def setup_services_and_clients(self):
        """设置服务和服务客户端(Set Services and Clients)"""
        # LLM相关客户端(LLM-related clients)
        if self.language == 'English':
            self.client = speech.OpenAIAPI(vllm_api_key, vllm_base_url)
        else:
            self.client = speech.OpenAIAPI(api_key, base_url)

        self.set_tool_client = self.create_client(SetTools, 'agent_process/set_tool')
        self.set_model_client = self.create_client(SetModel, 'agent_process/set_model')
        self.set_prompt_client = self.create_client(SetString, 'agent_process/set_prompt')

        # 导航客户端(Navigation clients)
        self.set_pose_client = self.create_client(SetPose2D, 'navigation_controller/set_pose')
        
        # 巡线客户端(Line following clients)
        self.line_follower_enter_client = self.create_client(Trigger, 'line_following/enter')
        self.line_follower_exit_client = self.create_client(Trigger, 'line_following/exit')
        self.line_follower_start_client = self.create_client(SetBool, 'line_following/set_running')
        self.line_follower_set_target_client = self.create_client(SetColor, 'line_following/set_color')

        
        self.get_current_pose_client = self.create_client(GetRobotPose, '/arm_kinematics/get_current_pose')
        self.get_current_pose_client.wait_for_service()
        self.set_pose_target_client = self.create_client(SetRobotPose, '/arm_kinematics/set_pose_target')
        self.set_pose_target_client.wait_for_service()

        # 语音唤醒客户端(Voice wakeup clients)
        self.awake_client = self.create_client(SetBool, 'vocal_detect/enable_wakeup')


    def setup_subs_and_pubs(self):
        """设置订阅者和发布者(Setup subscribers and publishers)"""
        # LLM相关(LLM related)
        self.create_subscription(Tools, 'agent_process/tools', self.tools_callback, 1, callback_group=self.cb_group)
        self.create_subscription(String, 'agent_process/result', self.llm_result_callback, 1)
        self.tools_result_pub = self.create_publisher(Tools, 'agent_process/tools_result', 1)

        # 导航相关(Navigation related)
        qos_profile = QoSProfile(depth=10)
        qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos_profile.reliability = QoSReliabilityPolicy.RELIABLE
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_pose_callback, qos_profile)
        self.create_subscription(Bool, 'navigation_controller/reach_goal', self.reach_goal_callback, 1, callback_group=self.cb_group)

        # 语音相关(Voice related)
        self.create_subscription(Bool, 'vocal_detect/wakeup', self.wakeup_callback, 1, callback_group=self.cb_group)
        self.tts_text_pub = self.create_publisher(String, 'tts_node/tts_text', 1)
        self.create_subscription(Bool, 'tts_node/play_finish', self.play_audio_finish_callback, 1, callback_group=self.cb_group)

        # 底盘控制(Chassis control)
        self.cmd_vel_pub = self.create_publisher(Twist, '/controller/cmd_vel', 1)

        # 巡线相关(Line following)
        self.cmd_vel_subscription = self.create_subscription(Twist,'/controller/cmd_vel',self.cmd_vel_callback,10)

        # 机械臂(Robot arm)
        self.joints_pub = self.create_publisher(ServosPosition, 'servo_controller', 1)
        self.agc = ActionGroupController(
            self.joints_pub, 
            '/home/ubuntu/software/actionset_editor/ActionGroups'
        )
        self.controller = ControllerClient()
        # 服务(Service)
        self.transport_mode = self.create_service(SetString,'~/transport_mode',self.transport_mode_callback)
        self.transport_target_color = self.create_service(SetString,'~/transport_color',self.transport_color_callback)

        # 图像发布(Image publisher)
        self.result_image_publisher = self.create_publisher(Image, '~/result_image', 10)

        # 图像同步(Image synchronizer)
        if self.camera != 'usb_cam':
            image_sub = message_filters.Subscriber(self, Image, self.image_topic)
            depth_sub = message_filters.Subscriber(self, Image, self.depth_topic)
            info_sub = message_filters.Subscriber(self, CameraInfo, self.info_topic)
            ts = message_filters.ApproximateTimeSynchronizer([image_sub, depth_sub, info_sub], 3, 0.02)
            ts.registerCallback(self.image_sync_callback)
        else:
            self.create_subscription(Image, self.image_topic, self.image_callback, 1)
        self.buzzer_pub = self.create_publisher(BuzzerState, 'ros_robot_controller/set_buzzer', 1)

    def setup_timers(self):
        """设置定时器(Timer)"""
        self.timer = self.create_timer(0.0, self.init_process, callback_group=self.cb_group)

    def _wait_for_services(self, timeout_sec=5.0):
        """等待关键服务就绪(Wait for critical services to be available)"""
        services = [
            (self.set_model_client, 'set_model'),
            (self.set_prompt_client, 'set_prompt'),
            (self.set_tool_client, 'set_tool'),
            (self.line_follower_enter_client, 'line_follower_enter'),
            (self.line_follower_set_target_client, 'line_follower_set_target'),
            (self.line_follower_start_client, 'line_follower_start'),

        ]
        
        for client, name in services:
            if not client.wait_for_service(timeout_sec=timeout_sec):
                self.get_logger().warn(f'Service {name} not available after {timeout_sec} seconds')

    def amcl_pose_callback(self, msg):
        """处理AMCL位姿信息(Slove the pose information from AMCL)"""
        position = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation

        # 四元数转欧拉角(Convert quaternion to euler angle)
        t3 = +2.0 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        t4 = +1.0 - 2.0 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z)
        yaw_z = math.atan2(t3, t4)
        yaw_deg = math.degrees(yaw_z)

        self.current_pose = {
            "x": position.x,
            "y": position.y,
            "yaw_degrees": yaw_deg
        }

    def reach_goal_callback(self, msg):
        """到达目标回调(Arrived at goal)"""
        self.get_logger().info('Reached goal')
        self.reach_goal = msg.data

    def llm_result_callback(self, msg):
        """LLM结果回调(LLM result)"""
        self.llm_result = msg.data
        self.get_logger().info(f'{LogColors.YELLOW}{LogColors.BOLD}LLM Reply: {self.llm_result}{LogColors.RESET}')

        # 非列表响应才进行语音播报(Speak out the response if it is not a list response)
        text_to_speak = self.llm_result
        is_list_response = re.search(r'^\s*\d+\.', text_to_speak, re.MULTILINE)
        if not is_list_response:
            tts_msg = String()
            tts_msg.data = text_to_speak
            self.tts_text_pub.publish(tts_msg)

    def image_sync_callback(self, ros_image, ros_depth_image, depth_camera_info):
        """同步图像回调(Callback of synchronized images)"""
        try:
            bgr_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
            depth_image = np.ndarray(
                shape=(ros_depth_image.height, ros_depth_image.width), 
                dtype=np.uint16, 
                buffer=ros_depth_image.data
            )
            depth_camera_info = depth_camera_info

            if self.image_pair_queue.full():
                self.image_pair_queue.get()
            self.image_pair_queue.put((bgr_image, depth_image, depth_camera_info))
        except Exception as e:
            self.get_logger().error(f"Image sync error: {str(e)}")

    def image_callback(self, ros_image):
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        bgr_image = np.array(cv_image, dtype=np.uint8)
        if self.image_queue.full():
            # If the queue is full, discard the oldest image(如果队列已满，丢弃最旧的图像)
            self.image_queue.get()
        # Put the image into the queue(将图像放入队列)
        self.image_queue.put(bgr_image)

    def get_endpoint(self):
        endpoint = self.send_request(self.get_current_pose_client, GetRobotPose.Request()).pose
        self.endpoint = common.xyz_quat_to_mat([endpoint.position.x, endpoint.position.y, endpoint.position.z],
                                        [endpoint.orientation.w, endpoint.orientation.x, endpoint.orientation.y, endpoint.orientation.z])
        return self.endpoint
    

    def get_pixel_distance(self, pixel_coords_list):
        """获取像素距离(get pixel distance)"""
        try:
            _, depth_image, _ = self.image_pair_queue.get()
            distances = []
            
            for pixel_xy in pixel_coords_list:
                x, y = pixel_xy  
                pixel_depth = depth_image[y, x]
                if 0 < pixel_depth < 30000:
                    distance = round(float(pixel_depth / 1000), 3)
                else:
                    distance = 0.0
                distances.append(distance)
            return f"{distances}.cm"
        except Exception as e:
            self.get_logger().error(f"Get pixel distance error: {str(e)}")
            return "[]"

    def get_obejct_pixel(self, user_query):
        """获取物体像素位置(x, y)，返回列表(get object pixel position (x, y), return list)"""
        try:
            rgb_image, _, _ = self.image_pair_queue.get()
            vllm_result_str = self.client.vllm(user_query, rgb_image, prompt=PROMPT, model=self.vllm_model_name)

            # 提取JSON部分(extract JSON part)
            if "```json" in vllm_result_str:
                json_part = vllm_result_str.split("```json")[1].split("```")[0]
            else:
                json_part = vllm_result_str

            detected_objects = json.loads(json_part.strip())
            return str(detected_objects)
        except Exception as e:
            self.get_logger().error(f"Get object pixel error: {str(e)}")
            return "[]"

    def move_to_location(self, destination):
        """移动到指定位置(move to a specific location)"""
        self.controller.traveling(gait=-1,time=1.0, steps=0)
        time.sleep(1)
        self.controller.traveling(gait=2, direction=math.radians(180), time=1.0, steps=2)
        time.sleep(3)
        if destination not in position_dict:
            return f"移动失败：未知的目标地点 '{destination}'。(Failed to move: Unknown destination '{destination}'.)"

        self.reach_goal = False
        msg = SetPose2D.Request()
        p = position_dict[destination]
        msg.data.x = float(p[0])
        msg.data.y = float(p[1])
        msg.data.roll = p[2]
        msg.data.pitch = p[3]
        msg.data.yaw = p[4]
        
        self.send_request(self.set_pose_client, msg)
        self.get_logger().info(f"Navigation goal '{destination}' sent. Waiting for arrival...")

        # 等待到达目标
        while not self.reach_goal:
            time.sleep(0.1)
        if 'raw_material_warehouse' in destination:
            twist = Twist()
            twist.linear.x = 0.03
            twist.angular.z = 0.0
            time.sleep(1)
        if self.reach_goal:
            msg = BuzzerState()
            msg.freq = 2500
            msg.on_time = 0.1
            msg.off_time = 0.5
            msg.repeat = 2
            self.buzzer_pub.publish(msg)
            return f"已成功抵达{destination}(Success reached {destination})"
        else:
            return f"移动超时，未能到达{destination}(Failed to reach {destination})"

    def get_current_location(self):
        """获取当前位置"""
        timeout_sec = 10.0
        start_time = time.time()

        while self.current_pose is None and (time.time() - start_time) < timeout_sec:
            time.sleep(0.1)

        if self.current_pose:
            x = self.current_pose['x']
            y = self.current_pose['y']
            location_string = self.find_nearest_location(x, y, position_dict)
            return location_string
        else:
            return "抱歉，我现在还无法确定自己的位置信息。(Sorry I can't determine my location now.)"

    def find_nearest_location(self, current_x, current_y, position_dict):
        """查找最近的位置(Find the nearest location)"""
        min_distance = float('inf')
        nearest_location_name = None

        for location_name, coords in position_dict.items():
            target_x, target_y = coords[0], coords[1]
            distance = math.sqrt((current_x - target_x)**2 + (current_y - target_y)**2)
            
            if distance < min_distance:
                min_distance = distance
                nearest_location_name = location_name

        if nearest_location_name:
            if min_distance < 0.2:
                return f'我现在在{nearest_location_name}(I am at {nearest_location_name})'
            else:
                return f'我现在在{nearest_location_name}附近(I am near {nearest_location_name})'
        else:
            return '我现在暂时不知道在哪里(I am not sure where I am now)'

    def describe_current_view(self, question):
        """描述当前视图(Describe the current view)"""
        try:

            if self.camera != 'usb_cam':
                rgb_image, _, _ = self.image_pair_queue.get(block=True)
            else:
                rgb_image = self.image_queue.get(block=True)
            if self.language == 'Chinese':
                VLLM_PROMPT = textwrap.dedent(f"""
                作为我的机器人管家，请仔细观察摄像头捕捉到的画面，并根据以下问题给出一个简洁、人性化的回答。
                不要进行反问，字数在10到40字之间。
                注意有以下几点：
                1、不要识别木质托盘或者圆形台子,如果看到有的话，不要返回相关关于它的信息（一定要注意）
                2、巴黎铁塔的颜色看到的一律被设定为古铜色
                问题是："{question}
            """)
                description = self.client.vllm(question, rgb_image, prompt=VLLM_PROMPT, model=self.vllm_model_name)
            else:
                VLLM_PROMPT = textwrap.dedent(f"""
                To be my robot butler, please observe the image captured by camera carefully. 
                Please give a concise and humanized answer to the following question. 
                Don't ask any questions back. The length of your answer should be between 10 to 40 words.
                "{question}
            """)
                description = self.client.vllm(question, rgb_image, prompt=VLLM_PROMPT, model=self.vllm_model_name)

            self.get_logger().info(f'{LogColors.YELLOW}{LogColors.BOLD}LLM Reply: {description}{LogColors.RESET}')

            # 直接发布到TTS进行语音播报(publish to tts)
            tts_msg = String()
            tts_msg.data = description
            self.tts_text_pub.publish(tts_msg)

            return f"画面描述任务已成功执行。得到的结果是{description}(The description of the view has been successfully executed. The result is {description})"

        except Exception as e:
            self.get_logger().error(f"Describe current view error: {str(e)}")
            return "无法描述当前画面(Unable to describe the current view)"

    def line_following(self, color):
        """巡线功能(Line following function)"""
        self.line_follower_enter_client.call_async(Trigger.Request())
        self.line_following_start = True

        # 设置目标颜色(Set target color)
        color_msg = SetColor.Request()
        color_msg.data = color
        self.line_follower_set_target_client.call_async(color_msg)

        # 启动巡线(Start line following)
        start_msg = SetBool.Request()
        start_msg.data = True
        self.line_follower_start_client.call_async(start_msg)

        self.is_task_running = True
        return f"好的，马上开始沿着{color}线行驶。(Okay, I will follow the {color} line.)"

    def cmd_vel_callback(self, msg):
        """cmd_vel话题回调函数(cmd_vel topic callback function)"""
        if self.line_following_start:
            self.current_linear_x = msg.linear.x
            self.current_angular_z = msg.angular.z


    def lidar_scan_detect(self, scan_detect):
        """激光雷达障碍物检测(LiDAR obstacle detection)"""
        self.get_logger().info(f'{LogColors.GREEN}{LogColors.BOLD}start lidar_scan_detect{LogColors.RESET}')
        time.sleep(2)

        stop_distance = 0.3
        self.current_linear_x = 0.0
        self.current_angular_z = 0.0
        while self.line_following_start:
            if abs(self.current_linear_x) < 0.001 and abs(self.current_angular_z) < 0.001:
                if self.line_following_count < 500:
                    self.line_following_count += 1
                    if self.line_following_count % 20 == 0:
                        self.get_logger().info(f'{LogColors.GREEN}{LogColors.BOLD}Detected zero velocity{LogColors.RESET}')
                else:
                    start_msg = SetBool.Request()
                    start_msg.data = False
                    self.line_follower_start_client.call_async(start_msg) 
                    self.line_following_count = 0
                    time.sleep(2)
                    self.line_follower_exit_client.call_async(Trigger.Request())
                    time.sleep(1)
                    return '检测到速度为0,已停止巡线(Detected zero velocity, stop line following)'
            else:
                self.line_following_count = 0

            time.sleep(0.01)

        if not self.line_following_start:
            return '已停止巡线(Stop line following)'
        else:
            return '巡线检测超时(Line following timeout)'

    def left_callback(self, msg):
        """左侧距离回调(Left distance callback)"""
        if self.line_following_start:
            self.current_left = msg.data

    def right_callback(self, msg):
        """右侧距离回调(Right distance callback)"""
        if self.line_following_start:
            self.current_right = msg.data


    def get_object_box_distance(self, user_query):
        """
        获取目标物体的坐标和距离(Get the coordinates and distance of the target object)
        """
        if self.camera != 'usb_cam':
            rgb_image, depth_image, _ = self.image_pair_queue.get(block=True)
        else:
            return '机器人需要切换深度摄像头才能执行这个功能，请重新传达指令吧(The robot needs to switch the depth camera before executing this function, please re-transmit the order)'

        vllm_result_str = self.client.vllm(user_query, rgb_image, prompt=OBJ_DISTANCE_DETECT, model=self.vllm_model_name)

        if "```json" in vllm_result_str:
            json_part = vllm_result_str.split("```json")[1].split("```")[0]
        else:
            json_part = vllm_result_str

        try:
            detected_objects = json.loads(json_part.strip())
        except json.JSONDecodeError as e:
            self.get_logger().error(f"JSON解析错误(JSON Error): {e}, 原始响应(Origin message): {vllm_result_str}")
            return "无法解析检测结果(No detected objects)"
        
        self.get_box_flag = True
        
        # 初始化 interface_box_list 为列表
        if not hasattr(self, 'interface_box_list'):
            self.interface_box_list = []
        else:
            self.interface_box_list.clear()  # 清空之前的检测结果
        
        distances = []
        
        if isinstance(detected_objects, list) and len(detected_objects) > 0:
            for obj in detected_objects:
                if 'xyxy' in obj and len(obj['xyxy']) == 4 and 'object' in obj:
                    xmin, ymin, xmax, ymax = obj['xyxy']
                    box = [xmin, ymin, xmax, ymax]
                    object_name = obj['object']
                    
                    # 存储物体名称和框坐标信息(Save the object name and box coordinates information)
                    self.interface_box_list.append({
                        "name": object_name,
                        "box": box
                    })
                    
                    # 计算距离(Calculate the distance)
                    if self.camera != 'usb_cam':  # 如果有深度图像(If there is a depth image)
                        roi = depth_image[ymin+1:ymax-1, xmin+1:xmax-1]
                        valid_depths = roi[(roi > 0) & (roi < 30000)]
                        if len(valid_depths) > 0:
                            avg_depth = np.mean(valid_depths)
                            distance = round(float(avg_depth / 1000), 3)
                            distances.append(f"{object_name}: {distance}m")
                        else:
                            distances.append(f"{object_name}:error to detect")
                    else:
                        distances.append(f"{object_name}: need depthc cam Camera")
            
            # self.get_logger().info(f"Detect {len(detected_objects)} : {distances}")
            # self.get_logger().info(f"Detect Information {self.interface_box_list}")
            
            # 设置绘制标志(Set draw flag)
            with self.draw_lock:
                self.draw_flag = True
            
            # 返回所有目标的距离信息(Return all target distances)
            if distances:
                return ", ".join(distances)
            else:
                return "未检测到有效目标(No valid target detected)"
        else:
            self.get_logger().info("未检测到任何目标(No target detected)")
            return "未检测到目标(No target detected)"



    def robot_move_control(self, direction, rotation, step):
        """机器人移动控制(Robot move control)"""
        if rotation != 0.0:
            self.controller.traveling(gait=2, stride=0.0, height=30.0, direction=direction, rotation=rotation, time=1.0, steps=step, relative_height=True, interrupt=True )
            time.sleep(1)
        else:
            self.controller.traveling(gait=2, stride=45.0, height=30.0, direction=direction, rotation=rotation, time=1.0, steps=step, relative_height=True, interrupt=True )
            time.sleep(1)

        self.is_task_running = True


        end_time = time.time() + float(step)  # 持续时间(duration)
        while time.time() < end_time and self.is_task_running:
            time.sleep(0.05)

        return "move finished"

    def obj_box_detect(self, user_query):
        """获取物体边界框(Get object bounding box)"""
        self.get_logger().info(f"ObjTracking: {user_query}")
        time.sleep(3)
        box_message = self.box_find(user_query)
        return box_message

    def obj_pick_box_detect(self, user_query):
        """获取物体边界框(Get object accelerated bounding box)"""
        self.get_logger().info(f"ObjTracking: {user_query}")
        self.arm_transport_pick()
        time.sleep(3)
        box_message = self.box_find(user_query)
        return box_message

    def box_find(self,user_query):
        try:
            if self.camera != 'usb_cam':
                rgb_image, _, _ = self.image_pair_queue.get(block=True)
            else:
                rgb_image = self.image_queue.get(block=True)
            vllm_result_str = self.client.vllm(user_query, rgb_image, prompt=OBJ_TRACK_PROMPT, model=self.vllm_model_name)

            if "```json" in vllm_result_str:
                json_part = vllm_result_str.split("```json")[1].split("```")[0]
            else:
                json_part = vllm_result_str

            detected_objects = json.loads(json_part.strip())
            self.get_box_flag = True

            if 'xyxy' in detected_objects:
                self.box = detected_objects['xyxy']
                self.get_logger().info('Detected objects: %s' % str(self.box))
                self.is_task_running = True

            return f'已经找到了物体的位置(The object location has been found.)' + str(detected_objects)
        except Exception as e:
            self.get_logger().error(f"Get object box error: {str(e)}")
            return "{}"

    def object_track(self, box):
        """物体追踪(Object Tracking)"""
        self.get_logger().info('Object Tracking: %s' % str(box))
        with self.draw_lock:
            self.draw_flag = True
            self.box = ast.literal_efval(box)

        self.object_detect_box = True
        self.object_mode = 'track'
        return 'Start Tracking'


    def pick_up(self, position):
        self.get_logger().info('Object Tracking:1')

        if position[2] < 0.1:
            yaw = 80
        else:
            yaw = 30
        position[0] -= 0.01
        position[2] -= 0.01
        msg = set_pose_target(position, yaw, [-180.0, 180.0], 1.0)
        res = self.send_request(self.set_pose_target_client, msg)
        if res.pulse:
            servo_data = res.pulse
            set_servo_position(self.joints_pub, 1, ((19, servo_data[0]), ))
            time.sleep(1)
            set_servo_position(self.joints_pub, 1.5, ((19, servo_data[0]),(20, servo_data[1]), (21, servo_data[2]),(22, servo_data[3]), (23, servo_data[4])))
            time.sleep(1.5)
            set_servo_position(self.joints_pub, 0.5, ((24, 500),))
            time.sleep(1)
        else:
            self.get_logger().info('Object Tracking:888'+str(res))
        self.get_logger().info('Object Tracking:2')

        set_servo_position(self.joints_pub, 1.5, ((19, 500), (20, 700), (21, 155), (22, 70), (23, 500), (24, 500)))
        time.sleep(1.5)  
        if self.name == "足球":
            time.sleep(0.2)
        else:
            time.sleep(15)  

        self.get_logger().info('Object Tracking:3')

    def pick_handle(self, image):
        """拾取处理(Pcick)"""
        twist = Twist()
        if not self.pick:
            object_center_x, object_center_y, object_angle, box = self.get_track_position()
            if object_center_x > 0:
                self.pick = True
                self.color_detect_center = [object_center_x, object_center_y]

                self.get_logger().info(f"Target locked at: {object_center_x}, {object_center_y}")

            else:
                # 如果没找到，可以保持原地等待或轻微搜索，这里选择继续循环
                pass
            # if self.transport_debug == 'pick':
            #     self.detect_count += 1
            #     if self.detect_count > 10:
            #         self.detect_count = 0
            #         self.pick_stop_y = object_center_y
            #         self.pick_stop_x = object_center_x
            #         data = common.get_yaml_data(self.config_path)
            #         data['/**']['ros__parameters']['pick_stop_pixel_coordinate'] = [self.pick_stop_x, self.pick_stop_y]
            #         common.save_yaml_data(data, self.config_path)
            #         self.transport_debug = 'none'
            #     self.get_logger().info('x_y: ' + str([object_center_x, object_center_y]))  # Print the pixel of the current object's center(打印当前物体中心的像素)
            # elif object_center_x > 0:
            #     # 线性PID控制(Linear PID control)
            #     self.linear_pid.SetPoint = self.pick_stop_y
            #     # self.get_logger().info(f' object_center_y >>>>{object_center_y}')
            #     # self.get_logger().info(f' y >>>>{abs(object_center_y - self.pick_stop_y)}')
            #     if abs(object_center_y - self.pick_stop_y) <= self.d_y:
            #         object_center_y = self.pick_stop_y

            #     if self.status != "align":
            #         self.linear_pid.update(object_center_y)  # Update PID(更新pid)
            #         output = self.linear_pid.output
            #         tmp = math.copysign(self.linear_base_speed, output) + output
            #         # self.get_logger().info(f'{tmp}')
            #         self.linear_speed = tmp/10
            #         if tmp > 0.1:
            #             self.linear_speed = 0.01
            #         if tmp < -0.1:
            #             self.linear_speed = -0.01
            #         if abs(tmp) <= 0.0075:
            #             self.linear_speed = 0

            #     # 角速度PID控制(angular velocity PID control)
            #     self.angular_pid.SetPoint = self.pick_stop_x
            #     # self.get_logger().info(f' object_center_x >>>>{object_center_x}')
            #     # self.get_logger().info(f' x >>>>{abs(object_center_x - self.pick_stop_x)}')
            #     if abs(object_center_x - self.pick_stop_x) <= self.d_x:
            #         object_center_x = self.pick_stop_x

            #     if self.status != "align":
            #         self.angular_pid.update(object_center_x)  # Update PID(更新pid)
            #         output = self.angular_pid.output
            #         tmp = math.copysign(self.angular_base_speed, output) + output

            #         self.angular_speed = tmp/5
            #         if tmp > 1.0:
            #             self.angular_speed = 0.2
            #         if tmp < -1.0:
            #             self.angular_speed = -0.2
            #         if abs(tmp) <= 0.038:
            #             self.angular_speed = 0
            #     if abs(self.linear_speed) == 0 and abs(self.angular_speed) == 0:
            #         self.count_turn += 1
            #         if self.count_turn > 5:
            #             self.count_turn = 5
            #             self.status = "align"
            #             if self.count_stop < 10:  # If there is no movement detected for 10 consecutive times(连续10次都没在移动)
            #                 if object_angle < 40: # Do not use 45, because unstable values at 45 may cause repeated movement(不取45，因为如果在45时值的不稳定会导致反复移动)
            #                     object_angle += 90
            #                 self.yaw_pid.SetPoint = 90
            #                 if abs(object_angle - 90) <= 1:
            #                     object_angle = 90
            #                 self.yaw_pid.update(object_angle)  # Update PID(更新pid)
            #                 self.yaw_angle = self.yaw_pid.output
            #                 if object_angle != 90:
            #                     if abs(self.yaw_angle) <=0.038:
            #                         self.count_stop += 1
            #                     else:
            #                         self.count_stop = 0
            #                     twist.linear.y = float(-2 * 0.3 * math.sin(self.yaw_angle / 15))
            #                     twist.angular.z = float(self.yaw_angle/8)
            #                 else:
            #                     self.count_stop += 1
            #             elif self.count_stop <= 20:
            #                 self.d_x = 30
            #                 self.d_y = 30
            #                 self.count_stop += 1
            #                 self.status = "adjust"
            #             else:
            #                 self.count_stop = 0
            #                 self.pick = True
            #     else:
            #         if self.count_stop >= 10:
            #             self.count_stop = 10
            #         self.count_turn = 0
            #         if self.status != 'align':
            #             twist.linear.x = float(self.linear_speed)
            #             twist.angular.z = float(self.angular_speed)
        self.stop_single = self.pick
        with self.draw_lock:
            if not self.stop_single:
                self.color_detect_box = box
                self.color_detect_center = [object_center_x, object_center_y]
                self.draw_flag = True
            else:
                self.draw_flag = False

        # self.cmd_vel_pub.publish(twist)
        return image, self.stop_single


    def place_handle(self, image):
        twist = Twist()
        if not self.place:
            object_center_x, object_center_y, object_angle,box = self.color_detect(image)  # Obtain the center and angle of the object color(获取物体颜色的中心和角度)
            if self.transport_debug == 'place':
                self.detect_count += 1
                if self.detect_count > 10:
                    self.detect_count = 0
                    self.place_stop_y = object_center_y
                    self.place_stop_x = object_center_x
                    data = common.get_yaml_data(self.config_path)
                    data['/**']['ros__parameters']['place_stop_pixel_coordinate'] = [self.place_stop_x, self.place_stop_y]
                    common.save_yaml_data(data, self.config_path)
                    self.transport_debug = 'none'
                self.get_logger().info('x_y: ' + str([object_center_x, object_center_y]))  # Print the pixel of the current object's center(打印当前物体中心的像素)
            elif object_center_x > 0:
                ########Motor PID processing(电机pid处理)#########
                # Use the x and y coordinates of the image's center point as the set value, and the current x and y coordinates as the input(以图像的中心点的x，y坐标作为设定的值，以当前x，y坐标作为输入)#
                self.linear_pid.SetPoint = self.place_stop_y
                if abs(object_center_y - self.place_stop_y) <= self.d_y:
                    object_center_y = self.place_stop_y
                self.linear_pid.update(object_center_y)  # Update PID(更新pid)
                output = self.linear_pid.output
                tmp = math.copysign(self.linear_base_speed, output) + output

                self.linear_speed = tmp/10
                if tmp > 0.1:
                    self.linear_speed = 0.01
                if tmp < -0.1:
                    self.linear_speed = -0.01
                if abs(tmp) <= 0.0075:
                    self.linear_speed = 0

                self.angular_pid.SetPoint = self.place_stop_x
                if abs(object_center_x - self.place_stop_x) <= self.d_x:
                    object_center_x = self.place_stop_x

                self.angular_pid.update(object_center_x)  # Update PID(更新pid)
                output = self.angular_pid.output
                tmp = math.copysign(self.angular_base_speed, output) + output

                self.angular_speed = tmp/5
                if tmp > 1.0:
                    self.angular_speed = 0.2
                if tmp < -1.0:
                    self.angular_speed = -0.2
                if abs(tmp) <= 0.035:
                    self.angular_speed = 0

                if abs(self.linear_speed) == 0 and abs(self.angular_speed) == 0:
                    self.place = True
                else:
                    twist.linear.x = float(self.linear_speed)
                    twist.angular.z = float(self.angular_speed)

        self.stop_single = self.place
        with self.draw_lock:
            if not self.stop_single:
                self.color_detect_box = box
                self.color_detect_center = [object_center_x, object_center_y]
                self.draw_flag = True
            else:
                self.draw_flag = False

        return image, self.stop_single


    def arm_transport_param_init_function(self):
        self.pick_stop_x, self.pick_stop_y = 292, 370
        self.place_stop_x, self.place_stop_y = 305, 325
        try:
            with open(self.config_path, 'r') as file:
                config_data = yaml.safe_load(file)

            if config_data:
                # 查找参数部分
                params = None
                if '/**' in config_data:
                    params = config_data['/**'].get('ros__parameters')
                elif 'ros__parameters' in config_data:
                    params = config_data['ros__parameters']
                
                if params:
                    coords = params.get('pick_stop_pixel_coordinate')
                    if coords and len(coords) >= 2:
                        self.pick_stop_x, self.pick_stop_y = coords[0], coords[1]
                    
                    coords = params.get('place_stop_pixel_coordinate')
                    if coords and len(coords) >= 2:
                        self.place_stop_x, self.place_stop_y = coords[0], coords[1]
            
            self.get_logger().info(f'Coordinates: pick({self.pick_stop_x}, {self.pick_stop_y}), place({self.place_stop_x}, {self.place_stop_y})')
            
        except Exception as e:
            self.get_logger().warning(f'Config error: {e}, using defaults')


    
    def arm_transport_pick_init(self):
        self.get_logger().info(f"Init Pick Arm")
        set_servo_position(self.joints_pub, 1.5, ((19, 500), (20, 700), (21, 155), (22, 70), (23, 500), (24, 730)))
        time.sleep(2)
        self.controller.traveling(gait=-2,time=1.0, steps=0)
        time.sleep(1)

    def arm_transport_pick(self,debug=False):
        time.sleep(0.5)
        set_servo_position(self.joints_pub, 1.5, ((19, 500), (20, 80), (21, 330), (22, 710), (23, 500), (24, 730)))
        time.sleep(2)
        set_servo_position(self.joints_pub, 1.5, ((19, 500), (20, 80), (21, 330), (22, 710), (23, 500), (24, 730)))
        time.sleep(1.5)
        if not debug:
            # set_servo_position(self.joints_pub, 0.5, ((19, 500), (20, 220), (21, 290), (22, 280), (23, 500), (24, 500)))
            # time.sleep(0.5)
            # set_servo_position(self.joints_pub, 0.5, ((19, 500), (20, 220), (21, 290), (22, 280), (23, 500), (24, 500)))
            # time.sleep(0.3)
            # set_servo_position(self.joints_pub, 1.5, ((19, 500), (20, 700), (21, 155), (22, 70), (23, 500), (24, 500)))
            # time.sleep(1.5)
            # set_servo_position(self.joints_pub, 1.5, ((19, 500), (20, 700), (21, 155), (22, 70), (23, 500), (24, 500)))
            time.sleep(0.3)
        else:
            time.sleep(5)
            self.arm_transport_pick_init()

    def arm_transport_place(self,debug=False):
        self.controller.traveling(gait=-2,time=1.0, steps=0)
        time.sleep(1)

        set_servo_position(self.joints_pub, 1.5, ((19, 500), (20, 100), (21, 470), (22, 300), (23, 500), (24, 500)))
        time.sleep(1.5)
        set_servo_position(self.joints_pub, 0.3, ((19, 500), (20, 100), (21, 470), (22, 300), (23, 500), (24, 500)))
        time.sleep(0.3)
        if not debug:
            set_servo_position(self.joints_pub, 0.5, ((19, 500), (20, 200), (21, 310), (22, 325), (23, 500), (24, 700)))
            time.sleep(0.5)
            set_servo_position(self.joints_pub, 0.3, ((19, 500), (20, 200), (21, 310), (22, 325), (23, 500), (24, 700)))
            time.sleep(0.3)
            set_servo_position(self.joints_pub, 1.5, ((19, 500), (20, 700), (21, 155), (22, 70), (23, 500), (24, 500)))
            time.sleep(1.5)
            set_servo_position(self.joints_pub, 1.5, ((19, 500), (20, 700), (21, 155), (22, 70), (23, 500), (24, 500)))
            time.sleep(0.3)
        else:
            time.sleep(5)
            self.arm_transport_pick_init()
 
    def get_track_position(self):
        """
        从追踪器获取物体中心坐标
        Get object center coordinates from the tracker
        """
        center_x, center_y, angle, box = -1, -1, 90, None # 默认角度设为90，因为包围盒没有旋转角度
        
        # 检查是否正在追踪且有有效的坐标
        if self.start_track and self.track_box_p1 is not None and self.track_box_p2 is not None:
            # 计算中心点
            x1, y1 = self.track_box_p1
            x2, y2 = self.track_box_p2
            
            center_x = int((x1 + x2) / 2)
            center_y = int((y1 + y2) / 2)
            
            # 构造Box格式以兼容后续绘图逻辑
            box = np.array([
                [x1, y1],
                [x2, y1],
                [x2, y2],
                [x1, y2]
            ], dtype=np.int32)
            
        return center_x, center_y, angle, box


    def arm_transport_function(self, object_name, action):
        """
        修改后的机械臂搬运函数，支持物体名称输入
        """
        self.get_logger().info(f"Starting transport task for object: {object_name}, Action: {action}")
        self.name = object_name
        self.pick = False
        self.place = False
        self.color_detect_center = None
 
        if action == 'pick':
            self.arm_transport_pick_init()
            
            # --- 使用 VLLM 寻找物体并获取初始 Box ---
            self.get_logger().info(f"Scanning for {object_name}...")
            # 构造 Prompt 让模型找物体
            find_query = f"Detect the {object_name}."
            # 调用 box_find 获取初始位置 (这会更新 self.box 和 self.get_box_flag)
            box_msg = self.box_find(find_query) 
            
            # 检查是否找到
            if not self.get_box_flag or self.box is None or len(self.box) == 0:
                return f"抱歉，我没有在画面中找到{object_name}。(Sorry, I cannot find {object_name}.)"
            
            # ---  启动被动追踪模式 ---
            # 我们需要 object_track_thread 计算坐标，但不要它直接发 cmd_vel
            # 因为我们将在 pick_handle 里进行精细的 PID 控制
            self.object_detect_box = True # 通知线程初始化追踪器
            self.object_mode = 'passive_track' # 设置为被动模式)
            
            time.sleep(1.0) # 等待追踪器稳定
            
        elif action == 'place':
            self.stop_single = True 
            self.get_logger().info("Direct place mode: Skipping alignment loop.")


        self.count_stop = 0
        self.count_turn = 0
        self.linear_pid.clear()
        self.angular_pid.clear()
        self.stop_single = False

        # 设置PID参数阈值
        if 'pick' == action:
            self.d_y = 10
            self.d_x = 10
            self.start_pick = True
        elif 'place' == action:
            self.d_y = 10
            self.d_x = 10
            self.start_place = True
            
        # --- 进入控制循环 ---
        while not self.stop_single:
            try:
                if self.camera != 'usb_cam':
                    rgb_image, depth_image, depth_camera_info = self.image_pair_queue.get(block=True, timeout=1)
                else:
                    rgb_image = self.image_queue.get(block=True, timeout=1)
                
                if 'pick' == action:
                    # 调用修改后的 pick_handle
                    _, stop_single = self.pick_handle(rgb_image)
                    
                elif 'place' == action:
                    self.stop_single = True 
                    # 如果放置也要改，类似处理，这里假设放置还是基于颜色的或者固定位置
                    # 如果放置需要基于追踪，需要编写 place_handle_track
                    # _, stop_single = self.place_handle(rgb_image) 

            except queue.Empty:
                if not self.running: break
                continue
            except Exception as e:
                self.get_logger().error(f"Transport loop error: {str(e)}")
                continue
        
        # 任务结束清理
        self.start_track = False # 停止追踪
        self.object_mode = ''    # 重置模式
        
        time.sleep(1)
        if 'pick' == action:

            self.get_logger().info('Pick Action!!')
            self.pick_stop_x = self.color_detect_center[0]
            self.pick_stop_y = self.color_detect_center[1]
            roi = [int(self.pick_stop_y) - 5, int(self.pick_stop_y) + 5, int(self.pick_stop_x) - 5, int(self.pick_stop_x) + 5]
                  
            roi_distance = depth_image[roi[0]:roi[1], roi[2]:roi[3]]
            
            valid_mask = (roi_distance > 0) & (roi_distance < 10000)
            if np.any(valid_mask):
                dist = round(float(roi_distance[valid_mask].mean()/1000.0), 3)
                dist += 0.015 # 物体半径补偿(object radius compensation)
                # dist += 0.035 # 误差补偿(error compensation)
                K = depth_camera_info.k
                self.get_endpoint()
                position = depth_pixel_to_camera((self.pick_stop_x, self.pick_stop_y), dist, (K[0], K[4], K[2], K[5]))
                self.get_logger().info('\033[1;32m%s\033[0m' % "position"+str(position))
                self.get_logger().info('\033[1;32m%s\033[0m' % "dist"+str(dist))

                position[0] -= 0.01  # rgb相机和深度相机tf有1cm偏移(the RGB camera and depth camera TFs have a 1cm offset)
                position[1] -= 0.035 
                position[2] += 0.02

                pose_end = np.matmul(self.hand2cam_tf_matrix, common.xyz_euler_to_mat(position, (0, 0, 0)))  # 转换的末端相对坐标(the relative coordinates at the end of the transformation)
                world_pose = np.matmul(self.endpoint, pose_end)  # 转换到机械臂世界坐标(transform into the world coordinates of the robotic arm)
                pose_t, pose_R = common.mat_to_xyz_euler(world_pose)
                self.stamp = time.time()
                self.moving = True
                self.get_logger().info('\033[1;32m%s\033[0m' % "stop"+str(pose_t))
                self.pick_up(pose_t)


            self.place_finish = False
            self.start_pick = False
            self.pick = False
        elif 'place' == action:
            self.get_logger().info('Place Action!!')
            self.arm_transport_place()
            self.place_finish = True
            self.start_place = False


        self.last_transprot_action = action
        return f'{action} {object_name} 完成! (Action {action} {object_name} finished!)'

    def color_detect(self, img):
        """颜色检测(color detection)"""
        img_h, img_w = img.shape[:2]
        frame_resize = cv2.resize(img, self.image_proc_size, interpolation=cv2.INTER_NEAREST)
        frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)
        frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)

        frame_mask = cv2.inRange(frame_lab, 
                               tuple(self.lab_data['lab']['Stereo'][self.target_color]['min']),
                               tuple(self.lab_data['lab']['Stereo'][self.target_color]['max']))

        eroded = cv2.erode(frame_mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
        dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))

        contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
        center_x, center_y, angle, box = -1, -1, -1, None

        if contours:
            areaMaxContour, area_max = common.get_area_max_contour(contours, 10)
            if areaMaxContour is not None and area_max > 10:
                rect = cv2.minAreaRect(areaMaxContour)
                angle = rect[2]
                box = np.intp(cv2.boxPoints(rect))
                
                for j in range(4):
                    box[j, 0] = int(common.val_map(box[j, 0], 0, self.image_proc_size[0], 0, img_w))
                    box[j, 1] = int(common.val_map(box[j, 1], 0, self.image_proc_size[1], 0, img_h))

                self.arm_transport_box = box
                ptime_start_x, ptime_start_y = box[0, 0], box[0, 1]
                pt3_x, pt3_y = box[2, 0], box[2, 1]
                center_x, center_y = int((ptime_start_x + pt3_x) / 2), int((ptime_start_y + pt3_y) / 2)

        return center_x, center_y, angle, box


    def get_available_locations(self):
        """获取可用位置(get available locations)"""
        self.get_logger().info("Querying available locations.")
        return json.dumps(position_dict, ensure_ascii=False, indent=2)

    def get_node_state(self, request, response):
        """获取节点状态(Get node state)"""
        return response

    def init_process(self):
        """初始化过程(Initialization process)"""
        self.timer.cancel()
        self._wait_for_services()

        # 设置模型
        msg = SetModel.Request()
        msg.model_type = 'llm_tools'
        if self.language == 'Chinese':
            msg.model = 'qwen3-max'
            msg.api_key = api_key 
            msg.base_url = base_url
        elif self.language == 'English':
            msg.model =  'qwen/qwen3-max'
            msg.api_key = vllm_api_key 
            msg.base_url = vllm_base_url
        self.send_request(self.set_model_client, msg)

        # 设置提示词(Set prompt)
        msg = SetString.Request()
        msg.data = content_string
        self.send_request(self.set_prompt_client, msg)

        # 设置工具(Set tools)
        tools_json = [json.dumps(tool, ensure_ascii=False) for tool in tools]
        msg = SetTools.Request()
        msg.tools = tools_json
        self.send_request(self.set_tool_client, msg)
        
        # 初始化机械臂(Init arm)
        set_servo_position(self.joints_pub, 1.5, ((19, 500), (20, 700), (21, 155), (22, 70), (23, 500), (24, 700)))
        time.sleep(1.5)

        # 启动处理线程(Start processing thread)
        threading.Thread(target=self.process, daemon=True).start()
        threading.Thread(target=self.display_thread, daemon=True).start()
        threading.Thread(target=self.object_track_thread, daemon=True).start()

        # 启动功能开启语音提示(Start function opening voice prompt)
        speech.play_audio(start_audio_path)

        self.create_service(Empty, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

    def send_request(self, client, msg):
        """发送请求并等待响应(Send request and wait for response)"""
        future = client.call_async(msg)
        while rclpy.ok():
            if future.done():
                try:
                    return future.result()
                except Exception as e:
                    self.get_logger().error(f"Service call failed: {str(e)}")
                    return None
            time.sleep(0.01)
        return None

    def tools_callback(self, msg):
        """工具回调(Tools callback)"""
        self.get_logger().info(f'{LogColors.GREEN}{LogColors.BOLD}AI Decision-Making:{LogColors.RESET}')
        self.get_logger().info(f'{LogColors.GREEN}{LogColors.BOLD}Tools id [{msg.id}]:{LogColors.RESET}')
        self.get_logger().info(f'{LogColors.GREEN}{LogColors.BOLD}Tools name [{msg.name}]:{LogColors.RESET}')
        self.get_logger().info(f'{LogColors.GREEN}{LogColors.BOLD}Tools data [{msg.data}]:{LogColors.RESET}')
        
        self.tools = [msg.id, msg.name, json.loads(msg.data)]

    def play_audio_finish_callback(self, msg):
        """音频播放完成回调(Audio playback finished callback)"""
        if msg.data:
            self.play_audio_finish = True
            awake_msg = SetBool.Request()
            awake_msg.data = True
            self.send_request(self.awake_client, awake_msg)

    def wakeup_callback(self, msg):
        """唤醒回调(Wakeup callback)"""
        if msg.data:
            self.get_logger().info('唤醒中断(Wakeup interrupt)')

            # 停止巡线(Stop line following)
            if self.line_following_start:
                request = SetBool.Request()
                request.data = False
                self.line_follower_start_client.call_async(request) 
                self.line_following_start = False


            # 停止运动(Stop movement)
            self.controller.traveling(gait=-1,time=1.0, steps=0)
            time.sleep(1)
            self.track.stop()
            self.is_task_running = False
            self.first_transprot = False
            with self.draw_lock:
                self.get_logger().info('Clear the information')
                self.draw_flag = False
                self.interface_box_list.clear()
                self.interface_box.clear()
                self.box = None
                self.get_box_flag = False
                self.object_detect_box = False
                self.start_track = False
                self.track_box_p1 = []
                self.track_box_p2 = []
        elif msg.data:
            self.get_logger().info('Wakeup received, but no interruptible task is running.')

    def process(self):
        """主处理循环(Main processing loop)"""
        while rclpy.ok():
            if self.tools:
                tool_id, tool_name, args_dict = self.tools
                res = None
                try:
                    if tool_name == 'describe_current_view':
                        question = args_dict.get('question')
                        if question:
                            res = self.describe_current_view(question)
                    elif tool_name == 'move_to_location':
                        destination = args_dict.get('destination')
                        if destination:
                            res = self.move_to_location(destination)
                    elif tool_name == 'get_available_locations':
                        res = self.get_available_locations()
                    elif tool_name == 'get_current_location':
                        res = self.get_current_location()
                    elif tool_name == 'line_following':
                        color = args_dict.get('color')
                        if color:
                            res = self.line_following(color)
                    elif tool_name == 'lidar_scan_detect':
                        scan_detect = args_dict.get('scan_detect')
                        if scan_detect:
                            res = self.lidar_scan_detect(scan_detect)
                    elif tool_name == 'robot_move_control':
                        if all(k in args_dict for k in ["direction", "rotation", "step"]):
                            res = self.robot_move_control(
                                direction=args_dict['direction'],
                                rotation=args_dict['rotation'],
                                step=args_dict['step'],
                        )

                    elif tool_name == 'obj_box_detect':
                        user_query = args_dict.get('user_query')
                        if user_query:
                            res = self.obj_box_detect(user_query)
                    elif tool_name == 'object_track':
                        box = args_dict.get('box')
                        if box:
                            res = self.object_track(box)
                    elif tool_name == 'color_place':
                        res = self.color_place()
                    elif tool_name == 'get_object_box_distance':
                        user_query = args_dict.get('user_query')
                        if user_query:
                            res = self.get_object_box_distance(user_query)

                    elif tool_name == 'arm_transport_function':
                        object_name = args_dict.get('object_name')
                        action = args_dict.get('action')
                        if object_name  and action:
                            res = self.arm_transport_function(object_name,action)

                    if res is not None:
                        self.tools_result_pub.publish(Tools(id=tool_id, name=tool_name, data=res))

                except Exception as e:
                    self.get_logger().error(f"Tool {tool_name} execution error: {str(e)}")
                    res = f"工具执行错误: {str(e)}(tool used failed{tool_name})"
                    self.tools_result_pub.publish(Tools(id=tool_id, name=tool_name, data=res))
                
                self.tools = []
                time.sleep(2)
            else:
                time.sleep(0.02)

    def display_thread(self):
        """显示线程(show thread)"""
        while self.running:
            try:
                if self.camera != 'usb_cam':
                    rgb_image, depth_image, _ = self.image_pair_queue.get(block=True, timeout=1)
                else:
                    rgb_image = self.image_queue.get(block=True, timeout=1)
                result_image = rgb_image.copy()
                with self.draw_lock:
                    if self.draw_flag:
                        # 颜色识别绘制(color detect)
                        if self.color_detect_box is not None and (self.start_pick or self.start_place):
                            cv2.drawContours(result_image, [self.color_detect_box], -1, (0, 255, 255), 2)
                            cv2.circle(result_image, (self.color_detect_center[0], self.color_detect_center[1]), 5, (0, 255, 255), -1)
                            cv2.line(result_image, (self.pick_stop_x, 0), (self.pick_stop_x, 480), (0, 255, 255), 2)
                            cv2.line(result_image, (0, self.pick_stop_y), (640, self.pick_stop_y), (0, 255, 255), 2)
                        # 物体追踪绘制(object tracking)
                        # if self.track_box_p1 and self.track_box_p2:
                        if self.track_box_p1 is not None and self.track_box_p2 is not None:

                            if self.start_track:
                                cv2.rectangle(result_image, self.track_box_p1, self.track_box_p2, (0, 255, 0), 2)
                            if not self.obj_move_finish and self.object_mode == 'pick':
                                cv2.line(result_image, (self.obj_pick_center_x, 0), (self.obj_pick_center_x, 480), (0, 255, 255), 2)
                                cv2.line(result_image, (0, self.obj_pick_center_y), (640, self.obj_pick_center_y), (0, 255, 255), 2)
                        if self.interface_box != []:
                            x_min, y_min, x_max, y_max = self.interface_box
                            cv2.rectangle(result_image, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
                        if self.interface_box_list != []:
                            for obj_info in self.interface_box_list:
                                box = obj_info.get("box", [])
                                name = obj_info.get("name", "unknown")
                                if box and len(box) == 4:
                                    x_min, y_min, x_max, y_max = box
                                    cv2.rectangle(result_image, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
                                    # label = name
                                    # (text_width, text_height), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
                                    # cv2.rectangle(result_image, (x_min, y_min - text_height - 5),(x_min + text_width, y_min),(0, 255, 0),-1)
                                    # cv2.putText(result_image, label,(x_min, y_min - 5),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255),2)
                ros_image = self.bridge_box.cv2_to_imgmsg(result_image, encoding="bgr8")
                self.result_image_publisher.publish(ros_image)
                if self.camera != 'usb_cam':
                    depth_color_map = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.2), cv2.COLORMAP_JET)
                    result_image = np.concatenate([result_image, depth_color_map, ], axis=1)
                cv2.imshow("result_image", result_image)

                key = cv2.waitKey(1)
                if key == ord('q') or key == 27:
                    self.running = False

            except queue.Empty:
                if not self.running:
                    break
                continue
            except Exception as e:
                self.get_logger().error(f"Display thread error: {str(e)}")
                continue



    def object_track_thread(self):
        """物体追踪线程(object tracking thread)"""
        dev = cuda.Device(0)
        ctx = dev.make_context()
        try:
            model_path = os.path.split(os.path.realpath(__file__))[0]
            back_exam_engine_path = os.path.join(model_path, "../resources/models/nanotrack_backbone_exam.engine")
            back_temp_engine_path = os.path.join(model_path, "../resources/models/nanotrack_backbone_temp.engine")
            head_engine_path = os.path.join(model_path, "../resources/models/nanotrack_head.engine")
            tracker = Tracker(back_exam_engine_path, back_temp_engine_path, head_engine_path)
            while self.running:
                try:
                    if self.camera != 'usb_cam':
                        image, depth_image, _ = self.image_pair_queue.get(block=True, timeout=1)
                    else:
                        image = self.image_queue.get(block=True, timeout=1)
                    img_h, img_w, _ = image.shape
                    
                    if self.object_detect_box and self.box is not None:
                        try:
                            box = self.box
                            box = [box[0], box[1], box[2] - box[0], box[3] - box[1]]
                            self.track.set_track_target(tracker,box, image)
                            self.start_track = True
                            self.object_detect_box = False
                            self.box = []
                            self.get_logger().info(f"Get the BOX!!!!!")
                        except (ValueError, TypeError) as e:
                            self.start_track = False

                    if self.start_track:

                        if self.object_mode == "track" or self.object_mode == 'passive_track':
                            if self.camera != 'usb_cam':
                                self.data = self.track.track(tracker,image,depth_image)
                            else:
                                self.data = self.track.track_pixel(tracker,image,chassis_move=True)
                            # self.get_logger().info(f'self.data[2]: {self.data[2]}')
                            # self.get_logger().info(f'self.data[3]: {self.data[3]}')

                        with self.draw_lock:
                            self.draw_flag = True
                            self.track_box_p1 = self.data[2]
                            self.track_box_p2 = self.data[3]

                            self.track_box_p1 = (max(0, self.track_box_p1[0]), min(img_w-1, self.track_box_p1[1]))
                            self.track_box_p2 = (max(0, self.track_box_p2[0]), min(img_h-1, self.track_box_p2[1]))
                        # self.get_logger().info(f'self.object_mode: {self.object_mode}')

                        # if self.object_mode == 'track' or self.object_mode == 'passive_track':
                        # # if self.object_mode == 'track':

                        #     twist = Twist()
                        #     twist.linear.x, twist.angular.z = self.data[0], self.data[1]
                        #     self.get_logger().info(f'speed: {twist.linear.x},{twist.angular.z}')
                        #     self.cmd_vel_pub.publish(twist)
                        #     if self.object_mode == 'pick':
                        #         center_x = self.track_box_p1[0] + (self.track_box_p2[0] - self.track_box_p1[0]) / 2
                        #         center_y = self.track_box_p1[1] + (self.track_box_p2[1] - self.track_box_p1[1]) / 2
                        #         if (center_y - self.obj_pick_center_y) < 28 and abs(center_x - self.obj_pick_center_x) < 28:
                        #             self.box_count += 1
                        #             self.get_logger().info(f'self.box_count{self.box_count}')
                        #             if self.box_count > 80:
                        #                 self.obj_pick_start = True
                        #                 self.cmd_vel_pub.publish(Twist())
                        #                 self.start_track = False
                    else:
                        self.box_count = 0
                        self.track_box_p1 = None
                        self.track_box_p2 = None
                        time.sleep(0.01)
                except queue.Empty:
                    if not self.running:
                        break
                    continue
                except Exception as e:
                    self.get_logger().error(f"Track thread error: {str(e)}")
                    time.sleep(0.01)
        finally:
            # Ensure the context is properly released(确保上下文被正确释放)
            ctx.pop()


    def transport_color_callback(self,req):
        self.target_color = req.data
        response.success = True
        return respon

    def transport_mode_callback(self, request, response):
        self.transport_debug = request.data
        self.start_debug = False
        # get arm_transport init param
        self.arm_transport_param_init_function()

        threading.Thread(target=self.object_transport_debug_thread, daemon=True).start()
        response.success = True
        return response

    def object_transport_debug_thread(self):
        if self.transport_debug != 'none':
            self.arm_transport_pick_init()
        if self.transport_debug == 'pick':
            self.get_logger().info(f'[Pick Mode]')
            self.arm_transport_pick(debug=True)
        elif self.transport_debug == 'place':
            self.get_logger().itransport_debugnfo(f'[Place Mode]')
            self.arm_transport_place(debug=True)
        if self.transport_debug == 'pick':
            self.target_color = 'red'

        elif self.transport_debug == 'place':
            self.target_color = 'blue'
        self.start_debug = False
        while self.running:
            if not self.start_debug:
                self.arm_transport_function(self.target_color,self.transport_debug)
                self.start_debug = True
                break
            else:
                time.sleep(0.01)
        self.transport_debug = 'none'
        self.get_logger().info(f'debug finish')
    
def main():
    rclpy.init()
    node = LLMControlMove('llm_control_move')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.destroy_node()
        rclpy.shutdown()
 
if __name__ == "__main__":
    main()
