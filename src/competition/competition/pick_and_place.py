#!/usr.bin/env python3
# encoding: utf-8
import os
import ast
import cv2
import time
import math
import copy
import queue
import rclpy
import signal
import threading
import numpy as np
import message_filters
from sdk import common
from sdk.pid import PID
from rclpy.node import Node
from std_msgs.msg import Bool
from cv_bridge import CvBridge
from std_msgs.msg import String
from std_srvs.srv import Trigger
from interfaces.msg import Pose2D
from interfaces.msg import CmdParam
from geometry_msgs.msg import Twist
from interfaces.srv import SetPose2D
from interfaces.srv import SetString
from rclpy.parameter import Parameter
from controller import step_controller
from interfaces.msg import ObjectsInfo
from interfaces.msg import ApriltagsInfo
from controller import controller_client 
from xf_mic_asr_offline import voice_play
from ros_robot_controller_msgs.msg import BuzzerState
from sensor_msgs.msg import Image, CameraInfo, LaserScan
from rcl_interfaces.msg import SetParametersResult
from servo_controller_msgs.msg import ServosPosition
from rcl_interfaces.srv import SetParametersAtomically
from arm_kinematics.kinematics_control import set_pose_target
from arm_kinematics_msgs.srv import GetRobotPose, SetRobotPose
from servo_controller.bus_servo_control import set_servo_position
from servo_controller.action_group_controller import ActionGroupController
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

def depth_pixel_to_camera(pixel_coords, depth, intrinsics):
    fx, fy, cx, cy = intrinsics
    px, py = pixel_coords
    x = (px - cx) * depth / fx
    y = (py - cy) * depth / fy
    z = depth
    return np.array([x, y, z])


class AutomaticPickNode(Node):
    config_path = '/home/ubuntu/ros2_ws/src/competition/config/automatic_pick_roi.yaml'
    lab_data = common.get_yaml_data("/home/ubuntu/software/lab_tool/lab_config.yaml")
    hand2cam_tf_matrix = [
    [0.0, 0.0, 1.0, -0.101],
    [-1.0, 0.0, 0.0, 0.011],
    [0.0, -1.0, 0.0, 0.045],
    [0.0, 0.0, 0.0, 1.0]
]
    pick_offset = [0.03,-0.005, 0.01]  # x, y, z 机械臂夹取时三轴的偏差调节
    '''
                x+
        y+    center    y-
                x-

                arm
    '''
    def __init__(self, name):
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.name = name
        self.image_proc_size = (640, 400)

        self.running = True
        # --- 用于显示距离的变量 ---
        self.current_dist_text = "Lidar: Wait..."
        self.current_dist_text_1 = "Follow: Wait..."
        
        # --- 流程控制标志位 ---
        self.pick = False # 是否正在夹取
        self.place = False # 是否正在放置
        self.start_pick = False # 是否开始夹取
        self.start_place = False # 是否开始放置
        self.start_height_calibration = False # 是否开始校准地面高度
        self.lidar_enable = False # 是否开启雷达避障
        self.step_check_enabled = False  # 是否开启高度检测

        # --- 上台阶前的对齐与居中标志位 ---
        self.is_aligning_step = False   # 是否正在进行上台阶前的旋转对齐
        self.is_centering_step = False  # 是否正在进行上台阶前的左右居中
        self.step_align_start_time = 0.0 # 对齐开始时间(用于超时保护)
        self.waiting_for_collapse = False  # 是否正在寻找坍塌区

        # --- 雷达避障相关变量 ---
        self.lidar_speed = 0.06  # 雷达避障时的线速度
        self.current_task = 'search_pick' # 当前任务，默认搜索夹取目标
        
        # --- 雷达避障参数 ---
        self.avg_dist_left = 0.0   # 左侧平均距离（拟合墙面）
        self.avg_dist_right = 0.0  # 右侧平均距离（拟合墙面）
        self.centered_target_dist = 0.3     # 期望离墙距离（用于正常循墙时的距离）
        self.stick_target_dist = 0.33    # 贴墙行驶安全距离（遇到路口，退出避障时，保持与墙的距离）

        # PID 参数 (需要根据实际机器人速度调整)
        self.kp_rotate = 0.25       # 居中对齐的旋转比例系数
        self.kp_strafe = 0.3       # 居中对齐的平移比例系数

        # --- 高度检测相关变量 ---
        self.step_detect_count = 0       # 滤波计数，防抖动
        self.plane_height_threshold = self.get_parameter('plane_height_threshold').value # 与地面的距离阈值（单位：米）

        self.target_color = "orange" # 目标识别的颜色
        self.linear_base_speed = 0.007 # 颜色识别时的基础线速度（单位：米/秒）
        self.angular_base_speed = 0.03 # 颜色识别时的基础角速度（单位：弧度/秒）
        self.yaw_pid = PID(P=0.015, I=0, D=0.000) # 颜色识别时的yaw角度PID控制器
        self.linear_pid = PID(P=0.0028, I=0, D=0) # 颜色识别时的线速度PID控制器
        self.angular_pid = PID(P=0.003, I=0, D=0) # 颜色识别时的角速度PID控制器
        self.center_roi = [245, 255, 315, 325] # 夹取识别时的中心ROI区域（x1, y1, x2, y2）

        pick_stop_param = self.get_parameter('pick_stop_pixel_coordinate').value
        self.pick_stop_x = pick_stop_param[0] # 夹取识别时的停止像素坐标（x）
        self.pick_stop_y = pick_stop_param[1] # 夹取识别时的停止像素坐标（y）
        place_stop_param = self.get_parameter('place_stop_pixel_coordinate').value
        self.place_stop_x = place_stop_param[0] # 放置识别时的停止像素坐标（x）
        self.place_stop_y = place_stop_param[1] # 放置识别时的停止像素坐标（y）

        self.status = "approach" # 当前状态，默认夹取识别
        self.count_stop = 0 # 颜色识别时的停止计数（用于判断是否到达停止位置）
        self.count_turn = 0 # 颜色识别时的转弯计数（用于判断是否完成转弯）
        self.detect_count = 0 # 颜色识别时的检测计数（用于判断是否完成夹取识别）

        self.d_y = 10  # 颜色识别时的垂直方向阈值（单位：像素）
        self.d_x = 10  # 颜色识别时的水平方向阈值（单位：像素）
        self.linear_speed = 0 # 颜色识别时的实际线速度（单位：米/秒）
        self.angular_speed = 0 # 颜色识别时的实际角速度（单位：弧度/秒）
        self.yaw_angle = 90 # 颜色识别时的目标yaw角度（单位：度）
        self.yaw = 500 # 机械臂夹取时23号舵机的脉冲值

        # --- 用于 YOLO (精对准) 的变量 ---
        self.yolo_tracking_mode = 'IDLE' # YOLO跟踪模式，默认空闲
        self.locked_target_class = 'poison' # 锁定的目标类别，默认毒弹
        self.final_grasp_target_info = None # 最终夹取目标中心坐标
        self.target_list = ['flammable','explosives','poison'] # 夹取识别时的目标类别列表

        self.grasp_detect_count = 0 # 夹取识别时的检测计数（用于判断是否完成夹取识别）
        self._last_grasp_center = None # 上一次夹取目标中心坐标
        self.position = None # 夹取目标的位置
        self.id = None # 标签码的ID（用于放置到不同层数）

        # --- 用于 AprilTag 的变量 ---
        self.apriltag_target_id = 3  # 目标AprilTag的ID（默认3）
        self.current_apriltag_id = None  # 当前识别到的AprilTag的ID
        self.locked_apriltag_id = None  # 锁定的AprilTag的ID
        
        # --- 目标丢失计数器 ---
        self.target_lost_count = 0  # 目标丢失计数器（用于判断是否丢失目标）
        self.LOST_THRESHOLD = 30    # 连续30帧丢失才切换模式
        self.color_found_count = 0       # 当前连续识别到的帧数
        self.FOUND_THRESHOLD = 40        # 阈值：40帧

        # --- 目标找回计数器 (防震荡) ---
        self.rediscover_count = 0        # 记录雷达模式下连续看到目标的帧数
        self.REDISCOVER_THRESHOLD = 40    # 连续40帧看到才切回视觉

        # --- 1. 初始化共享数据变量 (给一个安全的默认值) ---
        self.min_dist_front = 9.99  # 前方距离（单位：米）
        self.min_dist_left = 9.99   # 左侧距离（单位：米）
        self.min_dist_right = 9.99  # 右侧距离（单位：米）
        
        # --- 2. 状态机变量 ---
        self.is_forcing_turn = False  # 是否正在强制转弯
        self.turn_start_time = 0.0    # 转弯开始时间戳
        self.big_gap = 0.65           # 大间隙距离（单位：米）
        
        self.is_up_steps = False # 是否正在上台阶
        self.lidar_hole = False  # 是否处于跨越坍塌区模式

        self.ALIGN_ANGLE_THRESHOLD = 0.15  # 斜率k误差阈值（约3度），超过此值先旋转，阈值太小会导致机器人来回震荡
        self.ALIGN_OFFSET_THRESHOLD = 0.04 # 位移误差阈值（4cm），超过此值再平移

        self.right_k = None # 右侧斜率k（用于巡墙）
        self.left_k = None # 左侧斜率k（用于巡墙）
        self.follow_side = 'LEFT'  # 初始默认巡左墙 ('LEFT' 或 'RIGHT')
        self.min_dist_corner_left = 9.99  # 左侧角落距离（单位：米）
        self.min_dist_corner_right = 9.99  # 右侧角落距离（单位：米）
        self.left_blocked_count = 0  # 左侧被遮挡帧数（用于判断是否被遮挡）
        self.right_blocked_count = 0  # 右侧被遮挡帧数（用于判断是否被遮挡）
        self.left_blocked = False  # 是否左侧被遮挡
        self.right_blocked = False  # 是否右侧被遮挡
        
        # --- 强制巡墙控制变量 ---
        self.force_side_expire_time = 0.0 # 强制巡墙状态的过期时间戳

        self.hole_time = 0 # 跨越坍塌区计时
        self.rotate = False # 是否正在旋转（用于巡墙）

        self.language = os.environ['ASR_LANGUAGE']
        self.declare_parameter('status', 'start')
        self.bridge = CvBridge()
        self.image_queue = queue.Queue(maxsize=2)
        self.depth_image_queue = queue.Queue(maxsize=2)
        self.camera_info_queue = queue.Queue(maxsize=2)
        self.yolo_image_queue = queue.Queue(maxsize=2)
        self.enable_display = self.get_parameter('enable_display').value
        self.debug = self.get_parameter('debug').value
        self.controller = controller_client.ControllerClient()

        self.buzzer_pub = self.create_publisher(BuzzerState, 'ros_robot_controller/set_buzzer', 1)
        self.joints_pub = self.create_publisher(ServosPosition, '/servo_controller', 1)
        self.cmd_vel_pub = self.create_publisher(Twist, '/controller/cmd_vel', 1)
        self.cmd_param_pub = self.create_publisher(CmdParam, '/step_controller/cmd_param', 1) 
        self.image_pub = self.create_publisher(Image, '~/image_result', 1)

        lidar_qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, lidar_qos)
        self.create_subscription(Image, '/depth_cam/rgb/image_raw', self.image_callback, 1)
        self.create_subscription(Image, '/depth_cam/depth/image_raw', self.depth_image_callback, 1)
        self.create_subscription(CameraInfo , '/depth_cam/depth/camera_info', self.camera_info_callback, 1)
        self.create_subscription(String , '/competition/locked_target_class', self.locked_target_class_callback, 1)
        self.create_subscription(ApriltagsInfo, '/apriltag_detect/apriltag_info',  self.apriltag_info_callback, 1)
        self.create_subscription(ObjectsInfo, '/yolo/object_detect', self.get_object_callback, 1)
        self.create_subscription(Image, '/yolo/object_image', self.yolo_image_callback, 1)
        self.create_subscription(Bool, '/cross_bridge/finish', self.cross_bridge_finish_callback, 1)

        self.create_service(Trigger, '~/pick', self.start_pick_callback)
        self.create_service(Trigger, '~/place', self.start_place_callback) 
        self.create_service(Trigger, '~/lidar_enable', self.lidar_enable_callback)
        self.create_service(Trigger, '~/check_step_hazard', self.check_step_hazard_callback)
        self.create_service(Trigger, '~/is_up_steps', self.is_up_steps_callback)

        self.start_yolo_client = self.create_client(Trigger, '/yolo/start')
        self.stop_yolo_client = self.create_client(Trigger, '/yolo/stop')
        self.get_current_pose_client = self.create_client(GetRobotPose, '/arm_kinematics/get_current_pose')
        self.set_pose_target_client = self.create_client(SetRobotPose, '/arm_kinematics/set_pose_target')
        self.client = self.create_client(Trigger, '/controller_manager/init_finish')
        self.client.wait_for_service()

        self.get_logger().info('\033[1;32m雷达避障已开启，正在寻找红色目标...\033[0m')

        threading.Thread(target=self.action_thread, daemon=True).start()
        threading.Thread(target=self.main, daemon=True).start()
        self.timer = self.create_timer(0.05, self.lidar_control_loop)  # 20Hz 控制循环
        self.get_logger().info('\033[1;32m%s\033[0m' % 'Automatic Pick Node Started')

        set_servo_position(self.joints_pub, 1, ((19, 500), (20, 700), (21, 155), (22, 70), (23, 500), (24, 700)))
        time.sleep(1)

        if self.debug == 'pick':
            self.get_logger().info('\033[1;32m%s\033[0m' % '将机器人放置到夹取道具前的位置，准备执行拾取动作')
            self.controller.traveling(gait=-2, time=1, steps=0)
            time.sleep(1)
            set_servo_position(self.joints_pub, 1.5, ((19, 500), (20, 200), (21, 380), (22, 240), (23, 500), (24, 700)))
            time.sleep(5)
            set_servo_position(self.joints_pub, 2, ((19, 500), (20, 700), (21, 155), (22, 70), (23, 500), (24, 700)))
            time.sleep(2)
            msg = Trigger.Request()
            self.start_pick_callback(msg, Trigger.Response())

        if self.debug == 'place':
            self.get_logger().info('\033[1;32m%s\033[0m' % '将机器人放置到放置道具前的位置，准备执行拾取动作')
            self.controller.traveling(gait=-2, time=1, steps=0)
            time.sleep(1)
            self.get_logger().info("执行 ID=3 的放置动作")
            set_servo_position(self.joints_pub, 1.5, ((19, 500), (20, 445), (21, 70), (22, 650), (23, 500), (24, 500)))                
            time.sleep(5)
            self.controller.traveling(gait=2, time=1,direction=math.radians(180), steps=2) # 后退2步
            time.sleep(3)
            self.get_logger().info("执行 ID=2 的放置动作")
            set_servo_position(self.joints_pub, 1.5, ((19, 500), (20, 700), (21, 155), (22, 70), (23, 500), (24, 500)))
            time.sleep(1.5)
            self.controller.traveling(gait=2, time=1,direction=math.radians(0), steps=2) # 前进2步
            time.sleep(3)
            self.get_logger().info("执行 ID=1 的放置动作")
            set_servo_position(self.joints_pub, 1.5, ((19, 500), (20, 50), (21, 240), (22, 800), (23, 500), (24, 500)))
            time.sleep(1.5)
            self.controller.traveling(gait=2, time=1,direction=math.radians(180), steps=2) # 后退2步
            time.sleep(3)
            set_servo_position(self.joints_pub, 2, ((19, 500), (20, 700), (21, 155), (22, 70), (23, 500), (24, 700)))
            time.sleep(1.5)
            msg = Trigger.Request()
            self.start_place_callback(msg, Trigger.Response())

        if self.debug == 'height':
            self.get_logger().info('\033[1;32m%s\033[0m' % '将机器人放置到平整地面，准备开始校准地面高度')
            set_servo_position(self.joints_pub, 1.5, ((19, 500), (20, 700), (21, 155), (22, 70), (23, 500), (24, 700)))
            time.sleep(1.5)
            self.controller.traveling(gait=-1, time=1, steps=0)
            time.sleep(2)
            self.start_height_calibration = True


    def play(self, name):
        voice_play.play(name, language=self.language)

    def set_normal_posture(self):
        """恢复正常站立姿态"""
        cmd_param = CmdParam()
        cmd_param.gait = 2
        cmd_param.period = 1.0
        cmd_param.pose = 'SLAM_POSE'
        cmd_param.height = 20
        self.cmd_param_pub.publish(cmd_param)
        time.sleep(1.0)

    def set_hole_posture(self):
        """恢复跨越坍塌区姿态"""
        cmd_param = CmdParam()
        cmd_param.gait = 2
        cmd_param.period = 1.0
        cmd_param.pose = 'HOLE_POSE'
        cmd_param.height = 10
        self.cmd_param_pub.publish(cmd_param)
        time.sleep(1.0)

    def set_default_posture(self):
        """恢复夹取姿态"""
        cmd_param = CmdParam()
        cmd_param.gait = 2
        cmd_param.period = 1.0
        cmd_param.pose = 'DEFAULT_POSE'
        cmd_param.height = 20
        self.cmd_param_pub.publish(cmd_param)
        time.sleep(1.0)

    def set_up_steps_posture(self):
        """设置上台阶姿态"""
        cmd_param = CmdParam()
        cmd_param.gait = 2
        cmd_param.period = 2.0
        cmd_param.pose = 'SLAM_POSE' 
        cmd_param.height = 60
        self.cmd_param_pub.publish(cmd_param)
        time.sleep(2.0)

    def get_roi_distance(self, depth_image, roi):
        roi_image = depth_image[roi[0]:roi[1], roi[2]:roi[3]]
        try:
            distance = round(float(np.mean(roi_image[np.logical_and(roi_image > 0, roi_image < 30000)]) / 1000), 3)
        except:
            distance = 0
        return distance

    def cross_bridge_finish_callback(self, msg):
        self.lidar_enable_callback(None, None)
        
    def fit_line(self, xs, ys):
        """使用最小二乘法拟合直线 y = kx + b"""
        if len(xs) < 5: 
            return None, None
        try:
            # 拟合一次函数
            k, b = np.polyfit(xs, ys, 1)
            if np.isnan(k) or np.isnan(b):
                return None, None
            return k, b
        except:
            return None, None

    def camera_info_callback(self, camera_info):
        if self.camera_info_queue.full(): self.camera_info_queue.get()
        self.camera_info_queue.put(camera_info)

    def locked_target_class_callback(self, msg):
        self.locked_target_class = msg.data
        self.get_logger().info(f'\033[1;32m锁定目标类别: {self.locked_target_class}\033[0m')

    def depth_image_callback(self, ros_image):
        if self.depth_image_queue.full(): self.depth_image_queue.get()
        self.depth_image_queue.put(ros_image)

    def image_callback(self, ros_image):
        if self.image_queue.full(): self.image_queue.get()
        self.image_queue.put(self.bridge.imgmsg_to_cv2(ros_image, "bgr8"))

    def yolo_image_callback(self, ros_image):
        if self.yolo_image_queue.full(): self.yolo_image_queue.get()
        self.yolo_image_queue.put(self.bridge.imgmsg_to_cv2(ros_image, "bgr8"))

    def trigger_up_steps_action(self):
        """触发上台阶的动作序列"""
        self.set_up_steps_posture()
        self.is_up_steps = True
        self.turn_start_time = time.time()
        self.lidar_hole = True
        self.get_logger().info('\033[1;32m 姿态调整完毕，开始冲刺上台阶 \033[0m')

    def lidar_callback(self, lidar_data):
        # 1. 数据清洗：将 nan, inf, 0 全部替换为量程外数值
        ranges = np.array(lidar_data.ranges)
        ranges = np.nan_to_num(ranges, nan=9.99, posinf=9.99, neginf=9.99)
        ranges[ranges == 0] = 9.99
        
        angle_increment = lidar_data.angle_increment
        
          # 2. 区域定义
        # 前方: -20 ~ 20 (原有)
        # 侧方: 40 ~ 100 (原有，用于拟合墙壁)
        # 角落区: 15 ~ 50 度。这个区域如果有障碍物，会干扰墙壁拟合
        
        idx_front_limit = int(math.radians(20) / angle_increment)
        
        # 角落区索引 (15度 到 50度)
        idx_corner_start = int(math.radians(15) / angle_increment)
        idx_corner_end   = int(math.radians(50) / angle_increment)
        
        # 侧方拟合区索引 (40度 到 100度)
        idx_side_start = int(math.radians(40) / angle_increment)
        idx_side_end   = int(math.radians(100) / angle_increment)

        # 2. 区域提取与坐标转换
        left_pts_x, left_pts_y = [], []
        right_pts_x, right_pts_y = [], []
        

        for i, r in enumerate(ranges):
            angle = i * angle_increment
            # 转换为直角坐标 (x向前, y向左)
            x = r * math.cos(angle)
            y = r * math.sin(angle)

            # 收集侧墙点云用于拟合
            if 0.1 < r < 1.5:
                if idx_side_start < i < idx_side_end: # 左侧
                    left_pts_x.append(x)
                    left_pts_y.append(y)
                elif (len(ranges) - idx_side_end) < i < (len(ranges) - idx_side_start): # 右侧
                    right_pts_x.append(x)
                    right_pts_y.append(y)

        # 3. 计算关键指标
        # A. 前方最小值
        front_ranges = np.concatenate((ranges[:idx_front_limit], ranges[-idx_front_limit:]))
        valid_front = front_ranges[front_ranges > 0.05]
        self.min_dist_front = np.min(valid_front) if len(valid_front) > 0 else 9.99

        # B. 左右侧最小值与平均值 (用于路口判断)
        l_side_ranges = ranges[idx_side_start:idx_side_end]
        r_side_ranges = ranges[-idx_side_end:-idx_side_start]
        self.min_dist_left = np.min(l_side_ranges)
        self.min_dist_right = np.min(r_side_ranges)
        self.avg_dist_left = np.mean(l_side_ranges[l_side_ranges < 1.5])
        self.avg_dist_right = np.mean(r_side_ranges[r_side_ranges < 1.5])

     # C. 角落障碍物检测
        # 左前角
        l_corner_ranges = ranges[idx_corner_start:idx_corner_end]
        valid_l_corner = l_corner_ranges[(l_corner_ranges > 0.05) & (l_corner_ranges < 2.0)]
        self.min_dist_corner_left = np.min(valid_l_corner) if len(valid_l_corner) > 0 else 9.99
        
        # 右前角
        r_corner_ranges = ranges[-idx_corner_end:-idx_corner_start]
        valid_r_corner = r_corner_ranges[(r_corner_ranges > 0.05) & (r_corner_ranges < 2.0)]
        self.min_dist_corner_right = np.min(valid_r_corner) if len(valid_r_corner) > 0 else 9.99

        # C. 直线拟合 (用于航向/位移解耦)
        self.left_k, self.left_b = self.fit_line(left_pts_x, left_pts_y)
        self.right_k, self.right_b = self.fit_line(right_pts_x, right_pts_y)

        self.current_dist_text = f"F:{self.min_dist_front:.2f} L:{self.avg_dist_left:.2f} R:{self.avg_dist_right:.2f}"


    def lidar_control_loop(self):
        twist = Twist()
        # =========================================================================
        # 阶段1: 上台阶前的旋转对齐 (Align)
        # =========================================================================
        if self.is_aligning_step:
            # 超时保护 
            if time.time() - self.step_align_start_time > 10.0:
                self.get_logger().warn("对齐超时，强制进入居中阶段")
                self.is_aligning_step = False
                self.is_centering_step = True
                self.step_align_start_time = time.time() # 重用计时器
                return

            # 获取当前墙壁斜率 (优先用左墙，没有则用右墙)
            current_k = 0.0
            if self.left_k is not None:
                current_k = self.left_k
            elif self.right_k is not None:
                current_k = self.right_k
            
            # P控制旋转
            # 阈值：斜率绝对值小于 0.05 (约3度) 认为对齐
            if abs(current_k) > 0.03:
                twist.linear.x = 0.0
                twist.linear.y = 0.0
                # k > 0 代表车头偏右，需要左转 (z > 0)
                twist.angular.z = float(np.clip(current_k , -0.1, 0.1)) 
                self.cmd_vel_pub.publish(twist)
                self.get_logger().info(f"台阶对齐中... k={current_k:.3f}")
            else:
                # 对齐完成，停车并切换状态
                self.cmd_vel_pub.publish(Twist())
                time.sleep(0.5)
                self.get_logger().info("台阶对齐完成 -> 开始居中")
                self.is_aligning_step = False
                self.is_centering_step = True
                self.step_align_start_time = time.time()
            return

        # =========================================================================
        # 阶段2: 上台阶前的左右居中 (Center)
        # =========================================================================
        if self.is_centering_step:
            # 超时保护 (例如5秒)
            if time.time() - self.step_align_start_time > 10.0:
                self.get_logger().warn("居中超时，强制开始上台阶")
                self.is_centering_step = False
                self.trigger_up_steps_action() # 封装后的触发函数
                return

            # 计算左右偏差 (左距 - 右距)
            # 假设 avg_dist_left 和 avg_dist_right 在 lidar_callback 中实时更新
            err_offset = self.avg_dist_left - self.avg_dist_right
            
            # 阈值：偏差小于 0.03 (3cm) 认为居中
            if abs(err_offset) > 0.01:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                # err > 0 (左边宽)，需要向左平移(+y) ??? 
                # 坐标系: y+ 是左。
                # 如果左边距离大(err>0)，说明车偏右，确实需要向左平移(+y)。
                twist.linear.y = float(np.clip(err_offset * 0.5, -0.04, 0.04))
                self.cmd_vel_pub.publish(twist)
                self.get_logger().info(f"台阶居中中... err={err_offset:.3f}")
            else:
                # 居中完成
                self.cmd_vel_pub.publish(Twist())
                time.sleep(0.5)
                self.get_logger().info("台阶居中完成 -> 执行上台阶程序")
                self.is_centering_step = False
                self.trigger_up_steps_action() # 触发原有上台阶逻辑
            return


       # ------------------ 特殊状态保持 (上台阶/强制转弯) ------------------
        if self.is_up_steps:
            if time.time() - self.turn_start_time < 11:
                twist.linear.x = 0.04
                self.cmd_vel_pub.publish(twist)
                return
            else:
                self.controller.traveling(gait=-2, time=1, steps=0)
                time.sleep(1)
                self.set_normal_posture() # 切换为普通/收缩姿态
                self.is_up_steps = False
                self.waiting_for_collapse = True # 开始寻找坍塌区
                self.lidar_enable = True
                self.get_logger().info("\033[1;33m上台阶完成，当前：正常姿态 + 居中避障，寻找坍塌区...\033[0m")
                return

        if self.is_forcing_turn:
            self.controller.traveling(gait=2, time=1, steps=10) #前进
            time.sleep(10)
            self.controller.traveling(gait=2, time=1, rotation=-0.9, steps=10) # 旋转对齐
            time.sleep(11)
            self.lidar_enable = False
            self.lidar_hole = False
            self.is_forcing_turn = False
            self.play('finish')
            return

        if self.lidar_hole or self.waiting_for_collapse:
            if abs(self.min_dist_left - self.min_dist_right) > 0.1 and self.min_dist_front < self.big_gap and time.time() - self.hole_time > 10:
                self.get_logger().info("触发大间隙，开始计时,准备进入强制转弯状态")
                self.is_forcing_turn = True
                self.turn_start_time = time.time()
                # 下一个定时器周期就会进入状态1
                return 

            # 2.2 细微调整逻辑 (保持你原有的逻辑)
            if abs(self.min_dist_left - self.min_dist_right) > 0.01:
                twist.linear.x = 0.03
                turn_speed = 0.07
                
                if self.min_dist_front > 0.3:
                    if self.min_dist_left > self.min_dist_right:
                        twist.angular.z = turn_speed 
                    else:
                        twist.angular.z = -turn_speed
                else:
                    twist.linear.x = 0.00
                    twist.angular.z = -turn_speed
            else:
                twist.linear.x = 0.03

            self.cmd_vel_pub.publish(twist)
            return  # 如果进入了 lidar_hole 模式，就不执行下面的避障逻辑

        if not self.lidar_enable:
            return       
        # =========================================================================
        # 1. 状态管理：检测角落障碍物并切换巡航边
        # =========================================================================
        OBSTACLE_THRESHOLD = 0.4  # 前侧方障碍物判定阈值

        # --- A. 滤波检测角落状态 ---
        if self.min_dist_corner_left < OBSTACLE_THRESHOLD:
            self.left_blocked_count += 1
            if self.left_blocked_count >= 15: # 稍微降低滤波帧数加快反应
                self.left_blocked = True
        else:
            self.left_blocked_count = 0
            self.left_blocked = False


        if self.min_dist_corner_right < OBSTACLE_THRESHOLD:
            self.right_blocked_count += 1
            if self.right_blocked_count >= 15:
                self.right_blocked = True
        else:
            self.right_blocked_count = 0
            self.right_blocked = False
        # --- A.2 判断是否处于强制巡墙状态 ---
        is_force_side_active = time.time() < self.force_side_expire_time
        # --- B. 巡航边切换逻辑 ---
            # --- B. 巡航边切换逻辑  ---
        # 只有在【非强制锁定状态】下，才允许自动切换
        if not is_force_side_active: 
            if self.follow_side == 'LEFT':
                if self.left_blocked and not self.right_blocked:
                    self.follow_side = 'RIGHT'
                    self.get_logger().warn("【切换】左前受阻 -> 切换巡航右墙")

            elif self.follow_side == 'RIGHT':
                if self.right_blocked and not self.left_blocked:
                    self.follow_side = 'LEFT'
                    self.get_logger().warn("【切换】右前受阻 -> 切换巡航左墙")
        else:
            # 可选：打印调试信息，确认锁定生效
            self.get_logger().info(f"强制巡墙中: {self.follow_side}, 剩余 {self.force_side_expire_time - time.time():.1f}s", throttle_duration_sec=2)
            pass

        # =========================================================================
        # 2. 路口与尽头逻辑 (Priority 1)
        # =========================================================================
        # 判定条件：左右宽度差大，且角落没有障碍物干扰（如果有角落障碍，应优先交给下面的巡墙逻辑去避障）
        is_intersection = abs(self.avg_dist_left - self.avg_dist_right) > 0.3
        if is_intersection:
            # --- 2.1 尽头/转弯处理 ---
            if self.min_dist_front < 0.45:
                twist.linear.x = 0.0
                twist.linear.y = 0.0
                # 向空旷的一侧原地旋转
                if self.avg_dist_left > self.avg_dist_right:
                    if self.rotate:
                        self.rotate = False
                        self.controller.traveling(gait=2, time=1, direction=math.radians(90), steps=2)
                        time.sleep(2)
                    twist.angular.z = 0.15 # 左转
                    self.get_logger().info("【路口-尽头】前方受阻 -> 原地左转")
                else:
                    if self.rotate == False:
                        self.rotate = True
                        self.controller.traveling(gait=2, time=1, direction=math.radians(270), steps=2)
                        time.sleep(2)
                    twist.angular.z = -0.15 # 右转
                    self.get_logger().info("【路口-尽头】前方受阻 -> 原地右转")
                
                self.cmd_vel_pub.publish(twist)
                return # 此时正在转弯，不执行后续逻辑

            # --- 2.2 路口直行通过 ---
            else:
                # 即使是单侧空旷，也要主动对齐最近的墙壁保持 self.stick_target_dist 距离
                twist.linear.x = 0.05 # 保持稳步直行
                # 判断贴哪边的墙（选距离近的那一边）
                if self.avg_dist_left < self.avg_dist_right:
                    # ============== 贴左墙模式 ==============
                    # 1. 平移控制：距离 > 0.3 则向左平移(+y)靠近，反之向右(-y)
                    dist_err = self.avg_dist_left - self.stick_target_dist
                    twist.linear.y = float(np.clip(dist_err * self.kp_strafe *0.5, -0.03, 0.03))
                    
                    # 2. 旋转控制：利用斜率 k 修正车头
                    # 左墙 k>0 代表车头偏右(远离墙)，需左转(+z)
                    k_err = self.left_k if self.left_k is not None else 0.0
                    twist.angular.z = float(np.clip(k_err * self.kp_rotate, -0.2, 0.2))
                    self.get_logger().info(f"【路口-贴左】Dist:{self.avg_dist_left:.2f} Err:{dist_err:.2f} Y:{twist.linear.y:.2f}")

                else:
                    # ============== 贴右墙模式 ==============
                    # 1. 平移控制：距离 > 0.3 则向右平移(-y)靠近，反之向左(+y)
                    # 注意：右侧逻辑平移方向与左侧相反
                    dist_err = self.avg_dist_right - self.stick_target_dist
                    twist.linear.y = float(np.clip(-dist_err * self.kp_strafe *0.5, -0.03, 0.03))
                    
                    # 2. 旋转控制：利用斜率 k 修正车头
                    # 右墙 k>0 代表车头偏右(撞向墙)，需左转(+z)修正
                    k_err = self.right_k if self.right_k is not None else 0.0
                    twist.angular.z = float(np.clip(k_err * self.kp_rotate, -0.2, 0.2))
                    
                    self.get_logger().info(f"【路口-贴右】Dist:{self.avg_dist_right:.2f} Err:{dist_err:.2f} Y:{twist.linear.y:.2f}")

                self.cmd_vel_pub.publish(twist)
                return # 直行通过路口时，忽略后续的通用巡航逻辑

        # =========================================================================
        # 3. 数据提取 (为单墙巡航做准备)
        # =========================================================================
        current_k = 0.0
        current_offset = 0.0
        data_valid = False

        if self.follow_side == 'LEFT':
            if self.left_k is not None:
                current_k = self.left_k
                current_offset = self.left_b - self.centered_target_dist 
                data_valid = True
            else:
                # 用右墙数据补充
                if self.right_k is not None:
                    current_k = self.right_k
                    current_offset = self.right_b + self.centered_target_dist
                    data_valid = True

        elif self.follow_side == 'RIGHT':
            if self.right_k is not None:
                current_k = self.right_k
                current_offset = self.right_b + self.centered_target_dist
                data_valid = True
            else:
                if self.left_k is not None:
                    current_k = self.left_k
                    current_offset = self.left_b - self.centered_target_dist
                    data_valid = True

        # =========================================================================
        # 4. 单墙巡航逻辑：先旋 -> 再平移 -> 后直走 (Priority 2)
        # =========================================================================
        

        if not data_valid:
            # 如果完全没有墙壁数据（且未触发路口逻辑），缓慢直行
            twist.linear.x = 0.02
            self.cmd_vel_pub.publish(twist)
            return

        # --- 动作1：旋转对齐 (Align Heading) ---
        if abs(current_k) > self.ALIGN_ANGLE_THRESHOLD:
            twist.linear.x = 0.01 
            twist.linear.y = 0.0
            # k > 0 代表车头向右偏，需左转(z+)
            twist.angular.z = float(np.clip(current_k * self.kp_rotate * 1.0, -0.2, 0.2))
            self.get_logger().info(f"【巡墙-1 旋转】车身不正(k={current_k:.2f}) -> 原地对齐")

        # --- 动作2：平移居中 (Strafe Center) ---
        elif abs(current_offset) > self.ALIGN_OFFSET_THRESHOLD:
            twist.linear.x = 0.0 
            twist.angular.z = 0.0 
            # offset > 0 (偏左/b大)，需右移(-y) ??? 
            # 修正逻辑回顾：
            # 左墙: off = b(实测) - 0.3。 若 b=0.5(远了/偏右)，off=0.2。需左移(+y)。
            # 右墙: off = b(实测负) + 0.3。若 b=-0.5(远了/偏左)，off=-0.2。需右移(-y)。
            # 结论: twist.y = offset * kp (正对正)
            twist.linear.y = float(np.clip(current_offset * self.kp_strafe * 0.8, -0.03, 0.03))
            self.get_logger().info(f"【巡墙-2 平移】距离不准(off={current_offset:.2f}) -> {self.follow_side}")

        # --- 动作3：正常前进 (Move Forward) ---
        else:
            # 前方防撞
            if self.min_dist_front < 0.45:
                twist.angular.z = 0.01 
                twist.linear.x = 0.01 
            else:
                twist.linear.x = self.lidar_speed

            twist.linear.y = 0.0
            twist.angular.z = 0.0
            
            # P控制微调
            if abs(current_k) > 0.02:
                twist.angular.z = float(np.clip(-current_k * self.kp_rotate * 0.5, -0.1, 0.1)) 
            if abs(current_offset) > 0.02:
                twist.linear.y = float(np.clip(current_offset * self.kp_strafe * 0.8, -0.03, 0.03))
        self.current_dist_text_1 = f"Mode:{self.follow_side} k:{current_k:.2f} off:{current_offset:.2f} "
        self.cmd_vel_pub.publish(twist)

    def get_endpoint(self):
        endpoint = self.send_request(self.get_current_pose_client, GetRobotPose.Request()).pose
        self.endpoint = common.xyz_quat_to_mat([endpoint.position.x, endpoint.position.y, endpoint.position.z],
                                        [endpoint.orientation.w, endpoint.orientation.x, endpoint.orientation.y, endpoint.orientation.z])
        return self.endpoint
    
    def set_parameter(self, client, name, value):
        req = SetParametersAtomically.Request()
        req.parameters = [Parameter(name, Parameter.Type.STRING, value).to_parameter_msg()]
        client.call_async(req)

    def get_object_callback(self, msg):
        if self.yolo_tracking_mode == 'TRACKING_FOR_GRASP':
            for i in msg.objects:
                if i.class_name == self.locked_target_class:
                    box_coords = i.box
                    center_x = (box_coords[0] + box_coords[2] + box_coords[4] + box_coords[6]) / 4
                    center_y = (box_coords[1] + box_coords[3] + box_coords[5] + box_coords[7]) / 4

                    self.yaw = 500 + int(i.angle/ 240 * 1000)

                    current_center = (int(center_x), int(center_y))
                    if self._last_grasp_center is None:
                        self._last_grasp_center = current_center
                        self.grasp_detect_count = 1
                    else:
                        dx = abs(current_center[0] - self._last_grasp_center[0])
                        dy = abs(current_center[1] - self._last_grasp_center[1])
                        if dx <= 8 and dy <= 8:
                            self.grasp_detect_count += 1
                        else:
                            self._last_grasp_center = current_center
                            self.grasp_detect_count = 1

                    if self.grasp_detect_count >= 10:
                        self.final_grasp_target_info = {"center_x": center_x, "center_y": center_y}
                        self.yolo_tracking_mode = 'IDLE'
                        self.grasp_detect_count = 0
                        self._last_grasp_center = None
                        break

    def get_world_pose(self, object_center_x, object_center_y):
        try:
            depth_camera_info = self.camera_info_queue.get(block=True, timeout=2)
            depth_image_msg = self.depth_image_queue.get(block=True, timeout=2)
            depth_image = self.bridge.imgmsg_to_cv2(depth_image_msg, '16UC1')
        except queue.Empty:
            return None
        roi = [int(object_center_y) - 5, int(object_center_y) + 5, int(object_center_x) - 5, int(object_center_x) + 5]
        roi[0] = max(0, roi[0]); roi[1] = min(depth_image.shape[0], roi[1]); roi[2] = max(0, roi[2]); roi[3] = min(depth_image.shape[1], roi[3])
        roi_distance = depth_image[roi[0]:roi[1], roi[2]:roi[3]]
        valid_mask = (roi_distance > 0) & (roi_distance < 10000)
        if np.any(valid_mask):
            dist = round(float(roi_distance[valid_mask].mean() / 1000.0), 3) + 0.015
            K = depth_camera_info.k
            self.get_endpoint()
            position = depth_pixel_to_camera((object_center_x, object_center_y), dist, (K[0], K[4], K[2], K[5]))
            position[0] -= 0.01
            pose_end = np.matmul(self.hand2cam_tf_matrix, common.xyz_euler_to_mat(position, (0, 0, 0)))
            world_pose = np.matmul(self.endpoint, pose_end)
            pose_t, _ = common.mat_to_xyz_euler(world_pose)
            self.position = pose_t
        else:
            self.position = None
            
    def apriltag_info_callback(self, msg):
        if msg.data:
            self.current_apriltag_id = msg.data[0].id

    def start_pick_callback(self, request, response):
        self.set_default_posture()
        self.trigger_pick_process()
        response.success = True
        return response 
    
    def lidar_enable_callback(self, request, response):
        self.lidar_enable = True
        if response is not None:
            response.success = True
        return response

    def is_up_steps_callback(self, request, response):
        self.set_up_steps_posture()
        self.turn_start_time = time.time()
        self.is_up_steps = True     
        self.get_logger().info('\033[1;32m，开始视觉对准\033[0m')

        response.success = True
        return response 

    def check_step_hazard_callback(self, request, response):
        self.current_task = 'mission_completed'
        self.start_pick = False
        self.start_place = False
        self.lidar_enable = True
        self.step_check_enabled = True 
        response.success = True
        return response 

    def trigger_pick_process(self):
        self.get_logger().info('\033[1;32m检测到抓取目标(橙色)，关闭雷达，开始视觉对准\033[0m')
        self.lidar_enable = False
        self.step_check_enabled = False # 关闭高度检测
        set_servo_position(self.joints_pub, 1.0, ((19, 500), (20, 700), (21, 155), (22, 70), (23, 500), (24, 700)))
        time.sleep(1)
        self.controller.traveling(gait=-2, time=1, steps=0)
        time.sleep(1)

        self.status = "approach"
        self.count_stop = 0
        self.count_turn = 0
        self.target_lost_count = 0  
        self.linear_pid.clear()
        self.angular_pid.clear()
        self.yaw_pid.clear()
        self.d_x = 10 
        self.d_y = 10
        self.target_color = "orange" 
        self.pick = True 
        self.start_pick = True

    def start_place_callback(self, request, response):
        self.set_default_posture()
        self.trigger_place_process()
        response.success = True
        return response

    def trigger_place_process(self):
        self.get_logger().info('\033[1;32m检测到放置目标，关闭雷达，开始视觉对准 PLACE\033[0m')
        self.lidar_enable = False
        self.step_check_enabled = False # 关闭高度检测
        set_servo_position(self.joints_pub, 1, ((19, 500), (20, 700), (21, 155), (22, 70), (23, 500), (24, 500)))
        time.sleep(1)
        self.controller.traveling(gait=-2, time=1, steps=0)
        time.sleep(1)

        self.current_task = 'search_place'
        self.target_color = 'blue'
        self.status = "approach"
        self.count_stop = 0
        self.count_turn = 0
        self.target_lost_count = 0  # 重置丢失计数
        self.linear_pid.clear()
        self.angular_pid.clear()
        self.yaw_pid.clear()
        self.d_x = 10
        self.d_y = 10

        self.pick = False
        self.place = False
        self.start_place = True

    def send_request(self, client, msg):
        future = client.call_async(msg)
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()


    def color_detect(self, img):
        img_h, img_w = img.shape[:2]
        frame_resize = cv2.resize(img, (320, 200), interpolation=cv2.INTER_NEAREST)
        frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)
        frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)
        detect_color = self.target_color
        frame_mask = cv2.inRange(frame_lab, tuple(self.lab_data['lab']['Stereo'][detect_color]['min']),
                                 tuple(self.lab_data['lab']['Stereo'][detect_color]['max']))
        eroded = cv2.erode(frame_mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
        dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
        contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
        center_x, center_y, angle = -1, -1, -1
        
        if contours:
            areaMaxContour, area_max = common.get_area_max_contour(contours, 10)
            if areaMaxContour is not None and area_max > 100: 
                rect = cv2.minAreaRect(areaMaxContour)
                angle = rect[2]
                box = np.intp(cv2.boxPoints(rect))
                
                for j in range(4):
                    box[j, 0] = int(common.val_map(box[j, 0], 0, 320, 0, img_w))
                    box[j, 1] = int(common.val_map(box[j, 1], 0, 200, 0, img_h))
                
                draw_color = (0, 255, 255) # 默认黄
                if detect_color == 'orange': draw_color = (0, 0, 255)
                elif detect_color == 'blue': draw_color = (255, 0, 0)
                
                cv2.drawContours(img, [box], -1, draw_color, 2)
                
                if self.current_task == 'search_pick' or self.start_pick:
                    center_x = int((box[0, 0] + box[2, 0]) / 2)
                    center_y = int((box[0, 1] + box[2, 1]) / 2)
                elif self.current_task == 'search_place' or self.start_place:
                    center_x = int((box[0, 0] + box[2, 0]) / 2)
                    y_coords = box[:, 1]
                    bottom_point_index = np.argmax(y_coords)
                    bottom_point = box[bottom_point_index] 
                    center_y = int(bottom_point[1])
                cv2.circle(img, (center_x, center_y), 5, (0, 0, 255), -1)
        return center_x, center_y, angle

    def action_thread(self):
        while rclpy.ok():
            if not self.running:
                time.sleep(0.1)
                continue
                
            if self.pick:
                self.pick = False 
                self.get_logger().info(f"--- 开始 {self.target_color} 抓取流程 ---")
                self.final_grasp_target_info, self.position = None, None
                self.locked_apriltag_id = None
                # 阶段1: 粗对准
                self.get_logger().info("阶段1: 粗对准...")
                start_time = time.time()
                while self.start_pick  and rclpy.ok():
                    if time.time() - start_time > 3000: 
                        self.get_logger().error("粗对准超时")
                        self.start_pick = False 
                        break
                    time.sleep(0.1)
                self.controller.traveling(gait=-2, time=1, steps=0)
                time.sleep(1)

                # 阶段3: 低头精确定位
                set_servo_position(self.joints_pub, 2, ((19, 500), (20, 700), (21, 155), (22, 70), (23, 500), (24, 700)))
                time.sleep(2)
                msg = BuzzerState()
                msg.freq = 2500
                msg.on_time = 0.1
                msg.off_time = 0.5
                msg.repeat = 1
                self.buzzer_pub.publish(msg)
                self.send_request(self.start_yolo_client, Trigger.Request())
                self.yolo_tracking_mode = 'TRACKING_FOR_GRASP'
                start_time = time.time()
                while self.final_grasp_target_info is None and rclpy.ok():
                    if time.time() - start_time > 100: break
                    time.sleep(0.1)
                
                self.send_request(self.stop_yolo_client, Trigger.Request())
                self.yolo_tracking_mode = 'IDLE'

                if self.final_grasp_target_info is None:
                    self.lidar_enable = True
                    self.controller.traveling(gait=-1, time=1, steps=0)
                    continue

                # 阶段4: 抓取执行
                self.get_logger().info("阶段4: 执行抓取...")
                self.get_world_pose(self.final_grasp_target_info["center_x"], self.final_grasp_target_info["center_y"])

                if self.position is not None:
                    self.position[0] += self.pick_offset[0]
                    self.position[1] += self.pick_offset[1]
                    self.position[2] += self.pick_offset[2]
                    msg = set_pose_target(self.position, 80.0, [-180.0, 180.0], 1.0)
                    res = self.send_request(self.set_pose_target_client, msg)
                    if res.pulse:
                        servo_data = res.pulse
                        set_servo_position(self.joints_pub, 1, ((19, servo_data[0]), ))
                        time.sleep(1)
                        set_servo_position(self.joints_pub, 1.5, ((19, servo_data[0]),(20, servo_data[1]), (21, servo_data[2]),(22, servo_data[3]), (23, self.yaw)))
                        time.sleep(1.5)
                        set_servo_position(self.joints_pub, 0.5, ((24, 450),))
                        time.sleep(1)
                        set_servo_position(self.joints_pub, 2, ((19, 500), (20, 700), (21, 155), (22, 70), (23, 500), (24, 500)))
                        time.sleep(2)
                        if self.debug in ['pick', 'pick_debug']:
                            return
                        self.get_logger().info('\033[1;32m抓取成功! 重新开启雷达\033[0m')
                        if self.locked_target_class == 'flammable':
                            self.play('finish_pick_flammable')
                        elif self.locked_target_class == 'explosives':
                            self.play('finish_pickexplosives')
                        else:
                            self.play('finish_pick_poison')
                        #抬头锁定标签码
                        set_servo_position(self.joints_pub, 2, ((19, 500), (20, 750), (21, 240), (22, 160), (23, 500), (24, 500)))
                        time.sleep(4)

                        if self.current_apriltag_id is not None:
                            self.locked_apriltag_id = self.current_apriltag_id
                        
                        self.get_logger().info(str(f"\033[1;32m锁定到的 AprilTag ID: {self.locked_apriltag_id}\033[0m"))
                        
                        if self.locked_apriltag_id == 1:
                            self.play('first_layer')
                        elif self.locked_apriltag_id == 2:
                            self.play('second_layer')
                        elif self.locked_apriltag_id == 3:
                            self.play('third_layer')
                        set_servo_position(self.joints_pub, 2, ((19, 500), (20, 700), (21, 155), (22, 70), (23, 500), (24, 500)))
                        time.sleep(2)
                        self.controller.traveling(gait=2, time=1, direction=math.radians(180), steps=5)
                        time.sleep(5)

                        self.current_task = 'search_place' 
                        self.target_color = 'blue'         
                        self.lidar_enable = True          
                        self.controller.traveling(gait=-1, time=1, steps=0)
                        time.sleep(1)                        

                    else:
                        self.get_logger().error("机械臂规划失败")
                        msg = BuzzerState()
                        msg.freq = 2500
                        msg.on_time = 0.1
                        msg.off_time = 0.5
                        msg.repeat = 3
                        self.buzzer_pub.publish(msg)
                        self.lidar_enable = True
                        self.controller.traveling(gait=2, time=1, steps=0)
                else:
                    self.get_logger().error("坐标计算失败")
                    msg = BuzzerState()
                    msg.freq = 2500
                    msg.on_time = 0.1
                    msg.off_time = 0.5
                    msg.repeat = 3
                    self.buzzer_pub.publish(msg)
                    self.lidar_enable = True
                    self.controller.traveling(gait=2, time=1, steps=0)

            elif self.place:
                self.place = False 
                self.get_logger().info(f"--- 开始放置流程 (TagID: {self.locked_apriltag_id}) ---")
                time.sleep(1)
                
                if self.locked_apriltag_id == 1:
                    self.get_logger().info("执行 ID=1 的放置动作")
                    set_servo_position(self.joints_pub, 1.5, ((19, 500), (20, 50), (21, 240), (22, 800), (23, 500), (24, 500)))
                
                elif self.locked_apriltag_id == 2:
                    self.get_logger().info("执行 ID=2 的放置动作")
                    set_servo_position(self.joints_pub, 1.5, ((19, 500), (20, 280), (21, 155), (22, 680), (23, 500), (24, 500)))
                
                elif self.locked_apriltag_id == 3:
                    self.get_logger().info("执行 ID=3 的放置动作")
                    set_servo_position(self.joints_pub, 1.5, ((19, 500), (20, 445), (21, 70), (22, 650), (23, 500), (24, 500)))

                else:
                    self.get_logger().info(f"执行 默认/其他ID 的放置动作 (ID={self.locked_apriltag_id})")
                    set_servo_position(self.joints_pub, 1.5, ((19, 500), (20, 445), (21, 70), (22, 650), (23, 500), (24, 500)))                
                time.sleep(1.5)

                if self.debug in ['place', 'place_debug']:
                    self.controller.traveling(gait=2, time=1, steps=4)
                    time.sleep(5)
                else:
                    self.controller.traveling(gait=2, time=1, steps=7)
                    time.sleep(8)
                set_servo_position(self.joints_pub, 0.5, ((24, 700),))
                time.sleep(0.5)
                if self.debug in ['place', 'place_debug']:
                    return
                self.controller.traveling(gait=2, time=1, direction=math.radians(180), steps=2)
                time.sleep(3)
                set_servo_position(self.joints_pub, 1.5, ((19, 500), (20, 700), (21, 155), (22, 70), (23, 500), (24, 700)))
                time.sleep(1.5)
                self.play('place')

                self.controller.traveling(gait=2, time=1, direction=math.radians(180), steps=4)#后退
                time.sleep(5)
                self.controller.traveling(gait=2, time=1, rotation=0.9, steps=9) # 旋转对齐
                time.sleep(10)
                self.controller.traveling(gait=2, time=1, steps=6)#前进
                time.sleep(7)
                self.get_logger().info('\033[1;32m放置完成! 重新开启雷达，开启台阶检测\033[0m')

                self.controller.traveling(gait=-1, time=1, steps=0)
                time.sleep(2)
                
                self.lidar_enable = True
                self.step_check_enabled = True # 开启台阶检测 ---
                
                self.follow_side = 'RIGHT'          # 1. 强制设定为巡右墙
                self.force_side_expire_time = time.time() + 11.0 # 2. 设定11秒锁定时间
                self.get_logger().warn(">>> 强制锁定：巡航右墙 (持续15秒) <<<")
                self.stick_target_dist = 0.31 #修改循墙行驶的距离
                self.current_task = 'mission_completed'  # 将任务状态改为完成，main函数将不再进行颜色匹配

            else:
                time.sleep(0.01)

    def pick_handle(self, image, depth_image, depth_camera_info):
        twist = Twist()

        if not self.pick or self.debug == 'pick': 
            object_center_x, object_center_y, object_angle = self.color_detect(image)
            if self.debug == 'pick':
                self.detect_count += 1
                if self.detect_count > 20:
                    self.detect_count = 0
                    self.pick_stop_y = object_center_y
                    self.pick_stop_x = object_center_x
                    data = common.get_yaml_data(self.config_path)
                    data['/**']['ros__parameters']['pick_stop_pixel_coordinate'] = [self.pick_stop_x, self.pick_stop_y]
                    common.save_yaml_data(data, self.config_path)
                    self.debug = 'pick_debug'
                self.get_logger().info('x_y: ' + str([object_center_x, object_center_y]))  # 打印当前物体中心的像素(print the pixel of the current object's center)

            elif object_center_x > 0:
                self.target_lost_count = 0 # 清零丢失计数

                # 如果之前是雷达模式
                if self.lidar_enable:
                    self.rediscover_count += 1
                    cv2.putText(image, f"Rediscovering: {self.rediscover_count}/{self.REDISCOVER_THRESHOLD}", 
                           (10, 280), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                
                    if self.rediscover_count >= self.REDISCOVER_THRESHOLD:
                        self.get_logger().info(f"\033[1;32m稳定重新发现抓取目标({self.target_color})！切换回视觉追踪模式。\033[0m")

                        self.controller.traveling(gait=-2, time=1, steps=0) 
                        time.sleep(2)
                        self.start_pick = True
                        self.lidar_enable = False # 关闭雷达
                        
                        # 重置 PID 防止积分瞬间暴冲
                        self.linear_pid.clear()
                        self.angular_pid.clear()
                        self.yaw_pid.clear()
                        self.status = "approach" # 重置为接近状态，重新调整位置
                        self.count_stop = 0
                else:
                    self.rediscover_count = 0

                if self.status != "align":
                    self.linear_pid.SetPoint = self.pick_stop_y
                    if abs(object_center_y - self.pick_stop_y) <= self.d_y: object_center_y = self.pick_stop_y
                    self.linear_pid.update(object_center_y)
                    tmp_linear = self.linear_base_speed + self.linear_pid.output
                    self.linear_speed = np.clip(tmp_linear / 7, -0.02, 0.02)
                    if abs(tmp_linear) <= 0.0075: self.linear_speed = 0

                    self.angular_pid.SetPoint = self.pick_stop_x
                    if abs(object_center_x - self.pick_stop_x) <= self.d_x: object_center_x = self.pick_stop_x
                    self.angular_pid.update(object_center_x)
                    tmp_angular = self.angular_base_speed + self.angular_pid.output
                    self.angular_speed = np.clip(tmp_angular / 3, -0.2, 0.2)
                    if abs(tmp_angular) <= 0.038: self.angular_speed = 0

                twist.linear.x = float(self.linear_speed)
                twist.angular.z = float(self.angular_speed)

                if abs(self.linear_speed) == 0 and abs(self.angular_speed) == 0:
                    self.count_turn += 1
                    if self.count_turn > 5:
                        self.count_turn = 5
                        self.status = "align" 
                        
                        if self.count_stop < 5:
                            if object_angle < 40: object_angle += 90
                            self.yaw_pid.SetPoint = 90
                            if abs(object_angle - 90) <= 1: object_angle = 90
                            self.yaw_pid.update(object_angle)
                            self.yaw_angle = self.yaw_pid.output
                            
                            if object_angle != 90:
                                if abs(self.yaw_angle) <= 0.038:
                                    self.count_stop += 1
                                else:
                                    self.count_stop = 0
                                twist.linear.y = float(-2 * 0.3 * math.sin(self.yaw_angle / 2))
                                twist.angular.z = float(self.yaw_angle / 2)
                            else:
                                self.count_stop += 1
                        
                        elif self.count_stop <= 20:
                            self.d_x = 30
                            self.d_y = 30
                            self.count_stop += 1
                            self.status = "adjust"
                        
                        else:
                            self.get_logger().info("粗对准完成，停止移动。")
                            self.cmd_vel_pub.publish(Twist()) 
                            self.start_pick = False 
                else:
                    self.count_turn = 0
                    self.status = "approach"
            else:
                self.rediscover_count = 0 # 找回计数清零
                # 未检测到目标（丢失）
                self.target_lost_count += 1
                # 如果连续丢失超过阈值，且雷达还未开启
                if self.target_lost_count > self.LOST_THRESHOLD:
                    if not self.lidar_enable:
                        self.get_logger().warn(f"\033[1;33m丢失 {self.target_color} 目标！切换至雷达避障/搜索模式。\033[0m")
                        self.controller.traveling(gait=-1, time=1, steps=0) # 切换行走步态搜索
                        time.sleep(2)
                        self.lidar_enable = True

            if self.start_pick and not self.lidar_enable:
                self.cmd_vel_pub.publish(twist)

            return image

    def place_handle(self, image):
        twist = Twist()
        object_center_x, object_center_y, object_angle = self.color_detect(image)
        if not self.place or self.debug == 'place':
            if self.debug == 'place':
                self.detect_count += 1
                if self.detect_count > 20:
                    self.detect_count = 0
                    self.place_stop_y = object_center_y
                    self.place_stop_x = object_center_x
                    data = common.get_yaml_data(self.config_path)
                    data['/**']['ros__parameters']['place_stop_pixel_coordinate'] = [self.place_stop_x, self.place_stop_y]
                    common.save_yaml_data(data, self.config_path)
                    self.debug = 'place_debug'
                self.get_logger().info('x_y: ' + str([object_center_x, object_center_y]))  # Print the pixel of the current object's center(打印当前物体中心的像素)

            elif object_center_x > 0: 
                self.target_lost_count = 0
                
                if self.lidar_enable:
                    self.rediscover_count += 1
                    cv2.putText(image, f"Rediscovering: {self.rediscover_count}/{self.REDISCOVER_THRESHOLD}", 
                            (10, 280), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    
                    if self.rediscover_count >= self.REDISCOVER_THRESHOLD:

                        self.get_logger().info(f"\033[1;32m稳定重新发现放置目标！切换回视觉追踪模式。\033[0m")
                        self.controller.traveling(gait=-2, time=1, steps=0)
                        time.sleep(2)
                        self.lidar_enable = False
                        self.start_place = True

                        self.linear_pid.clear()
                        self.angular_pid.clear()
                        self.yaw_pid.clear()
                        self.status = "approach"
                        self.count_stop = 0
                else:
                    self.rediscover_count = 0
                if self.status == "approach":
                    self.linear_pid.SetPoint = self.place_stop_y
                    if abs(object_center_y - self.place_stop_y) <= self.d_y: object_center_y = self.place_stop_y
                    self.linear_pid.update(object_center_y)
                    tmp_linear = self.linear_base_speed + self.linear_pid.output
                    self.linear_speed = np.clip(tmp_linear / 7, -0.02, 0.02)
                    if abs(tmp_linear) <= 0.0075: self.linear_speed = 0

                    self.angular_pid.SetPoint = self.place_stop_x
                    if abs(object_center_x - self.place_stop_x) <= self.d_x: object_center_x = self.place_stop_x
                    self.angular_pid.update(object_center_x)
                    tmp_angular = self.angular_base_speed + self.angular_pid.output
                    self.angular_speed = np.clip(tmp_angular / 3, -0.2, 0.2)
                    if abs(tmp_angular) <= 0.038: self.angular_speed = 0

                if abs(self.linear_speed) == 0 and abs(self.angular_speed) == 0:
                    self.count_turn += 1
                    if self.count_turn > 5:
                        self.count_turn = 5
                        self.status = "align"
                else:
                    self.count_turn = 0
                
                if self.status == "align":
                    if self.count_stop < 5:
                        if object_angle < 40: object_angle += 90
                        self.yaw_pid.SetPoint = 90
                        if abs(object_angle - 90) <= 1: object_angle = 90
                        self.yaw_pid.update(object_angle)
                        self.yaw_angle = self.yaw_pid.output
                        
                        if abs(self.yaw_pid.output) <= 0.038:
                            self.count_stop += 1
                        else:
                            self.count_stop = 0
                        
                        twist.linear.y = float(-2 * 0.3 * math.sin(self.yaw_angle / 2))
                        twist.angular.z = float(self.yaw_angle / 2)
                        self.get_logger().warn(f"\033[1;33m线性速度 {twist.linear.y} \033[0m")
                        self.get_logger().warn(f"\033[1;33m角度 {twist.angular.z} \033[0m")

                    else:
                        self.status = "adjust" 
                
                if self.status == "adjust":
                    if self.count_stop <= 20:
                        self.d_x = 30
                        self.d_y = 30
                        self.count_stop += 1
                    else:
                        self.get_logger().info("\033[1;32m放置对准完成\033[0m")
                        self.controller.traveling(gait=-2, time=1, steps=0)
                        self.start_place = False          
                        self.place = True                 
                
                twist.linear.x = float(self.linear_speed)
                if self.status == "align":
                    twist.angular.z = float(self.yaw_angle / 2)
                else:
                    twist.angular.z = float(self.angular_speed)
            else:
                self.rediscover_count = 0 # 找回计数清零

                # 未检测到目标（丢失）
                self.target_lost_count += 1
                if self.target_lost_count > self.LOST_THRESHOLD:
                    if not self.lidar_enable:
                        self.get_logger().warn(f"\033[1;33m丢失放置目标！切换至雷达避障/搜索模式。\033[0m")
                        self.controller.traveling(gait=-1, time=1, steps=0) # 切换行走步态搜索
                        time.sleep(2)
                        self.lidar_enable = True

        if self.start_place and not self.lidar_enable:
            self.cmd_vel_pub.publish(twist)
        
        return image

    def main(self):
        while rclpy.ok():
            if not self.running:
                time.sleep(0.1)
                continue
            try:
                # 获取图像数据
                image = self.image_queue.get(block=True, timeout=0.1)
                depth_image_msg = self.depth_image_queue.get(block=True, timeout=0.1)
                depth_camera_info = self.camera_info_queue.get(block=True, timeout=0.1)
                depth_image = self.bridge.imgmsg_to_cv2(depth_image_msg, '16UC1')

                try:
                    yolo_image = self.yolo_image_queue.get(block=False)
                except queue.Empty:
                    pass

                # 提取内参
                K = depth_camera_info.k
                intrinsics = (K[0], K[4], K[2], K[5]) # fx, fy, cx, cy
            except queue.Empty:
                continue
            
            if image is None or image.size == 0:
                continue

            result_image = image.copy()
            
            # --- 地面高度校准 (Debug 模式) ---
            if self.start_height_calibration:
                current_dist = self.get_roi_distance(depth_image, self.center_roi)
                if current_dist > 0:
                    self.detect_count += 1
                    cv2.putText(result_image, f"Calibrating Height: {self.detect_count}/30", (10, 360), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
                    cv2.putText(result_image, f"Current Dist: {current_dist:.3f}m", (10, 380), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                    
                    if self.detect_count >= 30:
                        self.plane_height_threshold = current_dist
                        self.get_logger().info(f"\033[1;32m校准完成！地面高度阈值已更新为: {self.plane_height_threshold:.3f}m\033[0m")
                        
                        # 保存到配置文件
                        try:
                            data = common.get_yaml_data(self.config_path)
                            if '/**' not in data: data['/**'] = {}
                            if 'ros__parameters' not in data['/**']: data['/**']['ros__parameters'] = {}
                            data['/**']['ros__parameters']['plane_height_threshold'] = self.plane_height_threshold
                            common.save_yaml_data(data, self.config_path)
                            self.get_logger().info(f"配置文件已更新: {self.config_path}")
                        except Exception as e:
                            self.get_logger().error(f"保存配置文件失败: {str(e)}")
                        
                        self.detect_count = 0
                        self.start_height_calibration = False
                else:
                    cv2.putText(result_image, "Waiting for valid depth...", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                    self.get_logger().warn("校准中：未获取到有效深度数据")

            # --- 自动检测/雷达巡航模式逻辑 ---
            # 只有在开启雷达，且没有开始抓取/放置任务时，才进行识别
            if self.lidar_enable and not self.start_pick and not self.start_place:
                
                # --- 寻找坍塌区落差检测 ---
                if self.waiting_for_collapse:

                    current_dist = self.get_roi_distance(depth_image, self.center_roi)

                    # 检测到落差超过 0.04m 
                    if current_dist > 0 and (current_dist - self.plane_height_threshold) > 0.04:
                        self.step_detect_count += 1
                        if self.step_detect_count > 2:
                            self.get_logger().info(f"\033[1;32m检测到坍塌区(落差: {current_dist - self.plane_height_threshold:.3f}m)，切换跨越姿态！\033[0m")
                            # 蜂鸣器提示
                            msg = BuzzerState()
                            msg.freq = 3000
                            msg.on_time = 0.5
                            msg.off_time = 0.5
                            msg.repeat = 2
                            self.buzzer_pub.publish(msg)
                            self.waiting_for_collapse = False # 停止寻找
                            self.lidar_hole = True           # 进入正式跨越模式
                            self.set_hole_posture()      # 切换为 HOLE_POSE
                            self.hole_time = time.time()

                            self.step_detect_count = 0
                    else:
                        self.step_detect_count = 0  

                # --- 台阶检测  ---
                if self.step_check_enabled:
                    diff = self.plane_height_threshold - self.get_roi_distance(depth_image, self.center_roi)
                    # 绘制辅助线
                    cv2.line(result_image, (315, 300), (325, 300), (0, 0, 255) if diff < self.plane_height_threshold else (0, 255, 0), 2)
                    
                    if diff > 0.05: # 检测到障碍/台阶
                        self.step_detect_count += 1
                        if self.step_detect_count > 3:
                            self.get_logger().info(f"\033[1;31m检测到台阶/障碍 (diff {diff:.3f}m)，停车！\033[0m")
                            self.lidar_enable = False
                            self.step_check_enabled = False
                            self.cmd_vel_pub.publish(Twist()) # 强制停车
                            
                            # 蜂鸣器报警
                            msg = BuzzerState()
                            msg.freq = 3000
                            msg.on_time = 0.5
                            msg.off_time = 0.5
                            msg.repeat = 1
                            self.buzzer_pub.publish(msg)

                             # 3. 开启雷达，进入对齐状态，而不是直接 is_up_steps
                            self.lidar_enable = True 
                            self.is_aligning_step = True
                            self.step_align_start_time = time.time()

                    else:
                        self.step_detect_count = 0
    
                # --- 颜色识别逻辑 
                current_frame_has_target = False

                    # 如果任务状态是 completed，这里不会执行，实现了放置后停止识别
                if self.current_task == 'search_pick':
                    self.target_color = 'orange'
                    cx, cy, _ = self.color_detect(result_image)
                    if cx > 0: 
                        current_frame_has_target = True
                        self.color_found_count += 1
                        cv2.putText(result_image, f"Confirming Pick: {self.color_found_count}/{self.FOUND_THRESHOLD}", 
                                    (10, 250), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                        
                        if self.color_found_count >= self.FOUND_THRESHOLD:
                            self.color_found_count = 0
                            self.trigger_pick_process()
                
                elif self.current_task == 'search_place':
                    self.target_color = 'blue'
                    cx, cy, _ = self.color_detect(result_image)
                    if cx > 0: 
                        current_frame_has_target = True
                        self.color_found_count += 1
                        cv2.putText(result_image, f"Confirming Place: {self.color_found_count}/{self.FOUND_THRESHOLD}", 
                                    (10, 250), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
                        
                        if self.color_found_count >= self.FOUND_THRESHOLD:
                            self.color_found_count = 0
                            self.trigger_place_process()

                # 如果本帧没有检测到任何有效目标，重置计数器
                if not current_frame_has_target:
                    self.color_found_count = 0

            # --- 任务执行逻辑 ---
            if self.start_pick:
                # 安全检查 2: 接收处理后的图像，防止返回 None
                processed = self.pick_handle(result_image, depth_image, depth_camera_info)
                if processed is not None:
                    result_image = processed

            elif self.start_place:
                processed = self.place_handle(result_image)
                if processed is not None:
                    result_image = processed

            # --- 绘制 UI 信息 ---
            # 绘制停止点标记
            if self.start_pick:
                cv2.drawMarker(result_image, (self.pick_stop_x, self.pick_stop_y), (0, 255, 0), cv2.MARKER_CROSS, 20, 2)
            elif self.start_place:
                cv2.drawMarker(result_image, (self.place_stop_x, self.place_stop_y), (255, 0, 0), cv2.MARKER_CROSS, 20, 2)

            # 绘制状态文字
            mode_text = "Lidar: ON" if self.lidar_enable else "Lidar: OFF"
            cv2.putText(result_image, mode_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255) if not self.lidar_enable else (0, 255, 0), 2)
            cv2.putText(result_image, f"Task: {self.current_task}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
            
            # 显示当前追踪颜色 (如果是雷达模式，显示 Search: Color，如果是追踪模式，显示 Tracking: Color)
            target_str = self.target_color
            status_prefix = "Search" if self.lidar_enable else "Tracking"
            cv2.putText(result_image, f"{status_prefix}: {target_str}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)
            cv2.putText(result_image, self.current_dist_text, (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.putText(result_image, self.current_dist_text_1, (10, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            status_str = self.status
            status_prefix = "status_str" 
            cv2.putText(result_image, f"{status_prefix}: {status_str}", (10, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (105, 0, 255), 2)

            # --- 显示图像 ---
            if self.enable_display:
                # 安全检查 3: 确保图像尺寸有效
                if result_image is not None and result_image.shape[0] > 0 and result_image.shape[1] > 0:

                    if self.yolo_tracking_mode == 'TRACKING_FOR_GRASP':
                        result_image = yolo_image

                    cv2.imshow(self.name, result_image)
                    key = cv2.waitKey(1)
                    if key & 0xFF == ord('q'):
                        break
                else:
                    # 避免空图报错
                    pass
            # 发布图像
            if result_image is not None:
                try:
                    self.image_pub.publish(self.bridge.cv2_to_imgmsg(result_image, "bgr8"))
                except Exception:
                    pass

def main():
    node = AutomaticPickNode('automatic_pick')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.destroy_node()
        rclpy.shutdown()
 
if __name__ == "__main__":
    main()