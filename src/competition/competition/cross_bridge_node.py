#!/usr.bin/env python3
# encoding: utf-8
import os
import cv2
import time
import math
import enum
import rclpy
import queue
import signal
import threading
import numpy as np
from sdk import common
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist
from interfaces.msg import CmdParam
from interfaces.srv import SetString 
from controller import step_controller
from controller import controller_client 
from sensor_msgs.msg import Image, CameraInfo
from servo_controller_msgs.msg import ServosPosition
from rclpy.callback_groups import ReentrantCallbackGroup
from servo_controller.bus_servo_control import set_servo_position

from sensor_msgs.msg import Image, CameraInfo, LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class State(enum.Enum):
    PRE_UP_LIDAR_ALIGN = 1   # 上台阶前的雷达对齐
    UP_STEPS = 2             # 初始上台阶
    POST_UP_ROTATING = 3     # 上台阶后的旋转回正
    ALIGNING = 4             # 原地调整/对准
    CROSSING = 5             # 直行穿越
    FINISHED = 6             # 穿越完成
    LIDAR_ALIGNING = 7       # 雷达旋转对齐
    DONE = 8                 # 结束

class CrossBridgeNode(Node):
    def __init__(self, name):
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.name = name
        signal.signal(signal.SIGINT, self.shutdown)
        self.running = False
        
        self.state = State.PRE_UP_LIDAR_ALIGN
        self.current_pose = 'DEFAULT_POSE' # 当前姿态
        self.desired_height = 20 # 步态高度 (mm)
        self.cross_count = 0 # 完成计数，用于判断是否完成穿越
        self.timestamp = 0    # 记录当前时间戳，用于状态转换
        self.up_steps_time = 11.0       # 上台阶时间 (秒)
        self.descend_duration = 14.0 # 下台阶时间 (秒)
        
        # 桥面/地面的有效深度范围 (单位: 毫米)
        # 你需要根据实际情况调整这个范围，目的是把“桥面”和“深渊”分开
        self.min_depth = 410  # 最近距离
        self.max_depth = 450  # 最远距离 (超过这个距离认为是地面或者深渊)

        # 视觉对准的PID 参数 (简单的P控制)
        self.yaw_p = 0.05    # 角度修正系数
        self.lat_p = 0.0005   # 横向修正系数DEAD_ZONE_X 
        self.image_center_x = 320 # 图像中心 X 坐标
        self.aligned_counter = 0 # 对齐计数器，用于判断是否对齐

        # --- 雷达居中相关参数 ---
        self.min_dist_front = 9.99 # 前方触发距离 (米)
        self.avg_dist_left = 2.0  # 左侧平均距离 (米)
        self.avg_dist_right = 2.0  # 右侧平均距离 (米)
        self.left_wall_slope = 0.0  # 左侧墙的斜率
        self.pre_align_timestamp = 0.0 # 上一次对齐时间戳
        
        # 穿越完成后雷达控制增益
        self.kp_rotate = 0.5   # 转向对齐增益
        self.kp_strafe = 0.1   # 平移居中增益

        self.debug = self.get_parameter('debug').value
        self.image_queue = queue.Queue(maxsize=2)
        self.controller = controller_client.ControllerClient()
        self.joints_pub = self.create_publisher(ServosPosition, '/servo_controller', 1) 
        self.cmd_vel_pub = self.create_publisher(Twist, '/controller/cmd_vel', 1) 
        self.cmd_param_pub = self.create_publisher(CmdParam, '/step_controller/cmd_param', 1) 
        self.finish_pub = self.create_publisher(Bool, '~/finish', 1) 
        
        self.client = self.create_client(Trigger, '/controller_manager/init_finish')
        self.client.wait_for_service()

        self.create_service(Trigger, '~/enter', self.enter_srv_callback) 
        self.create_service(Trigger, '~/exit', self.exit_srv_callback)   
        timer_cb_group = ReentrantCallbackGroup()
        self.create_subscription(Bool, '/narrow_slit_traversal/finish', self.narrow_finish_callback, 1, callback_group=timer_cb_group)

        lidar_qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, lidar_qos)

        threading.Thread(target=self.main, daemon=True).start()
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'Cross Bridge/Hole Node Started (Vision Upgrade)')

    def lidar_callback(self, lidar_data):
        ranges = np.array(lidar_data.ranges)
        # 清洗数据
        ranges[ranges == 0] = 9.99
        ranges[np.isinf(ranges)] = 9.99
        
        angle_increment = lidar_data.angle_increment
        
        # 2. 区域提取与坐标转换 (拟合左墙)
        left_pts_x, left_pts_y = [], []
        for i, r in enumerate(ranges):
            angle = i * angle_increment
            # 收集左侧墙壁点云 (30度 到 100度)
            if 0.1 < r < 1.0:
                if math.radians(30) < angle < math.radians(100):
                    # 转换为直角坐标 (x向前, y向左)
                    x = r * math.cos(angle)
                    y = r * math.sin(angle)
                    left_pts_x.append(x)
                    left_pts_y.append(y)

        # 3. 计算关键指标
        # A. 拟合左墙斜率
        left_wall_slope, _ = self.fit_line(left_pts_x, left_pts_y)
        if left_wall_slope is not None:
            self.left_wall_slope = left_wall_slope
        else:
            self.left_wall_slope = 0.0

        # B. 侧边区域用于计算平均距离（居中用）
        idx_30 = int(math.radians(30) / angle_increment)
        idx_90 = int(math.radians(90) / angle_increment)
        left_side = ranges[idx_30 : idx_90]
        right_side = ranges[-idx_90 : -idx_30]

        # C. 前方区域用于避障
        idx_front = int(math.radians(20) / angle_increment)
        front_ranges = np.concatenate((ranges[:idx_front], ranges[-idx_front:]))

        # 过滤有效范围 (0.1m 到 2.0m)
        valid_l = left_side[(left_side > 0.1) & (left_side < 2.0)]
        valid_r = right_side[(right_side > 0.1) & (right_side < 2.0)]
        valid_f = front_ranges[(front_ranges > 0.1) & (front_ranges < 2.0)]

        # 更新均值和最小值
        self.avg_dist_left = np.mean(valid_l) if len(valid_l) > 0 else 1.0
        self.avg_dist_right = np.mean(valid_r) if len(valid_r) > 0 else 1.0
        self.min_dist_front = np.min(valid_f) if len(valid_f) > 0 else 9.99

    def narrow_finish_callback(self, msg):
        self.enter_srv_callback(None, None)
        self.get_logger().info(' self.narrow = msg.data ')

    def get_node_state(self, request, response):
        response.success = True
        return response

    def depth_callback(self, ros_depth_image):
        depth_image = np.ndarray(shape=(ros_depth_image.height, ros_depth_image.width), dtype=np.uint16,
                                 buffer=ros_depth_image.data)
        if self.image_queue.full():
            self.image_queue.get()
        self.image_queue.put(depth_image)

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

    def shutdown(self, signum, frame):
        self.running = False

    def exit_srv_callback(self, request, response):
        self.running = False
        self.controller.traveling(gait=-2, time=1, steps=0)
        if response: response.success = True
        return response

    def set_normal_posture(self):
        """恢复正常站立姿态"""
        set_servo_position(self.joints_pub, 1, ((19, 500), (20, 720), (21, 120), (22, 140), (23, 500), (24, 700)))
        time.sleep(1.0)  
        cmd_param = CmdParam()
        cmd_param.gait = 2
        cmd_param.period = 1.0
        cmd_param.pose = 'SLAM_POSE'
        cmd_param.height = 20
        self.cmd_param_pub.publish(cmd_param)
        time.sleep(2.0)  # 确保姿态变形完成

    def set_down_steps_posture(self):
        """设置上下台阶姿态"""
        cmd_param = CmdParam()
        cmd_param.gait = 2
        cmd_param.period = 2.0
        cmd_param.pose = 'SLAM_POSE' 
        cmd_param.height = 60
        self.cmd_param_pub.publish(cmd_param)
        time.sleep(1.0)  # 确保姿态变形完成

    def enter_srv_callback(self, request, response):
        self.running = True
        self.cross_count = 0
        # --- 上台阶前的对齐逻辑 ---
        self.get_logger().info("Starting Pre-Up Lidar Alignment...")
        self.pre_align_timestamp = time.time() + 5.0 # 先对齐5秒
        self.state = State.PRE_UP_LIDAR_ALIGN

        self.image_sub = self.create_subscription(Image, '/depth_cam/depth/image_raw', self.depth_callback, 1)
        if response:
            response.success = True
        return response

    # --- 核心算法：分析桥面 ---
    def process_bridge_vision(self, depth_image):
        """
        处理深度图，返回：
        1. 桥面中心点的 X 坐标 (center_x)
        2. 桥面的倾斜角度 (angle) - 垂直为0，左偏为负，右偏为正
        3. 是否找到有效桥面 (found)
        4. 处理后的用于显示的图像 (debug_img)
        """
        h, w = depth_image.shape
        # 1. 预处理：裁剪感兴趣区域 (ROI)
        # 我们只关心前方地面，不需要看太远或太近的边缘
        roi_top = 100
        roi_bottom = 380
        roi_img = depth_image[roi_top:roi_bottom, :]
        
        # 2. 二值化：提取“桥面高度”
        # 这里的 min_depth 和 max_depth 非常关键，需要调试
        # 只有在这个距离范围内的物体才被认为是桥面
        mask = cv2.inRange(roi_img, self.min_depth, self.max_depth)
        
        # 3. 形态学操作：去噪
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # 4. 寻找轮廓
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        center_x = 0
        angle = 0
        width= 0
        found = False
        debug_img = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR) # 转彩色用于画图
        
        if contours:
            # 找到面积最大的轮廓（假设最大的就是桥）
            max_cnt = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(max_cnt)
            
            # 只有面积足够大才算找到了
            if area > 2000:
                found = True
                # 拟合最小外接矩形 (Center(x,y), (width, height), angle_of_rotation)
                rect = cv2.minAreaRect(max_cnt)
                     
                #使用最长边向量法计算角度 ---
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                cv2.drawContours(debug_img, [box], 0, (0, 0, 255), 2)
                
                # 计算矩形四条边的长度，找出长边
                # box[0], box[1], box[2], box[3] 是四个顶点
                edge1_len = np.linalg.norm(box[0] - box[1])
                edge2_len = np.linalg.norm(box[1] - box[2])
                
                # 确定主要方向向量 (vx, vy)
                # 注意：图像坐标系中，y向下是正
                if edge1_len > edge2_len:
                    # 边 0-1 是长边
                    vx = box[1][0] - box[0][0]
                    vy = box[1][1] - box[0][1]
                else:
                    # 边 1-2 是长边
                    vx = box[2][0] - box[1][0]
                    vy = box[2][1] - box[1][1]

                # 现在的向量 (vx, vy) 就是桥的走向
                # 也就是桥在图像中的斜率
                
                # 使用 atan2 计算角度 (单位：度)
                # atan2(y, x) 返回的是与 X 轴的夹角
                # 垂直向下的直线的角度应该是 90度 (因为 y 是正的)
                raw_angle = math.degrees(math.atan2(vy, vx))
                
                # 我们希望：垂直向下时 angle = 0
                # 向右歪 ( / ) 时 angle > 0
                # 向左歪 ( \ ) 时 angle < 0
                
                # 转换公式：
                # 如果线是垂直的 (vx=0, vy=1)，atan2 返回 90。 我们要 0。 -> 90 - 90 = 0
                # 如果线是水平的 (vx=1, vy=0)，atan2 返回 0。  我们要 -90。
                
                # 处理 atan2 的周期性 (-180 到 180)
                # 我们先把向量统一成 "y 是向下的" (保证 vy > 0)
                if vy < 0:
                    vx = -vx
                    vy = -vy
                    
                # 重新计算简单的偏差角
                # 垂直线的 vx 应该是 0
                # 利用反正切直接求偏差：atan(dx / dy)
                # 当 dx=0 (垂直), angle=0
                # 当 dx>0 (向右偏), angle>0
                if vy != 0:
                    angle = math.degrees(math.atan(vy / vx))
                else:
                    angle = 90.0 # 极端的水平情况

                # ===计算路面宽度 ===
                w_rect, h_rect = rect[1]
                width = max(w_rect, h_rect) 
                
                # === 在画面上打印宽度 ===
                cv2.putText(debug_img, f"Width: {width:.1f}", (10, 90), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

                # 计算中心点
                center_x = int(rect[0][0])
                center_y = int(rect[0][1])
                cv2.circle(debug_img, (center_x, center_y), 5, (0, 255, 0), -1)
                
                # 计算角度 (确保角度是相对于垂直方向的偏角)
                # minAreaRect 的角度范围比较诡异，通常是 -90 到 0 或 0 到 90
                # 我们需要把它转换成：垂直向前是0度，向左偏是负，向右偏是正
                width , height = rect[1]
                raw_angle = rect[2]
                
                if width < height:
                    angle = raw_angle
                else:
                    angle = raw_angle + 90
                
                # 简单的角度修正，限制在 +/- 90度内
                if angle > 90: angle -= 180
                
                # 画出中心线方向
                rows, cols = debug_img.shape[:2]
                vx = math.sin(math.radians(angle))
                vy = -math.cos(math.radians(angle)) # y轴向下为正，所以向上是负
                cv2.line(debug_img, (center_x, center_y), (int(center_x + vx * 100), int(center_y + vy * 100)), (255, 0, 0), 2)

        return center_x, angle, width, found, debug_img

 # --- 分步对准策略：先修角度，再修平移 ---
    def new_move_policy(self, center_x, angle, width, found):
        self.twist = Twist()
        desired_pose = self.current_pose
        desired_height = self.desired_height
        # --- 阈值设置 ---
        DEAD_ZONE_ANGLE = 3.0 # 度：角度容忍范围
        DEAD_ZONE_X = 10     # 像素：横向容忍范围
        period = 1.0

        # -1. 状态：上台阶前的雷达对齐 (PRE_UP_LIDAR_ALIGN)
        if self.state == State.PRE_UP_LIDAR_ALIGN:
            # 1. 旋转对齐左侧墙壁 (让 left_wall_slope 趋于 0)
            # 2. 平移居中 (让 avg_dist_left 和 avg_dist_right 趋于相等)
            slope = self.left_wall_slope
            
            # 先旋转对齐 (使用最小二乘法拟合的斜率)
            if abs(slope) > 0.15:
                self.twist.angular.z = float(np.clip(slope * 2.0, -0.15, 0.15))
                self.twist.linear.y = 0.0
            # 再平移居中
            elif self.avg_dist_left > 0.45:
                self.twist.angular.z = 0.0
                self.twist.linear.y = 0.03
            elif self.avg_dist_left < 0.4:
                self.twist.angular.z = 0.0
                self.twist.linear.y = -0.03                
            else:
                self.twist.angular.z = 0.0
                self.twist.linear.y = 0.0
            
            if time.time() > self.pre_align_timestamp:
                if abs(slope) < 0.15 and 0.4 < self.avg_dist_left < 0.45:
                    self.twist = Twist()
                    self.cmd_vel_pub.publish(self.twist)
                    
                    # 切换到上台阶准备动作
                    self.controller.traveling(gait=-1, time=1, steps=0)
                    time.sleep(1.0)
                    self.set_down_steps_posture()
                    self.timestamp = time.time() + self.up_steps_time
                    self.state = State.UP_STEPS
                else:
                    # 如果时间到了但还没对齐好，再延时一点
                    self.pre_align_timestamp = time.time() + 1.0

        # 0. 状态：初始上台阶 (UP_STEPS)
        elif self.state == State.UP_STEPS:
            if time.time() < self.timestamp:
                self.twist.linear.x = 0.04 
                self.cmd_vel_pub.publish(self.twist)

            else:
                self.twist.linear.x = 0.0
                self.cmd_vel_pub.publish(self.twist)
                self.set_normal_posture()
                time.sleep(2.0) # 确保姿态恢复
                
                self.state = State.POST_UP_ROTATING

        # 0.1 状态：上台阶后的旋转回正 (POST_UP_ROTATING)
        elif self.state == State.POST_UP_ROTATING:
            # 调整舵机角度，确保摄像头能看到前方大约 0.5米-1.5米的桥面
            set_servo_position(self.joints_pub, 1, ((19, 500), (20, 727), (21, 70), (22, 160), (23, 500), (24, 700)))
            time.sleep(1.0)
            # 进入旋转状态
            self.controller.traveling(gait=2, time=1, rotation=-0.9, steps=9)
            time.sleep(10.0)

            self.state = State.ALIGNING
            self.get_logger().info("Transition to ALIGNING state.")

        # 1. 状态：对准阶段 (ALIGNING)
        elif self.state == State.ALIGNING:
            desired_pose = 'DEFAULT_POSE'
            desired_height = 20
            period = 1.0
            if not found:
                self.get_logger().warn("丢失目标，暂停动作")
                self.twist.linear.x = 0.0
                self.twist.linear.y = 0.0
                self.twist.angular.z = 0.1
            else:
                # 计算误差
                err_x = self.image_center_x - center_x 
                slope = self.left_wall_slope # 上桥后对准仍使用雷达斜率
                # 情况A：位移(视觉)或角度(雷达)偏差较大 -> 执行调整
                if abs(err_x) > DEAD_ZONE_X or abs(slope) > 0.1:
                    self.aligned_counter = 0
                    # 优先修正角度 (使用雷达斜率)
                    if abs(slope) > 0.1:
                        self.twist.angular.z = float(np.clip(slope * 2.0, -0.1, 0.1))
                        self.twist.linear.y = 0.0
                        self.get_logger().info(f"Aligning Angle (Lidar): Slope={slope:.3f}", throttle_duration_sec=1.0)
                    # 角度对准后，再修正位移 
                    else:
                        lat_speed = err_x * self.lat_p
                        self.twist.linear.y = float(np.clip(lat_speed, -0.02, 0.02))
                        self.twist.angular.z = 0.0
                        self.get_logger().info(f"Aligning Position (Vision): ErrX={err_x:.1f}", throttle_duration_sec=1.0)
                # 情况B：当前帧达标 -> 进入观察期
                else:
                    self.aligned_counter += 1
                    self.get_logger().info(f"\033[1;33m 预备对准... 稳定性计数: {self.aligned_counter}/10 (ErrX: {err_x:.1f}) \033[0m")
                    
                    # 停车观察
                    self.twist.linear.x = 0.0
                    self.twist.linear.y = 0.0
                    self.twist.angular.z = 0.0
                    
                    # 只有连续 10 帧（约0.5-1秒）都对准了，才切换状态
                    if self.aligned_counter > 10:
                        self.get_logger().info('\033[1;32m >>> 稳定对准！开始穿越 <<< \033[0m')
                        self.cmd_vel_pub.publish(self.twist)
                        time.sleep(1.0)
                        set_servo_position(self.joints_pub, 1, ((19, 500), (20, 680), (21, 100), (22, 140), (23, 500), (24, 700)))
                        time.sleep(1)
                        self.state = State.CROSSING

                        self.min_depth = 350  # 最近距离
                        self.max_depth = 400

        # 2. 状态：穿越阶段 (CROSSING)
        elif self.state == State.CROSSING:
            desired_pose = 'NARROW_POSE'
            desired_height = 5
            if not found:
                # 丢失目标的处理：稍微增加容错，不要立即判定完成
                self.lost_target_count += 1 
                if self.lost_target_count > 20: 
                    self.state = State.FINISHED
                self.twist.linear.x = 0.0
            else:
                self.lost_target_count = 0 # 重置丢失计数
                
                # 纠偏计算
                err_x = self.image_center_x - center_x
                
                self.twist.linear.x = 0.015 # 前进速度
                
                # 同时修正位移(视觉)和角度(视觉)
                rot_speed = err_x * self.yaw_p * 0.5  # 使用深度视觉角度进行修正
                
                self.twist.angular.z = float(np.clip(rot_speed, -0.03, 0.03))
        
                self.cross_count += 1

                if self.cross_count > 400:  #结束桥梁识别
                    self.twist.angular.z = 0.0

                if self.cross_count > 600: 
                    self.get_logger().warn('结束穿越')
                    self.state = State.LIDAR_ALIGNING

        elif self.state == State.LIDAR_ALIGNING:
            desired_pose = 'SLAM_POSE'
            desired_height = 10
            period = 1.0

            diff = self.avg_dist_left - self.avg_dist_right
            diff = np.clip(diff, -0.3, 0.3) # 限制最大偏差感应      

            if  abs(diff) > 0.02:
                # 核心控制逻辑
                # P 控制：
                # 如果左边距离大 (diff > 0)，说明靠右了，需要向左平移 (+y) 且向左转头 (+z)
                self.twist.linear.y = float(np.clip(diff * self.kp_strafe, -0.04, 0.04))
                self.twist.angular.z = float(np.clip(diff * self.kp_rotate, -0.2, 0.2))
                
                self.get_logger().info(f"Lidar Centering: L_avg={self.avg_dist_left:.2f}, R_avg={self.avg_dist_right:.2f}, Vy={self.twist.linear.y:.3f}")
            else:
                self.state = State.FINISHED

        elif self.state == State.FINISHED:
            self.get_logger().info('\033[1;32m 准备下台阶... \033[0m')
            self.state = State.DONE
            self.timestamp = time.time() + self.descend_duration

        elif self.state == State.DONE:
            desired_pose = 'SLAM_POSE'
            desired_height = 60
            period = 2.0
            self.get_logger().info('\033[1;32m 开始下台阶... \033[0m')
            if time.time() < self.timestamp :
                self.twist.linear.x = 0.04
            else:
                desired_height = 20
                period = 1.0
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
                self.running = False
                msg = Bool()
                msg.data = True
                self.finish_pub.publish(msg)
                self.get_logger().info('\033[1;32m 结束.. \033[0m')

        # --- 执行姿态切换 ---
        if desired_pose != self.current_pose or desired_height != self.desired_height:
            self.get_logger().info(f"Changing Pose to {desired_pose}")
            self.get_logger().info(f"Changing Height to {desired_height}")

            cmd_param = CmdParam()
            cmd_param.gait = 2
            cmd_param.period = period
            cmd_param.pose = desired_pose
            cmd_param.height = desired_height
            self.cmd_param_pub.publish(cmd_param)
            self.current_pose = desired_pose
            self.desired_height = desired_height
            time.sleep(1.5)

        self.cmd_vel_pub.publish(self.twist)

    def main(self):
        while rclpy.ok():
            if not self.running:
                time.sleep(0.1)
                continue

            try:
                depth_image = self.image_queue.get(block=True, timeout=1)
            except queue.Empty:
                continue
            
            # --- 调用视觉处理函数 ---
            center_x, angle, width, found, debug_img = self.process_bridge_vision(depth_image)
            
            # --- 调用控制策略 ---
            self.new_move_policy(center_x, angle, width, found)

            # 显示调试图像
            if self.debug or True: # 强制显示方便调试
                cv2.putText(debug_img, f"State: {self.state.name}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
                cv2.putText(debug_img, f"Ang: {angle:.1f} X: {center_x}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                cv2.imshow('Bridge Vision', debug_img)
                key = cv2.waitKey(1)
                if key == ord('q'):
                    self.running = False

def main():
    rclpy.init()
    node = CrossBridgeNode('cross_bridge')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
