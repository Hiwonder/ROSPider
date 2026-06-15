#!/usr.bin/env python3
# encoding: utf-8
# 窄缝穿越专用程序 - 带雷达侧向检测停止功能
import math
import time
import rclpy
import threading
import numpy as np
from rclpy.node import Node
from app.common import Heart
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from interfaces.msg import CmdParam
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from controller import controller_client 
from servo_controller_msgs.msg import ServosPosition
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from servo_controller.bus_servo_control import set_servo_position

MAX_SCAN_ANGLE = 90  # 激光的扫描角度(用于前方检测)

class GapTraverseNode(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)

        self.name = name
        
        # --- 参数设置 ---
        self.speed = 0.04         # 前进/平移速度 (米/秒)
        self.turn_speed = 0.3     # 旋转速度 (弧度/秒)
        self.detect_dist = 0.33    # 前方触发距离 (米)
        self.side_stop_dist = 0.315 # 侧向停止距离 (米) - 小于此距离停止平移
        self.down_steps_time = 8.0 # 下台阶直行时间 (秒)
        self.max_shift_time = 8.0  # 最大平移超时时间(防止雷达失效)
        self.traverse_time = 13.0  # 穿越危墙时间 (秒)
        self.align_timeout = 10.0  # 对齐最大超时时间
        self.step = -1   #机器人状态， -1:下台阶, 0:巡航检测, 1:旋转对齐, 2:平移对齐, 3:旋转, 4:变形, 5:穿越, 6:恢复    
        self.timestamp = 0    # 记录当前时间戳，用于状态转换
        self.direction = 1   #平移时检测的方向，1左, -1右
        self.align_threshold = 0.02 # 旋转对齐允许的角度误差(弧度)

        # --- 内部变量 ---
        self.running = False      
        self.latest_scan = None   
        self.lidar_sub = None
        self.lock = threading.RLock()
        self.controller = controller_client.ControllerClient()
        
        # --- ROS2 通信 ---
        self.cmd_vel_pub = self.create_publisher(Twist, '/controller/cmd_vel', 1)
        self.joints_pub = self.create_publisher(ServosPosition, 'servo_controller', 1)
        self.cmd_param_pub = self.create_publisher(CmdParam, '/step_controller/cmd_param', 1) 
        self.finish_pub = self.create_publisher(Bool, '~/finish', 1) 

        # 服务
        self.create_service(Trigger, '~/enter', self.enter_srv_callback) 
        self.create_service(Trigger, '~/exit', self.exit_srv_callback)   
        self.debug = self.get_parameter('debug').value
        
        self.client = self.create_client(Trigger, '/controller_manager/init_finish')
        self.client.wait_for_service()

        self.get_logger().info('\033[1;32m%s\033[0m' % 'Narrow Gap Traversal Node Started')
        
        if self.debug:
            self.set_narrow_posture()
            time.sleep(1.0)  # 确保姿态变形完成
            self.enter_srv_callback(None, None)
            self.get_logger().info('\033[1;32m%s\033[0m' % 'DEBUG MODE STARTED')

    def reset_state(self):
        self.step = -1
        self.direction = 1
        self.latest_scan = None

    def set_normal_posture(self):
        """恢复正常站立姿态"""
        set_servo_position(self.joints_pub, 1, ((19, 500), (20, 720), (21, 120), (22, 140), (23, 500), (24, 700)))
        time.sleep(1.5)  # 确保姿态变形完成
        cmd_param = CmdParam()
        cmd_param.gait = 2
        cmd_param.period = 1.0
        cmd_param.pose = 'SLAM_POSE'
        cmd_param.height = 20
        self.cmd_param_pub.publish(cmd_param)
        time.sleep(2.0)  # 确保姿态变形完成


    def set_narrow_posture(self):
        """设置窄缝穿越姿态 """
        set_servo_position(self.joints_pub, 1, ((19, 500), (20, 835), (21, 215), (22, 60), (23, 500), (24, 700)))
        time.sleep(1.5)  # 确保姿态变形完成
        cmd_param = CmdParam()
        cmd_param.gait = 2
        cmd_param.period = 1.0
        cmd_param.pose = 'SIDE_SHIFT_POSE'
        cmd_param.height = 10
        self.cmd_param_pub.publish(cmd_param)
        time.sleep(1.0)  # 确保姿态变形完成
        
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
        self.get_logger().info('Start Running')
        self.reset_state()
        self.running = True

        if self.lidar_sub is None:
            self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 1)

        self.set_down_steps_posture()
        self.timestamp = time.time() + self.down_steps_time
        self.get_logger().info(f"Step -1: Start Down Steps (Forward {self.down_steps_time}s)")

        self.processing_thread = threading.Thread(target=self.processing_loop, daemon=True)
        self.processing_thread.start()

        if response:
            response.success = True
            response.message = "Started"
        return response

    def exit_srv_callback(self, request, response):
        self.get_logger().info('Stop Running')
        self.running = False
        self.controller.traveling(gait=-1, time=1, steps=0)
        
        if self.lidar_sub is not None:
            self.destroy_subscription(self.lidar_sub)
            self.lidar_sub = None
        
        if response:
            response.success = True
            response.message = "Stopped"
        return response

    def lidar_callback(self, msg):
        if self.running:
            with self.lock:
                self.latest_scan = msg


    def calculate_align_error(self, scan_data):
        try:
            # 1. 生成所有点的角度数组
            # angles 的长度等于 ranges 的长度，对应每个点的真实角度
            num_points = len(scan_data.ranges)
            angles = scan_data.angle_min + np.arange(num_points) * scan_data.angle_increment
            ranges = np.array(scan_data.ranges)

            # 2. 定义右侧的角度范围 (弧度)
            # 右侧: 240度 ~ 3155度 (-2.09 ~ -0.78 rad)
            min_side_rad = math.radians(240)
            max_side_rad = math.radians(315)

            # 3. 提取左侧和右侧的有效点
            # 逻辑：角度在范围内 AND 距离在有效范围内(0.1m - 3.0m) AND 数据有效(非inf/nan)
            valid_dist = (np.isfinite(ranges)) & (ranges > 0.05) & (ranges < 3.0)
            
            # 右侧掩码
            right_mask = (angles > math.radians(240)) & (angles < math.radians(315)) & valid_dist

            # 提取数据
            r_ranges = ranges[right_mask]
            r_angles = angles[right_mask]

            # 5. 决策：选哪边？
            chosen_ranges = None
            chosen_angles = None
            points_threshold = 3 

            if len(r_ranges) > points_threshold:
                chosen_ranges = r_ranges
                chosen_angles = r_angles
            else:
                self.get_logger().warn(f"Align Fail: Not enough points!  R={len(r_ranges)}")
                return None

            # 6. 拟合直线计算误差
            # 转换到笛卡尔坐标系
            x_vals = chosen_ranges * np.cos(chosen_angles)
            y_vals = chosen_ranges * np.sin(chosen_angles)
            
            # 拟合 y = mx + c (拟合侧面墙壁直线)
            # 理想情况下侧面墙壁平行于x轴，m (斜率) 应该接近 0
            A = np.vstack([x_vals, np.ones(len(x_vals))]).T
            m, c = np.linalg.lstsq(A, y_vals, rcond=None)[0]
            
            # 误差角度
            error = math.atan(m)
            return error

        except Exception as e:
            self.get_logger().error(f"Align Algorithm Error: {e}")
            return None


    def get_side_distance(self, scan_data, direction):
        """
        根据移动方向(1左, -1右)获取侧面的最小距离
        检测范围: 侧向30度到100度
        """
        try:
            angle_inc = scan_data.angle_increment
            # 计算索引范围
            idx_30 = int(math.radians(30) / angle_inc)
            idx_100 = int(math.radians(100) / angle_inc)
            
            # 限制索引不超过数组长度
            max_len = len(scan_data.ranges)
            idx_100 = min(idx_100, max_len - 1)
            
            if direction == 1: # 向左移动，检测左侧
                # 假设 ranges[0] 是正前方，正索引向左增加
                side_ranges = np.array(scan_data.ranges[idx_30 : idx_100])
            else: # 向右移动，检测右侧
                # 假设 ranges[-1] 是右侧开始
                side_ranges = np.array(scan_data.ranges[-idx_100 : -idx_30])
                
            # 过滤无效数据 (inf, nan, 0)
            valid_ranges = side_ranges[np.isfinite(side_ranges)]
            valid_ranges = valid_ranges[valid_ranges > 0.05] # 过滤掉极小值噪点
            
            if len(valid_ranges) > 0:
                return valid_ranges.min()
            else:
                return 10.0 # 无有效数据，返回较大值
                
        except Exception as e:
            self.get_logger().warn(f"Lidar processing error: {e}")
            return 10.0

    def processing_loop(self):
        while rclpy.ok():
            if not self.running:
                time.sleep(0.1)
                continue

            current_time = time.time()

            # =========================================
            # 1. 执行阶段 (Execution Phase)
            # =========================================
            if current_time < self.timestamp:
                twist = Twist()
                
                # Step -1: 下台阶直行
                if self.step == -1:
                    twist.linear.x = self.speed 
                    self.cmd_vel_pub.publish(twist)

                # Step 2: 预平移 (带雷达检测的移动)
                elif self.step == 2:
                    twist.linear.y = self.speed * -self.direction
                    self.cmd_vel_pub.publish(twist)

                # Step 3: 穿越
                elif self.step == 5: 
                    twist.angular.z = 0.0
                    twist.linear.y = self.speed * -self.direction
                    self.cmd_vel_pub.publish(twist)
                             
                # 对于Step 1，我们需要在移动过程中实时检测，如果不满足条件要提前break出执行阶段
                if self.step != 1  and self.step != 2: 
                    time.sleep(0.05)
                    continue 
                # Step 1 将继续向下执行进入Transition Phase进行检测

            # =========================================
            # 2. 状态切换阶段 (Transition Phase)
            # =========================================

            # --- Step -1 -> 0: 下台阶结束 -> 开始巡航 ---
            if self.step == -1:
                if current_time >= self.timestamp:
                    self.step = 0
                    self.timestamp = time.time() + 1
                    self.set_normal_posture()
                    self.get_logger().info("Step -1->0: Down Steps Done. Start Cruising...")

            # --- Step 0: 巡航与检测前方障碍 ---
            elif self.step == 0:
                scan_data = None
                with self.lock:
                    if self.latest_scan is not None:
                        scan_data = self.latest_scan
                
                if scan_data is None:
                    time.sleep(0.05)
                    continue

                try:
                    max_index = int(math.radians(MAX_SCAN_ANGLE / 2.0) / scan_data.angle_increment)
                    left = np.array(scan_data.ranges[:max_index])
                    right = np.array(scan_data.ranges[::-1][:max_index])
                    left = left[np.isfinite(left)]
                    right = right[np.isfinite(right)]
                    
                    min_l = left.min() if len(left) > 0 else 10.0
                    min_r = right.min() if len(right) > 0 else 10.0
                    min_dist = min(min_l, min_r)

                    twist = Twist()
                    if min_dist < self.detect_dist:
                        self.cmd_vel_pub.publish(Twist()) # 停车
                        self.step = 1
                        self.get_logger().info("开始旋转对齐.")

                        # 设置一个较长的超时时间作为安全机制
                        self.timestamp = time.time() + self.align_timeout

                    else:
                        twist.linear.x = self.speed
                        self.cmd_vel_pub.publish(twist)
                except Exception as e:
                    self.get_logger().error(f"Error in Step 0: {e}")

        # --- Step 1 -> 2: 旋转对齐 -> 平移对齐 ---
            elif self.step == 1:
                # 1. 确保获取数据 !!!
                scan_data = None
                with self.lock:
                    if self.latest_scan is not None:
                        scan_data = self.latest_scan
                
                aligned = False
                twist = Twist()

                # 2. 如果有数据，尝试计算
                if scan_data is not None:
                    error = self.calculate_align_error(scan_data)
                    
                    if error is not None and error != 0.0:

                        if abs(error) < self.align_threshold:
                            aligned = True
                            twist.angular.z = 0.0
                        else:
                            # P控制旋转
                            # 侧面对齐拟合的是 y=mx+c。
                            # 如果 m>0 (error>0)，说明车头偏向墙壁（假设左墙），需要顺时针转？或者逆时针？
                            # 通常建议系数先设为 1.0，如果越转越歪，改成 -1.0
                            kp = 0.2
                            twist.angular.z = float(kp * error)

                            # 速度限制
                            max_rot = 0.3
                            twist.angular.z = max(min(twist.angular.z, max_rot), -max_rot)
                            
                            # 最小速度补偿 (防止低速电机不转)
                            if 0 < abs(twist.angular.z) < 0.03:
                                twist.angular.z = 0.03 * (1 if twist.angular.z > 0 else -1)
                    else:
                        self.get_logger().warn("Align Fail: Not enough wall points detected!", throttle_duration_sec=1.0)
                        twist.angular.z = 0.0
                    self.get_logger().info(f"Alignin: {twist.angular.z} rad")

                # 3. 发布指令
                self.cmd_vel_pub.publish(twist)

                # 4. 退出条件
                if aligned or current_time >= self.timestamp:
                    self.cmd_vel_pub.publish(Twist()) # 停车
                    time.sleep(0.5)
                    
                    if aligned:
                        self.get_logger().info(f"Step 11->1: Alignment SUCCESS.")
                    else:
                        self.get_logger().warn(f"Step 11->1: Alignment TIMEOUT. (Did not reach threshold)")

                    self.step = 2
                    self.timestamp = time.time() + self.max_shift_time
                    
            # ---  Step 2 -> 3: 平移中检测雷达距离 -> 旋转 ---
            elif self.step == 2:
                # 获取雷达数据
                scan_data = None
                with self.lock:
                    if self.latest_scan is not None:
                        scan_data = self.latest_scan
                
                stop_shift = False
                min_side = 10.0

                if scan_data:
                    check_dir = -self.direction # 1为左, -1为右
                    min_side = self.get_side_distance(scan_data, check_dir)
                    
                    if min_side < self.side_stop_dist:
                        self.get_logger().info(f"Side Obstacle Detected ({min_side:.3f}m < {self.side_stop_dist}m). Stopping Shift.")
                        self.controller.traveling(gait=-2, time=1, steps=0)
                        time.sleep(1.0) # 确保停止 
                        stop_shift = True
                
                # 超时或者检测到侧面距离小于阈值
                if stop_shift or current_time >= self.timestamp:
                    self.step = 3
                    self.controller.traveling(gait=2, time=1, rotation=0.9,  steps=8)
                    self.get_logger().info(f"Step 1->2: 旋轉8s)")
                    time.sleep(9.0) # 确保停止

                    self.get_logger().info(f"Step 1->2: Shift Done. Start Rotating ({self.timestamp:.2f}s)")

            # --- Step 3 -> 4: 旋转结束 -> 变形并后退 ---
            elif self.step == 3:
                self.step = 4
                self.set_narrow_posture()
                self.controller.traveling(gait=2, time=1, direction=math.radians(180), steps=2)
                time.sleep(3.0) # 确保停止
                self.get_logger().info(f"Step 2->3: Rotate Done. Start Morphing )")

            # --- Step 4 -> 5: 变形结束 -> 穿越 ---
            elif self.step == 4:
                self.step = 5
                self.timestamp = time.time() + self.traverse_time
                self.get_logger().info(f"Step 3->4: Morph Done. Start Traversing ({self.traverse_time}s)")

            # --- Step 5 -> 6: 穿越结束 -> 前进并回复姿态 ---
            elif self.step == 5:
                if current_time >= self.timestamp:
                    self.step = 6
                    self.controller.traveling(gait=2, time=1, direction=math.radians(0), steps=2)
                    time.sleep(3.0) # 确保停止
                    self.set_normal_posture()
                    self.get_logger().info(f"Step 4->5: Traverse Done. Start Restoring)")

            # --- Step 6 -> 结束: 恢复结束 -> 旋转回正 ---
            elif self.step == 6:
                if current_time >= self.timestamp:
                    self.step = 7
                    self.controller.traveling(gait=2, time=1, rotation=-0.9,  steps=9)
                    time.sleep(10.0) # 确保停止
                    
                    self.exit_srv_callback(None, None) 
                    self.get_logger().info("Step 6->End: Task Completed. Rotating back done.")
                    msg = Bool()
                    msg.data = True
                    self.finish_pub.publish(msg)
                    self.get_logger().info("self.finish_pub.publish(msg)")

            time.sleep(0.05)

def main():
    node = GapTraverseNode('narrow_slit_traversal')
    rclpy.spin(node)

if __name__ == "__main__":
    main()