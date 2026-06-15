#!/usr/bin/env python3
# encoding: utf-8
# 雷达避障，跟随功能(lidar obstacle avoidance and lidar following function)
import os
import math
import time
import rclpy
import threading
import numpy as np
import sdk.pid as pid
import sdk.common as common
from rclpy.node import Node
from app.common import Heart
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from controller import controller_client 
from interfaces.srv import SetInt64, SetFloat64List
from servo_controller_msgs.msg import ServosPosition
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from servo_controller.bus_servo_control import set_servo_position

CAR_WIDTH = 0.4  # meter
MAX_SCAN_ANGLE = 90  # 激光的扫描角度,去掉总是被遮挡的部分degree(the scanning angle of Lidar. The covered part is always eliminated)

class LidarController(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)

        self.name = name
        self.running_mode = 0
        self.threshold = 0.3  # meters
        self.scan_angle = math.radians(90)  # radians
        self.speed = 0.05
        self.last_act = 0
        self.timestamp = 0
        self.angle_data = []
        # pid参数       
        self.pid_yaw = pid.PID(0.35, 0, 0.00001)
        self.pid_dist = pid.PID(0.35, 0, 0.000001)
        self.lock = threading.RLock()
        self.controller = controller_client.ControllerClient()
        self.lidar_sub = None

        self.cmd_vel_pub = self.create_publisher(Twist, '/controller/cmd_vel', 1)  # 底盘控制(chassis control)
        self.create_service(Trigger, '~/enter', self.enter_srv_callback)  # 进入玩法(enter the game)
        self.create_service(Trigger, '~/exit', self.exit_srv_callback)  # 退出玩法(exit the game)
        self.create_service(SetInt64, '~/set_running', self.set_running_srv_callback)  # 开启玩法(start the game)
        Heart(self, self.name + '/heartbeat', 5, lambda _: self.exit_srv_callback(request=Trigger.Request(), response=Trigger.Response()))  # 心跳包(heartbeat package)
        self.create_service(SetFloat64List, '~/set_param', self.set_parameters_srv_callback)  # 参数设置(set parameter)
        self.joints_pub = self.create_publisher(ServosPosition, 'servo_controller', 1)
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

    def get_node_state(self, request, response):
        response.success = True
        return response

    def reset_value(self):
        '''
        重置参数(reset parameter)
        :return:
        '''
        self.running_mode = 0
        self.threshold = 0.6
        self.scan_angle = math.radians(90)
        self.last_act = 0
        self.speed = 0.05
        self.timestamp = 0
        self.pid_yaw.clear()
        self.pid_dist.clear()

    def enter_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % 'lidar enter')
        self.reset_value()
        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, qos)  # 订阅雷达数据(subscribe to Lidar data)
        set_servo_position(self.joints_pub, 1, ((24, 500), (23, 500), (22, 150), (21, 130), (20, 720), (19, 500)))
        self.controller.traveling(gait=-2, time=1, steps=0)
        response.success = True
        response.message = "enter"
        return response

    def exit_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % 'lidar exit')
        self.reset_value()
        self.controller.traveling(gait=-2, time=1, steps=0)
        
        response.success = True
        response.message = "exit"
        return response

    def set_running_srv_callback(self, request, response):
        '''
        开启不同功能(enable different functions)
        :param req: 0关闭，1避障，2跟随，3警卫(0 close, 1 obstacle avoidance, 2 Lidar following, 3 Lidar guarding)
        :return:
        '''
        self.get_logger().info(str(request))
        new_running_mode = request.data
        self.get_logger().info('\033[1;32m%s\033[0m' % ("set_running " + str(new_running_mode)))
        if not 0 <= new_running_mode <= 3:
            response.success = False
            response.message = "Invalid running mode {}".format(new_running_mode)
        else:
            response.success = True
            response.message = "set_running"
            self.timestamp = 0
            with self.lock:
                self.running_mode = new_running_mode
                if self.running_mode == 0 :
                    self.controller.traveling(gait=-2, time=1, steps=0)

        return response

    def set_parameters_srv_callback(self, request, response):
        '''
        设置避障阈值，速度参数(set the threshold of obstacle avoidance and speed)
        :param req:
        :return:
        '''
        new_parameters = request.data
        new_threshold, new_scan_angle, new_speed = new_parameters
        self.get_logger().info("\033[1;32mn_t:{:2f}, n_a:{:2f}, n_s:{:2f}\033[0m".format(new_threshold, new_scan_angle, new_speed))
        if not 0.3 <= new_threshold <= 1.5:
            response.success = False
            response.message = "New threshold ({:.2f}) is out of range (0.3 ~ 1.5)".format(new_threshold)
            return response
        if not 0 <= new_scan_angle <= 90:
            response.success = False
            response.message = "New scan angle ({:.2f}) is out of range (0 ~ 90)"
            return response
        if not new_speed > 0:
            response.success = False
            response.message = "Invalid speed"
            return response

        with self.lock:
            self.threshold = new_threshold
            self.scan_angle = math.radians(new_scan_angle)
            self.speed = new_speed
        return response

    def lidar_callback(self, lidar_data):
        # 雷达订阅回调(Lidar subscription callback)
        twist = Twist()
        # 数据大小 = 扫描角度/每扫描一次增加的角度(data size= scanning angle/ the increased angle per scan)
        max_index = int(math.radians(MAX_SCAN_ANGLE / 2.0) / lidar_data.angle_increment)
        left_ranges = lidar_data.ranges[:max_index]  # 左半边数据 (the left data)
        right_ranges = lidar_data.ranges[::-1][:max_index]  # 右半边数据 (the right data)
        
        with self.lock:
            # 根据设定取数据 (get the data according to the settings)
            angle = self.scan_angle / 2
            angle_index = int(angle / lidar_data.angle_increment + 0.50)
            left_range, right_range = np.array(left_ranges[:angle_index]), np.array(right_ranges[:angle_index])
            if self.running_mode == 1 and self.timestamp <= time.time():
                left_nonzero = left_range.nonzero()
                right_nonzero = right_range.nonzero()
                left_nonan = np.isfinite(left_range[left_nonzero])
                right_nonan = np.isfinite(right_range[right_nonzero])
                min_dist_left_ = left_range[left_nonzero][left_nonan]
                min_dist_right_ = right_range[right_nonzero][right_nonan]
                # 取左右最近的距离(Take the nearest distance left and right)
                if len(min_dist_left_) > 1 and len(min_dist_right_) > 1:
                    min_dist_left = min_dist_left_.min()
                    min_dist_right = min_dist_right_.min()
                    # self.get_logger().info(f"self.threshold,{self.threshold}")

                    if min_dist_left <= self.threshold and min_dist_right > self.threshold:  # 左侧有障碍(there is obstacle at left)
                        # self.get_logger().info('\033[1;32m%s\033[0m' % '左侧有障碍')
                        twist.linear.x = 0.0  # 转弯时向前速度尽量小(keep the forward velocity as low as possible when turning)
                        max_angle = math.radians(90)  # 转90度(turn 90 degree)
                        w = self.speed * 4.0  # 转弯速度尽量大(keep the turning speed as high as possible)
                        twist.angular.z = -w
                        if self.last_act != 0 and self.last_act != 1:
                            twist.angular.z = w
                        self.last_act = 1
                        self.cmd_vel_pub.publish(twist)
                        self.timestamp = time.time() + (max_angle / w / 4)
                    elif min_dist_left <= self.threshold and min_dist_right <= self.threshold:  # 两侧都有障碍
                        # self.get_logger().info('\033[1;32m%s\033[0m' % '两侧都有障碍')
                        twist.linear.x = 0.0   # 转弯时向前速度尽量小(keep the forward velocity as low as possible when turning)
                        w = self.speed * 4.0  # 转弯速度尽量大(keep the turning speed as high as possible)
                        twist.angular.z = w
                        self.last_act = 3
                        self.cmd_vel_pub.publish(twist)
                        # 转180度
                        self.timestamp = time.time() + (math.radians(180) / w / 4)
                    elif min_dist_left > self.threshold and min_dist_right <= self.threshold:  # 右侧有障碍(there is obstacles at right)
                        # self.get_logger().info('\033[1;32m%s\033[0m' % '右侧有障碍')
                        twist.linear.x = 0.0   # 转弯时向前速度尽量小(keep the forward velocity as low as possible when turning)
                        max_angle = math.radians(90)  # 转90度 (turn 90 degrees)
                        w = self.speed * 4.0  # 转弯速度尽量大(keep the turning speed as high as possible)
                        twist.angular.z = w
                        if self.last_act != 0 and self.last_act != 2:
                            twist.angular.z = -w
                        self.last_act = 2
                        self.cmd_vel_pub.publish(twist)
                        self.timestamp = time.time() + (max_angle / w / 4)
                    else:  # 没有障碍(none obstacle)
                        self.last_act = 0
                        twist.linear.x = self.speed  # 直行
                        self.cmd_vel_pub.publish(twist)
                    


            elif self.running_mode == 2:
                # 拼合距离数据, 从右半侧逆时针到左半侧(the merged distance data from right half counterclockwise to the left half)
                ranges = np.append(right_range[::-1], left_range)
                nonzero = ranges.nonzero()
                nonan = np.isfinite(ranges[nonzero])
                dist_ = ranges[nonzero][nonan]
                if len(dist_) > 0:
                    dist = dist_.min()
                    min_index = list(ranges).index(dist)
                    angle = -angle + lidar_data.angle_increment * min_index  # 计算最小值对应的角度(calculate the angle corresponding to the minimum value)

                    if dist < self.threshold and abs(math.degrees(angle)) > 15:  # 控制左右(control the left and right)
                        self.pid_yaw.update(-angle)
                        twist.angular.z = common.set_range(self.pid_yaw.output, -self.speed * 10.0, self.speed * 10.0)
                    else:
                        self.pid_yaw.clear()

                    if dist < self.threshold and abs(0.2 - dist) > 0.02:  # 控制前后(control the front and back)
                        self.pid_dist.update(self.threshold / 2 - dist)
                        twist.linear.x = common.set_range(self.pid_dist.output, -self.speed, self.speed)
                    else:
                        self.pid_dist.clear()
                    if abs(twist.angular.z) < 0.05:
                        twist.angular.z = 0.0
                    if abs(twist.linear.x) < 0.012:
                        twist.linear.x = 0.0
                    self.cmd_vel_pub.publish(twist)

            elif self.running_mode == 3:
                # 拼合距离数据, 从右半侧逆时针到左半侧(the merged distance data from right half counterclockwise to the left half)
                ranges = np.append(right_range[::-1], left_range)
                nonzero = ranges.nonzero()
                nonan = np.isfinite(ranges[nonzero])
                dist_ = ranges[nonzero][nonan]
                if len(dist_) > 1:
                    dist = dist_.min()
                    min_index = list(ranges).index(dist)
                    
                    angle = -angle + lidar_data.angle_increment * min_index  # 计算最小值对应的角度(calculate the angle corresponding to the minimum value)
                    if dist < self.threshold and abs(math.degrees(angle)) > 15:  # 控制左右(control the left and right)
                        self.pid_yaw.update(-angle)
                        twist.angular.z = common.set_range(self.pid_yaw.output, -self.speed * 10.0, self.speed * 10.0)
                    else:
                        self.pid_yaw.clear()
                    if abs(twist.angular.z) < 0.05:
                        twist.angular.z = 0.0
                    self.cmd_vel_pub.publish(twist)

            else:
                time.sleep(0.01)


def main():
    node = LidarController('lidar_app')
    rclpy.spin(node)  # 循环等待ROS2退出(Loop waiting for ROS2 to Exit)

if __name__ == "__main__":
    main()


