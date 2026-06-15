#!/usr/bin/env python3
#!/usr/bin/env python3
# encoding: utf-8
# 过独木桥(cross bridge)

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
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist
from interfaces.msg import CmdParam
from controller import step_controller
from sensor_msgs.msg import Image, CameraInfo
from servo_controller_msgs.msg import ServosPosition
from servo_controller.bus_servo_control import set_servo_position

import matplotlib.pyplot as plt
class State(enum.Enum):
    NORMAL = 0
    CROOSS_BRIDGE = 1

class CrossBridgeNode(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.name = name
        signal.signal(signal.SIGINT, self.shutdown)
        self.running = True
        self.turn = False
        self.K = None
        self.D = None
        self.current_pose = 'DEFAULT_POSE'
        self.count_status = False
        self.plane_high = self.get_parameter('plane_distance').value
        self.plane_high_1 = self.get_parameter('plane_distance_1').value
        self.debug = self.get_parameter('debug').value
        self.twist = Twist()
        self.image_queue = queue.Queue(maxsize=2)
        self.left_roi_1 = [245, 255, 115, 125]
        self.left_roi = [245, 255, 185, 195]
        self.center_roi = [245, 255, 315, 325]
        self.right_roi = [245, 255, 445, 455]
        self.right_roi_1 = [245, 255, 515, 525]
        self.controller = step_controller.StepController()
        self.debug = self.get_parameter('debug').value
        self.joints_pub = self.create_publisher(ServosPosition, '/servo_controller', 1) # 舵机控制(servo control)
        self.cmd_vel_pub = self.create_publisher(Twist, '/controller/cmd_vel', 1)  # 底盘控制(chassis control)
        self.cmd_param_pub = self.create_publisher(CmdParam, '/step_controller/cmd_param', 1) # 行走姿态控制  
        self.image_sub = self.create_subscription(Image, '/depth_cam/depth/image_raw', self.depth_callback, 1)
        self.camera_info_sub = self.create_subscription(CameraInfo, '/depth_cam/rgb/camera_info', self.camera_info_callback, 1)


        self.client = self.create_client(Trigger, '/controller_manager/init_finish')
        self.client.wait_for_service()

        self.controller.set_build_in_pose('DEFAULT_POSE', 1)
        set_servo_position(self.joints_pub, 1, ((19, 500), (20, 727), (21, 80), (22, 160), (23, 500), (24, 700)))
        time.sleep(1)

        threading.Thread(target=self.main, daemon=True).start()
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

    def get_node_state(self, request, response):
        response.success = True
        return response

    def depth_callback(self, ros_depth_image):
        depth_image = np.ndarray(shape=(ros_depth_image.height, ros_depth_image.width), dtype=np.uint16,
                                 buffer=ros_depth_image.data)
        if self.image_queue.full():
            # 如果队列已满，丢弃最旧的图像(if the queue is full, discard the oldest image)
            self.image_queue.get()
        # 将图像放入队列(put the image into the queue)
        self.image_queue.put(depth_image)

    def camera_info_callback(self, msg):
        self.K = msg.k
        self.D = msg.d

    def shutdown(self, signum, frame):
        self.running = False
        self.get_logger().info('\033[1;32m%s\033[0m' % "shutdown")

    def get_roi_distance(self, depth_image, roi):
        roi_image = depth_image[roi[0]:roi[1], roi[2]:roi[3]]
        try:
            distance = round(float(np.mean(roi_image[np.logical_and(roi_image > 0, roi_image < 30000)]) / 1000), 3)
        except:
            distance = 0
        return distance

    def move_policy(self, left_distance, left_distance_1, center_distance, right_distance, right_distance_1):
        self.get_logger().info('\033[1;31m%s\033[0m' % str(abs(left_distance_1 - self.plane_high))+str(abs(right_distance_1 - self.plane_high)))

        if abs(left_distance_1 - self.plane_high) > 0.04 and abs(right_distance_1 - self.plane_high) > 0.0 :
            self.twist.linear.x = 0.02
            desired_pose = 'NARROW_POSE'
            desired_height = 5
            time.sleep(1.5)
            self.state = State.CROOSS_BRIDGE
        else:
            self.twist.linear.x = 0.0
            desired_pose  = 'DEFAULT_POSE'
            desired_height = 20        
            time.sleep(1.5)
            self.state = State.NORMAL
        
        if desired_pose != self.current_pose:
            # 只有在需要改变姿态时才发送指令 (Only send a command when a pose change is needed)            
            cmd_param = CmdParam()
            cmd_param.gait = 2
            cmd_param.period = 1.0
            cmd_param.pose = desired_pose
            cmd_param.height = desired_height
            self.cmd_param_pub.publish(cmd_param)
            
            # 更新当前姿态状态 (Update the current pose state)
            self.current_pose = desired_pose

        if self.state == State.NORMAL:
            if abs(left_distance - self.plane_high) > 0.06:
                if abs(center_distance - self.plane_high) > 0.06:
                    self.twist.angular.z = -0.2
                else:
                    self.twist.angular.z = -0.1
            elif abs(right_distance - self.plane_high) > 0.06:
                if abs(center_distance - self.plane_high) > 0.06:
                    self.twist.angular.z = 0.2
                else:
                    self.twist.angular.z = 0.1
            else:
                self.twist.angular.z = 0.0
        else :
            self.twist.angular.z = 0.0

        if abs(left_distance - self.plane_high) > 0.06 and abs(right_distance - self.plane_high) > 0.06 and abs(center_distance - self.plane_high) > 0.06:
            self.twist = Twist()
        self.get_logger().info('\033[1;32m%s\033[0m' % str(self.twist))

        self.cmd_vel_pub.publish(self.twist)

    def main(self):
        count = 0
        count_1 = 0
        while self.running:
            try:
                depth_image = self.image_queue.get(block=True, timeout=1)
            except queue.Empty:
                if not self.running:
                    break
                else:
                    continue
            depth_color_map = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.45), cv2.COLORMAP_JET)
            if self.debug:
                cv2.circle(depth_color_map, (int((self.left_roi[2] + self.left_roi[3]) / 2), int((self.left_roi[0] + self.left_roi[1]) / 2)), 10, (0, 0, 0), -1)
                cv2.circle(depth_color_map, (int((self.center_roi[2] + self.center_roi[3]) / 2), int((self.center_roi[0] + self.center_roi[1]) / 2)), 10, (0, 0, 0), -1)
                cv2.circle(depth_color_map, (int((self.right_roi[2] + self.right_roi[3]) / 2), int((self.right_roi[0] + self.right_roi[1]) / 2)), 10, (0, 0, 0), -1)
                cv2.circle(depth_color_map, (int((self.left_roi_1[2] + self.left_roi_1[3]) / 2), int((self.left_roi_1[0] + self.left_roi_1[1]) / 2)), 10, (0, 0, 0), -1)
                cv2.circle(depth_color_map, (int((self.right_roi_1[2] + self.right_roi_1[3]) / 2), int((self.right_roi_1[0] + self.right_roi_1[1]) / 2)), 10, (0, 0, 0), -1)

                left_distance = self.get_roi_distance(depth_image, self.left_roi)
                center_distance = self.get_roi_distance(depth_image, self.center_roi)
                right_distance = self.get_roi_distance(depth_image, self.right_roi)
                left_distance_1 = self.get_roi_distance(depth_image, self.left_roi_1)
                right_distance_1 = self.get_roi_distance(depth_image, self.right_roi_1)

                count += 1
                self.get_logger().info(str([left_distance_1, left_distance, center_distance, right_distance, right_distance_1]))
                # self.get_logger().info(str([left_distance, center_distance, right_distance]))
                if count > 50 and not math.isnan(center_distance):
                    count = 0
                    self.plane_high = center_distance

                    self.controller.set_build_in_pose('NARROW_POSE', 1)
                    time.sleep(1)
                    self.count_status = True
                if self.count_status:
                    count_1 += 1
                if count_1 > 50 and not math.isnan(center_distance):
                    count_1 = 0
                    self.count_status = False
                    self.plane_high_1 = center_distance
                    data = {'/**': {'ros__parameters': {}}}
                    data['/**']['ros__parameters']['plane_distance'] = self.plane_high 
                    data['/**']['ros__parameters']['plane_distance_1'] = self.plane_high_1
                    common.save_yaml_data(data, os.path.join(
                        os.path.abspath(os.path.join(os.path.split(os.path.realpath(__file__))[0], '../../..')),
                        'config/bridge_plane_distance.yaml'))
                    self.controller.set_build_in_pose('DEFAULT_POSE', 1)
                    time.sleep(1)
                    self.debug = False
            else:
                left_roi_1 = [self.left_roi_1[0] - 50, self.left_roi_1[1] - 50, self.left_roi_1[2] - 60, self.left_roi_1[3] - 60] 
                left_roi = [self.left_roi[0] - 50, self.left_roi[1] - 50, self.left_roi[2] - 20, self.left_roi[3] - 20] 
                center_roi = [self.center_roi[0] - 50, self.center_roi[1] - 50, self.center_roi[2], self.center_roi[3]] 
                right_roi = [self.right_roi[0] - 50, self.right_roi[1] - 50, self.right_roi[2] + 20, self.right_roi[3] + 20] 
                right_roi_1 = [self.right_roi_1[0] - 50, self.right_roi_1[1] - 50, self.right_roi_1[2] + 60, self.right_roi_1[3] + 60] 
                cv2.circle(depth_color_map, (int((left_roi_1[2] + left_roi_1[3]) / 2), int((left_roi_1[0] + left_roi_1[1]) / 2)), 10, (0, 0, 0), -1)
                cv2.circle(depth_color_map, (int((left_roi[2] + left_roi[3]) / 2), int((left_roi[0] + left_roi[1]) / 2)), 10, (0, 0, 0), -1)
                cv2.circle(depth_color_map, (int((right_roi[2] + right_roi[3]) / 2), int((right_roi[0] + right_roi[1]) / 2)), 10, (0, 0, 0), -1)
                cv2.circle(depth_color_map, (int((center_roi[2] + center_roi[3]) / 2), int((center_roi[0] + center_roi[1]) / 2)), 10, (0, 0, 0), -1)
                cv2.circle(depth_color_map, (int((right_roi_1[2] + right_roi_1[3]) / 2), int((right_roi_1[0] + right_roi_1[1]) / 2)), 10, (0, 0, 0), -1)

                left_distance = self.get_roi_distance(depth_image, left_roi)
                center_distance = self.get_roi_distance(depth_image, self.center_roi)
                right_distance = self.get_roi_distance(depth_image, right_roi)
                left_distance_1 = self.get_roi_distance(depth_image, left_roi_1)
                right_distance_1 = self.get_roi_distance(depth_image, right_roi_1)
              
                # self.get_logger().info(str([left_distance_1, left_distance, center_distance, right_distance, right_distance_1]))

             
                if math.isnan(left_distance):
                    left_distance = 0
                if math.isnan(left_distance_1):
                    left_distance_1 = 0
                if math.isnan(center_distance):
                    center_distance = 0
                if math.isnan(right_distance):
                    right_distance = 0
                if math.isnan(right_distance_1):
                    right_distance_1 = 0
                threading.Thread(target=self.move_policy, args=(left_distance, left_distance_1, center_distance, right_distance, right_distance_1), daemon=True).start()

                # self.move_policy(left_distance, left_distance_1, center_distance, right_distance, right_distance_1)

            cv2.imshow('depth_color_map', depth_color_map)
            k = cv2.waitKey(1) & 0xFF
            if k == 27 or k == ord('q'):
                self.running = False
        self.cmd_vel_pub.publish(Twist())
        self.get_logger().info('\033[1;32m%s\033[0m' % 'shutdown')
        rclpy.shutdown()

def main():
    node = CrossBridgeNode('cross_bridge')
    rclpy.spin(node)
    node.destroy_node()

if __name__ == "__main__":
    main()
