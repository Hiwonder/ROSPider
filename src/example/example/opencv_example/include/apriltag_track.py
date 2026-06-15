#!/usr/bin/env python3
# encoding: utf-8
# 标签跟踪(apriltag tracking)

import os
import cv2
import math
import queue
import rclpy
import threading
import numpy as np
import sdk.pid as pid
import sdk.common as common
from rclpy.node import Node
from app.common import Heart
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from app.common import ColorPicker
from geometry_msgs.msg import Twist
from controller import step_controller
from interfaces.msg import ApriltagsInfo
from std_srvs.srv import SetBool, Trigger
from rclpy.callback_groups import ReentrantCallbackGroup
from interfaces.srv import SetPoint, SetFloat64, SetString
from servo_controller_msgs.msg import ServosPosition
from servo_controller.bus_servo_control import set_servo_position


class OjbectTrackingNode(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.name = name
        self.tag_id = ''
        self.set_above = False
        self.set_callback = False
        self.tracker = None
        self.is_running = True
        self.lock = threading.RLock()
        self.image_sub = None
        self.result_image = None
        self.image_height = None
        self.image_width = None
        self.pid_yaw = pid.PID(0.005, 0.0, 0.000001)
        self.pid_dist = pid.PID(0.002, 0.0, 0.00)
        self.last_color_circle = None
        self.x = 0
        self.distance = 0
        self.x_stop = 320
        self.d_stop = 15
        self.pro_size = (320, 240)
        self.bridge = CvBridge()
        self.controller = step_controller.StepController()

        self.target_tag = self.get_parameter('target_tag').value
        self.get_logger().info('\033[1;32mself.target_tag%s\033[0m' % self.target_tag)

        self.image_queue = queue.Queue(2)
        self.cmd_vel_pub = self.create_publisher(Twist, '/controller/cmd_vel', 1)
        self.image_sub = self.create_subscription(Image, '/apriltag_detect/image_result' , self.image_callback, 1)  # 画面订阅
        self.apriltag_info_sub = self.create_subscription(ApriltagsInfo, '/apriltag_detect/apriltag_info',  self.apriltag_info_callback, 1) # 标签信息订阅

        self.set_target_tag_srv = self.create_service(SetPoint, '~/set_target_tag', self.set_target_tag_srv_callback)
        self.joints_pub = self.create_publisher(ServosPosition, 'servo_controller', 1)

        self.timer_cb_group = ReentrantCallbackGroup()
        self.client = self.create_client(Trigger, '/controller_manager/init_finish',  callback_group=self.timer_cb_group)
        self.client.wait_for_service()
        self.timer = self.create_timer(0.0, self.init_process, callback_group=self.timer_cb_group)

        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

    def init_process(self):
        self.timer.cancel()
        self.controller.set_build_in_pose('DEFAULT_POSE', 1)
        joint_angle = [500, 750, 200, 150, 500, 700]
        set_servo_position(self.joints_pub, 1, ((19, joint_angle[0]), (20, joint_angle[1]), (21, joint_angle[2]), (22, joint_angle[3]), (23, joint_angle[4]), (24, joint_angle[5])))
        threading.Thread(target=self.main, daemon=True).start()

    def get_node_state(self, request, response):
        response.success = True
        return response
    
    def apriltag_info_callback(self, msg):
        data = msg.data
        if data != []:
            self.x = data[0].x
            self.distance = data[0].d
            self.tag_id = data[0].id

        else:
            self.x = 0
            self.distance = 0
            self.tag_id = ''

    def set_target_tag_srv_callback(self, request, response):
        with self.lock:
            self.target_tag = request
            self.get_logger().info('\033[1;32mset_target_color %s\033[0m' % self.target_tag)

        response.success = True
        response.message = "set_target_color"
        return response

    def main(self):
        while True:
            try:
                image = self.image_queue.get(block=True, timeout=1)
            except queue.Empty:
                continue

            result = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            twist = Twist()
            if self.tag_id == self.target_tag and self.distance !=0 and self.x !=0:
                if abs(self.distance - self.d_stop) > 1:
                    self.pid_dist.update(self.distance - self.d_stop)
                    twist.linear.x = common.set_range(self.pid_dist.output, -0.01, 0.01) * -3
                else:
                    twist.linear.x = 0.0
                    self.pid_dist.clear()

                if abs(self.x - self.x_stop) > 20:
                    self.pid_yaw.update(self.x - self.x_stop)
                    twist.angular.z = common.set_range(self.pid_yaw.output, -1, 1) * 0.2
                else:
                    twist.angular.z = 0.0
                    self.pid_yaw.clear()
                self.cmd_vel_pub.publish(twist)

            else:
                self.cmd_vel_pub.publish(Twist())
                self.pid_dist.clear()
                self.pid_yaw.clear()
            cv2.imshow("image", result)
            k = cv2.waitKey(1)
            if k != -1:
                break

        self.cmd_vel_pub.publish(Twist())
        rclpy.shutdown()

    def exit_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % 'object tracking exit')

        with self.lock:
            self.is_running = False
            self.cmd_vel_pub.publish(Twist())
            
        response.success = True
        response.message = "exit"
        return response

    def image_callback(self, ros_image):
        # 将ros格式(rgb)转为opencv的rgb格式(convert RGB format of ROS to that of OpenCV)
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, "rgb8")
        rgb_image = np.array(cv_image, dtype=np.uint8)
        self.image_height, self.image_width = rgb_image.shape[:2]

        result_image = np.copy(rgb_image)  # 显示结果用的画面(the image used for display the result)
        with self.lock:

            if self.image_queue.full():
                # 如果队列已满，丢弃最旧的图像(if the queue is full, discard the oldest image)
                self.image_queue.get()
                # 将图像放入队列(put the image into the queue)
            self.image_queue.put(result_image)

def main():
    node = OjbectTrackingNode('object_tracking')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

