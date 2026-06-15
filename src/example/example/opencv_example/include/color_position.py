#!/usr/bin/env python3
# encoding: utf-8
# 色块定位

import os
import cv2
import time
import queue
import rclpy
import signal
import threading
import numpy as np
from rclpy.node import Node
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
from controller import step_controller
from rclpy.executors import MultiThreadedExecutor
from interfaces.msg import ColorsInfo, ColorDetect
from servo_controller_msgs.msg import ServosPosition
from ros_robot_controller_msgs.msg import BuzzerState
from rclpy.callback_groups import ReentrantCallbackGroup
from interfaces.srv import SetColorDetectParam, SetCircleROI
from servo_controller.bus_servo_control import set_servo_position

class ColorPositionNode(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.name = name
        self.running = True
        self.center = None
        self.color = ''
        self.count = 0
        self.target_color = self.get_parameter('color').value
        self.image_queue = queue.Queue(maxsize=2)
        signal.signal(signal.SIGINT, self.shutdown)

        self.create_subscription(ColorsInfo, '/color_detect/color_info', self.get_color_callback, 1)
        self.create_subscription(Image, '/color_detect/image_result', self.image_callback, 1)
        timer_cb_group = ReentrantCallbackGroup()
        self.create_service(Trigger, '~/start', self.start_srv_callback, callback_group=timer_cb_group) # 进入玩法(enter the game)

        self.client = self.create_client(Trigger, '/controller_manager/init_finish')
        self.client.wait_for_service()
        self.controller = step_controller.StepController()
        self.buzzer_pub = self.create_publisher(BuzzerState, 'ros_robot_controller/set_buzzer', 1)
        self.joints_pub = self.create_publisher(ServosPosition, 'servo_controller', 1)

        self.set_color_client = self.create_client(SetColorDetectParam, '/color_detect/set_param', callback_group=timer_cb_group)
        self.set_roi_client = self.create_client(SetCircleROI, '/color_detect/set_circle_roi', callback_group=timer_cb_group)
        self.set_color_client.wait_for_service()
        self.set_roi_client.wait_for_service()

        self.timer = self.create_timer(0.0, self.init_process, callback_group=timer_cb_group)

    def init_process(self):
        self.timer.cancel()

        self.start_srv_callback(Trigger.Request(), Trigger.Response())
        self.controller.set_build_in_pose('DEFAULT_POSE', 1)
        joint_angle = [500, 670, 40, 210, 500, 700]
        set_servo_position(self.joints_pub, 1, ((19, joint_angle[0]), (20, joint_angle[1]), (21, joint_angle[2]), (22, joint_angle[3]), (23, joint_angle[4]), (24, joint_angle[5])))

        threading.Thread(target=self.main, daemon=True).start()
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

    def get_node_state(self, request, response):
        response.success = True
        return response

    def shutdown(self, signum, frame):
        self.running = False

    def send_request(self, client, msg):
        future = client.call_async(msg)
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()

    def start_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "start color position")

        msg = SetColorDetectParam.Request()
        color_msg = ColorDetect()
        color_msg.color_name = self.target_color
        color_msg.detect_type = 'circle'
      
        msg.data = [color_msg]
        res = self.send_request(self.set_color_client, msg)
        if res.success:
            self.get_logger().info('\033[1;32m%s\033[0m' % 'set color success')
        else:
            self.get_logger().info('\033[1;32m%s\033[0m' % 'set color fail')
         
        response.success = True
        response.message = "start"
        return response
     

    def get_color_callback(self, msg):
        data = msg.data
        if data != []:
            if data[0].radius > 10:
                # self.center = data[0]
                self.color = data[0].color
                self.center = (data[0].x, data[0].y)
                self.get_logger().info('\033[1;32m(x, y):%s\033[0m' % str(self.center))
            else:
                self.color = ''
                self.center = None
        else:
            self.color = ''
            self.center = None


    def main(self):
        while self.running:
            try:
                image = self.image_queue.get(block=True, timeout=1)
            except queue.Empty:
                if not self.running:
                    break
                else:
                    continue
            if self.color in ['red', 'green', 'blue'] :
                self.count += 1
                if self.count > 30:
                    self.count = 0
                    self.target_color = self.color
            else:
                self.count = 0
            if image is not None:
                if self.center is not None:
                    cv2.circle(image, (int(self.center[0]), int(self.center[1])), 5, (0, 255, 255), -1)
                    string = "({:0.1f}, {:0.1f})".format(self.center[0], self.center[1])
                    cv2.putText(image, string, (int(self.center[0]), int(self.center[1] + 16)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                cv2.imshow('image', image)
                key = cv2.waitKey(1)
                if key == ord('q') or key == 27:  # 按q或者esc退出(Press Q or Esc to exit)
                    self.running = False
        rclpy.shutdown()

    def image_callback(self, ros_image):
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8,
                               buffer=ros_image.data)  # 原始 RGB 画面(original RGB image)

        if self.image_queue.full():
            # 如果队列已满，丢弃最旧的图像(if the queue is full, discard the oldest image)
            self.image_queue.get()
            # 将图像放入队列(put the image into the queue)
        self.image_queue.put(rgb_image)

def main():
    node = ColorPositionNode('color_position')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
 
if __name__ == "__main__":
    main()

