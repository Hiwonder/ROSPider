#!/usr/bin/env python3
# encoding: utf-8
# 颜色识别(color recognition)

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

class ColorRecognitionNode(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.name = name
        self.running = True
        self.center = None
        self.start_action = False
        self.target_color = ''
        self.color = ''
        self.count = 0
        self.image_queue = queue.Queue(maxsize=2)
        self.controller = step_controller.StepController()
        signal.signal(signal.SIGINT, self.shutdown)

        self.create_subscription(ColorsInfo, '/color_detect/color_info', self.get_color_callback, 1)
        self.create_subscription(Image, '/color_detect/image_result', self.image_callback, 1)
        timer_cb_group = ReentrantCallbackGroup()
        self.create_service(Trigger, '~/start', self.start_srv_callback, callback_group=timer_cb_group) # 进入玩法(enter the game)

        self.client = self.create_client(Trigger, '/controller_manager/init_finish')
        self.client.wait_for_service()
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
        joint_angle = [500, 750, 200, 150, 500, 700]
        set_servo_position(self.joints_pub, 1, ((19, joint_angle[0]), (20, joint_angle[1]), (21, joint_angle[2]), (22, joint_angle[3]), (23, joint_angle[4]), (24, joint_angle[5])))

        threading.Thread(target=self.action, daemon=True).start()
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
        self.get_logger().info('\033[1;32m%s\033[0m' % "start color recognition")

        msg = SetColorDetectParam.Request()
        msg_red = ColorDetect()
        msg_red.color_name = 'red'
        msg_red.detect_type = 'circle'
        msg_green = ColorDetect()
        msg_green.color_name = 'green'
        msg_green.detect_type = 'circle'
        msg_blue = ColorDetect()
        msg_blue.color_name = 'blue'
        msg_blue.detect_type = 'circle'
        msg.data = [msg_red, msg_green, msg_blue]
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
                self.center = data[0]
                self.color = data[0].color
            else:
                self.color = ''
        else:
            self.color = ''

    def action(self):
        while self.running:
            if self.start_action:
                self.get_logger().info('\033[1;32mcolor: %s\033[0m' % self.target_color)
                msg = BuzzerState()
                msg.freq = 2500
                msg.on_time = 0.1
                msg.off_time = 0.5
                msg.repeat = 1
                self.buzzer_pub.publish(msg)
                if self.target_color == 'red':
                    set_servo_position(self.joints_pub, 0.5, (((22, 200), )))
                    time.sleep(0.5)
                    set_servo_position(self.joints_pub, 0.5, (((22, 100), )))
                    time.sleep(0.5)
                    set_servo_position(self.joints_pub, 0.5, (((22, 150), )))
                    time.sleep(0.5)
                elif self.target_color == 'green':
                    set_servo_position(self.joints_pub, 0.5, (((19, 400), )))
                    time.sleep(0.5)
                    set_servo_position(self.joints_pub, 0.5, (((19, 600), )))
                    time.sleep(0.5)
                    set_servo_position(self.joints_pub, 0.5, (((19, 500), )))
                    time.sleep(0.5)
                elif self.target_color == 'blue':
                    set_servo_position(self.joints_pub, 0.5, (((23, 400), )))
                    time.sleep(0.5)
                    set_servo_position(self.joints_pub, 0.5, (((23, 600), )))
                    time.sleep(0.5)
                    set_servo_position(self.joints_pub, 0.5, (((23, 500), )))
                    time.sleep(0.5)
                self.start_action = False
            else:
                time.sleep(0.01)

    def main(self):
        count = 0
        while self.running:
            try:
                image = self.image_queue.get(block=True, timeout=1)
            except queue.Empty:
                if not self.running:
                    break
                else:
                    continue
            if self.color in ['red', 'green', 'blue']:
                if not self.start_action :
                    self.count += 1
                    if self.count > 30:
                        self.count = 0
                        self.target_color = self.color
                        self.start_action = True
                else:
                    count = 0
            if image is not None:
                cv2.imshow('image', image)
                key = cv2.waitKey(1)
                if key == ord('q') or key == 27:  # 按q或者esc退出(Press Q or Esc to exit)
                    self.running = False
        self.controller.run_action('init')
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
    node = ColorRecognitionNode('color_recognition')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
 
if __name__ == "__main__":
    main()

