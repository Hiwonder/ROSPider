#!/usr/bin/env python3
# encoding: utf-8

# 语音控制移动(voice control move)
import os
import json
import math
import time
import rclpy
import threading
import numpy as np
import sdk.pid as pid
import sdk.common as common
from rclpy.node import Node
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Int32
from controller import controller_client
from xf_mic_asr_offline import voice_play
from servo_controller_msgs.msg import ServosPosition
from ros_robot_controller_msgs.msg import BuzzerState
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from servo_controller.action_group_controller import ActionGroupController

MAX_SCAN_ANGLE = 240  # 激光的扫描角度,去掉总是被遮挡的部分degree(laser scanning angle, removing obstructed degrees)
CAR_WIDTH = 0.4  # meter

class VoiceControMovelNode(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)
        
        self.angle = None
        self.words = None
        self.running = True
        self.haved_stop = False
        self.lidar_follow = False
        self.start_follow = False
        self.last_status = Twist()
        self.threshold = 3
        self.speed = 0.3
        self.stop_dist = 0.4
        self.count = 0
        self.scan_angle = math.radians(90)
        self.declare_parameter('move', False)
        self.move = self.get_parameter('move').value

        self.pid_yaw = pid.PID(1.6, 0, 0.16)
        self.pid_dist = pid.PID(1.7, 0, 0.16)

        self.language = os.environ['ASR_LANGUAGE']
        self.controller = controller_client.ControllerClient()
        self.agc_controller = ActionGroupController(self.create_publisher(ServosPosition, 'servo_controller', 1), '/home/ubuntu/software/actionset_editor/ActionGroups')
        self.cmd_vel_pub = self.create_publisher(Twist, '/controller/cmd_vel', 1)
        self.buzzer_pub = self.create_publisher(BuzzerState, '/ros_robot_controller/set_buzzer', 1)
        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(String, '/asr_node/voice_words', self.words_callback, 1)
        self.create_subscription(Int32, '/awake_node/angle', self.angle_callback, 1)

        self.client = self.create_client(Trigger, '/asr_node/init_finish')
        self.client.wait_for_service()  # 阻塞等待(blocking wait)
        self.declare_parameter('delay', 0)
        time.sleep(self.get_parameter('delay').value)

        self.get_logger().info('唤醒口令: 小幻小幻(Wake up word: hello hiwonder)')
        self.get_logger().info('唤醒后15秒内可以不用再唤醒(No need to wake up within 15 seconds after waking up)')
        self.get_logger().info('控制指令: 左转 右转 前进 后退 过来 跳个舞吧(Voice command: turn left/turn right/go forward/go backward/come here /dance)')
        self.time_stamp = time.time()
        self.current_time_stamp = time.time()
        threading.Thread(target=self.main, daemon=True).start()
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.play('running')

        if self.language == 'Chinese':
            self.get_logger().info('\033[1;32m%s\033[0m' % '准备就绪')
        else:
            self.get_logger().info('\033[1;32m%s\033[0m' % 'I am ready')




    def get_node_state(self, request, response):
        response.success = True
        return response

    def play(self, name):
        voice_play.play(name, language=self.language)

    def words_callback(self, msg):
        self.words = json.dumps(msg.data, ensure_ascii=False)[1:-1]
        if self.language == 'Chinese':
            self.words = self.words.replace(' ', '')
        self.get_logger().info('words:%s' % self.words)
        if self.words is not None and self.words not in ['唤醒成功(wake-up-success)', '休眠(Sleep)', '失败5次(Fail-5-times)',
                                                         '失败10次(Fail-10-times']:
            pass
        elif self.words == '唤醒成功(wake-up-success)':
            self.play('awake')
        elif self.words == '休眠(Sleep)':
            msg = BuzzerState()
            msg.freq = 1000
            msg.on_time = 0.1

            msg.off_time = 0.01
            msg.repeat = 1
            self.buzzer_pub.publish(msg)

    def angle_callback(self, msg):
        self.angle = msg.data
        self.get_logger().info('angle:%s' % self.angle)
        self.start_follow = False
        self.start_follow = False 


    def main(self):
        while True:
            if self.words is not None:
                self.move = True
                twist = Twist()
                if self.words == '前进' or self.words == 'go forward':
                    self.play('go')
                    self.time_stamp = time.time() + 4
                    twist.linear.x = 0.05
                elif self.words == '后退' or self.words == 'go backward':
                    self.play('back')
                    self.time_stamp = time.time() + 4
                    twist.linear.x = -0.05
                elif self.words == '左转' or self.words == 'turn left':
                    self.play('turn_left')
                    self.time_stamp = time.time() + 4
                    twist.angular.z = 0.3
                elif self.words == '右转' or self.words == 'turn right':
                    self.play('turn_right')
                    self.time_stamp = time.time() + 4
                    twist.angular.z = -0.3
                elif self.words == '左平移' or self.words == 'move left':
                    self.play('move_left')
                    self.time_stamp = time.time() + 4
                    twist.linear.y = 0.05
                elif self.words == '右平移' or self.words == 'move right':
                    self.play('move_right')
                    self.time_stamp = time.time() + 4
                    twist.linear.y = -0.05
                elif self.words == '跳个舞吧' or self.words == 'dance':
                    self.play('dance')
                    self.agc_controller.run_action('twist')

                elif self.words == '过来' or self.words == 'come here':
                    self.play('come')
                    self.get_logger().info('\033[1;32m%s\033[0m' % self.angle)

                    if 270 > self.angle > 90:
                        twist.angular.z = -0.3
                        self.time_stamp = time.time() + abs(math.radians(self.angle - 90) / twist.angular.z)
                    else:
                        twist.angular.z = 0.3
                        if self.angle <= 90:
                            self.angle = 90 - self.angle
                        else:
                            self.angle = 450 - self.angle
                        self.time_stamp = time.time() + abs(math.radians(self.angle) / twist.angular.z)
                    self.lidar_follow = True
                elif self.words == '休眠(Sleep)':
                    time.sleep(0.01)
                self.words = None
                self.haved_stop = False
                if self.move:
                    self.cmd_vel_pub.publish(twist)

            else:
                time.sleep(0.01)
            self.current_time_stamp = time.time()
            if self.time_stamp < self.current_time_stamp and not self.haved_stop and self.move:
                self.controller.traveling(gait=-2, time=1, steps=0)
                self.haved_stop = True
                if self.lidar_follow:
                    self.lidar_follow = False
                    self.start_follow = True




def main():
    node = VoiceControMovelNode('voice_control_move')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
