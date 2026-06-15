#!/usr/bin/env python3
# encoding: utf-8

# 手势控制(hand gesture control)
import os
import cv2
import math
import time
import rclpy
import threading
import numpy as np
from rclpy.node import Node
from app.common import Heart
from std_msgs.msg import String 
from interfaces.msg import Points
from geometry_msgs.msg import Twist
from controller import controller_client 
from std_srvs.srv import SetBool, Trigger
from rclpy.executors import MultiThreadedExecutor
from servo_controller_msgs.msg import ServosPosition
from ros_robot_controller_msgs.msg import BuzzerState
from rclpy.callback_groups import ReentrantCallbackGroup
from servo_controller.bus_servo_control import set_servo_position
from servo_controller.action_group_controller import ActionGroupController


class HandGestureControlNode(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)
        self.name = name
        self.image = None
        self.image_sub = None
        self.running = True
        self.last_point = [0, 0]
        self.linear_speed = 0.06
        self.th = None
        self.act = True
        self.gesture = None
        self.thread_running = True
        self.gesture_counter = 0
        self.last_gesture = None

        self.controller = controller_client.ControllerClient()
        self.agc_controller = ActionGroupController(self.create_publisher(ServosPosition, 'servo_controller', 1), '/home/ubuntu/software/actionset_editor/ActionGroups')

        self.cmd_vel_pub = self.create_publisher(Twist, '/controller/cmd_vel', 1)  # 底盘控制(chassis control)
        self.buzzer_pub = self.create_publisher(BuzzerState, '/ros_robot_controller/set_buzzer', 1)

        timer_cb_group = ReentrantCallbackGroup()
        self.create_service(Trigger, '~/enter', self.enter_srv_callback, callback_group=timer_cb_group)  # 进入玩法(enter the game)
        self.create_service(Trigger, '~/exit', self.exit_srv_callback, callback_group=timer_cb_group)  # 退出玩法(exit the game)
        self.create_service(SetBool, '~/set_running', self.set_running_srv_callback)  # 开启玩法(start the game)
        self.joints_pub = self.create_publisher(ServosPosition, 'servo_controller', 1)

        Heart(self, self.name + '/heartbeat', 5, lambda _: self.exit_srv_callback(request=Trigger.Request(), response=Trigger.Response()))  # 心跳包(heartbeat package)

        self.set_hand_trajectory_enter_client = self.create_client(Trigger, '/hand_trajectory/enter', callback_group=timer_cb_group)
        self.set_hand_trajectory_exit_client = self.create_client(Trigger, '/hand_trajectory/exit', callback_group=timer_cb_group)
        self.set_hand_trajectory_stop_client = self.create_client(Trigger, '/hand_trajectory/stop', callback_group=timer_cb_group)
        self.set_hand_trajectory_start_client = self.create_client(Trigger, '/hand_trajectory/start', callback_group=timer_cb_group)
        self.set_hand_trajectory_start_client.wait_for_service()
        self.set_hand_trajectory_stop_client.wait_for_service()
        self.set_hand_trajectory_enter_client.wait_for_service()
        self.set_hand_trajectory_exit_client.wait_for_service()
        threading.Thread(target=self.do_act, daemon=True).start()

        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')
  

    def get_node_state(self, request, response):
        response.success = True
        return response

    def send_request(self, client, msg):
        future = client.call_async(msg)
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()

    def enter_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "hand gesture control enter")
        set_servo_position(self.joints_pub, 1.5, ((24, 500), (23, 500), (22, 150), (21, 180), (20, 810), (19, 500)))

        self.controller.traveling(gait=-2, time=1, steps=0)
        self.create_subscription(String, '/hand_trajectory/gesture', self.gesture_callback, 1)  # 手势识别结果订阅(subscribe to the camera)

        self.send_request(self.set_hand_trajectory_enter_client, Trigger.Request())
        if self.image_sub is None:
            self.image_sub = self.create_subscription(Points, '/hand_trajectory/points', self.get_hand_points_callback, 1)
        self.thread_running = False
        
        response.success = True
        response.message = "enter"
        return response

    def exit_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "hand gesture control exit")
        self.controller.traveling(gait=-2, time=1, steps=0)
        self.thread_running = False
        self.send_request(self.set_hand_trajectory_exit_client, Trigger.Request())
        
        try:
            if self.image_sub is not None:
                self.destroy_subscription(self.image_sub)
                self.image_sub = None
        except Exception as e:
            self.get_logger().error(str(e))
        response.success = True
        response.message = "exit"
        return response
   
    def set_running_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "set_running")
        
        if request.data:
            self.thread_running = True
            self.send_request(self.set_hand_trajectory_start_client, Trigger.Request())
        else:
            self.thread_running = False
            self.send_request(self.set_hand_trajectory_stop_client, Trigger.Request())
        self.controller.traveling(gait=-2, time=1, steps=0)
        
        response.success = True
        response.message = "set_running"
        return response
    
    def buzzer_warn(self):
        msg = BuzzerState()
        msg.freq = 1900
        msg.on_time = 0.2
        msg.off_time = 0.01
        msg.repeat = 1
        self.buzzer_pub.publish(msg)

    def gesture_callback(self, msg):
        current_gesture = msg.data
        # 如果手势发生变化，重置计数器
        if current_gesture != self.last_gesture or not self.act:
            self.gesture_counter = 0
            self.last_gesture = current_gesture
            return
        # 相同手势时增加计数器
        self.gesture_counter += 1  
        # 达到稳定阈值时更新最终手势
        if self.gesture_counter >= 10 and self.gesture != current_gesture:
            self.gesture = current_gesture


    def do_act(self):
        ACTION = {  'rock' : 'attack',
                    'thumbs_up' : 'twist_l',
                    'OK' : 'wave',
                    'yes' : 'yes'}
        
        LOCK_SERVOS={'19':500, '20':810, '21':180, '22':150,  '23':500,  '24':700}
        while self.act:
            if self.gesture is not None and self.gesture in ACTION:
                self.act = False
                self.buzzer_warn()
                run = self.agc_controller.run_action(ACTION[self.gesture],lock_servos=LOCK_SERVOS)
                self.controller.traveling(gait=-2, time=1, steps=0)
                self.gesture = None

                if run == None:
                    self.act = True
            else:
                time.sleep(0.02)


    def move_action(self, *args):
        status = 0
        t_start = time.time()
        while self.thread_running:
            current_time = time.time()
            if status == 0 and t_start < current_time:
                status = 1
                
                twist = args[0]
                self.cmd_vel_pub.publish(twist)
                t_start = current_time + args[1]/50.0/self.linear_speed

            elif status == 1 and t_start < current_time:
                status = 0
                self.controller.traveling(gait=-2, time=1, steps=0)
                break
            time.sleep(0.01)

    def get_hand_points_callback(self, msg):
        points = []
        left_and_right = [0]
        up_and_down = [0]
        if len(msg.points) >= 5:
            for i in msg.points:
                if int(i.x) - self.last_point[0] > 0:
                    left_and_right.append(1)
                else:
                    left_and_right.append(-1)
                if int(i.y) - self.last_point[1] > 0:
                    up_and_down.append(1)
                else:
                    up_and_down.append(-1)
                points.extend([(int(i.x), int(i.y))])
                self.last_point = [int(i.x), int(i.y)]
            line = cv2.fitLine(np.array(points), cv2.DIST_L2, 0, 0.01, 0.01)
            angle = int(abs(math.degrees(math.acos(line[0][0]))))
            twist = Twist()
            # self.get_logger().info('\033[1;32mangle: %s\033[0m' % str(angle))
            if 0 <= angle < 30:
                if sum(left_and_right) > 0:
                    twist.linear.y = float(self.linear_speed)
                else:
                    twist.linear.y = float(-self.linear_speed)

            elif 60 < angle <= 90:
                if sum(up_and_down) > 0:
                    twist.linear.x = float(-self.linear_speed)
                else:
                    twist.linear.x = float(self.linear_speed)
            if self.th is None:
                self.th = threading.Thread(target=self.move_action, args=(twist, len(points)))
                self.th.start()
            else:
                if not self.th.is_alive():
                    self.th = threading.Thread(target=self.move_action, args=(twist, len(points)))
                    self.th.start()
                else:
                    self.thread_running = False
                    time.sleep(0.1)

def main():
    node = HandGestureControlNode('hand_gesture')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
 
if __name__ == "__main__":
    main()
    
