#!/usr/bin/env python3
# encoding: utf-8
# @Author: Aiden
# @Date: 2023/11/10
import math
import rclpy
import threading
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from ros_robot_controller_msgs.srv import GetBusServoState
from ros_robot_controller_msgs.msg import ServoPosition, ServosPosition, GetBusServoCmd

class ServoState:
    def __init__(self, name='', position=''):
        self.name = name
        self.position = position

class ServoManager(Node):
    def __init__(self, connected_ids=[]):
        super().__init__('servo_manager', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.servos = {}
        self.position = []
        self.connected_ids = connected_ids

        timer_cb_group = ReentrantCallbackGroup()
        for key, name in self.connected_ids.items():
            name = f'{name}.init'
            position = self.get_parameter(name).value
            self.position.append(position)

    
        for i in connected_ids:
            index = int(i) - 1
            if 0 <= index < len(self.position):

                self.servos[i] = ServoState(connected_ids[i],self.position[index])

        self.servo_position_pub = self.create_publisher(ServosPosition, 'ros_robot_controller/bus_servo/set_position', 1)
        self.client = self.create_client(GetBusServoState, 'ros_robot_controller/bus_servo/get_state', callback_group=timer_cb_group)
        self.client.wait_for_service()
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')



    def connect(self):
        # 通过读取id来检测舵机
        # threading.Thread(target=self.check,daemon=True).start()
        pass
        

    def get_position(self):
        # self.get_logger().info('\033[1;32m%s\033[-2m' % str(self.servos))

        return self.servos

    def send_request(self, client, msg):
        future = client.call_async(msg)
        while rclpy.ok():
            # self.get_logger().info('ID %s ' % str(future))
            if future.done() and future.result():
                self.get_logger().info('ID %s ' % str(future.result()))

                return future.result()

    def get_servo_id(self, servo_id):
        request = GetBusServoState.Request()
        cmd = GetBusServoCmd()
        cmd.id = int(servo_id)
        cmd.get_id = int(1)
        cmd.get_position = int(1)
        request.cmd = [cmd] 
        for i in range(0, 25):

            self.get_logger().info('ID %s ' % str(request))
            res = self.send_request(self.client, request)

            response = res.response
            if response[0].present_id == servo_id:
                return True
        return False

    def set_position(self, duration, position):
        duration = 0.02 if duration < 0.02 else 30 if duration > 30 else duration
        msg = ServosPosition()
        msg.duration = float(duration)
        for i in position:
            position = int(i.position)
            position = 0 if position < 0 else 1000 if position > 1000 else position
            self.servos[str(i.id)].position = position  # 记录发送的位置
            servo_msg = ServoPosition()
            servo_msg.id = i.id
            servo_msg.position = position
            msg.position.append(servo_msg)
        self.servo_position_pub.publish(msg)

