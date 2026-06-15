#!/usr/bin/env python3
# encoding: utf-8
# @Author: Aiden
# @Date: 2023/11/10
from servo_controller_msgs.msg import ServoPosition, ServosPosition


def set_servo_position(pub, duration, positions):
    msg = ServosPosition()
    msg.duration = float(duration)
    new_position_list = []
    for i in positions:
        position = ServoPosition()
        position.id = i[0]
        position.position = float(i[1])
        new_position_list.append(position)
    msg.position = new_position_list
    msg.position_unit = "pulse"
    pub.publish(msg)

if __name__ == '__main__':
    import time
    import rclpy
    from rclpy.node import Node

    rclpy.init()
    node = Node('servo_control_demo')
    pub = node.create_publisher(ServosPosition, 'servo_controller', 1)
    while rclpy.ok():
        set_servo_position(pub, 0.02, ((1, 400),))
        time.sleep(1)
        set_servo_position(pub, 0.02, ((1, 600),))
        time.sleep(1)
