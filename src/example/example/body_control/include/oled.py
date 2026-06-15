#!/usr/bin/env python3
# encoding: utf-8
#OLED液晶显示屏使用

import time
import rclpy
from std_srvs.srv import Trigger
from ros_robot_controller_msgs.msg import OLEDState


def main():
    rclpy.init()
    node = rclpy.create_node('oled')
    oled_pub = node.create_publisher(OLEDState, '/ros_robot_controller/set_oled', 1)

    node.client = node.create_client(Trigger, '/controller_manager/init_finish')
    node.client.wait_for_service()
    
    msg = OLEDState()
    msg.index = 1
    msg.text = 'Hello word'
    oled_pub.publish(msg)
    time.sleep(0.2)

    msg = OLEDState()
    msg.index = 2
    msg.text = 'Hello Hiwonder' 
    oled_pub.publish(msg)
    
if __name__ == '__main__':
    main()
