#!/usr/bin/env python3
# encoding: utf-8
# 六足逆运动学控制

import time
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from controller import step_controller

class BodyIk(Node):
    def __init__(self):
        super().__init__('body_ik')
        self.step_controller = step_controller.StepController()
        self.client = self.create_client(Trigger, '/controller_manager/init_finish')
        self.client.wait_for_service()

def main(args=None):
    rclpy.init(args=args)
    node = BodyIk()
    try:
        for i in range(3):
            joints = node.step_controller.set_leg_position(2, (0, 140, -50), 2)
            node.get_logger().info('\033[1;32m%s\033[0m' % str(joints))
            time.sleep(2.5)
            joints = node.step_controller.set_leg_position(2, (0, 250, -50), 2)
            node.get_logger().info('\033[1;32m%s\033[0m' % str(joints))
            time.sleep(2.5)

        node.get_logger().info('\033[1;32m%s\033[0m' % 'stop')
    except Exception as e:
        node.get_logger().error(str(e))
    node.step_controller.destroy_node()  # 清理节点
    rclpy.shutdown()  # 关闭 ROS 2

if __name__ == '__main__':
    main()

 
