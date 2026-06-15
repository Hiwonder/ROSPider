#!/usr/bin/env python3
# encoding: utf-8
# 躯干姿态调节

import time
import rclpy
import signal
from rclpy.node import Node
from std_srvs.srv import Trigger
from controller import step_controller

class PostureAdjustment(Node):
    def __init__(self):
        super().__init__('posture_adjustment')
        self.step_controller = step_controller.StepController()
        self.client = self.create_client(Trigger, '/controller_manager/init_finish')
        self.client.wait_for_service()

        self.step_controller.set_build_in_pose('DEFAULT_POSE', 1)
        time.sleep(1)

    def transform(self):
        self.step_controller.transform_pose_euler((0, 0, 0), 'xyz', (-15, 0, 0), 1) # 旋转
        time.sleep(1)
        self.step_controller.transform_pose_euler((0, 0, 0), 'xyz', (0, 15, 0), 1) # 旋转
        time.sleep(1)
        self.step_controller.transform_pose_euler((0, 0, 0), 'yxz', (-15, 15, 0), 2) #欧拉角注意顺序
        time.sleep(2)
        self.step_controller.transform_pose_euler((40, 0, 0), 'xyz', (0, 0, 0), 1) # 平移， 单位为 mm
        time.sleep(1)
        self.step_controller.transform_pose_euler((0, 40, 0), 'xyz', (0, 0, 0), 1) # 平移， 单位为 mm
        time.sleep(1)
        self.step_controller.transform_pose_euler((-40, -40, 0), 'xyz', (0, 0, 0), 2) # 平移， 单位为 mm
        time.sleep(1)

    def reset(self):
        self.step_controller.set_build_in_pose('DEFAULT_POSE', 1)
        time.sleep(1)

def main(args=None):
    rclpy.init(args=args)
    node = PostureAdjustment()
    
        # 定义自定义信号处理函数，确保在接收到 SIGINT 时发送停止指令
    def signal_handler(sig, frame):
        node.get_logger().info('\033[1;32m%s\033[0m' % 'Received SIGINT, stopping...')
        node.reset()  # 发送停止指令
        # 清理节点和 shutdown ROS2
        node.destroy_node()
        rclpy.shutdown()
        exit(0)
        
    signal.signal(signal.SIGINT, signal_handler)

    try:
        node.transform()
        node.reset()
        node.get_logger().info('\033[1;32m%s\033[0m' % 'stop')  
    except Exception as e:
        node.get_logger().error(str(e))

    node.step_controller.destroy_node()  # 清理节点
    rclpy.shutdown()  # 关闭 ROS 2

if __name__ == '__main__':
    main()