#!/usr/bin/env python3
# encoding: utf-8
# 机体舞动（质心）

import math
import time
import rclpy
import signal
from rclpy.node import Node
from std_srvs.srv import Trigger
from kinematics import kinematics_calculate
from controller import step_controller, build_in_pose

class BodyCircle(Node):
    def __init__(self):
        super().__init__('body_circle')
        self.step_controller = step_controller.StepController()
        self.client = self.create_client(Trigger, '/controller_manager/init_finish')
        self.client.wait_for_service()

        self.step_controller.set_build_in_pose('DEFAULT_POSE_M', 1)
        time.sleep(1)

    def gen_circle(self, r):
        points = []
        for i in range(180, 0, -5):
            x = r * math.cos(math.radians(i))
            y = r * math.sin(math.radians(i))
            points.append((x, y))

        for i in range(360, 180, -5):
            x = r * math.cos(math.radians(i))
            y = r * math.sin(math.radians(i))
            points.append((x, y))
        return points


    def wave(self):
        """
        机体扭动
        """
        self.step_controller.set_pose_base(build_in_pose.DEFAULT_POSE_M, 1)
        org_pose = tuple(build_in_pose.DEFAULT_POSE_M)
        time.sleep(0.8)
        for i in range(10, 40, +3):
            points = self.gen_circle(i)
            for x, y in points:
                pose = kinematics_calculate.transform_euler(org_pose, (x, y, 0), 'xyz', (0, 0, 0), degrees=False)
                self.step_controller.set_pose_base(pose, 0.02)
                time.sleep(0.02)


    def reset(self):
        self.step_controller.set_build_in_pose('DEFAULT_POSE_M', 1)
        time.sleep(1)

def main(args=None):
    rclpy.init(args=args)
    node = BodyCircle()
    
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
        node.wave()
        node.reset()
        node.get_logger().info('\033[1;32m%s\033[0m' % 'stop')  
    except Exception as e:
        node.get_logger().error(str(e))

    node.step_controller.destroy_node()  # 清理节点
    rclpy.shutdown()  # 关闭 ROS 2

if __name__ == '__main__':
    main()