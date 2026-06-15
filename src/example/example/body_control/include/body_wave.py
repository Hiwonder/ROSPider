#!/usr/bin/env python3
# encoding: utf-8
# 机体扭动
import math
import time
import rclpy
import signal
from rclpy.node import Node
from std_srvs.srv import Trigger
from kinematics import kinematics_calculate
from controller import step_controller, build_in_pose


class BodyWave(Node):
    def __init__(self):
        super().__init__('body_wave')
        self.step_controller = step_controller.StepController()
        self.client = self.create_client(Trigger, '/controller_manager/init_finish')
        self.client.wait_for_service()

        self.step_controller.set_build_in_pose('DEFAULT_POSE_M', 1)
        time.sleep(1)

    def wave(self):
        """
        机体扭动
        """
        duration = 0.03
        org_pose = tuple(build_in_pose.DEFAULT_POSE_M)
        time.sleep(0.8)
        # 逐渐加快并加大摇摆幅度
        for j in range(7, 16, 2):
            i = 90 
            j = min(15, j)
            while i <= 360 + 85:
                if i == 90 and j == 7:
                    t = 0.5
                else:
                    t = duration
                i += 4 + j * 0.30
                x = math.sin(math.radians(i)) * (0.018 * (j + ((i - 90) / 360) * 2))
                y = math.cos(math.radians(i)) * (0.018 * (j + ((i - 90) / 360) * 2))
                pose = kinematics_calculate.transform_euler(org_pose, (0, 0, 0), 'xy', (x, y), degrees=False)
                self.step_controller.set_pose_base(pose, t)
                time.sleep(t)

        # 逐渐放慢和减小摇摆幅度
        for j in range(15, 4, -3):
            i = 360 + 85
            while i >= 90:
                i += -(4 + j * 0.30)
                k = 360 + 90 - i + 90
                x = math.sin(math.radians(k)) * (0.018 * (j + (1 - (i - 90) / 360) * -3))
                y = math.cos(math.radians(k)) * (0.018 * (j + (1 - (i - 90) / 360) * -3))
                pose = kinematics_calculate.transform_euler(org_pose, (0, 0, 0), 'xy', (x, y), degrees=False)
                self.step_controller.set_pose_base(pose, duration)
                time.sleep(duration)

    def reset(self):
        self.step_controller.set_build_in_pose('DEFAULT_POSE_M', 1)
        time.sleep(1)

def main(args=None):
    rclpy.init(args=args)
    node = BodyWave()
    
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
