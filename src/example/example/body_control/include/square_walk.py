#!/usr/bin/env python3
# encoding: utf-8
# 矩形平移

import signal
import math
import time
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from controller.controller_client import ControllerClient

class SquareWalk(Node):
    def __init__(self):
        super().__init__('square_walk')
        self.controller = ControllerClient()
        self.client = self.create_client(Trigger, '/controller_manager/init_finish')
        self.client.wait_for_service()

    def move(self,
             gait=1,              # 步态类型，0为停止运动、1为波纹步态、2为三角步态
             stride=40.0,         # 步幅（mm），默认40mm，范围 0~65
             height=15.0,         # 步高（mm），默认15mm， 范围 0~50
             direction=0,         # 移动方向（角度），单位：弧度；0 为前进
             rotation=0.0,        # 旋转角度（正数为逆时针旋转，负数为顺时针旋转）
             step_time=1,         # 单步执行时间（秒），默认1秒
             steps=0,             # 步数，0表示持续运动
             interrupt=True,      # 是否允许中断，默认允许
             relative_height=False# 高度是否为相对值，默认绝对高度
             ):
        """
        控制机器人移动的方法
        """
        self.controller.traveling(
            gait=gait,
            stride=stride,
            height=height,
            direction=direction,
            rotation=rotation,
            time=step_time,
            steps=steps,
            interrupt=interrupt,
            relative_height=relative_height
        )

def main():
    # 初始化 ROS2 客户端库
    rclpy.init()
    node = SquareWalk()

    # 定义自定义信号处理函数，确保在接收到 SIGINT 时发送停止指令
    def signal_handler(sig, frame):
        node.get_logger().info('\033[1;32m%s\033[0m' % 'Received SIGINT, stopping...')
        node.move(gait=0)  # 发送停止指令
        # 清理节点和 shutdown ROS2
        node.destroy_node()
        rclpy.shutdown()
        exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    # 机器人按正方形运动的各个方向（单位：角度）
    direction_angles = [0, 90, 180, 270]

    try:
        # 按顺序执行每个方向的运动
        for angle in direction_angles:
            node.get_logger().info('\033[1;32m%s\033[0m' % f'Moving in direction: {angle}°')
            # 转换角度为弧度后发送运动指令
            node.move(gait=1, direction=math.radians(angle))
            time.sleep(4)  # 等待一段时间后换下一个方向

    except KeyboardInterrupt:
        node.get_logger().info('\033[1;32m%s\033[0m' % 'KeyboardInterrupt detected, stopping...')
        node.move(gait=0)

    finally:
        # 最后确保机器人停止运动
        node.move(gait=0)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
