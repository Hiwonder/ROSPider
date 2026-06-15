#!/usr/bin/env python3
# encoding: utf-8
# 斜向移动

import math
import time
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from controller.controller_client import ControllerClient

class Diagonally(Node):
    def __init__(self):
        super().__init__('diagonally')
        # 声明参数并设置默认值
        self.declare_parameter('direction', 45)
        
        # 获取参数值
        self.direction = self.get_parameter('direction').value

        self.controller = ControllerClient()
        self.client = self.create_client(Trigger, '/controller_manager/init_finish')
        self.client.wait_for_service()

    def move(
        self,
        gait=1,              # 步态类型，0为停止运动、1为波纹步态、2为三角步态
        stride=40.0,         # 步幅（mm），默认40mm，范围 0~65
        height=15.0,         # 步高（mm），默认15mm， 范围 0~50
        direction=0,         # 移动方向（角度），范围 0°~360°，0°为前进，逆时针方向递增
        rotation=0.0,        # 旋转角度，默认不旋转， 正数为逆时针旋转，负数为顺时针旋转z
        time=1,              # 单步用时（秒），默认1秒
        steps=0,             # 步数，0表示持续运动
        interrupt=True,      # 是否允许中断，默认允许
        relative_height=False # 高度是否为相对值，默认绝对高度
    ):
        """控制机器人移动的方法

        Args:
            gait (int): 步态类型 (例如1=波纹步态）
            stride (float): 步幅大小（毫米）
            height (float): 抬腿高度（毫米）
            direction (float): 移动方向(0=前,180=后）
            rotation (float): 旋转角度（正数右转，负数左转）
            time (float): 单步执行时间（秒）
            steps (int): 移动步数(0=持续）
            interrupt (bool): 是否允许被新指令中断
            relative_height (bool): 高度是否为相对值
        """
        self.controller.traveling(
            gait=gait,
            stride=stride,
            height=height,
            direction=direction,
            rotation=rotation,
            time=time,
            steps=steps,
            interrupt=interrupt,
            relative_height=relative_height
        )


def main(args=None):
    rclpy.init(args=args)
    node = Diagonally()

    node.move(gait=1, direction=math.radians(node.direction))   
    node.get_logger().info('\033[1;32m%s\033[0m' % f'Movement angle {node.direction} degrees')
    time.sleep(5)  # 等待 5 秒
    node.move(gait=0) # 停止 
    node.get_logger().info('\033[1;32m%s\033[0m' % 'stop')

    node.controller.destroy_node()  # 清理节点
    rclpy.shutdown()  # 关闭 ROS 2

if __name__ == '__main__':
    main()

 