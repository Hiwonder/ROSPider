#!/usr/bin/env python3

import argparse
import sys
import rospy
import math
from rospider_controller import client


class MovingNode:
    def __init__(self):
        rospy.init_node("moving_node", anonymous=True, log_level=rospy.INFO)
        self.rospider = client.Client(self)
    
    def start(self, d):
        if d == 'left':
            d = math.radians(90)
        elif d == 'right':
            d = math.radians(270)
        else:
            sys.exit(-1)
        self.rospider.traveling(
                  gait=1, # RIPPER步态
                  stride=40.0, # 步幅 40mm
                  height=25.0, # 步高 25mm
                  direction=d, # 逆时针为正方向 90度移动就是左移, 270 右移
                  rotation=0.0,
                  time=1.2, # 每步的用时
                  steps=0, # 走多少步, 0步就是一直走，直到被新的指令改变
                  interrupt=True,
                  relative_height=False)
    
    def stop(self):
        rospy.loginfo("stop")
        self.rospider.traveling(gait=0)
    


if __name__ == "__main__":
    argv = rospy.myargv(argv=sys.argv) # ros 会在启动 py 文件时传入自己的一些参数，myargv 会去掉这些，返回原始参数
    parser = argparse.ArgumentParser() # 参数解析器
    parser.add_argument('left_or_right', metavar='left or right', nargs='?', type=str, help="左移或右移", default="left") # 添加要解析的参数
    argv = parser.parse_args(argv[1:]) # 对输入参数进行解析

    node = MovingNode() # 建立相关资源
    rospy.sleep(5) # 稍等一下下, 订阅或者注册发布之后可能不会马上生效要等一下下
    rospy.loginfo("start")
    node.start(argv.left_or_right) # 直行
    rospy.on_shutdown(node.stop) # 注册退出时的回调， 退出时停止机器人
    rospy.spin() # 等待退出, spin() 会保持进行的活跃但是并不做什么事

