#!/usr/bin/env python3

import sys
import argparse
import rospy
import math
from rospider_controller import client


class MovingNode:
    def __init__(self):
        rospy.init_node("moving_node", anonymous=True, log_level=rospy.INFO)
        self.rospider = client.Client(self)
    
    def start(self):
        a = [45, 360 - 45]
        while True:
            for i in a:
                self.rospider.traveling(
                  gait=1, # RIPPER步态
                  stride=40.0, # 步幅 40mm
                  height=20.0, # 步高 20mm
                  direction=math.radians(i), 
                  rotation=0.0,
                  time=0.8, # 每步的用时
                  steps=0, # 走多少步, 0步就是一直走，直到被新的指令改变
                  interrupt=True,
                  relative_height=False)
                rospy.sleep(4)
    
    def stop(self):
        rospy.loginfo("stop")
        self.rospider.traveling(gait=-2)
    


if __name__ == "__main__":
    argv = rospy.myargv(argv=sys.argv) # ros 会在启动 py 文件时传入自己的一些参数，myargv 会去掉这些，返回原始参数
    parser = argparse.ArgumentParser() # 参数解析器
    # parser.add_argument('direction', metavar='direction', nargs='?', type=float, help="指定平移方向", default=0) # 添加要解析的参数
    # argv = parser.parse_args(argv[1:]) # 对输入参数进行解析

    node = MovingNode() # 建立相关资源
    rospy.sleep(3) # 稍等一下下, 订阅或者注册发布之后可能不会马上生效要等一下下
    rospy.loginfo("start")

    rospy.on_shutdown(node.stop) # 注册退出时的回调， 退出时停止机器人
    node.start() # 向指定方向行走运动


