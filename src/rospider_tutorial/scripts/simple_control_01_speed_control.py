#!/usr/bin/env python3

import argparse
import sys
import rospy
from rospider_controller import client


class MovingNode:
    def __init__(self):
        rospy.init_node("moving_node", anonymous=True, log_level=rospy.INFO)
        self.rospider = client.Client(self)
    
    def start(self, period, stride):
        self.rospider.traveling(
                  gait=1, # RIPPER步态
                  stride=stride, # 步幅
                  height=15.0, # 步高 25mm
                  direction=0, # 180 方向移动就是后移
                  rotation=0.0,
                  time=period, # 每步的用时
                  steps=0, # 走多少步, 0步就是一直走，直到被新的指令改变
                  interrupt=True,
                  relative_height=False)
    
    def stop(self):
        rospy.loginfo("stop")
        self.rospider.traveling(gait=0)
    

if __name__ == "__main__":
    node = MovingNode() # 建立相关资源
    argv = rospy.myargv(argv=sys.argv)
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--period', "-p", type=float, help="每步间隔，单位: 秒", default=0.8)
    parser.add_argument('--stride', "-s", type=float, help="步幅", default=40)
    argv = parser.parse_args(argv[1:]) # 解析输入参数
    rospy.sleep(3) # 稍等一下下, 订阅或者注册发布之后可能不会马上生效要等一下下
    node.start(argv.period, argv.stride) # 直行

