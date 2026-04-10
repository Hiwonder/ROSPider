#!/usr/bin/env python3

import sys
import argparse
import rospy
import math
from rospider_controller import rospider


class MovingNode:
    def __init__(self):
        rospy.init_node("moving_node", anonymous=True, log_level=rospy.INFO)
        self.rospider = rospider.rospider(self, pwm=False)
    
    def start(self):
        for i in range(3):
            joints = self.rospider.set_leg_position(2, (0, 150, -80), 2)
            print(joints)
            rospy.sleep(2.5)
            joints = self.rospider.set_leg_position(2, (0, 250, -80), 2)
            print(joints)
            rospy.sleep(2.5)

    
    


if __name__ == "__main__":
    argv = rospy.myargv(argv=sys.argv) # ros 会在启动 py 文件时传入自己的一些参数，myargv 会去掉这些，返回原始参数

    node = MovingNode() # 建立相关资源
    rospy.sleep(3)
    node.start() # 运行


