#!/usr/bin/env python3

import sys
import argparse
import rospy
import math
from rospider_controller import rospider

class Control:
    def __init__(self):
        rospy.init_node("moving_node", anonymous=True, log_level=rospy.INFO)
        self.rospider = rospider.rospider(self)
        self.rospider.set_build_in_pose('DEFAULT_POSE_M', 1)
        rospy.sleep(1.0)
    
    def transform(self, translation, axis, rotation, duration):
        x, y, z = translation
        u, v, w = rotation
        self.rospider.transform_pose_2((x, y, z), axis, (u,v,w), duration, degrees=True)

    def do(self):
        self.transform((0, 0, 0), 'yxz', (-10, 0, 0), 1) # 旋转
        rospy.sleep(1)
        self.transform((0, 0, 0), 'xyz', (0, 10, 0), 1) #欧拉角注意顺序
        rospy.sleep(1)
        self.transform((0, 0, 0), 'yxz', (-10, 10, 0), 2)  #欧拉角注意顺序
        rospy.sleep(2)
        self.transform((20, 0, 0), 'xyz', (0, 0, 0), 1) # 平移， 单位为 mm
        rospy.sleep(1)
        self.transform((0, 20, 0), 'xyz', (0, 0, 0), 1)
        rospy.sleep(1)
        self.transform((-20, -20, 0), 'xyz', (0, 0, 0), 1)
        rospy.sleep(1)
    
    def reset(self):
        self.rospider.set_build_in_pose('DEFAULT_POSE_M', 1)
    
if __name__ == "__main__":
    argv = rospy.myargv(argv=sys.argv) # ros 会在启动 py 文件时传入自己的一些参数，myargv 会去掉这些，返回原始参数

    node = Control() # 建立相关资源
    rospy.loginfo("start")
    rospy.on_shutdown(node.reset)
    rospy.sleep(3)
    node.do() 


