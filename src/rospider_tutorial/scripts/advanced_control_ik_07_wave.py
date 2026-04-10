#!/usr/bin/env python3

import sys
import argparse
import rospy
import math
from rospider_controller import build_in_pose, kinematics_api, rospider

class Control:
    def __init__(self):
        rospy.init_node("moving_node", anonymous=True, log_level=rospy.INFO)
        self.rospider = rospider.rospider(self)

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
        duration = 0.03
        self.rospider.set_pose_base(build_in_pose.DEFAULT_POSE_M, 0.8)
        org_pose = tuple(build_in_pose.DEFAULT_POSE_M)
        rospy.sleep(0.8)
        for i in range(10, 50, +3):
            points = self.gen_circle(i)
            for x, y in points:
                pose = kinematics_api.transform_euler(org_pose, (x, y, 0), 'xyz', (0, 0, 0), degrees=False)
                self.rospider.set_pose_base(pose, 0.02)
                rospy.sleep(0.02)

    
    def reset(self):
        # 恢复正常状态
        self.rospider.set_pose_base(build_in_pose.DEFAULT_POSE_M, 1)
        rospy.sleep(1)


if __name__ == "__main__":
    node = Control() # 建立相关资源
    rospy.loginfo("start")
    rospy.on_shutdown(node.reset)
    rospy.sleep(3)
    node.wave() 


