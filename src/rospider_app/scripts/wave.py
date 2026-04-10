#!/usr/bin/env python3

import time
import math
import rospy
from rospider_controller.rospider import rospider
from rospider_controller import kinematics_api
from rospider_controller import build_in_pose
from scipy.spatial.transform import Rotation as R
import kinematics


class WaveMove:
    def __init__(self, name):
        rospy.init_node(name)
        self.pitch = 0
        self.roll = -0.2
        self.pitch = -0.2
        self.robot = rospider(None, True)
        self.robot.set_pose(build_in_pose.DEFAULT_POSE_M, 1)
        rospy.sleep(1)
        self.pose = list(self.robot.pose)
    
    def wave(self):
        duration = 0.03
        rospy.sleep(0.5)
        for j in range(7, 20, 2):
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
                pose = kinematics_api.transform_euler(self.pose, (0, 0, 0), 'xy', (x, y), degrees=False)
                self.robot.set_pose_base(pose, t)
                rospy.sleep(t)

        for j in range(15, 4, -3):
            i = 360 + 85
            while i >= 90:
                i += -(4 + j * 0.30)
                k = 360 + 90 - i + 90
                x = math.sin(math.radians(k)) * (0.018 * (j + (1 - (i - 90) / 360) * -3))
                y = math.cos(math.radians(k)) * (0.018 * (j + (1 - (i - 90) / 360) * -3))
                pose = kinematics_api.transform_euler(self.pose, (0, 0, 0), 'xy', (x, y), degrees=False)
                self.robot.set_pose_base(pose, duration)
                rospy.sleep(duration)
        self.robot.set_pose(build_in_pose.DEFAULT_POSE_M, 1)

def main():
    wave_move_node = WaveMove('wave_move')
    try:
        wave_move_node.wave()
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))


if __name__ == "__main__":
    main()
