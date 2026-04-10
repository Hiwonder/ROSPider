#!/usr/bin/env python3
# encoding: utf-8
import os
import math
import time
import rospy
import threading
import std_msgs.msg
from rospider_sdk import buzzer

class RestartBringupNode:
    def __init__(self, name):
        rospy.init_node(name, anonymous=True)
        rospy.Subscriber('/restart_bringup', std_msgs.msg.Empty, self.callback) 

    # 进入玩法
    def callback(self, _):
        buzzer.beep(0.4, 3, rospy.sleep)
        os.system("sudo systemctl restart rospider_bringup.service")


if __name__ == "__main__":
    node = RestartBringupNode('restart_bringup')
    try:
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))

