#!/usr/bin/env python3
# coding=utf8
import json
import math
import rospy
from rospider_controller import client
from std_msgs.msg import String, Int32

words = ''
angle = None
def words_callback(msg):
    global words
    words = json.dumps(msg.data, ensure_ascii=False)[1:-1]
    print('words: ', words)

def angle_callback(msg):
    global angle
    angle = msg.data
    print('angle: ', angle)

if __name__ == "__main__":
    rospy.init_node('test', anonymous=True)
    rospy.Subscriber('/voice_words', String, words_callback)
    rospy.Subscriber('/mic/awake/angle', Int32, angle_callback)
    robot = client.Client(None)
    rospy.sleep(1)
    words = ""
    while not rospy.is_shutdown():
        try:
            if words == "":
                rospy.sleep(0.05);
                continue

            if words in "向左转":
                robot.cmd_vel(0, 0, 0.3)
                rospy.sleep(2)
                robot.traveling(gait=-2) 
            elif words in "向右转":
                robot.cmd_vel(0, 0, -0.3)
                rospy.sleep(2)
                robot.traveling(gait=-2) 
            elif words in "向左移动":
                robot.cmd_vel(0, 0.05, 0)
                rospy.sleep(2)
                robot.traveling(gait=-2) 
            elif words in "向右移动":
                robot.cmd_vel(0, 0.05, 0)
                rospy.sleep(2)
                robot.traveling(gait=-2) 
            elif words in "前进go":
                robot.cmd_vel(0.06, 0, 0)
                rospy.sleep(2)
                robot.traveling(gait=-2) 
            elif words in "后退":
                robot.cmd_vel(-0.06, 0, 0)
                rospy.sleep(2)
                self.robot.traveling(gait=-2) 
            else:
                pass
            words = ''
        except Exception as e:
            rospy.logerr(str(e))
