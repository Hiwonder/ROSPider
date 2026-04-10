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

def start():
    t = 0.5
    rospider.set_pose_euler((0, 0, 160), (0, 0, 0))
    rospy.sleep(2)
    # 左右
    #rospider.pose_transform_euler((0, 60, 0), (0, 0, 0), t)
    #rospy.sleep(0.75)
    #rospider.pose_transform_euler((0, -120, 0), (0, 0, 0), 2*t)
    #rospy.sleep(1.25)
    #rospider.pose_transform_euler((0, 120, 0), (0, 0, 0), 2*t)
    #rospy.sleep(1.25)
    #rospider.pose_transform_euler((0, -60, 0), (0, 0, 0), t)
    #rospy.sleep(0.75)

    '''
    # 左右斜
    rospider.pose_transform_euler((0, 0, 0), (math.radians(20), 0, 0), t)
    rospy.sleep(1.05)
    rospider.pose_transform_euler((0, 0, 0), (math.radians(-40), 0, 0), 2*t)
    rospy.sleep(2.05)
    rospider.pose_transform_euler((0, 0, 0), (math.radians(20), 0, 0), t)
    rospy.sleep(1.05)
    # 左右扭
    rospider.pose_transform_euler((0, 0, 0), (0, 0, math.radians(20)), t)
    rospy.sleep(1.05)
    rospider.pose_transform_euler((0, 0, 0), (0, 0, math.radians(-40)), 2*t)
    rospy.sleep(2.05)
    rospider.pose_transform_euler((0, 0, 0), (0, 0, math.radians(20)), t)
    rospy.sleep(1.05)
    rospider.set_pose_euler((0, 0, 160), (0, 0, 0))
    rospy.sleep(2)
    '''
if __name__ == "__main__":
    rospy.init_node('test', anonymous=True)
    rospy.Subscriber('/voice_words', String, words_callback)
    rospy.Subscriber('/mic/awake/angle', Int32, angle_callback)
    rospider = client.Client(None)
    rospy.sleep(1)
    SPEED = 0.5
    while not rospy.is_shutdown():
        try:
            if angle is not None:
                if angle > 180:
                    angle = 360 - angle
                    speed = SPEED
                else:
                    speed = -SPEED
                if angle > 20:
                    rospider.cmd_vel(0, 0, speed)
                    rospy.sleep(abs(1.3*math.radians(angle)/SPEED))
                    rospider.traveling(gait=-2) 
                angle = None
            words = ''
        except Exception as e:
            rospy.logerr(str(e))
