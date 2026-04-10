#!/usr/bin/env python3

import rospy
from rospider_controller import rospider

class Control:
    def __init__(self):
        rospy.init_node("moving_node", anonymous=True, log_level=rospy.INFO)
        self.rospider = rospider.rospider(self)
        self.rospider.set_build_in_pose('DEFAULT_POSE', 1)
        rospy.sleep(1)
        self.rospider.set_step_mode(1, 40, 15, 0, 0, 0.8, repeat=0) # 机器人直行
    
    def transform(self):
        for _ in range(10):
            for i in range(20):
                self.rospider.transform_pose_2((0, 0, 4), 'xyz', (0,0,0), 0.1) # 升高机体
                rospy.sleep(0.1)
            for i in range(20):
                self.rospider.transform_pose_2((0, 0, -4), 'xyz', (0,0,0), 0.1) # 降低机体
                rospy.sleep(0.1)

    def reset(self):
        self.rospider.set_build_in_pose('DEFAULT_POSE', 1)
        rospy.sleep(1)
    
if __name__ == "__main__":
    node = Control() # 建立相关资源
    rospy.loginfo("start")
    rospy.on_shutdown(node.reset) # 退出的时候重置
    rospy.sleep(3)
    try:
        node.transform()
    except Exception as e:
        rospy.logerr(str(e))


