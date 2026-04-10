#!/usr/bin/env python3

import rospy
from rospider_controller import client


class MovingNode:
    def __init__(self):
        rospy.init_node("moving_node", anonymous=True, log_level=rospy.INFO)
        self.rospider = client.Client(self)
    
    def start(self):
        self.rospider.traveling(
                  gait=1, # RIPPER步态
                  stride=40.0, # 步幅 60mm
                  height=15.0, # 步高 25mm
                  direction=0, # 180 方向移动就是后移
                  rotation=0.0,
                  time=1, # 每步的用时
                  steps=0, # 走多少步, 0步就是一直走，直到被新的指令改变
                  interrupt=True,
                  relative_height=False)
    
    def stop(self):
        rospy.loginfo("stop")
        self.rospider.traveling(gait=0)
    

if __name__ == "__main__":
    node = MovingNode() # 建立相关资源
    rospy.sleep(5) # 稍等一下下, 订阅或者注册发布之后可能不会马上生效要等一下下
    node.start() # 直行
    rospy.on_shutdown(node.stop) # 注册退出时的回调， 退出时停止机器人
    rospy.spin() # 等待退出, spin() 会保持进行的活跃但是并不做什么事

