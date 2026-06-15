#!/usr/bin/env python3
# encoding: utf-8
# 标签定位

import cv2
import rclpy
import threading
import numpy as np
from rclpy.node import Node
from apriltag import apriltag
from cv_bridge import CvBridge
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
from controller import step_controller
from interfaces.msg import ApriltagsInfo
from servo_controller_msgs.msg import ServosPosition
from rclpy.callback_groups import ReentrantCallbackGroup
from servo_controller.bus_servo_control import set_servo_position

class TagNode(Node):
    def __init__(self):
        super().__init__('apriltag_position')
        self.id = ''
        self.center = None
        self.bridge = CvBridge()
        self.controller = step_controller.StepController()
        timer_cb_group = ReentrantCallbackGroup()
        self.joints_pub = self.create_publisher(ServosPosition, 'servo_controller', 1)

        self.image_sub = self.create_subscription(Image, '/apriltag_detect/image_result' , self.image_callback, 1)  # 画面订阅
        self.apriltag_info_sub = self.create_subscription(ApriltagsInfo, '/apriltag_detect/apriltag_info',  self.apriltag_info_callback, 1) # 标签信息订阅

        self.timer = self.create_timer(0.0, self.init_process, callback_group=timer_cb_group)
    
    def init_process(self):
        self.timer.cancel()

        self.controller.set_build_in_pose('DEFAULT_POSE', 1)
        joint_angle = [500, 750, 200, 150, 500, 700]
        set_servo_position(self.joints_pub, 1, ((19, joint_angle[0]), (20, joint_angle[1]), (21, joint_angle[2]), (22, joint_angle[3]), (23, joint_angle[4]), (24, joint_angle[5])))


    def image_callback(self, msg):
        rgb_image = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
        result_image = np.copy(rgb_image)

        if self.id :
            self.get_logger().info(f"ID: {str(self.id)}, X: {self.center[0]:.2f}, Y: {self.center[1]:.2f}, W: {self.width:.2f}")

        cv2.imshow('image', cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR))
        cv2.waitKey(1)

    def apriltag_info_callback(self, msg):
        data = msg.data
        if data != []:
            self.center = (data[0].x, data[0].y)
            self.width = data[0].w
            self.id = data[0].id
        else:
            self.id = ''


def main(args=None):
    rclpy.init(args=args)
    node = TagNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
