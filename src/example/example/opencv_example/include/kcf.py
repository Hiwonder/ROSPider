#!/usr/bin/env python3
# encoding: utf-8
#KCF追踪

import cv2
import rclpy
import queue
import signal
import threading
import numpy as np
import sdk.pid as pid
import sdk.common as common
from rclpy.node import Node
from cv_bridge import CvBridge
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image 
from geometry_msgs.msg import Twist
from controller import controller_client
from rclpy.executors import MultiThreadedExecutor
from servo_controller_msgs.msg import ServosPosition
from rclpy.callback_groups import ReentrantCallbackGroup
from servo_controller.bus_servo_control import set_servo_position

class KcfTrackNode(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)

        self.bridge = CvBridge()  # 用于ROS Image消息与OpenCV图像之间的转换
        self.center = None
        self.tracker = None
        self.enable_select = False
        self.running = True
        self.name = name
        self.image_queue = queue.Queue(maxsize=2)
        self.pid_x = pid.PID(0.05, 0.0, 0.0)
        self.controller = controller_client.ControllerClient()
        signal.signal(signal.SIGINT, self.shutdown)

        self.joints_pub = self.create_publisher(ServosPosition, '/servo_controller', 1) # 舵机控制
        self.cmd_vel_pub = self.create_publisher(Twist, '/controller/cmd_vel', 1)  # 底盘控制(chassis control)

        self.image_sub = self.create_subscription(Image, '/depth_cam/rgb/image_raw', self.image_callback, 1)  # 摄像头订阅(subscribe to the camera)

        timer_cb_group = ReentrantCallbackGroup()
        self.client = self.create_client(Trigger, '/controller_manager/init_finish')
        self.client.wait_for_service()

        self.timer = self.create_timer(0.0, self.init_process, callback_group=timer_cb_group)

    def init_process(self):
        self.timer.cancel()

        self.controller.traveling(gait=-2, time=1, steps=0)
        joint_angle = [500, 750, 200, 150, 500, 700]
        set_servo_position(self.joints_pub, 1, ((19, joint_angle[0]), (20, joint_angle[1]), (21, joint_angle[2]), (22, joint_angle[3]), (23, joint_angle[4]), (24, joint_angle[5])))

        threading.Thread(target=self.main, daemon=True).start()
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'Press “S" in the picture window to start tracking the target, then press the "space" to start tracking')



    def get_node_state(self, request, response):
        response.success = True
        return response

    def shutdown(self, signum, frame):
        self.running = False
        self.controller.traveling(gait=-2, time=1, steps=0)

    def image_callback(self, ros_image):
        # 将画面转为 opencv 格式(convert the screen to opencv format)
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        bgr_image = np.array(cv_image, dtype=np.uint8)

        if self.image_queue.full():
            # 如果队列已满，丢弃最旧的图像
            self.image_queue.get()
        # 将图像放入队列
        self.image_queue.put(bgr_image)


    def main(self):
        while self.running:
            rgb_image = self.image_queue.get()
            result_image = np.copy(rgb_image)
            factor = 1
            rgb_image = cv2.resize(rgb_image, (int(result_image.shape[1]/ factor), int(result_image.shape[0]/ factor)))

            if self.tracker is None:
                if self.enable_select:
                    roi = cv2.selectROI("result", result_image, False)
                    roi =  tuple(int(i / factor)for i in roi)

                    if roi[2] > 0 and roi[3] > 0:
                        param = cv2.TrackerKCF.Params()
                        param.detect_thresh = 0.2
                        self.tracker = cv2.TrackerKCF_create(param)
                        self.tracker.init(rgb_image, roi)

            else:
                twist = Twist()
                status, box = self.tracker.update(rgb_image)
                if status:
                    p1 = int(box[0] * factor), int(box[1] * factor)
                    p2 = p1[0] + int(box[2] * factor), p1[1] + int(box[3] * factor)
                    cv2.rectangle(result_image, p1, p2, (255, 255, 0), 2)
                    center_x, center_y = (p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2
           

                    self.pid_x.SetPoint = result_image.shape[1]/2 
                    self.pid_x.update(center_x)

                    self.pid_x.SetPoint = result_image.shape[1]/2 
                    self.pid_x.update(center_x)
                    if self.pid_x.output :
                        twist.angular.z = (common.set_range(self.pid_x.output, -10, 10)/ 10.0) * 0.3
                        self.cmd_vel_pub.publish(twist)
                    else:
                        self.cmd_vel_pub.publish(Twist())

                else:
                    self.cmd_vel_pub.publish(Twist())

            cv2.imshow("result", result_image)

            key = cv2.waitKey(1)
            if key == ord('s'): # 按下s开始选择追踪目标(press 's' to start selecting the tracking target)
                self.controller.traveling(gait=-2, time=1, steps=0)
                self.tracker = None
                self.enable_select = True

        rclpy.shutdown()

def main():
    node = KcfTrackNode('kcf_track')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
 
if __name__ == "__main__":
    main()
