#!/usr/bin/env python3
# encoding: utf-8
# 颜色跟踪(color tracking)

import os
import cv2
import time
import math
import queue
import rclpy
import threading
import numpy as np
import sdk.pid as pid
import sdk.common as common
from rclpy.node import Node
from app.common import Heart
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from app.common import ColorPicker
from geometry_msgs.msg import Twist
from interfaces.msg import CmdParam
from arm_kinematics import transform
from controller import controller_client
from std_srvs.srv import SetBool, Trigger
from arm_kinematics_msgs.srv import SetRobotPose
from interfaces.srv import SetPoint, SetFloat64, SetString
from servo_controller_msgs.msg import ServosPosition
from arm_kinematics.kinematics_control import set_pose_target
from servo_controller.bus_servo_control import set_servo_position


display_size = [int(640*6/4), int(400*6/4)]
class ObjectTracker:
    def __init__(self, color, node):
        self.node = node
        self.pid_yaw = pid.PID(0.009, 0.0, 0.001)
        self.pid_dist = pid.PID(0.002, 0.0, 0.0)
        self.y_dis = 500
        self.x_init = transform.link3 + transform.tool_link
        self.last_color_circle = None
        self.lost_target_count = 0
        self.target_lab, self.target_rgb = color
        self.weight_sum = 1.0
        self.x_stop = 320
        self.y_stop = 300
        self.pro_size = (320, 200)

    def __call__(self, image, result_image, threshold, color=None, use_color_picker=True):
        twist = Twist()
        h, w = image.shape[:2]
        image = cv2.resize(image, self.pro_size)
        image = cv2.cvtColor(image, cv2.COLOR_RGB2LAB)  # RGB转LAB空间(convert RGB to LAB space)
        image = cv2.GaussianBlur(image, (5, 5), 5)

        if use_color_picker: 
            min_color = [int(self.target_lab[0] - 50 * threshold * 2),
                         int(self.target_lab[1] - 50 * threshold),
                         int(self.target_lab[2] - 50 * threshold)]
            max_color = [int(self.target_lab[0] + 50 * threshold * 2),
                         int(self.target_lab[1] + 50 * threshold),
                         int(self.target_lab[2] + 50 * threshold)]
            target_color = self.target_lab, min_color, max_color
            lowerb = tuple(target_color[1])
            upperb = tuple(target_color[2])
        else:
            lowerb = tuple(color['min'])
            upperb = tuple(color['max'])
        mask = cv2.inRange(image, lowerb, upperb) # 二值化(binarization)

        eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 腐蚀(erode)
        dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 膨胀(dilate)
        contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # 找出轮廓(find contours)
        contour_area = map(lambda c: (c, math.fabs(cv2.contourArea(c))), contours)  # 计算各个轮廓的面积(calculate the area of each contour)
        contour_area = list(filter(lambda c: c[1] > 40, contour_area))  # 剔除>面积过小的轮廓(remove contours with area that is too small)
        circle = None
        if len(contour_area) > 0:
            if self.last_color_circle is None:
                contour, area = max(contour_area, key=lambda c_a: c_a[1])
                circle = cv2.minEnclosingCircle(contour)
            else:
                (last_x, last_y), last_r = self.last_color_circle
                circles = map(lambda c: cv2.minEnclosingCircle(c[0]), contour_area)
                circle_dist = list(map(lambda c: (c, math.sqrt(((c[0][0] - last_x) ** 2) + ((c[0][1] - last_y) ** 2))),
                                       circles))
                circle, dist = min(circle_dist, key=lambda c: c[1])
                if dist < 100:
                    circle = circle
        if circle is not None:
            self.lost_target_count = 0
            (x, y), r = circle
            x = x / self.pro_size[0] * w
            y = y / self.pro_size[1] * h
            r = r / self.pro_size[0] * w

            cv2.circle(result_image, (self.x_stop, self.y_stop), 15, (255, 255, 0), -1)
            result_image = cv2.circle(result_image, (int(x), int(y)), int(r), (self.target_rgb[0],
                                                                               self.target_rgb[1],
                                                                               self.target_rgb[2]), 2)
            vx = 0
            vw = 0
            if abs(y - self.y_stop) > 60:
                self.pid_dist.update(y - self.y_stop)
                twist.linear.x = common.set_range(self.pid_dist.output, -0.05, 0.05)
            elif abs(y - self.y_stop) > 30:
                self.pid_dist.update(y - self.y_stop)
                twist.linear.x = common.set_range(self.pid_dist.output, -0.01, 0.01)
            else:
                self.pid_dist.clear()

            if abs(x - self.x_stop) > 40:
                self.pid_yaw.update(x - self.x_stop)
                twist.angular.z = common.set_range(self.pid_yaw.output, -0.1, 0.1)
            else:
                self.pid_yaw.clear()
                
        return result_image, twist

class OjbectTrackingNode(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.name = name
        self.color = ''
        self.set_above = False
        self.set_callback = False
        self.color_picker = None
        self.tracker = None
        self.is_running = False
        self.threshold = 0.1
        self.dist_threshold = 0.3
        self.lock = threading.RLock()
        self.image_sub = None
        self.result_image = None
        self.image_height = None
        self.image_width = None
        self.bridge = CvBridge()
        self.use_color_picker = True
        self.camera_type = 'Stereo'
        self.x_init = transform.link3 + transform.tool_link

        self.lab_data = common.get_yaml_data("/home/ubuntu/software/lab_tool/lab_config.yaml")
        self.image_queue = queue.Queue(2)
        self.controller = controller_client.ControllerClient()
        self.cmd_vel_pub = self.create_publisher(Twist, '/controller/cmd_vel', 1)
        self.result_publisher = self.create_publisher(Image, '~/image_result',  1)
        self.enter_srv = self.create_service(Trigger, '~/enter', self.enter_srv_callback)
        self.exit_srv = self.create_service(Trigger, '~/exit', self.exit_srv_callback)
        self.set_running_srv = self.create_service(SetBool, '~/set_running', self.set_running_srv_callback)
        self.set_color_srv = self.create_service(SetString, '~/set_color', self.set_color_srv_callback)
        self.set_target_color_srv = self.create_service(SetPoint, '~/set_target_color', self.set_target_color_srv_callback)
        self.get_target_color_srv = self.create_service(Trigger, '~/get_target_color', self.get_target_color_srv_callback)
        self.set_threshold_srv = self.create_service(SetFloat64, '~/set_threshold', self.set_threshold_srv_callback)
        self.joints_pub = self.create_publisher(ServosPosition, 'servo_controller', 1)
        self.cmd_param_pub = self.create_publisher(CmdParam, '/step_controller/cmd_param', 1) # 行走姿态控制  

        Heart(self, self.name + '/heartbeat', 5, lambda _: self.exit_srv_callback(request=Trigger.Request(), response=Trigger.Response()))  # 心跳包(heartbeat package)
        self.debug = self.get_parameter('debug').value

        if self.debug: 
            threading.Thread(target=self.main, daemon=True).start()
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

    def get_node_state(self, request, response):
        response.success = True
        return response

    def main(self):
        while True:
            try:
                image = self.image_queue.get(block=True, timeout=1)
            except queue.Empty:
                continue

            result = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
 
            cv2.imshow("image", cv2.resize(result, (display_size[0], display_size[1])))
            if self.debug and not self.set_callback:
                self.set_callback = True
                # 设置鼠标点击事件的回调函数(set callback function for mouse clicking event)
                cv2.setMouseCallback("image", self.mouse_callback)
            k = cv2.waitKey(1)
            if k != -1:
                break
            if self.debug and not self.set_above:
                cv2.moveWindow('image', 1920 - display_size[0], 0)
                os.system("wmctrl -r image -b add,above")
                self.set_above = True
        rclpy.shutdown()

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.get_logger().info("x:{} y{}".format(x, y))
            msg = SetPoint.Request()
            if self.image_height is not None and self.image_width is not None:
                msg.data.x = x / display_size[0]
                msg.data.y = y / display_size[1]
                self.set_target_color_srv_callback(msg, SetPoint.Response())

    def enter_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % 'object tracking enter')
        with self.lock:
            self.is_running = False
            self.threshold = 0.5
            self.tracker = None
            self.color_picker = None
            self.dist_threshold = 0.3
            self.color = ''
            if self.image_sub is None:
                self.image_sub = self.create_subscription(Image, '/depth_cam/rgb/image_raw', self.image_callback, 1)  # 摄像头订阅(subscribe to the camera)

            set_servo_position(self.joints_pub, 1, ((24, 500), (23, 500), (22, 150), (21, 130), (20, 720), (19, 500)))

            cmd_param = CmdParam()
            cmd_param.pose = 'DEFAULT_POSE'
            cmd_param.gait = 1
            cmd_param.height = 10
            cmd_param.period = 1.0
            self.cmd_param_pub.publish(cmd_param)
        response.success = True
        response.message = "enter"
        return response

    def exit_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % 'object tracking exit')
        try:
            if self.image_sub is not None:
                self.destroy_subscription(self.image_sub)
                self.image_sub = None
        except Exception as e:
            self.get_logger().error(str(e))
        with self.lock:
            self.is_running = False
            self.color_picker = None
            self.tracker = None
            self.threshold = 0.5
            self.dist_threshold = 0.3
            twist = Twist()
            self.cmd_vel_pub.publish(twist)
            self.controller.traveling(gait=-2, time=1, steps=0)
        response.success = True
        response.message = "exit"
        return response

    def set_target_color_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % 'set_target_color')
        with self.lock:
            self.use_color_picker = True
            x, y = request.data.x, request.data.y
            if x == -1 and y == -1:
                self.color_picker = None
                self.tracker = None
            else:
                self.tracker = None
                self.color_picker = ColorPicker(request.data, 10)
        response.success = True
        response.message = "set_target_color"
        return response

    def get_target_color_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % 'get_target_color')
        response.success = False
        response.message = "get_target_color"
        with self.lock:
            if self.tracker is not None:
                response.success = True
                rgb = self.tracker.target_rgb
                response.message = "{},{},{}".format(int(rgb[0]), int(rgb[1]), int(rgb[2]))
        return response

    def set_running_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % 'set_running')
        with self.lock:
            self.is_running = request.data
            if not self.is_running:
                twist = Twist()
                self.cmd_vel_pub.publish(twist)
                self.controller.traveling(gait=-2, time=1, steps=0)
        response.success = True
        response.message = "set_running"
        return response

    def set_threshold_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % 'threshold')
        with self.lock:
            self.threshold = request.data
            response.success = True
            response.message = "set_threshold"
            return response

    def set_color_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % 'set_color')
        with self.lock:
            self.color = request.data
            self.use_color_picker = False
        response.success = True
        response.message = "set_color"
        return response
    
    def send_request(self, client, msg):
        future = client.call_async(msg)
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()
            
    def image_callback(self, ros_image):
        # 将ros格式(rgb)转为opencv的rgb格式(convert RGB format of ROS to that of OpenCV)
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, "rgb8")
        rgb_image = np.array(cv_image, dtype=np.uint8)
        self.image_height, self.image_width = rgb_image.shape[:2]

        result_image = np.copy(rgb_image)  # 显示结果用的画面(the image used for display the result)
        with self.lock:
            if self.use_color_picker:
                # 颜色拾取器和识别追踪互斥, 如果拾取器存在就开始拾取(color picker and object tracking are mutually exclusive. If the color picker exists, start picking colors)
                if self.color_picker is not None:  # 拾取器存在(color pick exists)
                    target_color, result_image = self.color_picker(rgb_image, result_image)
                    if target_color is not None:
                        self.color_picker = None
                        self.tracker = ObjectTracker(target_color, self)
                        # self.get_logger().info("target color: {}".format(target_color))
                else:
                    if self.tracker is not None:
                        try:
                            result_image, twist = self.tracker(rgb_image, result_image, self.threshold)
                            if self.is_running:
                                self.cmd_vel_pub.publish(twist)
                            else:
                                self.tracker.pid_dist.clear()
                                self.tracker.pid_yaw.clear()
                        except Exception as e:
                            self.get_logger().error(str(e))
            else:
                if self.color in common.range_rgb:
                    self.tracker = ObjectTracker([None, common.range_rgb[self.color]], self)
                    result_image, twist = self.tracker(rgb_image, result_image, self.threshold, self.lab_data['lab'][self.camera_type][self.color], False)
                    if self.is_running:
                        self.cmd_vel_pub.publish(twist)
                    else:
                        self.tracker.pid_dist.clear()
                        self.tracker.pid_yaw.clear()
        if self.debug:
            if self.image_queue.full():
                # 如果队列已满，丢弃最旧的图像(if the queue is full, discard the oldest image)
                self.image_queue.get()
                # 将图像放入队列(put the image into the queue)
            self.image_queue.put(result_image)
        else:
            # 将opencv的格式(bgr)转为ros的rgb格式(convert BGR format of OpenCV to RGB format of ROS)
            self.result_publisher.publish(self.bridge.cv2_to_imgmsg(result_image, "rgb8"))

def main():
    node = OjbectTrackingNode('object_tracking')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

