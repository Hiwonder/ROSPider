#!/usr/bin/env python3
# encoding: utf-8
# @data:2022/11/18
# @author:aiden
# 追踪拾取(tracking and picking)
import os
import ast
import cv2
import time
import math
import queue
import rclpy
import threading
import numpy as np
from sdk import common
from sdk.pid import PID
from rclpy.node import Node
from cv_bridge import CvBridge
from std_msgs.msg import Bool
from app.common import ColorPicker
from sensor_msgs.msg import Image
from interfaces.msg import Pose2D
from std_srvs.srv import Trigger, Empty
from controller import controller_client 
from geometry_msgs.msg import Twist, Point
from xf_mic_asr_offline import voice_play
from servo_controller_msgs.msg import ServosPosition
from interfaces.srv import SetPose2D, SetPoint, SetBox
from servo_controller.bus_servo_control import set_servo_position
from servo_controller.action_group_controller import ActionGroupController

class AutomaticTransportNode(Node):
    config_path = '/home/ubuntu/ros2_ws/src/large_models_examples/config/automatic_transport_roi.yaml'

    def __init__(self, name):
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.name = name
        # 颜色识别(color recognition)
        self.image_proc_size = (320, 240)
        
        self.color_picker_point = []
        self.threshold = 0.5
        self.color_picker = None
        self.min_color = []
        self.max_color = []
        self.box = []
        
        self.mouse_click = False
        self.selection = None  # 实时跟踪鼠标的跟踪区域
        self.track_window = None  # 要检测的物体所在区域
        self.drag_start = None  # 标记，是否开始拖动鼠标
        self.start_circle = True
        self.start_click = False
        self.pick_finish = False
        self.place_finish = False

        self.running = True
        self.detect_count = 0
        self.start_pick = False
        self.start_place = False
        self.linear_base_speed = 0.007
        self.angular_base_speed = 0.03

        self.yaw_pid = PID(P=0.025, I=0, D=0.000)
        self.linear_pid = PID(P=0.003, I=0, D=0)
        self.angular_pid = PID(P=0.003, I=0, D=0)

        self.linear_speed = 0
        self.angular_speed = 0
        self.yaw_angle = 90

        self.pick_stop_x = 320
        self.pick_stop_y = 388
        self.place_stop_x = 320
        self.place_stop_y = 388
        self.stop = True

        self.d_y = 10
        self.d_x = 10

        self.pick = False
        self.place = False

        self.status = "approach"
        self.count_stop = 0
        self.count_turn = 0

        self.declare_parameter('status', 'start')
        self.bridge = CvBridge()
        self.image_queue = queue.Queue(maxsize=2)
        self.display_box = True
        self.start_time = time.time()
        self.start = self.get_parameter('start').value
        self.enable_display = self.get_parameter('enable_display').value
        self.debug = self.get_parameter('debug').value
        self.image_name = 'image'

        self.language = os.environ['ASR_LANGUAGE']

        self.joints_pub = self.create_publisher(ServosPosition, '/servo_controller', 1)
        self.mecanum_pub = self.create_publisher(Twist, '/controller/cmd_vel', 1)
        self.image_pub = self.create_publisher(Image, '~/image_result', 1)

        self.create_subscription(Image, 'depth_cam/rgb/image_raw', self.image_callback, 1)
        
        self.create_service(Trigger, '~/pick', self.start_pick_callback)
        self.create_service(Trigger, '~/place', self.start_place_callback) 
        self.create_service(SetPoint, '~/set_target_color', self.set_target_color_srv_callback)
        self.create_service(SetBox, '~/set_box', self.set_box_srv_callback)

        self.agc = ActionGroupController(self.create_publisher(ServosPosition, 'servo_controller', 1), '/home/ubuntu/software/arm_pc/ActionGroups')
        self.get_logger().info("Action Group Controller has been started")
        self.client = self.create_client(Trigger, '/controller_manager/init_finish')
        self.client.wait_for_service()
        self.controller = controller_client.ControllerClient()

        
        self.action_finish_pub = self.create_publisher(Bool, '~/action_finish', 1)
        
        set_servo_position(self.joints_pub, 2, ((19, 500), (20, 700), (21, 155), (22, 70), (23, 500), (24, 700)))
        time.sleep(2)

        self.get_logger().info("Automatic Pick Node has been started")
        if self.debug == 'pick':
            set_servo_position(self.joints_pub, 1.5, ((19, 500), (20, 150), (21, 350), (22, 280), (23, 500), (24, 700)))
            time.sleep(5)
            set_servo_position(self.joints_pub, 2, ((19, 500), (20, 700), (21, 155), (22, 70), (23, 500), (24, 700)))

            time.sleep(2)
            msg = Trigger.Request()
            self.start_pick_callback(msg, Trigger.Response())
        elif self.debug == 'place':
            set_servo_position(self.joints_pub, 1.5, ((19, 500), (20, 110), (21, 430), (22, 390), (23, 500), (24, 460)))
            time.sleep(5)
            set_servo_position(self.joints_pub, 2, ((19, 500), (20, 700), (21, 155), (22, 70), (23, 500), (24, 700)))
            time.sleep(2)
            msg = Trigger.Request()
            self.start_place_callback(msg, Trigger.Response())

        threading.Thread(target=self.action_thread, daemon=True).start()
        threading.Thread(target=self.main, daemon=True).start()
        self.create_service(Empty, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

    def get_node_state(self, request, response):
        return response

    def set_box_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % 'set_box')
        self.box = [request.x_min, request.y_min, request.x_max, request.y_max]
        self.display_box = True
        self.start_time = time.time()
        self.controller.traveling(gait=-2, time=1, steps=0)
        response.success = True
        response.message = "set_box"
        return response

    def set_target_color_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % 'set_target_color')
        x, y = request.data.x, request.data.y
        if x == -1 and y == -1:
            self.min_color, self.max_color = [], []
            self.color_picker = None
        else:
            self.min_color, self.max_color = [], []
            self.color_picker = ColorPicker(request.data, 10)
        self.controller.traveling(gait=-2, time=1, steps=0)
        response.success = True
        response.message = "set_target_color"
        return response

    def start_pick_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "start pick")
        
        self.place_finish = False
        self.linear_speed = 0
        self.angular_speed = 0
        self.yaw_angle = 90

        param = self.get_parameter('pick_stop_pixel_coordinate').value
        self.get_logger().info('\033[1;32mget pick stop pixel coordinate: %s\033[0m' % str(param))
        self.pick_stop_x = param[0]
        self.pick_stop_y = param[1]
        self.stop = True

        self.d_y = 20
        self.d_x = 20

        self.pick = False
        self.place = False

        self.status = "approach"
        self.count_stop = 0
        self.count_turn = 0

        self.linear_pid.clear()
        self.angular_pid.clear()
        self.start_pick = True
        try:
            lab_data = common.get_yaml_data("/home/ubuntu/software/lab_tool/lab_config.yaml")
            self.min_color = lab_data['lab']['Stereo']['basket']['min']
            self.max_color = lab_data['lab']['Stereo']['basket']['max']
        except:
            self.min_color = [87, 108, 142]
            self.max_color = [255, 142, 199]

        response.success = True
        response.message = "start_pick"
        return response 


    def start_place_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "start place")
        set_servo_position(self.joints_pub, 0.8, ((19, 500), (20, 620), (21, 109), (22, 432), (23, 125), (24, 650)))
        time.sleep(1.5)
        set_servo_position(self.joints_pub, 1, ((19, 500), (20, 515), (21, 20), (22, 450), (23, 125), (24, 650)))
        time.sleep(1.5)

        self.pick_finish = False
        self.linear_speed = 0
        self.angular_speed = 0
        self.d_y = 30
        self.d_x = 30
        
        param = self.get_parameter('place_stop_pixel_coordinate').value
        self.get_logger().info('\033[1;32mget place stop pixel coordinate: %s\033[0m' % str(param))
        self.place_stop_x = param[0]
        self.place_stop_y = param[1]
        self.stop = True
        self.pick = False
        self.place = False

        self.linear_pid.clear()
        self.angular_pid.clear()
        self.start_place = True

        try:
            lab_data = common.get_yaml_data("/home/ubuntu/software/lab_tool/lab_config.yaml")
            self.min_color = lab_data['lab']['Stereo']['desk']['min']
            self.max_color = lab_data['lab']['Stereo']['desk']['max']
        except:
            self.min_color = [166, 127, 139]
            self.max_color = [255, 255, 255]

        response.success = True
        response.message = "start_place"
        return response 

    def send_request(self, client, msg):
        future = client.call_async(msg)
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()


    def color_detect(self, img):
        img_h, img_w = img.shape[:2]
        frame_resize = cv2.resize(img, self.image_proc_size, interpolation=cv2.INTER_NEAREST)
        frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)
        frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  # 将图像转换到LAB空间(convert image to LAB space)
        frame_mask = cv2.inRange(frame_lab, tuple(self.min_color), tuple(self.max_color))  # 对原图像和掩模进行位运算(perform bitwise operation on the original image and the mask)

        eroded = cv2.erode(frame_mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 腐蚀(erode)
        dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 膨胀(dilate)

        contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # 找出轮廓(find contours)
        center_x, center_y, angle = -1, -1, -1
        if len(contours) != 0:
            areaMaxContour, area_max = common.get_area_max_contour(contours, 10)  # 找出最大轮廓(find the largest contour)
            if areaMaxContour is not None:
                if 10 < area_max:  # 有找到最大面积(the maximum area has been found)
                    rect = cv2.minAreaRect(areaMaxContour)  # 最小外接矩形(the minimum bounding rectangle)
                    angle = rect[2]
                    box = np.intp(cv2.boxPoints(rect))  # 最小外接矩形的四个顶点(the four corner points of the minimum bounding rectangle)
                    for j in range(4):
                        box[j, 0] = int(common.val_map(box[j, 0], 0, self.image_proc_size[0], 0, img_w))
                        box[j, 1] = int(common.val_map(box[j, 1], 0, self.image_proc_size[1], 0, img_h))

                    cv2.drawContours(img, [box], -1, (0, 255, 255), 2)  # 画出四个点组成的矩形(draw the rectangle composed of the four points)
                    # 获取矩形的对角点(obtain the diagonal points of the rectangle)
                    ptime_start_x, ptime_start_y = box[0, 0], box[0, 1]
                    pt3_x, pt3_y = box[2, 0], box[2, 1]
                    radius = abs(ptime_start_x - pt3_x)
                    center_x, center_y = int((ptime_start_x + pt3_x) / 2), int((ptime_start_y + pt3_y) / 2)  # 中心点(center point)
                    cv2.circle(img, (center_x, center_y), 5, (0, 255, 255), -1)  # 画出中心点(draw the center point)

        return center_x, center_y, angle

    def action_thread(self):
        while True:
            if self.pick:
                self.min_color, self.max_color = [], []
                self.start_pick = False
                self.controller.traveling(gait=-2, time=1, steps=0)
                time.sleep(1)

                time.sleep(0.5)
                set_servo_position(self.joints_pub, 1.5, ((19, 500), (20, 220), (21, 290), (22, 280), (23, 500), (24, 700)))
                time.sleep(2)
                set_servo_position(self.joints_pub, 0.5, ((19, 500), (20, 220), (21, 290), (22, 280), (23, 500), (24, 700)))
                time.sleep(0.5)
                set_servo_position(self.joints_pub, 0.5, ((19, 500), (20, 220), (21, 290), (22, 280), (23, 500), (24, 450)))
                time.sleep(0.5)
                set_servo_position(self.joints_pub, 0.5, ((19, 500), (20, 220), (21, 290), (22, 280), (23, 500), (24, 450)))
                time.sleep(0.3)
                set_servo_position(self.joints_pub, 1.5, ((19, 500), (20, 700), (21, 15), (22, 215), (23, 500), (24, 450)))
                time.sleep(1.5)
                set_servo_position(self.joints_pub, 0.3, ((19, 500), (20, 700), (21, 15), (22, 215), (23, 500), (24, 450)))
                time.sleep(0.3)
                
                self.pick = False
                self.get_logger().info('pick finish')
                msg = Bool()
                msg.data = True
                self.action_finish_pub.publish(msg)
                self.pick_finish = True
                self.place = False
            elif self.place:
                self.min_color, self.max_color = [], []
                self.start_place = False
                self.mecanum_pub.publish(Twist())
                time.sleep(1)
                set_servo_position(self.joints_pub, 1.5, ((19, 500), (20, 110), (21, 430), (22, 390), (23, 500), (24, 460)))
                time.sleep(1.5)
                set_servo_position(self.joints_pub, 0.3, ((19, 500), (20, 110), (21, 430), (22, 390), (23, 500), (24, 460)))
                time.sleep(0.3)
                set_servo_position(self.joints_pub, 0.5, ((19, 500), (20, 110), (21, 430), (22, 390), (23, 500), (24, 700)))
                time.sleep(0.5)
                set_servo_position(self.joints_pub, 0.3, ((19, 500), (20, 110), (21, 430), (22, 390), (23, 500), (24, 700)))
                time.sleep(0.3)
                set_servo_position(self.joints_pub, 1.5, ((19, 500), (20, 700), (21, 155), (22, 70), (23, 500), (24, 700)))
                time.sleep(1.5)
                set_servo_position(self.joints_pub, 0.3, ((19, 500), (20, 700), (21, 155), (22, 70), (23, 500), (24, 700)))
                time.sleep(1.5)
                self.get_logger().info('place finish')
                self.place = False
                msg = Bool()
                msg.data = True
                self.action_finish_pub.publish(msg)
                self.place_finish = True
            else:
                time.sleep(0.01)

    def pick_handle(self, image):
        twist = Twist()

        if not self.pick or self.debug == 'pick':
            object_center_x, object_center_y, object_angle = self.color_detect(image)  # 获取物体颜色的中心和角度(obtain the center and angle of the object color)
            # self.get_logger().info(f'{object_center_x}, {object_center_y}, {object_angle}')
            if self.debug == 'pick':
                self.detect_count += 1
                if self.detect_count > 10:
                    self.detect_count = 0
                    self.pick_stop_y = object_center_y
                    self.pick_stop_x = object_center_x
                    data = common.get_yaml_data(self.config_path)
                    data['/**']['ros__parameters']['pick_stop_pixel_coordinate'] = [self.pick_stop_x, self.pick_stop_y]
                    common.save_yaml_data(data, self.config_path)
                    self.debug = False
                self.get_logger().info('x_y: ' + str([object_center_x, object_center_y]))  # 打印当前物体中心的像素(print the pixel of the current object's center)
            elif object_center_x > 0:
                ########电机pid处理(motor PID processing)#########
                # 以图像的中心点的x，y坐标作为设定的值，以当前x，y坐标作为输入(use the x and y coordinates of the image's center point as the set value, and the current x and y coordinates as the input)#
                self.linear_pid.SetPoint = self.pick_stop_y
                if abs(object_center_y - self.pick_stop_y) <= self.d_y:
                    object_center_y = self.pick_stop_y
                if self.status != "align":
                    self.linear_pid.update(object_center_y)  # 更新pid(update PID)
                    output = self.linear_pid.output
                    tmp = math.copysign(self.linear_base_speed, output) + output
                    self.linear_speed = tmp
                    if tmp > 0.4:
                        self.linear_speed = 0.4
                    if tmp < -0.4:
                        self.linear_speed = -0.4
                    if abs(tmp) <= 0.0075:
                        self.linear_speed = 0

                self.angular_pid.SetPoint = self.pick_stop_x
                if abs(object_center_x - self.pick_stop_x) <= self.d_x:
                    object_center_x = self.pick_stop_x
                if self.status != "align":
                    self.angular_pid.update(object_center_x)  # 更新pid(update PID)
                    output = self.angular_pid.output
                    tmp = math.copysign(self.angular_base_speed, output) + output

                    self.angular_speed = tmp
                    if tmp > 1.5:
                        self.angular_speed = 1.5
                    if tmp < -1.5:
                        self.angular_speed = -1.5
                    if abs(tmp) <= 0.038:
                        self.angular_speed = 0
                if abs(self.linear_speed) == 0 and abs(self.angular_speed) == 0:
                    self.count_turn += 1
                    if self.count_turn >= 3:
                        self.count_turn = 3
                        self.status = "align"
                        if self.count_stop < 3:  # 连续10次都没在移动(if there is no movement detected for 10 consecutive times)
                            if object_angle < 40: # 不取45，因为如果在45时值的不稳定会导致反复移动(do not use 45, because unstable values at 45 may cause repeated movement)
                                object_angle += 90
                            self.yaw_pid.SetPoint = 90
                            if abs(object_angle - 90) <= 3:
                                object_angle = 90
                            self.yaw_pid.update(object_angle)  # 更新pid(update PID)
                            self.yaw_angle = self.yaw_pid.output
                            if object_angle != 90:
                                if abs(self.yaw_angle) <=0.038:
                                    self.count_stop += 1
                                    # self.count_stop =0
                                twist.linear.y = float(-2 * 0.3 * math.sin(self.yaw_angle / 20))
                                twist.angular.z = float(self.yaw_angle/10)
                            else:
                                self.count_stop += 1
                        elif self.count_stop <=6:
                            self.d_x = 30
                            self.d_y = 30
                            self.count_stop += 1
                            self.status = "adjust"
                        else:
                            self.count_stop = 0
                            self.pick = True

                else:
                    if self.count_stop >= 3:
                        self.count_stop = 3
                    self.count_turn = 0
                    if self.status != 'align':
                        twist.linear.x = float(self.linear_speed)
                        twist.angular.z = float(self.angular_speed)

        self.mecanum_pub.publish(twist)

        return image


    def place_handle(self, image):
        twist = Twist()
        if not self.place or self.debug == 'place':
            object_center_x, object_center_y, object_angle = self.color_detect(image)  # 获取物体颜色的中心和角度(obtain the center and angle of the object color)
            if self.debug == 'place':
                self.detect_count += 1
                if self.detect_count > 10:
                    self.detect_count = 0
                    self.place_stop_y = object_center_y
                    self.place_stop_x = object_center_x
                    data = common.get_yaml_data(self.config_path)
                    data['/**']['ros__parameters']['place_stop_pixel_coordinate'] = [self.place_stop_x, self.place_stop_y]
                    common.save_yaml_data(data, self.config_path)
                    self.debug = False
                self.get_logger().info('x_y: ' + str([object_center_x, object_center_y]))  # 打印当前物体中心的像素(print the pixel of the current object's center)
            elif object_center_x > 0:
               ########电机pid处理(motor PID processing)#########
                # 以图像的中心点的x，y坐标作为设定的值，以当前x，y坐标作为输入(use the x and y coordinates of the image's center point as the set value, and the current x and y coordinates as the input)#
                self.linear_pid.SetPoint = self.place_stop_y
                if abs(object_center_y - self.place_stop_y) <= self.d_y:
                    object_center_y = self.place_stop_y
                if self.status != "align":
                    self.linear_pid.update(object_center_y)  # 更新pid(update PID)
                    output = self.linear_pid.output
                    tmp = math.copysign(self.linear_base_speed, output) + output
                    self.linear_speed = tmp/10
                    if tmp > 0.1:
                        self.linear_speed = 0.01
                    if tmp < -0.1:
                        self.linear_speed = -0.01
                    if abs(tmp) <= 0.0075:
                        self.linear_speed = 0

                self.angular_pid.SetPoint = self.place_stop_x
                if abs(object_center_x - self.place_stop_x) <= self.d_x:
                    object_center_x = self.place_stop_x
                if self.status != "align":
                    self.angular_pid.update(object_center_x)  # 更新pid(update PID)
                    output = self.angular_pid.output
                    tmp = math.copysign(self.angular_base_speed, output) + output

                    self.angular_speed = tmp/5
                    if tmp > 1.0:
                        self.angular_speed = 0.2
                    if tmp < -1.0:
                        self.angular_speed = -0.2
                    if abs(tmp) <= 0.035:
                        self.angular_speed = 0
                if abs(self.linear_speed) == 0 and abs(self.angular_speed) == 0:
                    self.count_turn += 1
                    if self.count_turn >= 3:
                        self.count_turn = 3
                        self.status = "align"
                        if self.count_stop < 3:  # 连续10次都没在移动(if there is no movement detected for 10 consecutive times)
                            if object_angle < 40: # 不取45，因为如果在45时值的不稳定会导致反复移动(do not use 45, because unstable values at 45 may cause repeated movement)
                                object_angle += 90
                            self.yaw_pid.SetPoint = 90
                            if abs(object_angle - 90) <= 3:
                                object_angle = 90
                            self.yaw_pid.update(object_angle)  # 更新pid(update PID)
                            self.yaw_angle = self.yaw_pid.output
                            # self.get_logger().info(f'{self.yaw_angle}')
                            if object_angle != 90:
                                if abs(self.yaw_angle) <=0.038:
                                    self.count_stop += 1
                                    # self.count_stop =0
                                twist.linear.y = float(-2 * 0.3 * math.sin(self.yaw_angle / 20))
                                twist.angular.z = float(self.yaw_angle/10)
                            else:
                                self.count_stop += 1
                        elif self.count_stop <=6:
                            self.d_x = 30
                            self.d_y = 30
                            self.count_stop += 1
                            self.status = "adjust"
                        else:
                            self.count_stop = 0
                            self.place = True

                else:
                    if self.count_stop >= 3:
                        self.count_stop = 3
                    self.count_turn = 0
                    if self.status != 'align':
                        twist.linear.x = float(self.linear_speed)
                        twist.angular.z = float(self.angular_speed)

        self.mecanum_pub.publish(twist)

        return image


    def image_callback(self, ros_image):
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, "rgb8")
        rgb_image = np.array(cv_image, dtype=np.uint8)
        if self.image_queue.full():
            # 如果队列已满，丢弃最旧的图像(if the queue is full, remove the oldest image)
            self.image_queue.get()
            # 将图像放入队列(put the image into the queue)
        self.image_queue.put(cv2.resize(rgb_image, (640, 480)))

    def main(self):
        while self.running:
            try:
                image = self.image_queue.get(block=True, timeout=1)
            except queue.Empty:
                if not self.running:
                    break
                else:
                    continue
            
            result_image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            if self.max_color and self.min_color:
                if self.start_pick:
                    self.stop = True

                    result_image = self.pick_handle(cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
                elif self.start_place:
                    self.stop = True
                    result_image = self.place_handle(cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
                else:
                    if self.stop:
                        self.stop = False
                    result_image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            
                cv2.line(result_image, (self.pick_stop_x, self.pick_stop_y - 10), (self.pick_stop_x, self.pick_stop_y + 10), (0, 255, 255), 2)
                cv2.line(result_image, (self.pick_stop_x - 10, self.pick_stop_y), (self.pick_stop_x + 10, self.pick_stop_y), (0, 255, 255), 2)
            if self.enable_display:
                cv2.imshow(self.image_name, result_image)
                cv2.waitKey(1)
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(result_image, "bgr8"))

        set_servo_position(self.joints_pub, 2, ((19, 500), (20, 700), (21, 15), (22, 215), (23, 500), (24, 200)))
        self.cmd_vel_pub.publish(Twist())
        rclpy.shutdown()

def main():
    node = AutomaticTransportNode('automatic_transport')
    rclpy.spin(node)
    node.destroy_node()
 
if __name__ == "__main__":
    main()

