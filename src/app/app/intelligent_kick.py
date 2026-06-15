#!/usr/bin/env python3
# encoding: utf-8

# 智能踢球
import os
import cv2
import time
import math
import queue
import rclpy
import signal
import threading
import numpy as np
from app.common import Heart
import sdk.pid as pid
import sdk.common as common 
from rclpy.node import Node
from cv_bridge import CvBridge
from app.common import ColorPicker 
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool, Trigger
from geometry_msgs.msg import Twist
from arm_kinematics import transform
from arm_kinematics_msgs.srv import SetRobotPose
from rclpy.executors import MultiThreadedExecutor
from servo_controller_msgs.msg import ServosPosition
from rclpy.callback_groups import ReentrantCallbackGroup
from interfaces.srv import SetPoint, SetFloat64, SetString
from controller import step_controller, controller_client
from servo_controller.bus_servo_control import set_servo_position

class CenterObj:
    def __init__(self, x, y, radius):
        self.x = int(x)
        self.y = int(y)
        self.radius = int(radius)

class IntelligentKickNode(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.name = name
        
        self.x_dis = 150
        self.y_dis = 500
        self.x_init = transform.link3 + transform.tool_link
        self.center = None
        self.running = True
        self.start = False
        self.kick = False
        self.display = False
        self.x_direction = 1
        self.y_direction = 1
        self.x_stop = 305
        self.y_stop = 230
        self.scale_y = 1.0
        self.pid_x = pid.PID(0.02, 0.0, 0.000001)
        self.pid_y = pid.PID(0.065, 0.0001, 0.000001)
        self.pid_X = pid.PID(0.02, 0.0, 0.000001)
        self.pid_Y = pid.PID(0.015, 0.0001, 0.000001)
        
        self.bridge = CvBridge()
        self.color_picker = None # 颜色拾取器对象
        self.target_lab = None   # 目标颜色LAB值
        self.target_rgb = None   # 目标颜色RGB值 (用于绘制)
        self.use_color_picker = True # 默认开启拾取模式
        self.threshold = 0.5     # 颜色阈值
        self.lab_config = common.get_yaml_data("/home/ubuntu/software/lab_tool/lab_config.yaml") # 读取本地颜色配置
        self.mouse_callback_set = False # 标记鼠标回调是否已设置
        self.pro_size = (320, 200)
        self.width = 640
        self.height = 400
        self.last_color_circle = None
        self.lost_target_count = 0
        self.color = None
        self.debug = self.get_parameter('debug').value
        self.camera = os.environ['DEPTH_CAMERA_TYPE']

        signal.signal(signal.SIGINT, self.shutdown)
        self.step_controller = step_controller.StepController()
        self.controller = controller_client.ControllerClient()
        self.image_queue = queue.Queue(maxsize=2)

        self.joints_pub = self.create_publisher(ServosPosition, '/servo_controller', 1) 
        self.cmd_vel_pub = self.create_publisher(Twist, '/controller/cmd_vel', 1)
        self.result_publisher = self.create_publisher(Image, '~/image_result', 1)
        
        self.image_sub = self.create_subscription(Image, '/depth_cam/rgb/image_raw', self.image_callback, 1)

        timer_cb_group = ReentrantCallbackGroup()
        self.set_color_srv = self.create_service(SetString, '~/set_color', self.set_color_srv_callback, callback_group=timer_cb_group) 
        self.set_threshold_srv = self.create_service(SetFloat64, '~/set_threshold', self.set_threshold_srv_callback)
        self.get_target_color_srv = self.create_service(Trigger, '~/get_target_color', self.get_target_color_srv_callback)

        self.create_service(SetPoint, '~/set_target_color', self.set_target_color_srv_callback, callback_group=timer_cb_group)

        self.client = self.create_client(Trigger, '/controller_manager/init_finish')
        self.client.wait_for_service()
        self.client = self.create_client(Trigger, '/arm_kinematics/init_finish')
        self.client.wait_for_service()
        
        self.create_service(Trigger, '~/enter', self.enter_srv_callback)
        self.create_service(Trigger, '~/exit', self.exit_srv_callback)
        self.create_service(SetBool, '~/set_running', self.set_running_srv_callback)

        self.arm_kinematics_client = self.create_client(SetRobotPose, '/arm_kinematics/set_pose_target')
        self.arm_kinematics_client.wait_for_service()

        Heart(self, self.name + '/heartbeat', 5, lambda _: self.exit_srv_callback(request=Trigger.Request(), response=Trigger.Response()))
        threading.Thread(target=self.main, daemon=True).start()

    def get_node_state(self, request, response):
        response.success = True
        return response
    
    def enter_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "intelligent kick enter")
        # 进入时重置相关参数，准备选色
        self.center = None
        self.start = False # 等待选色或set_running开启
        self.color_picker = None
        self.target_lab = None
        self.display = True
        if self.camera == 'usb_cam':
            self.y_stop = 320
        self.step_controller.set_build_in_pose('DEFAULT_POSE', 1)
        time.sleep(1.0)
        set_servo_position(self.joints_pub, 1, ((24, 500), (23, 500), (22, 150), (21, 100), (20, 665), (19, 500)))

        response.success = True
        response.message = "enter"
        return response
    
    def set_running_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % 'set_running: %s' % request.data)
        self.start = request.data
        
        if not self.start:
            self.controller.traveling(gait=-2, time=1, steps=0)
            set_servo_position(self.joints_pub, 1, ((24, 500), (23, 500), (22, 150), (21, 100), (20, 665), (19, 500)))
            self.y_dis = 500
            self.x_dis = 150

        response.success = True
        response.message = "set_running"
        return response

    def exit_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % 'intelligent kick exit ')
        self.start = False
        self.color_picker = None
        self.target_lab = None
        self.center = None
        self.cmd_vel_pub.publish(Twist())
        self.step_controller.set_build_in_pose('DEFAULT_POSE', 1)
        self.controller.traveling(gait=-2, time=1, steps=0)
        response.success = True
        response.message = "exit"
        return response

    def set_threshold_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % 'threshold')
        self.threshold = request.data
        response.success = True
        response.message = "set_threshold"
        return response

    def shutdown(self, signum, frame):
        self.running = False

    # --- 用于鼠标点击选色的服务回调 ---
    def set_target_color_srv_callback(self, request, response):
        self.get_logger().info('Start picking color at x:%.2f, y:%.2f' % (request.data.x, request.data.y))
        x, y = request.data.x, request.data.y
        if x == -1 and y == -1:
            self.color_picker = None
        else:
            self.use_color_picker = True
            # 初始化颜色拾取器，区域大小 10x10
            self.color_picker = ColorPicker(request.data, 10)
        
        response.success = True
        response.message = "picking"
        return response

    def set_color_srv_callback(self, request, response):
        color_name = request.data
        self.get_logger().info('\033[1;32m%s\033[0m' % ("set_color: " + color_name))
        
        if color_name in self.lab_config['lab']['Stereo']:
            self.color = self.lab_config['lab']['Stereo'][color_name]
            min_v = self.color['min']
            max_v = self.color['max']
            self.target_lab = [(min_v[0]+max_v[0])/2, (min_v[1]+max_v[1])/2, (min_v[2]+max_v[2])/2]
            
            self.target_rgb = common.range_rgb.get(color_name, (255, 255, 255))
            self.use_color_picker = False
            self.color_picker = None
            self.last_color_circle = None
            self.get_logger().info('Color set to %s' % color_name)
            response.success = True
            response.message = "set_color success"
        else:
            self.get_logger().warn('Color %s not found' % color_name)
            response.success = False
            response.message = "color not found"
        return response

    def get_target_color_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % 'get_target_color')
        response.success = False
        response.message = "get_target_color"
        if self.target_rgb is not None:
            response.success = True
            response.message = "{},{},{}".format(int(self.target_rgb[0]), int(self.target_rgb[1]), int(self.target_rgb[2]))
        return response

    def image_callback(self, ros_image):
        # 将 ROS 图像转换为 OpenCV 格式 (RGB)
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, "rgb8")
        rgb_image = np.array(cv_image, dtype=np.uint8)
        self.height, self.width, _ = rgb_image.shape
        self.pro_size = (self.width // 2, self.height // 2)
        if self.image_queue.full():
            self.image_queue.get()
        self.image_queue.put(rgb_image)


    def kick_ball(self, center):
        self.get_logger().info('kick_ball: start')

        if self.kick and self.center is not None:
            self.get_logger().info('kick_ball: executing kick')
            self.controller.traveling(gait=-2, time=1, steps=0)
            # 记录踢球前的球坐标
            pre_kick_center = self.center
            
            # 使用传入的 center 坐标计算
            # 计算距离时不应使用 scale_y 缩放，假设像素密度不变
            dis = 250 + (center.y - self.height // 2) 
            if self.camera == 'usb_cam':
                dis -= 40
            dis_y = 40 + (center.x - self.width // 2)

            self.step_controller.set_leg_position(1, (130, 160, -50), 0.3)
            time.sleep(0.5)
            self.step_controller.set_leg_position(1, (dis , dis_y, -50), 0.3)
            time.sleep(0.5)
            self.step_controller.set_leg_position(1, (160, 140, -70), 0.3)
            time.sleep(0.5)
            
            # 检测球是否还在原地
            if self.center is not None:
                # 计算踢球前后的坐标距离
                dx = self.center.x - pre_kick_center.x
                dy = self.center.y - pre_kick_center.y
                dist = math.sqrt(dx**2 + dy**2)
                
                self.get_logger().info('Check kick result: dist=%.2f' % dist)
                
                # 如果球移动距离非常小（比如小于20像素），认为没踢到
                if dist < 20:
                    self.get_logger().warn('Kick failed! Ball still at the same position. Backing up...')
                    self.controller.traveling(gait=2, time=1, direction=math.radians(180), steps=2)
                    time.sleep(2.0) # 等待动作完成
            else:
                self.get_logger().info('Kick success! Ball is gone.')
            
        self.kick = False
        self.get_logger().info('kick_ball: done')

    # --- OpenCV 鼠标回调 ---
    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            # 映射坐标归一化
            h, w = self.height, self.width 
            self.get_logger().info(f'self.height: {self.height}, self.width: {self.width}')

            req = SetPoint.Request()
            req.data.x = x / w
            req.data.y = y / h
            self.set_target_color_srv_callback(req, SetPoint.Response())

    # --- 图像处理逻辑 ---
    def process_image(self, image):
        h, w = image.shape[:2]
        # 如果正在使用取色器
        if self.use_color_picker and self.color_picker is not None:
            res, image = self.color_picker(image, image) # 拾取并画框
            if res is not None:
                # 拾取成功
                self.target_lab, self.target_rgb = res
                self.color_picker = None # 停止拾取
                self.last_color_circle = None
                self.get_logger().info("Color picked successfully!")
        
        # 如果有目标颜色，进行追踪
        if self.target_lab is not None:
            # 1. 预处理
            img_resize = cv2.resize(image, self.pro_size)
            img_lab = cv2.cvtColor(img_resize, cv2.COLOR_RGB2LAB)
            img_blur = cv2.GaussianBlur(img_lab, (5, 5), 5)
            
            # 2. 阈值计算
            if self.use_color_picker:
                lowerb = (int(self.target_lab[0] - 50 * self.threshold * 2),
                          int(self.target_lab[1] - 50 * self.threshold),
                          int(self.target_lab[2] - 50 * self.threshold))
                upperb = (int(self.target_lab[0] + 50 * self.threshold * 2),
                          int(self.target_lab[1] + 50 * self.threshold),
                          int(self.target_lab[2] + 50 * self.threshold))
            else:
                lowerb = tuple(self.color['min'])
                upperb = tuple(self.color['max'])
            
            # 3. 二值化与形态学
            mask = cv2.inRange(img_blur, lowerb, upperb)
            eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
            dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
            
            # 4. 轮廓查找
            contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
            contour_area = map(lambda c: (c, math.fabs(cv2.contourArea(c))), contours)
            contour_area = list(filter(lambda c: c[1] > 40, contour_area)) # 过滤小面积
            
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
                    if dist > 100:
                        circle = None

            if circle is not None:
                self.lost_target_count = 0
                self.last_color_circle = circle
                (x, y), r = circle
                
                # 映射回原图尺寸
                x = x / self.pro_size[0] * w
                y = y / self.pro_size[1] * h
                r = r / self.pro_size[0] * w

                # 更新 self.center 供后续逻辑使用
                self.center = CenterObj(x, y, r)
                
                # 画图
                if self.target_rgb:
                    cv2.circle(image, (int(x), int(y)), int(r), (255, 255, 0), 2) 
                    cv2.circle(image, (int(x), int(y)), 5, (255, 255, 0), -1)

            else:
                self.lost_target_count += 1
                if self.lost_target_count > 10:
                    self.center = None
                    self.last_color_circle = None
        else:
            self.center = None
            self.last_color_circle = None
            
        return image

    def main(self):
        while self.running:
            twist = Twist()
            try:
                # 获取原始图像
                image = self.image_queue.get(block=True, timeout=1)
            except queue.Empty:
                if not self.running:
                    break
                else:
                    continue

            # process_image 处理 RGB 图像并返回绘图后的 RGB 图像
            show_image = self.process_image(image)

            if self.start and self.target_lab is not None:
                if self.center is not None :
                    t1 = time.time()
                    center = self.center

                    self.pid_y.SetPoint = self.x_stop
                    self.pid_y.update(center.x)
                    self.y_dis += self.pid_y.output
                    
                    if self.y_dis < 350:
                        self.y_dis = 350
                    if self.y_dis > 650:
                        self.y_dis = 650
                    if self.x_dis < 150:
                        self.x_dis = 150
                    if self.x_dis > 250:
                        self.x_dis = 250

                    self.pid_x.SetPoint = self.y_stop
                    self.pid_x.update(center.y)
                    self.x_dis += self.pid_x.output

                    t2 = time.time()
                    t = t2 - t1
                    if t < 0.02:
                        time.sleep(0.02 - t)

                    set_servo_position(self.joints_pub, 0.02, ((19, self.y_dis), (22, int(self.x_dis))))
                    self.pid_Y.SetPoint = 500  # 19号舵机的初始值
                    self.pid_Y.update(self.y_dis)
                    self.pid_X.SetPoint = 200  # 22号舵机的初始值
                    self.pid_X.update(self.x_dis)

                    twist.angular.z = -self.pid_Y.output/20
                    # 动态阈值计算

                    if self.camera == 'usb_cam':
                        if self.pid_X.output < 1 or center.y < 305:
                            twist.linear.x = 0.02
                        elif self.pid_X.output > 1.2 or center.y > 335:
                            twist.linear.x = -0.02
                        elif 290 < center.x < 320 and 305 < center.y < 335 and abs(self.y_dis - 500) < 10 :
                            if not self.kick:
                                self.kick = True
                                threading.Thread(target=self.kick_ball, args=(center,)).start()
                    else:
                        if self.pid_X.output < 1 or center.y < 180:
                            twist.linear.x = 0.02
                        elif self.pid_X.output > 1.2 or center.y > 260:
                            twist.linear.x = -0.02
                        elif 290 < center.x < 320 and 205 < center.y < 260 and abs(self.y_dis - 500) < 10 :
                            if not self.kick:
                                self.kick = True
                                threading.Thread(target=self.kick_ball, args=(center,)).start()
                    # self.get_logger().info(f'x_dis: {self.x_dis}, y_dis: {self.y_dis}')

                    if not self.kick:
                        self.cmd_vel_pub.publish(twist)

                else:
                    self.cmd_vel_pub.publish(Twist())
                    
                    self.x_dis += 10 * self.x_direction 
                    self.y_dis += 10 * self.y_direction
                    # 到达左右边界时，纵向移动一次，x 方向翻转
                    if self.x_dis > 300:
                        self.x_dis = 300
                        self.x_direction = -1  # 改为向左扫
                          # 根据方向向上或向下扫一次

                    elif self.x_dis < 100:
                        self.x_dis = 100
                        self.x_direction = 1   # 改为向右扫

                    # 控制纵向上下边界范围（上下来回）
                    if self.y_dis > 750:
                        self.y_dis = 750
                        self.y_direction = -1
                    elif self.y_dis < 250:
                        self.y_dis = 250
                        self.y_direction = 1

                    # 设置舵机位置
                    set_servo_position(self.joints_pub, 0.2, ((19, int(self.y_dis)), (22, int(self.x_dis))))
                    time.sleep(0.2)
            else:
                time.sleep(0.02)
            # 绘制中心点 (此时 show_image 是 RGB，黄色应该是 (255, 255, 0))
            cv2.circle(show_image, (self.x_stop, self.y_stop), 10, (255, 255, 0), -1)
            if self.debug:
                if self.display:
                    # 转换成 BGR 供本地窗口显示
                    display_image = cv2.cvtColor(show_image, cv2.COLOR_RGB2BGR)
                    cv2.imshow('image', display_image)
                    
                    # 设置鼠标回调 (只设置一次)
                    if not self.mouse_callback_set:
                        cv2.setMouseCallback('image', self.mouse_callback)
                        self.mouse_callback_set = True
            else:
                # 发布时保持 RGB 格式
                self.result_publisher.publish(self.bridge.cv2_to_imgmsg(show_image, "rgb8"))

            cv2.waitKey(1)
            
        rclpy.shutdown()

def main():
    node = IntelligentKickNode('intelligent_kick') # 修正节点名称
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
 
if __name__ == "__main__":
    main()