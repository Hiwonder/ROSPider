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
import torch
import queue
import rclpy
import threading
import numpy as np
from sdk import common
from sdk.pid import PID
from rclpy.node import Node
from std_msgs.msg import Bool
from cv_bridge import CvBridge
from app.common import ColorPicker
from interfaces.msg import CmdParam
from std_srvs.srv import Trigger, Empty
from controller import controller_client 
from geometry_msgs.msg import Twist, Point
from interfaces.srv import SetPoint, SetBox
from ultralytics.utils.ops import scale_masks 
from sensor_msgs.msg import Image, CameraInfo
from servo_controller_msgs.msg import ServosPosition
from ultralytics.models.fastsam import FastSAMPredictor
from arm_kinematics.kinematics_control import set_pose_target
from arm_kinematics_msgs.srv import GetRobotPose, SetRobotPose
from servo_controller.bus_servo_control import set_servo_position
from servo_controller.action_group_controller import ActionGroupController

device = 'cuda' if torch.cuda.is_available() else 'cpu'
def prompt(results, bboxes=None, points=None, labels=None, texts=None, log=None):
    if bboxes is None and points is None and texts is None:
        return results
    prompt_results = []
    if not isinstance(results, list):
        results = [results]
    for result in results:
        if len(result) == 0:
            prompt_results.append(result)
            continue
        masks = result.masks.data
        if masks.shape[1:] != result.orig_shape:
            masks = scale_masks(masks[None], result.orig_shape)[0]
        idx = torch.zeros(len(result), dtype=torch.bool, device=device)
        if bboxes is not None:
            bboxes = torch.as_tensor(bboxes, dtype=torch.int32, device=device)
            bboxes = bboxes[None] if bboxes.ndim == 1 else bboxes
            bbox_areas = (bboxes[:, 3] - bboxes[:, 1]) * (bboxes[:, 2] - bboxes[:, 0])
            mask_areas = torch.stack([masks[:, b[1] : b[3], b[0] : b[2]].sum(dim=(1, 2)) for b in bboxes])
            full_mask_areas = torch.sum(masks, dim=(1, 2))
 
            u = mask_areas / full_mask_areas  
            u = torch.nan_to_num(u, nan=0.0) 
            indices = (u >= (torch.max(u) - 0.1)).nonzero(as_tuple=True)[1] 
            u1 = full_mask_areas / bbox_areas
            max_index = indices[torch.argmax(u1[indices])]
            idx[max_index] = True

        prompt_results.append(result[idx])

    return prompt_results

def depth_pixel_to_camera(pixel_coords, depth, intrinsics):
    fx, fy, cx, cy = intrinsics
    px, py = pixel_coords
    x = (px - cx) * depth / fx
    y = (py - cy) * depth / fy
    z = depth
    return np.array([x, y, z])


# display_size = [int(640*6/4), int(480*6/4)]
class AutomaticPickNode(Node):
    config_path = '/home/ubuntu/ros2_ws/src/large_models_examples/config/automatic_pick_roi.yaml'

    lab_data = common.get_yaml_data("/home/ubuntu/software/lab_tool/lab_config.yaml")
    hand2cam_tf_matrix = [
    [0.0, 0.0, 1.0, -0.101],
    [-1.0, 0.0, 0.0, 0.018],
    [0.0, -1.0, 0.0, 0.045],
    [0.0, 0.0, 0.0, 1.0]
]
    def __init__(self, name):
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.name = name
        # 颜色识别(color recognition)
        self.image_proc_size = (320, 200)
        
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
        self.position = [0, 0, 0]

        self.yaw_pid = PID(P=0.015, I=0, D=0.000)
        self.linear_pid = PID(P=0.0028, I=0, D=0)
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
        self.camera_info = queue.Queue(maxsize=2)
        self.depth_image_queue = queue.Queue(maxsize=2)
        self.display_box = True
        self.start_time = time.time()
        self.start = self.get_parameter('start').value
        self.enable_display = self.get_parameter('enable_display').value
        self.debug = self.get_parameter('debug').value
        self.image_name = 'image'
        cv2.namedWindow(self.image_name, 1)
        cv2.setMouseCallback(self.image_name, self.onmouse)

        code_path = os.path.abspath(os.path.split(os.path.realpath(__file__))[0])
        overrides = dict(conf=0.4, task="segment", mode="predict", model=os.path.join(os.path.dirname(code_path), 'resources/models', "FastSAM-x.pt"), save=False, imgsz=640)
        self.predictor = FastSAMPredictor(overrides=overrides)
        self.predictor(np.zeros((640, 400, 3), dtype=np.uint8))
        self.language = os.environ['ASR_LANGUAGE']
        self.camera = os.environ['DEPTH_CAMERA_TYPE']

        self.controller = controller_client.ControllerClient()


        self.joints_pub = self.create_publisher(ServosPosition, '/servo_controller', 1)
        self.cmd_vel_pub = self.create_publisher(Twist, '/controller/cmd_vel', 1)
        self.cmd_param_pub = self.create_publisher(CmdParam, '/step_controller/cmd_param', 1) 
        self.image_pub = self.create_publisher(Image, '~/image_result', 1)

        self.create_subscription(Image, 'depth_cam/rgb/image_raw', self.image_callback, 1)
        if self.camera == "aurora":
            self.create_subscription(Image, '/depth_cam/depth/image_raw', self.depth_image_callback, 1)
            self.create_subscription(CameraInfo , '/depth_cam/depth/camera_info', self.camera_info_callback, 1)

        self.create_service(Trigger, '~/pick', self.start_pick_callback)
        self.create_service(Trigger, '~/place', self.start_place_callback) 
        self.create_service(SetPoint, '~/set_target_color', self.set_target_color_srv_callback)
        self.create_service(SetBox, '~/set_box', self.set_box_srv_callback)

        self.agc = ActionGroupController(self.create_publisher(ServosPosition, 'servo_controller', 1), '/home/ubuntu/software/actionset_editor/ActionGroups')
        
        self.get_current_pose_client = self.create_client(GetRobotPose, '/arm_kinematics/get_current_pose')
        self.set_pose_target_client = self.create_client(SetRobotPose, '/arm_kinematics/set_pose_target')
        self.get_current_pose_client = self.create_client(GetRobotPose, '/arm_kinematics/get_current_pose')
        self.set_pose_target_client = self.create_client(SetRobotPose, '/arm_kinematics/set_pose_target')
        self.client = self.create_client(Trigger, '/controller_manager/init_finish')
        self.client.wait_for_service()
        
        self.action_finish_pub = self.create_publisher(Bool, '~/action_finish', 1)
        self.set_cmdparam()

        set_servo_position(self.joints_pub, 2, ((19, 500), (20, 700), (21, 155), (22, 70), (23, 500), (24, 700)))
        time.sleep(2)
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
        self.get_logger().info('\033[1;32m%s\033[0m' % 'debug'+str(self.debug))

    def get_node_state(self, request, response):
        return response
    
    def get_endpoint(self):
        endpoint = self.send_request(self.get_current_pose_client, GetRobotPose.Request()).pose
        self.endpoint = common.xyz_quat_to_mat([endpoint.position.x, endpoint.position.y, endpoint.position.z],
                                        [endpoint.orientation.w, endpoint.orientation.x, endpoint.orientation.y, endpoint.orientation.z])
        return self.endpoint

    def set_cmdparam(self):
        cmd_param = CmdParam()
        cmd_param.pose = 'DEFAULT_POSE'
        cmd_param.gait = 2
        cmd_param.height = 10
        cmd_param.period = 1.0
        self.cmd_param_pub.publish(cmd_param)

    # 鼠标点击事件回调函数
    def onmouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:  # 鼠标左键按下
            self.mouse_click = True
            self.drag_start = (x, y)  # 鼠标起始位置
            self.track_window = None
            self.selection = None
            self.start_circle = True
            self.start_click = False
        if self.drag_start:  # 是否开始拖动鼠标，记录鼠标位置
            xmin = min(x, self.drag_start[0])
            ymin = min(y, self.drag_start[1])
            xmax = max(x, self.drag_start[0])
            ymax = max(y, self.drag_start[1])
            self.selection = (xmin, ymin, xmax, ymax)
        if event == cv2.EVENT_LBUTTONUP:  # 鼠标左键松开
            if not self.debug:
                if not self.pick_finish:
                    msg = Trigger.Request()
                    self.start_pick_callback(msg, Trigger.Response())
                elif not self.place_finish:
                    msg = Trigger.Request()
                    self.start_place_callback(msg, Trigger.Response())
                else:
                    msg = Trigger.Request()
                    self.start_pick_callback(msg, Trigger.Response())
            self.mouse_click = False
            self.drag_start = None
            self.track_window = self.selection
            self.selection = None
        if event == cv2.EVENT_RBUTTONDOWN:
            self.mouse_click = False
            self.selection = None  # 实时跟踪鼠标的跟踪区域
            self.track_window = None  # 要检测的物体所在区域
            self.drag_start = None  # 标记，是否开始拖动鼠标
            self.start_circle = True
            self.start_click = False
            self.start_pick = False
            self.pick = False
            self.place = False
            self.max_color, self.min_color = [], []

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

        response.success = True
        response.message = "start_pick"
        return response 


    def start_place_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "start place")
        
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
        frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_RGB2LAB)  # 将图像转换到LAB空间(convert image to LAB space)
        frame_mask = cv2.inRange(frame_lab, tuple(self.min_color), tuple(self.max_color))  # 对原图像和掩模进行位运算(perform bitwise operation on the original image and the mask)

        eroded = cv2.erode(frame_mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 腐蚀(erode)
        dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 膨胀(dilate)

        contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # 找出轮廓(find contours)
        # cv2.imshow('image', dilated)
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


    def segment_handle(self, image, box, factor):
        h, w = image.shape[:2]
        
        box_center_x = int((box[0] + box[2]) / 2)
        box_center_y = int((box[1] + box[3]) / 2)
        
        final_x = box_center_x
        final_y = box_center_y

        try:
            everything_results = self.predictor(image)
            results = prompt(everything_results, bboxes=[box])
            
            if results and len(results) > 0 and results[0].masks is not None:
                mask_data = results[0].masks.data
                if not isinstance(mask_data, np.ndarray):
                    mask_data = mask_data.cpu().numpy()
                
                if mask_data.ndim == 3:
                    mask = mask_data[0] # (640, 640)
                else:
                    mask = mask_data
                
                mask = (mask * 255).astype(np.uint8)
                mh, mw = mask.shape
                if mh == 640 and h == 400:
                    pad_top = (mh - h) // 2  
                    pad_bot = mh - pad_top   
                    
                    mask = mask[pad_top:pad_bot, :] 

                contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
                areaMaxContour, area_max = common.get_area_max_contour(contours, 10)
                
                if areaMaxContour is not None and area_max > 10:
                    rect = cv2.minAreaRect(areaMaxContour)
                    (ai_cx, ai_cy), _, _ = rect
                    
                    ai_cx = int(ai_cx)
                    ai_cy = int(ai_cy)
                    
                    if (box[0] <= ai_cx <= box[2]) and (box[1] <= ai_cy <= box[3]):
                        final_x = ai_cx
                        final_y = ai_cy
                    else:
                        self.get_logger().warn('Point out of box, using Box Center.')

        except Exception as e:
            import traceback
            self.get_logger().error(f'Segment Error: {e}')
            self.get_logger().error(traceback.format_exc())

        final_x = max(0, min(final_x, w - 1))
        final_y = max(0, min(final_y, h - 1))

        cv2.circle(image, (final_x, final_y), 8, (0, 0, 255), -1)

        return float(final_x / w), float(final_y / h)

    def action_thread(self):
        while True:
            if self.pick:
                self.min_color, self.max_color = [], []
                self.start_pick = False

                self.controller.traveling(gait=-2, time=1, steps=0)
                time.sleep(1)
                if self.camera == "aurora":
                    if self.position[2] < 0.2:
                        yaw = 80
                    else:
                        yaw = 30
                    self.position[0] += 0.0  #添加X轴偏差值
                    self.position[1] += 0.01 #添加Y轴偏差值
                    self.position[2] += 0.0  #添加Z轴偏差值
                    msg = set_pose_target(self.position, yaw, [-180.0, 180.0], 1.0)
                    res = self.send_request(self.set_pose_target_client, msg)
                    self.get_logger().info('pick finish'+str(res.pulse))

                    if res.pulse:
                        servo_data = res.pulse
                        set_servo_position(self.joints_pub, 1, ((19, servo_data[0]), ))
                        time.sleep(1)
                        set_servo_position(self.joints_pub, 1.5, ((19, servo_data[0]),(20, servo_data[1]), (21, servo_data[2]),(22, servo_data[3]), (23, servo_data[4])))
                        time.sleep(1.5)
                        set_servo_position(self.joints_pub, 0.5, ((24, 450),))
                        time.sleep(1)
                        set_servo_position(self.joints_pub, 2, ((19, 500), (20, 700), (21, 155), (22, 70), (23, 500), (24, 450)))
                        time.sleep(2)
                    else:
                        self.get_logger().error('Gripping failed')
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
                else:
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
                self.controller.traveling(gait=-1, time=2, steps=0)
                time.sleep(2)
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

                self.controller.traveling(gait=-2, time=1, steps=0)
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
                time.sleep(0.3)

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
        if self.camera == "aurora":
            depth_camera_info = self.camera_info.get(block=True, timeout=1)
            depth_image = self.depth_image_queue.get(block=True, timeout=1)
            depth_image = np.ndarray(shape=(depth_image.height, depth_image.width), dtype=np.uint16, buffer=depth_image.data)

        if not self.pick or self.debug == 'pick':
            object_center_x, object_center_y, object_angle = self.color_detect(image)  # 获取物体颜色的中心和角度(obtain the center and angle of the object color)
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
                # 以图像的中心点的x，y坐标作为设定的值，以当前x，y坐标作为输入(use the x and y coordinates of the image's center point as the set value, and the current x and y coordinates as the input)#
                self.linear_pid.SetPoint = self.pick_stop_y
                # self.get_logger().info(f'{self.pick_stop_y} {self.d_y} {object_center_y} {object_center_y - self.pick_stop_y}')
                if abs(object_center_y - self.pick_stop_y) <= self.d_y:
                    object_center_y = self.pick_stop_y
                if self.status != "align":
                    self.linear_pid.update(object_center_y)  # 更新pid(update PID)
                    output = self.linear_pid.output
                    tmp = math.copysign(self.linear_base_speed, output) + output
                    # self.get_logger().info(f'{tmp}')
                    self.linear_speed = tmp/7
                    if tmp > 0.1:
                        self.linear_speed = 0.02
                    if tmp < -0.1:
                        self.linear_speed = -0.02
                    if abs(tmp) <= 0.0075:
                        self.linear_speed = 0

                self.angular_pid.SetPoint = self.pick_stop_x
                # self.get_logger().info(f'{self.pick_stop_x} {self.d_x} {object_center_x} {object_center_x - self.pick_stop_x}')
                if abs(object_center_x - self.pick_stop_x) <= self.d_x:
                    object_center_x = self.pick_stop_x
                if self.status != "align":
                    self.angular_pid.update(object_center_x)  # 更新pid(update PID)
                    output = self.angular_pid.output
                    tmp = math.copysign(self.angular_base_speed, output) + output

                    self.angular_speed = tmp/3
                    # self.get_logger().info(f'{self.angular_speed}')
                    if tmp > 1.0:
                        self.angular_speed = 0.2
                    if tmp < -1.0:
                        self.angular_speed = -0.2
                    if abs(tmp) <= 0.038:
                        self.angular_speed = 0
                if abs(self.linear_speed) == 0 and abs(self.angular_speed) == 0:
                    self.count_turn += 1
                    if self.count_turn > 5:
                        self.count_turn = 5
                        self.status = "align"
                        if self.count_stop < 10:  # 连续10次都没在移动(if there is no movement detected for 10 consecutive times)
                            if object_angle < 40: # 不取45，因为如果在45时值的不稳定会导致反复移动(do not use 45, because unstable values at 45 may cause repeated movement)
                                object_angle += 90
                            self.yaw_pid.SetPoint = 90
                            if abs(object_angle - 90) <= 1:
                                object_angle = 90
                            self.yaw_pid.update(object_angle)  # 更新pid(update PID)
                            self.yaw_angle = self.yaw_pid.output
                            if object_angle != 90:
                                if abs(self.yaw_angle) <=0.038:
                                    self.count_stop += 1
                                else:
                                    self.count_stop = 0
                                twist.linear.y = float(-2 * 0.3 * math.sin(self.yaw_angle / 10))
                                twist.angular.z = float(self.yaw_angle/3)
                            else:
                                self.count_stop += 1
                        elif self.count_stop <= 20:
                            self.d_x = 30
                            self.d_y = 30
                            self.count_stop += 1
                            self.status = "adjust"
                        else:
                            self.count_stop = 0
                            self.pick = True
                            if self.camera == "aurora": 
                    
                                roi = [int(object_center_y) - 5, int(object_center_y) + 5, int(object_center_x) - 5, int(object_center_x) + 5]
                                if roi[0] < 0:
                                    roi[0] = 0
                                if roi[1] > 400:
                                    roi[1] = 400
                                if roi[2] < 0:
                                    roi[2] = 0
                                if roi[3] > 640:
                                    roi[3] = 640                      
                                roi_distance = depth_image[roi[0]:roi[1], roi[2]:roi[3]]
                                
                                valid_mask = (roi_distance > 0) & (roi_distance < 10000)
                                if np.any(valid_mask):
                                    dist = round(float(roi_distance[valid_mask].mean()/1000.0), 3)
                                    dist += 0.015 # 误差补偿(error compensation)
                                    K = depth_camera_info.k
                                    self.get_endpoint()
                                
                                    position = depth_pixel_to_camera((object_center_x, object_center_y), dist, (K[0], K[4], K[2], K[5]))
                                    
                                    position[0] -= 0.01  # rgb相机和深度相机tf有1cm偏移(the RGB camera and depth camera TFs have a 1cm offset)
                                    pose_end = np.matmul(self.hand2cam_tf_matrix, common.xyz_euler_to_mat(position, (0, 0, 0)))  # 转换的末端相对坐标(the relative coordinates at the end of the transformation)
                                    world_pose = np.matmul(self.endpoint, pose_end)  # 转换到机械臂世界坐标(transform into the world coordinates of the robotic arm)
                                    pose_t, pose_R = common.mat_to_xyz_euler(world_pose)
                                    self.get_logger().info('\033[1;32m%s\033[0m' % "stop"+str(pose_t))
                                    self.position = pose_t
                else:
                    if self.count_stop >= 10:
                        self.count_stop = 10
                    self.count_turn = 0
                    if self.status != 'align':
                        twist.linear.x = float(self.linear_speed)
                        twist.angular.z = float(self.angular_speed)

            self.cmd_vel_pub.publish(twist)

        return image


    def place_handle(self, image):
        twist = Twist()
        h, _ = image.shape[:2]
        image_roi = image[:int(h*0.7), :]
        if not self.place or self.debug == 'place':
            object_center_x, object_center_y, object_angle = self.color_detect(image_roi)  # 获取物体颜色的中心和角度(obtain the center and angle of the object color)
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
                # 以图像的中心点的x，y坐标作为设定的值，以当前x，y坐标作为输入(use the x and y coordinates of the image's center point as the set value, and the current x and y coordinates as the input)#
                self.linear_pid.SetPoint = self.place_stop_y
                if abs(object_center_y - self.place_stop_y) <= self.d_y:
                    object_center_y = self.place_stop_y
                self.linear_pid.update(object_center_y)  # 更新pid(update PID)
                output = self.linear_pid.output
                tmp = math.copysign(self.linear_base_speed, output) + output

                self.linear_speed = tmp/5
                if tmp > 0.1:
                    self.linear_speed = 0.02
                if tmp < -0.1:
                    self.linear_speed = -0.02
                if abs(tmp) <= 0.0075:
                    self.linear_speed = 0

                self.angular_pid.SetPoint = self.place_stop_x
                if abs(object_center_x - self.place_stop_x) <= self.d_x:
                    object_center_x = self.place_stop_x

                self.angular_pid.update(object_center_x)  # 更新pid(update PID)
                output = self.angular_pid.output
                tmp = math.copysign(self.angular_base_speed, output) + output

                self.angular_speed = tmp/3
                if tmp > 1.0:
                    self.angular_speed = 0.3
                if tmp < -1.0:
                    self.angular_speed = -0.3
                if abs(tmp) <= 0.035:
                    self.angular_speed = 0

                if abs(self.linear_speed) == 0 and abs(self.angular_speed) == 0:
                    self.place = True
                else:
                    twist.linear.x = float(self.linear_speed)
                    twist.angular.z = float(self.angular_speed)

            self.cmd_vel_pub.publish(twist)

        return image


    def image_callback(self, ros_image):
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        rgb_image = np.array(cv_image, dtype=np.uint8)
        if not hasattr(self, 'debug_printed_size'):
            self.get_logger().info(f'\033[1;33m[DEBUG] Camera Raw Shape: {rgb_image.shape}\033[0m')
            self.debug_printed_size = True
        if self.image_queue.full():
            # 如果队列已满，丢弃最旧的图像(if the queue is full, remove the oldest image)
            self.image_queue.get()
            # 将图像放入队列(put the image into the queue)
        self.image_queue.put(rgb_image)

    def depth_image_callback(self, ros_image):

        if self.depth_image_queue.full():
            # 如果队列已满，丢弃最旧的图像(if the queue is full, remove the oldest image)
            self.depth_image_queue.get()
            # 将图像放入队列(put the image into the queue)
        self.depth_image_queue.put(ros_image)

    def camera_info_callback(self, camera_info):

        if self.camera_info.full():
            # 如果队列已满，丢弃最旧的图像(if the queue is full, remove the oldest image)
            self.camera_info.get()
            # 将图像放入队列(put the image into the queue)
        self.camera_info.put(camera_info)


    def main(self):
        while self.running:
            try:
                image = self.image_queue.get(block=True, timeout=1)
            except queue.Empty:
                if not self.running:
                    break
                else:
                    continue
            result_image = image.copy()
            # result_image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            if self.start_circle:
                # 用鼠标拖拽一个框来指定区域
                if self.track_window:  # 跟踪目标的窗口画出后，实时标出跟踪目标
                    cv2.rectangle(result_image, (self.track_window[0], self.track_window[1]),
                                  (self.track_window[2], self.track_window[3]), (0, 0, 255), 2)
                elif self.selection:  # 跟踪目标的窗口随鼠标拖动实时显示
                    cv2.rectangle(result_image, (self.selection[0], self.selection[1]), (self.selection[2], self.selection[3]),
                                  (0, 255, 255), 2)
                if self.mouse_click:
                    self.start_click = True
                if self.start_click:
                    if not self.mouse_click:
                        self.start_circle = False
                if not self.start_circle:
                    self.box = self.track_window
                    self.get_logger().info('\033[1;32m%s\033[0m' % '333333box: ' + str(self.box))
            if self.box:
                if self.display_box:
                    if time.time() - self.start_time > 5:
                        self.display_box = False
                    cv2.rectangle(result_image, (self.box[0], self.box[1]),
                                  (self.box[2], self.box[3]), (0, 255, 255), 2)

                else:
                    if self.start_pick:
                        point = Point()
                        self.get_logger().info('\033[1;33m[DEBUG] Calling segment_handle for PICK...\033[0m')

                        point.x, point.y = self.segment_handle(result_image, self.box, 1 / 4)
                        self.get_logger().info(f'\033[1;32m[DEBUG] Point for ColorPicker: x={point.x}, y={point.y}\033[0m')

                        # self.get_logger().info(f'222222222222 {point.x} {point.y}')
                        # cv2.circle(result_image, (int(point.x*result_image.shape[1]), int(point.y*result_image.shape[0])), 5, (0, 0, 255), -1)
                        # cv2.imshow('result_image', result_image)
                        # cv2.waitKey(5000)
                        self.color_picker = ColorPicker(point, 10)
                        self.box = []
                    elif self.start_place:
                        point = Point()
                        point.x, point.y = self.segment_handle(result_image, self.box, 3 / 4)
                        self.color_picker = ColorPicker(point, 10)
                        self.box = []
                
            if self.color_picker is not None:  # 拾取器存在(color pick exists)
                target_color, result_image = self.color_picker(image, result_image)
                if target_color is not None:
                    self.color_picker = None
                    self.min_color = [int(target_color[0][0] - 50 * self.threshold * 2),
                                 int(target_color[0][1] - 50 * self.threshold),
                                 int(target_color[0][2] - 50 * self.threshold)]
                    self.max_color = [int(target_color[0][0] + 50 * self.threshold * 2),
                                 int(target_color[0][1] + 50 * self.threshold),
                                 int(target_color[0][2] + 50 * self.threshold)]
                    # self.get_logger().info(f'{self.min_color} {self.max_color}')
            else:
                if self.max_color and self.min_color:
                    if self.start_pick:
                        self.stop = True
                        result_image = self.pick_handle(image)
                    elif self.start_place:
                        self.stop = True
                        result_image = self.place_handle(image)
                    else:
                        if self.stop:
                            self.stop = False
                        result_image = image
                
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
    node = AutomaticPickNode('automatic_pick')
    rclpy.spin(node)
    node.destroy_node()
 
if __name__ == "__main__":
    main()

