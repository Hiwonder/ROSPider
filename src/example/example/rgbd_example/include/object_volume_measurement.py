#!/usr/bin/python3
# coding=utf8
# 通过深度图识别物体并测量体积 (Classify objects and measure volume through depth map)
import os
import cv2
import time
import math
import rclpy
import queue
import signal
import threading
import numpy as np
import message_filters
from rclpy.node import Node
from sdk import common, fps
from controller import controller_client 
from interfaces.srv import SetStringList
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image, CameraInfo
from rclpy.executors import MultiThreadedExecutor
from servo_controller_msgs.msg import ServosPosition
from ros_robot_controller_msgs.msg import BuzzerState
from rclpy.callback_groups import ReentrantCallbackGroup
from arm_kinematics import kinematics_control
from arm_kinematics_msgs.srv import SetRobotPose, SetJointValue
from servo_controller.bus_servo_control import set_servo_position
from example.rgbd_example.include.position_change_detect import position_reorder

def depth_pixel_to_camera(pixel_coords, intrinsic_matrix):
    fx, fy, cx, cy = intrinsic_matrix[0], intrinsic_matrix[4], intrinsic_matrix[2], intrinsic_matrix[5]
    px, py, pz = pixel_coords
    x = (px - cx) * pz / fx
    y = (py - cy) * pz / fy
    z = pz
    return np.array([x, y, z])

class ObjectClassificationNode(Node):
    hand2cam_tf_matrix = [
        [0.0, 0.0, 1.0, -0.101],
        [-1.0, 0.0, 0.0, 0.011],
        [0.0, -1.0, 0.0, 0.045],
        [0.0, 0.0, 0.0, 1.0]
    ]

    def __init__(self, name):
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.fps = fps.FPS()
        self.moving = False
        self.count = 0
        self.running = True
        self.start = False
        self.shapes = None
        self.target_shapes = ''
        
        # 识别区域 ROI [y_min, y_max, x_min, x_max]
        self.roi = [50, 350, 150, 500] 
        
        self.endpoint = None
        self.last_position = 0, 0
        self.last_object_info_list = []
        signal.signal(signal.SIGINT, self.shutdown)
        self.image_queue = queue.Queue(maxsize=2)
        self.debug = self.get_parameter('debug').value
        self.plane_distance = self.get_parameter('plane_distance').value
        self.joints_pub = self.create_publisher(ServosPosition, '/servo_controller', 1)
        self.buzzer_pub = self.create_publisher(BuzzerState, '/ros_robot_controller/set_buzzer', 1)
        
        self.create_service(Trigger, '~/start', self.start_srv_callback)
        self.create_service(Trigger, '~/stop', self.stop_srv_callback)
        self.create_service(SetStringList, '~/set_shape', self.set_shape_srv_callback)
        
        rgb_sub = message_filters.Subscriber(self, Image, '/depth_cam/rgb/image_raw')
        depth_sub = message_filters.Subscriber(self, Image, '/depth_cam/depth/image_raw')
        info_sub = message_filters.Subscriber(self, CameraInfo, '/depth_cam/depth/camera_info')
        self.controller = controller_client.ControllerClient()

        sync = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub, info_sub], 3, 0.02)
        sync.registerCallback(self.multi_callback)
        
        self.client = self.create_client(Trigger, '/controller_manager/init_finish')
        self.client.wait_for_service()

        timer_cb_group = ReentrantCallbackGroup()
        self.set_joint_value_target_client = self.create_client(SetJointValue, '/arm_kinematics/set_joint_value_target', callback_group=timer_cb_group)
        self.set_joint_value_target_client.wait_for_service()
        self.kinematics_client = self.create_client(SetRobotPose, '/arm_kinematics/set_pose_target')
        self.kinematics_client.wait_for_service()

        self.timer = self.create_timer(0.0, self.init_process, callback_group=timer_cb_group)

    def init_process(self):
        self.timer.cancel()
        self.goto_default()
        self.controller.traveling(gait=-1, time=1, steps=0)
        time.sleep(1)

        if self.get_parameter('start').value:
            msg = SetStringList.Request()
            msg.data = ['sphere', 'cuboid', 'cylinder']
            self.set_shape_srv_callback(msg, SetStringList.Response())

        threading.Thread(target=self.main, daemon=True).start()
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'Node Started')

    def get_node_state(self, request, response):
        response.success = True
        return response

    def shutdown(self, signum, frame):
        self.running = False

    def send_request(self, client, msg):
        future = client.call_async(msg)
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()

    def set_shape_srv_callback(self, request, response):
        self.shapes = request.data
        self.start = True
        response.success = True
        response.message = "set_shape"
        return response

    def start_srv_callback(self, request, response):
        self.start = True
        response.success = True
        response.message = "start"
        return response

    def stop_srv_callback(self, request, response):
        self.start = False
        self.shapes = None
        self.moving = False
        self.target_shapes = ''
        response.success = True
        response.message = "stop"
        return response

    def goto_default(self):
        msg = kinematics_control.set_joint_value_target([500.0, 500.0, 130.0, 130.0, 500.0])
        endpoint = self.send_request(self.set_joint_value_target_client, msg)
        pose_t = endpoint.pose.position
        pose_r = endpoint.pose.orientation
        set_servo_position(self.joints_pub, 1, ((19, 500), (20, 500), (21, 130), (22, 130), (23, 500), (24, 700)))
        self.endpoint = common.xyz_quat_to_mat([pose_t.x, pose_t.y, pose_t.z], [pose_r.w, pose_r.x, pose_r.y, pose_r.z])
    
    def calculate_volume(self, obj_type, obj_info, intrinsic_matrix):
        fx, fy = intrinsic_matrix[0], intrinsic_matrix[4]
        dist_to_obj = obj_info[2]
        if dist_to_obj <= 0: return 0.0

        pixel_w = obj_info[3][5]
        pixel_h = obj_info[3][6]
        
        real_w_mm = (pixel_w * dist_to_obj) / fx
        real_h_mm = (pixel_h * dist_to_obj) / fy
        thickness_mm = max(0, self.plane_distance - dist_to_obj)

        if 'sphere' in obj_type:
            radius_mm = (real_w_mm + real_h_mm) / 4.0
            volume_mm3 = (4/3) * math.pi * (radius_mm ** 3)
        elif 'cuboid' in obj_type:
            volume_mm3 = real_w_mm * real_h_mm * thickness_mm
        elif 'cylinder' in obj_type:
            radius_mm = real_w_mm / 2.0
            volume_mm3 = math.pi * (radius_mm ** 2) * thickness_mm
        else:
            volume_mm3 = 0.0
            
        return volume_mm3 / 1000.0 # cm^3

    def multi_callback(self, ros_rgb_image, ros_depth_image, depth_camera_info):
        if self.image_queue.full(): self.image_queue.get()
        self.image_queue.put((ros_rgb_image, ros_depth_image, depth_camera_info))

    def cal_position(self, x, y, depth, intrinsic_matrix):
        position = depth_pixel_to_camera([x, y, depth / 1000], intrinsic_matrix)
        pose_end = np.matmul(self.hand2cam_tf_matrix, common.xyz_euler_to_mat(position, (0, 0, 0)))
        world_pose = np.matmul(self.endpoint, pose_end)
        pose_t, _ = common.mat_to_xyz_euler(world_pose)
        return pose_t

    def get_stable_distance(self, depth_image):
        ih, iw = depth_image.shape[:2]
        roi_depths = depth_image[self.roi[0]:self.roi[1], self.roi[2]:self.roi[3]]
        valid_depths = roi_depths[(roi_depths > 0) & (roi_depths < 1000)]
        if len(valid_depths) > 0:
            return float(np.median(valid_depths))
        else:
            return float(self.plane_distance)

    def get_contours(self, depth_image, stable_dist):
        """【优化】提取轮廓，仅限 ROI 区域"""
        ih, iw = depth_image.shape[:2]
        
        # 1. 创建 ROI 掩码
        roi_mask = np.zeros((ih, iw), dtype=np.uint8)
        roi_mask[self.roi[0]:self.roi[1], self.roi[2]:self.roi[3]] = 255
        
        # 2. 深度阈值处理
        # 过滤地面以下的杂讯
        processed_depth = np.where(depth_image > self.plane_distance - 10, 0, depth_image)  
        # 过滤物体高度以外的背景
        processed_depth = np.where(processed_depth > stable_dist + 40, 0, processed_depth)
        
        # 3. 强制将 ROI 之外的所有区域置为 0
        processed_depth = cv2.bitwise_and(processed_depth.astype(np.uint16), processed_depth.astype(np.uint16), mask=roi_mask)

        # 4. 归一化和二值化
        sim_depth_image_sort = np.clip(processed_depth, 0, self.plane_distance - 10).astype(np.float64) / (self.plane_distance - 10) * 255
        depth_gray = sim_depth_image_sort.astype(np.uint8)
        _, depth_bit = cv2.threshold(depth_gray, 1, 255, cv2.THRESH_BINARY)
        
        contours, _ = cv2.findContours(depth_bit, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        return contours

    def shape_recognition(self, rgb_image, depth_image, depth_color_map, intrinsic_matrix, stable_dist):
        object_info_list = []
        display_info_list = []
        image_height, image_width = depth_image.shape[:2]
        
        if stable_dist <= self.plane_distance:
            sphere_index, cuboid_index, cylinder_index = 0, 0, 0
            contours = self.get_contours(depth_image, stable_dist)
            
            for obj in contours:
                area = cv2.contourArea(obj)
                if area < 300: continue
                
                perimeter = cv2.arcLength(obj, True)
                approx = cv2.approxPolyDP(obj, 0.035 * perimeter, True)
                CornerNum = len(approx)
                (cx, cy), r = cv2.minEnclosingCircle(obj)
                center, (width, height), angle = cv2.minAreaRect(obj)
                
                if angle < -45: angle += 90
                if width > height and width / height > 1.5: angle += 90

                # 通过Mask获取物体像素
                mask = np.zeros((image_height, image_width), dtype=np.uint8)
                cv2.drawContours(mask, [obj], -1, 255, cv2.FILLED)
                selected_depths = depth_image[mask == 255]
                selected_depths = selected_depths[selected_depths > 0]

                if len(selected_depths) > 0:
                    depth = float(np.median(selected_depths))
                    depth_std = np.std(selected_depths)
                else:
                    continue

                objType = None
                if CornerNum > 4:
                    if depth_std > 2.0:
                        sphere_index += 1
                        objType = 'sphere_' + str(sphere_index)
                    else:
                        cylinder_index += 1
                        objType = "cylinder_" + str(cylinder_index)
                elif CornerNum == 4:
                    cuboid_index += 1
                    objType = "cuboid_" + str(cuboid_index)

                if objType is not None:
                    position = self.cal_position(cx, cy, depth, intrinsic_matrix)
                    x, y, w, h = cv2.boundingRect(approx)
                    volume = self.calculate_volume(objType, [objType, position, depth, [x, y, w, h, center, width, height], None, angle], intrinsic_matrix)
                    object_info_list.append([objType, position, depth, [x, y, w, h, center, width, height], rgb_image[int(cy), int(cx)], angle])
                    
                    # 绘制
                    cv2.putText(depth_color_map, objType, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                    cv2.drawContours(depth_color_map, [np.int0(cv2.boxPoints((center, (width, height), angle)))], -1, (0, 0, 255), 2)
                    display_info_list.append(f"{objType}, Vol: {volume:.2f}cm3, H: {self.plane_distance - depth:.1f}mm")

        # 屏幕左下角实时显示
        for i, info in enumerate(display_info_list):
            cv2.putText(depth_color_map, info, (20, image_height - 20 - i*25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        return object_info_list

    def main(self):
        count = 0
        while self.running:
            try:
                ros_rgb_image, ros_depth_image, depth_camera_info = self.image_queue.get(block=True, timeout=1)
                rgb_image = np.ndarray(shape=(ros_rgb_image.height, ros_rgb_image.width, 3), dtype=np.uint8, buffer=ros_rgb_image.data)
                depth_image = np.ndarray(shape=(ros_depth_image.height, ros_depth_image.width), dtype=np.uint16, buffer=ros_depth_image.data).copy()
                
                # 1. 获取稳定的地面距离
                stable_dist = self.get_stable_distance(depth_image)

                if self.debug:
                    count += 1
                    self.get_logger().info(f"Calibrating Ground: {stable_dist} mm")
                    if count > 50:
                        count = 0
                        data = {'/**': {'ros__parameters': {'plane_distance': int(stable_dist)}}}
                        common.save_yaml_data(data, os.path.join(os.path.abspath(os.path.join(os.path.split(os.path.realpath(__file__))[0], '../../..')), 'config/object_volume_measurement_plane_distance.yaml'))
                        self.buzzer_pub.publish(BuzzerState(freq=1900, on_time=0.2, off_time=0.01, repeat=1))
                        self.debug = False
                        self.plane_distance = int(stable_dist)
                else:
                    # 2. 准备深度图可视化
                    sim_depth_image = np.clip(depth_image, 0, 350).astype(np.float64) / 350 * 255
                    depth_color_map = cv2.applyColorMap(sim_depth_image.astype(np.uint8), cv2.COLORMAP_JET)
                    
                    # 【优化】视觉提示：将 ROI 之外的深度图变暗
                    mask_outside = np.ones(depth_color_map.shape[:2], dtype=np.uint8) * 255
                    mask_outside[self.roi[0]:self.roi[1], self.roi[2]:self.roi[3]] = 0
                    depth_color_map[mask_outside == 255] = (depth_color_map[mask_outside == 255] * 0.3).astype(np.uint8)

                    if not self.moving:
                        # 3. 形状识别
                        object_info_list = self.shape_recognition(rgb_image, depth_image, depth_color_map, depth_camera_info.k, stable_dist)
                        if self.start and object_info_list:
                            reorder_list = position_reorder(object_info_list, self.last_object_info_list, 20) if self.last_object_info_list else object_info_list
                            self.last_object_info_list = reorder_list

                    # 4. 绘制 ROI 框反馈
                    cv2.rectangle(depth_color_map, (self.roi[2], self.roi[0]), (self.roi[3], self.roi[1]), (255, 255, 255), 2)
                    cv2.rectangle(rgb_image, (self.roi[2], self.roi[0]), (self.roi[3], self.roi[1]), (255, 255, 0), 2)
                    
                    self.fps.update()
                    result_view = np.concatenate([depth_color_map, rgb_image], axis=1)
                    cv2.imshow("Object Classification (ROI Mode)", result_view)
                    if cv2.waitKey(1) in [ord('q'), 27]: self.running = False
                    
            except queue.Empty: continue
            except Exception as e: self.get_logger().error(f"Error in Main Loop: {e}")
        rclpy.shutdown()

def main():
    node = ObjectClassificationNode('object_classification')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()

if __name__ == "__main__":
    main()