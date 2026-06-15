#!/usr/bin/python3
# coding=utf8
# @data:2023/12/25
# 通过深度图识别物体进行分类(classify objects through depth map recognition)
# 机械臂向下识别(mechanical arm recognition downwards)
# 可以识别长方体，球，圆柱体，以及他们的颜色(it can recognize cuboids, spheres, cylinders, and their respective colors)
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
from interfaces.srv import SetStringList
from std_srvs.srv import Trigger, SetBool
from xf_mic_asr_offline import voice_play
from arm_kinematics import kinematics_control
from sensor_msgs.msg import Image, CameraInfo
from rclpy.executors import MultiThreadedExecutor
from servo_controller_msgs.msg import ServosPosition
from ros_robot_controller_msgs.msg import BuzzerState
from rclpy.callback_groups import ReentrantCallbackGroup
from arm_kinematics_msgs.srv import GetRobotPose, SetRobotPose, SetJointValue
from servo_controller.bus_servo_control import set_servo_position
from example.rgbd_example.include.position_change_detect import position_reorder
from servo_controller.action_group_controller import ActionGroupController

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
    pick_offset = [0.01, 0.01, 0.0, -0.01, 0.0]  # x1, x2, y1, y2, z
    '''
                
                 x1(+)
        y1(+)   center    y2(-)
                  x2

                  arm
                  car
    '''

    def __init__(self, name):
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.fps = fps.FPS()
        self.moving = False
        self.count = 0
        self.running = True
        self.start = False
        self.shapes = None
        self.colors = None
        self.target_shapes = ''
        self.roi = [100, 300, 170, 470]
        self.endpoint = None
        self.last_position = 0, 0
        self.last_object_info_list = []
        signal.signal(signal.SIGINT, self.shutdown)
        self.image_queue = queue.Queue(maxsize=2)
        self.debug = self.get_parameter('debug').value
        self.plane_distance = self.get_parameter('plane_distance').value
        self.lab_data = common.get_yaml_data("/home/ubuntu/software/lab_tool/lab_config.yaml")
        self.joints_pub = self.create_publisher(ServosPosition, '/servo_controller', 1)
        self.buzzer_pub = self.create_publisher(BuzzerState, '/ros_robot_controller/set_buzzer', 1)
        
        self.create_service(Trigger, '~/start', self.start_srv_callback)
        self.create_service(Trigger, '~/stop', self.stop_srv_callback)
        self.create_service(SetStringList, '~/set_shape', self.set_shape_srv_callback)
        self.create_service(SetStringList, '~/set_color', self.set_color_srv_callback)
        
        rgb_sub = message_filters.Subscriber(self, Image, '/depth_cam/rgb/image_raw')
        depth_sub = message_filters.Subscriber(self, Image, '/depth_cam/depth/image_raw')
        info_sub = message_filters.Subscriber(self, CameraInfo, '/depth_cam/depth/camera_info')

        # 同步时间戳, 时间允许有误差在0.03s(synchronize timestamps, allowing a time discrepancy of up to 0.03 seconds)
        sync = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub, info_sub], 3, 0.02)
        sync.registerCallback(self.multi_callback)
        self.client = self.create_client(Trigger, '/controller_manager/init_finish')
        self.client.wait_for_service()

       
        timer_cb_group = ReentrantCallbackGroup()
        self.set_joint_value_target_client = self.create_client(SetJointValue, '/arm_kinematics/set_joint_value_target', callback_group=timer_cb_group)
        self.set_joint_value_target_client.wait_for_service()
        self.kinematics_client = self.create_client(SetRobotPose, '/arm_kinematics/set_pose_target')
        self.kinematics_client.wait_for_service()

        self.controller = ActionGroupController(self.create_publisher(ServosPosition, 'servo_controller', 1), '/home/ubuntu/software/actionset_editor/ActionGroups')

        self.timer = self.create_timer(0.0, self.init_process, callback_group=timer_cb_group)

    def init_process(self):
        self.timer.cancel()
        self.controller.run_action("init")
        self.goto_default()

        if self.get_parameter('start').value:
            if self.get_parameter('category').value == 'shape':
                msg = SetStringList.Request()
                msg.data = ['sphere', 'cuboid', 'cylinder']
                self.set_shape_srv_callback(msg, SetStringList.Response())
            else:
                msg = SetStringListi.Request()
                msg.data = ['red', 'green', 'blue']
                self.set_color_srv_callback(msg, SetStringList.Response())

        threading.Thread(target=self.main, daemon=True).start()
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

    def get_node_state(self, request, response):
        response.success = True
        return response

    def shutdown(self, signum, frame):
        self.running = False
        self.get_logger().info('\033[1;32m%s\033[0m' % "shutdown")

    def send_request(self, client, msg):
        future = client.call_async(msg)
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()

    def set_shape_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "set_shape")
        self.colors = None
        self.shapes = request.data
        self.start = True
        response.success = True
        response.message = "set_shape"
        return response

    def set_color_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "set_color")
        self.shapes = None
        self.colors = request.data
        self.start = True
        response.success = True
        response.message = "set_color"
        return response

    def start_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "start")
        self.start = True
        response.success = True
        response.message = "start"
        return response

    def stop_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "stop")
        self.start = False
        self.colors = None
        self.shapes = None
        self.moving = False
        self.count = 0
        self.target_shapes = ''
        self.last_position = 0, 0
        self.last_object_info_list = []
        response.success = True
        response.message = "stop"
        return response

    def goto_default(self):
        msg = kinematics_control.set_joint_value_target([500.0, 470.0, 220.0, 90.0, 500.0])
        endpoint = self.send_request(self.set_joint_value_target_client, msg)
        pose_t = endpoint.pose.position
        pose_r = endpoint.pose.orientation
        set_servo_position(self.joints_pub, 1, ((19, 500), (20, 470), (21, 220), (22, 90), (23, 500), (24, 700)))
        self.endpoint = common.xyz_quat_to_mat([pose_t.x, pose_t.y, pose_t.z], [pose_r.w, pose_r.x, pose_r.y, pose_r.z])

    def move(self, obejct_info):
        shape, pose_t = obejct_info[:2]
        color, angle = obejct_info[-2:]
        msg = BuzzerState()
        msg.freq = 1900
        msg.on_time = 0.2
        msg.off_time = 0.01
        msg.repeat = 1
        self.buzzer_pub.publish(msg)
        time.sleep(1)
        if 'sphere' in shape:
            offset_z = -0.015 + self.pick_offset[-1]
        elif 'cylinder' in shape:
            offset_z = 0.02 + self.pick_offset[-1]
        else:
            offset_z = 0.01 + self.pick_offset[-1]
        if pose_t[0] > 0.21:
            offset_x = self.pick_offset[0]
        else:
            offset_x = self.pick_offset[1]
        if pose_t[1] > 0:
            offset_y = self.pick_offset[2]
        else:
            offset_y = self.pick_offset[3]
        pose_t[0] += offset_x
        pose_t[1] += offset_y
        pose_t[2] += offset_z
        msg = kinematics_control.set_pose_target(pose_t, 85)
        res1 = self.send_request(self.kinematics_client, msg)
        if res1.pulse:
            servo_data = res1.pulse
            set_servo_position(self.joints_pub, 1.5, ((19, servo_data[0]), (20, servo_data[1]), (21, servo_data[2]), (22, servo_data[3]), (23, servo_data[4])))
            time.sleep(1.5)
        pose_t[2] -= 0.05
        msg = kinematics_control.set_pose_target(pose_t, 85)
        res2 = self.send_request(self.kinematics_client, msg)
        if angle != 0:
            if 'sphere' in shape or ('cylinder' in shape and 'cylinder_horizontal_' not in shape):
                angle = 500
            else:
                angle = angle % 180
                angle = angle - 180 if angle > 90 else (angle + 180 if angle < -90 else angle)

                angle = 500 + int(1000 * (angle + res2.rpy[-1]) / 240)

        else:
            angle = 500
        if res2.pulse:
            servo_data = res2.pulse
            set_servo_position(self.joints_pub, 0.5, ((23, angle),))
            time.sleep(0.5)
            set_servo_position(self.joints_pub, 1, ((19, servo_data[0]), (20, servo_data[1]), (21, servo_data[2]), (22, servo_data[3]), (23, angle)))
            time.sleep(1)
            if shape == "sphere":
                set_servo_position(self.joints_pub, 0.6, ((24, 600),))
            else:
                set_servo_position(self.joints_pub, 0.6, ((24, 470),))
            time.sleep(0.6)
        if res1.pulse:
            servo_data = res1.pulse
            set_servo_position(self.joints_pub, 1, ((19, servo_data[0]), (20, servo_data[1]), (21, servo_data[2]), (22, servo_data[3]), (23, angle)))
            time.sleep(1)
        set_servo_position(self.joints_pub, 1, ((19, 500), (20, 640), (21, 150), (22, 130), (23, 500), (24, 470)))
        time.sleep(1)
        if self.colors is None:
            self.get_logger().info('shape: %s' % shape.split("_")[0])
            if "sphere" in shape:
                self.controller.run_action("target_1")
            if "cylinder" in shape:
                self.controller.run_action("target_2")
            if "cuboid" in shape:
                self.controller.run_action("target_3")
        else:
            color = self.color_comparison(color)
            self.get_logger().info('color: %s' % color)
            if "red" == color:
                self.controller.run_action("target_1")
            if "blue" == color:
                self.controller.run_action("target_3")
        self.goto_default()
        time.sleep(2)
        self.moving = False

    def multi_callback(self, ros_rgb_image, ros_depth_image, depth_camera_info):
        if self.image_queue.full():
            # 如果队列已满，丢弃最旧的图像(if the queue is full, discard the oldest image)
            self.image_queue.get()
        # 将图像放入队列(put the image into the queue)
        self.image_queue.put((ros_rgb_image, ros_depth_image, depth_camera_info))

    def cal_position(self, x, y, depth, intrinsic_matrix):
        position = depth_pixel_to_camera([x, y, depth / 1000], intrinsic_matrix)
        position[0] -= 0.01
        pose_end = np.matmul(self.hand2cam_tf_matrix, common.xyz_euler_to_mat(position, (0, 0, 0)))
        world_pose = np.matmul(self.endpoint, pose_end)
        pose_t, pose_r = common.mat_to_xyz_euler(world_pose)
        return pose_t

    def get_min_distance(self, depth_image):
        ih, iw = depth_image.shape[:2]
        # 屏蔽掉一些区域，降低识别条件，使识别跟可靠(mask certain areas to lower recognition conditions and enhance reliability)
        depth_image[:, :self.roi[2]] = np.array([[1000, ] * self.roi[2]] * ih)
        depth_image[:, self.roi[3]:] = np.array([[1000, ] * (iw - self.roi[3])] * ih)
        depth_image[self.roi[1]:, :] = np.array([[1000, ] * iw] * (ih - self.roi[1]))
        depth_image[:self.roi[0], :] = np.array([[1000, ] * iw] * self.roi[0])
        depth = np.copy(depth_image).reshape((-1,))
        depth[depth <= 0] = 55555  # 距离为0可能是进入死区，或者颜色问题识别不到，将距离赋一个大值(a distance of 0 may indicate entry into a dead zone or failure to recognize due to color issues. Assign a large value to the distance)

        min_index = np.argmin(depth)  # 距离最小的像素(the pixel with the minimum distance)
        min_y = min_index // iw
        min_x = min_index - min_y * iw

        min_dist = depth_image[min_y, min_x]  # 获取距离摄像头最近的物体的距离(get the distance of the object that is closest to the camera)
        return min_dist

    def get_contours(self, depth_image, min_dist):
        depth_image = np.where(depth_image > self.plane_distance - 10, 0, depth_image)  
        depth_image = np.where(depth_image > min_dist + 40, 0, depth_image)  # 将深度值大于最小距离15mm的像素置0(set pixels with depth values greater than the minimum distance of 15mm to 0)
        sim_depth_image_sort = np.clip(depth_image, 0, self.plane_distance - 10).astype(np.float64) / (self.plane_distance - 10) * 255
        depth_gray = sim_depth_image_sort.astype(np.uint8)
        _, depth_bit = cv2.threshold(depth_gray, 1, 255, cv2.THRESH_BINARY)

        contours, hierarchy = cv2.findContours(depth_bit, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        return contours

    def shape_recognition(self, rgb_image, depth_image, depth_color_map, intrinsic_matrix, min_dist):
        object_info_list = []
        image_height, image_width = depth_image.shape[:2]
        if min_dist <= self.plane_distance - 10:  # 大于这个值说明已经低于地面了，可能检测有误(If it is greater than 300, it means the object is lower than the gound, and the detection may be not correct)
            sphere_index = 0
            cuboid_index = 0
            cylinder_index = 0
            cylinder_horizontal_index = 0
            contours = self.get_contours(depth_image, min_dist)
            
            for obj in contours:
                area = cv2.contourArea(obj)
                # self.get_logger().info(srt(area))
                if area < 300:
                    continue
                # cv2.drawContours(depth_color_map, obj, -1, (255, 255, 0), 10)  # 绘制轮廓线(draw contour line)
                perimeter = cv2.arcLength(obj, True)  # 计算轮廓周长(calculate the perimeter of the contour)
                approx = cv2.approxPolyDP(obj, 0.035 * perimeter, True)  # 获取轮廓角点坐标(get the coordinates of the contour corner points)
                # cv2.drawContours(depth_color_map, approx, -1, (255, 0, 0), 4)  # 绘制轮廓线(get contour line)

                CornerNum = len(approx)
                (cx, cy), r = cv2.minEnclosingCircle(obj)
                center, (width, height), angle = cv2.minAreaRect(obj)
                if angle < -45:
                    angle += 89
                if width > height and width / height > 1.5:
                    angle = angle + 90
                depth = depth_image[int(cy), int(cx)]
                position = self.cal_position(cx, cy, depth, intrinsic_matrix)
                x, y, w, h = cv2.boundingRect(approx)
                mask = np.full((image_height, image_width), 0, dtype=np.uint8)
                
                cv2.drawContours(mask, [obj], -1, (255), cv2.FILLED)
                
                # 提取mask区域内的深度值
                # 创建一个临时图像，背景设为NaN (Not a Number)，物体区域设为深度值
                depth_values_in_roi = np.where(mask == 255, depth_image, np.nan)

                # 去除距离为0的无效点
                selected_depths = depth_image[mask == 255] # 取出所有物体像素
                selected_depths = selected_depths[selected_depths > 0] # 去除无效深度

                # 计算标准差
                if len(selected_depths) > 0:
                    depth_std = np.std(selected_depths)
                else:
                    depth_std = 0

                objType = None
                if CornerNum > 4:
                    if depth_std > 2.0 :
                        sphere_index += 1
                        angle = 0
                        objType = 'sphere_' + str(sphere_index)
                    else:
                        cylinder_index += 1
                        angle = 0
                        objType = "cylinder_" + str(cylinder_index)
                elif CornerNum == 4:
                    cuboid_index += 1
                    objType = "cuboid_" + str(cuboid_index)


                if objType is not None:
                    object_info_list.append([objType, position, depth, [x, y, w, h, center, width, height], rgb_image[int(center[1]), int(center[0])], angle])

                    cv2.rectangle(depth_color_map, (x, y), (x + w, y + h), (255, 255, 255), 2)
        return object_info_list

    def color_comparison(self, rgb):
        if rgb[0] > rgb[1] and rgb[0] > rgb[2]:
            return 'red'
        elif rgb[2] > rgb[1] and rgb[2] > rgb[1]:
            return 'blue'
        else:
            return None

    def main(self):
        count = 0
        while self.running:
            try:
                ros_rgb_image, ros_depth_image, depth_camera_info = self.image_queue.get(block=True, timeout=1)
            except queue.Empty:
                if not self.running:
                    break
                else:
                    continue
            try:
                rgb_image = np.ndarray(shape=(ros_rgb_image.height, ros_rgb_image.width, 3), dtype=np.uint8, buffer=ros_rgb_image.data)
                depth_image = np.ndarray(shape=(ros_depth_image.height, ros_depth_image.width), dtype=np.uint16, buffer=ros_depth_image.data)
                depth_image = depth_image.copy()
                min_dist = self.get_min_distance(depth_image)
                if self.debug:
                    count += 1
                    self.get_logger().info(str(min_dist))
                    if count > 50:
                        count = 0
                        data = {'/**': {'ros__parameters': {'plane_distance': {}}}}
                        data['/**']['ros__parameters']['plane_distance'] = int(min_dist)
                        common.save_yaml_data(data, os.path.join(
                            os.path.abspath(os.path.join(os.path.split(os.path.realpath(__file__))[0], '../../..')),
                            'config/object_classification_plane_distance.yaml'))
                        msg = BuzzerState()
                        msg.freq = 1900
                        msg.on_time = 0.2
                        msg.off_time = 0.01
                        msg.repeat = 1
                        self.buzzer_pub.publish(msg)
                        self.debug = False
                else:
                    #像素值限制在0到350的范围内, 将深度图像的像素值限制和归一化到0到255的范围内(Limit the pixel values of the depth image within the range of 0 to 350, and normalize the pixel values of the depth image to the range of 0 to 255)
                    sim_depth_image = np.clip(depth_image, 0, 350).astype(np.float64) / 350 * 255
                    
                    depth_color_map = cv2.applyColorMap(sim_depth_image.astype(np.uint8), cv2.COLORMAP_JET)
                    if not self.moving:
                        object_info_list = self.shape_recognition(rgb_image, depth_image, depth_color_map, depth_camera_info.k, min_dist)
                        if self.start:
                            reorder_object_info_list = object_info_list
                            if object_info_list:
                                if self.last_object_info_list:
                                    # 对比上一次的物体的位置来重新排序(reorder based on the contrast of the previous object's position)
                                    reorder_object_info_list = position_reorder(object_info_list, self.last_object_info_list, 20)
                            if reorder_object_info_list:
                                if not self.target_shapes:
                                    if self.shapes is not None:
                                        indices = [i for i, info in enumerate(reorder_object_info_list) if info[0].split('_')[0] in self.shapes]
                                    else:
                                        indices = [i for i, info in enumerate(reorder_object_info_list) if self.color_comparison(info[-2]) in self.colors]
                                    if indices:
                                        min_depth_index = min(indices, key=lambda i: reorder_object_info_list[i][2])
                                        self.target_shapes = reorder_object_info_list[min_depth_index][0].split('_')[0]
                                else:

                                    target_index = [i for i, info in enumerate(reorder_object_info_list) if info[0].split('_')[0] == self.target_shapes]
                                    if target_index:
                                        target_index = target_index[0]
                                        obejct_info = reorder_object_info_list[target_index]
                                        x, y, w, h, center, width, height = obejct_info[3]
                                        angle = obejct_info[-1]
                                        cv2.putText(depth_color_map, self.target_shapes, (x + w // 2, y + (h // 2) - 10), cv2.FONT_HERSHEY_COMPLEX, 1.0,
                                                    (0, 0, 0), 2, cv2.LINE_AA)
                                        cv2.putText(depth_color_map, self.target_shapes, (x + w // 2, y + (h // 2) - 10), cv2.FONT_HERSHEY_COMPLEX, 1.0,
                                                    (255, 255, 255), 1)
                                        cv2.drawContours(depth_color_map, [np.int0(cv2.boxPoints((center, (width, height), angle)))], -1,
                                                         (0, 0, 255), 2, cv2.LINE_AA)
                                        position = obejct_info[1]
                                        e_distance = round(math.sqrt(pow(self.last_position[0] - position[0], 2)) + math.sqrt(
                                            pow(self.last_position[1] - position[1], 2)), 5)
                                        if e_distance <= 0.005:
                                            self.count += 1
                                        else:
                                            self.count = 0
                                        if self.count > 5:
                                            self.count = 0
                                            self.target_shapes = None
                                            self.moving = True

                                            threading.Thread(target=self.move, args=(obejct_info,)).start()
                                        self.last_position = position
                                    else:
                                        self.target_shapes = None

                            self.last_object_info_list = reorder_object_info_list

                    cv2.rectangle(rgb_image, (self.roi[2], self.roi[0]), (self.roi[3], self.roi[1]), (255, 255, 0), 1)
                    # bgr_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)

                    self.fps.update()
                    result_image = np.concatenate([depth_color_map, rgb_image], axis=1)
                    cv2.imshow("depth", result_image)
                    key = cv2.waitKey(1)
                    if key == ord('q') or key == 27:  # 按q或者esc退出(press q or esc to exit)
                        self.running = False
            except Exception as e:
                self.get_logger().info(str(e))
        rclpy.shutdown()

def main():
    node = ObjectClassificationNode('object_classification')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()

if __name__ == "__main__":
    main()

