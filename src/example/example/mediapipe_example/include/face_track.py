#!/usr/bin/env python3
# encoding: utf-8
#人脸追踪
import os
import cv2
import time
import queue
import rclpy
import threading
import numpy as np
import sdk.pid as pid
import mediapipe as mp
from sdk import fps
from rclpy.node import Node
from cv_bridge import CvBridge
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
from controller import step_controller
from arm_kinematics_msgs.srv import SetRobotPose
from rclpy.executors import MultiThreadedExecutor
from servo_controller_msgs.msg import ServosPosition
from rclpy.callback_groups import ReentrantCallbackGroup
from arm_kinematics.kinematics_control import set_pose_target
from servo_controller.bus_servo_control import set_servo_position
from sdk.common import show_faces, mp_face_location, box_center, distance

class FaceTrackingNode(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.face_detector = mp.solutions.face_detection.FaceDetection(
            min_detection_confidence=0.3,
        )
        self.running = True
        self.bridge = CvBridge()
        self.fps = fps.FPS()
        self.controller = step_controller.StepController()
        self.image_queue = queue.Queue(maxsize=2)
        
        self.z_dis = 0.28
        self.y_dis = 500
        self.x_init = 0.18
        self.pid_z = pid.PID(0.00006, 0.0, 0.0)
        self.pid_y = pid.PID(0.055, 0.0, 0.0)
        self.detected_face = 0 
        self.joints_pub = self.create_publisher(ServosPosition, '/servo_controller', 1) # 舵机控制

        self.image_sub = self.create_subscription(Image, '/depth_cam/rgb/image_raw', self.image_callback, 1)  # 摄像头订阅(subscribe to the camera)

        self.result_publisher = self.create_publisher(Image, '~/image_result', 1)  # 图像处理结果发布(publish the image processing result)
        timer_cb_group = ReentrantCallbackGroup()
        self.create_service(Trigger, '~/start', self.start_srv_callback) # 进入玩法
        self.create_service(Trigger, '~/stop', self.stop_srv_callback, callback_group=timer_cb_group) # 退出玩法
        self.client = self.create_client(Trigger, '/controller_manager/init_finish')
        self.client.wait_for_service()
        self.client = self.create_client(Trigger, '/arm_kinematics/init_finish')
        self.client.wait_for_service()

        self.kinematics_client = self.create_client(SetRobotPose, '/arm_kinematics/set_pose_target')
        self.kinematics_client.wait_for_service()

        self.timer = self.create_timer(0.0, self.init_process, callback_group=timer_cb_group)

    def init_process(self):
        self.timer.cancel()

        self.init_action()
        self.start_srv_callback(Trigger.Request(), Trigger.Response())

        threading.Thread(target=self.main, daemon=True).start()
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

    def get_node_state(self, request, response):
        response.success = True
        return response

    def shutdown(self, signum, frame):
        self.running = False

    def init_action(self):
        self.controller.set_build_in_pose('DEFAULT_POSE', 1)

        msg = set_pose_target([self.x_init, 0.0, self.z_dis], 0.0, [-90.0, 90.0], 1.0)
        res = self.send_request(self.kinematics_client, msg)
        if res.pulse:
            servo_data = res.pulse
            set_servo_position(self.joints_pub, 1.5, ((24, 500), (23, 500), (22, servo_data[3]), (21, servo_data[2]), (20, servo_data[1]), (19, servo_data[0])))
            time.sleep(1.8)

    def send_request(self, client, msg):
        future = client.call_async(msg)
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()


    def start_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "start face track")
        
        self.start = True
        response.success = True
        response.message = "start"
        return response

    def stop_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "stop face track")
        self.start = False
        res = self.send_request(ColorDetect.Request())
        if res.success:
            self.get_logger().info('\033[1;32m%s\033[0m' % 'set face success')
        else:
            self.get_logger().info('\033[1;32m%s\033[0m' % 'set face fail')
        response.success = True
        response.message = "stop"
        return response
    
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
            bgr_image = self.image_queue.get()

            result_image = np.copy(bgr_image)
            results = self.face_detector.process(bgr_image)
            boxes, keypoints = mp_face_location(results, bgr_image)
            o_h, o_w = bgr_image.shape[:2]
            if len(boxes) > 0:
                self.detected_face += 1 
                self.detected_face = min(self.detected_face, 20) # 让计数总是不大于20(ensure that the count is never greater than 20)

                # 连续 5 帧识别到了人脸就开始追踪, 避免误识别(start tracking if a face is detected in five consecutive frames to avoid false positives)
                if self.detected_face >= 5:
                    center = [box_center(box) for box in boxes] # 计算所有人脸的中心坐标(calculate the center coordinate of all human faces)
                    dist = [distance(c, (o_w / 2, o_h / 2)) for c in center] # 计算所有人脸中心坐标到画面中心的距离(calculate the distance from the center of each detected face to the center of the screen)
                    face = min(zip(boxes, center, dist), key=lambda k: k[2]) # 找出到画面中心距离最小的人脸(identify the face with the minimum distance to the center of the screen)

                    center_x, center_y = face[1]
                    t1 = time.time()
                    self.pid_y.SetPoint = result_image.shape[1]/2 
                    self.pid_y.update(center_x)
                    self.y_dis += self.pid_y.output
                    if self.y_dis < 200:
                        self.y_dis = 200
                    if self.y_dis > 800:
                        self.y_dis = 800

                    self.pid_z.SetPoint = result_image.shape[0]/2 
                    self.pid_z.update(center_y)
                    self.z_dis += self.pid_z.output
                    if self.z_dis > 0.30:
                        self.z_dis = 0.30
                    if self.z_dis < 0.23:
                        self.z_dis = 0.23
                    msg = set_pose_target([self.x_init, 0.0, self.z_dis], 0.0, [-180.0, 180.0], 1.0)
                    res = self.send_request(self.kinematics_client, msg)
                    t2 = time.time()
                    t = t2 - t1
                    if t < 0.02:
                        time.sleep(0.02 - t)
                    if res.pulse:
                        servo_data = res.pulse
                        set_servo_position(self.joints_pub, 0.02, ((24, 500), (23, 500), (22, servo_data[3]), (21, servo_data[2]), (20, servo_data[1]), (19, int(self.y_dis))))
                    else:
                        set_servo_position(self.joints_pub, 0.02, ((19, int(self.y_dis)), ))

                result_image = show_faces( result_image, bgr_image, boxes, keypoints) # 在画面中显示识别到的人脸和脸部关键点(display the detected faces and facial key points on the screen)
            else: # 这里是没有识别到人脸的处理(here is the processing for when no face is detected)         
                if self.detected_face > 0:
                    self.detected_face -= 1
                else:
                    self.pid_z.clear()
                    self.pid_y.clear()


            self.result_publisher.publish(self.bridge.cv2_to_imgmsg(result_image, "bgr8"))
            self.fps.update()
            self.fps.show_fps(result_image)
            cv2.imshow("result", result_image)
            cv2.waitKey(1)

        self.init_action()
        rclpy.shutdown()


def main():
    node = FaceTrackingNode('face_tracking')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()

if __name__ == "__main__":
    main()
