#!/usr/bin/env python3
"""
这个程序实现了人脸追踪功能
运行现象：桌面显示识别结果画面， 将识别到的人脸在画面中框出
        机器人的头部云台跟随最靠近画面中心的人脸移动

"""
import cv2
import gc
import rospy
import time
import numpy as np
import mediapipe as mp
from sensor_msgs.msg import Image
from vision_utils import fps, box_center, distance
from utils import show_faces, mp_face_location
from rospider_sdk import pid, serial_servo


class FaceTrackingNode:
    def __init__(self):
        rospy.init_node("face_tracking_node", log_level=rospy.INFO) # 初始化节点

        # 人脸检测器
        self.face_detector = mp.solutions.face_detection.FaceDetection(
            # model_selection=0,
            min_detection_confidence=0.5,
        )
        # self.drawing = mp.solutions.drawing_utils

        self.pitch, self.yaw = 500, 500 # 云台俯仰角，偏航角
        self.pid_pitch = pid.PID(25, 0, 4) # 云台俯仰角 pid 
        self.pid_yaw = pid.PID(10, 0, 4) # 云台偏航角 pid
        serial_servo.set_position(19, self.yaw, 1000)

        self.fps = fps.FPS() # 帧率统计器
        self.detected_face = 0 # 连续识别到了人脸的帧数

        # 订阅和发布图像
        self.camera_rgb_prefix = rospy.get_param('/camera_rgb_prefix', 'camera/rgb')
        self.image_sub = rospy.Subscriber(self.camera_rgb_prefix + '/image_raw', Image, self.image_callback, queue_size=2)
        print(self.camera_rgb_prefix)
        self.timestamp = time.time()

    def image_callback(self, ros_image):
        rospy.logdebug('Received an image! ')
        # 原始 RGB 画面
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data) 
        result_image = np.copy(rgb_image)

        results = self.face_detector.process(rgb_image) # 识别图像中的人脸及人脸关键点
        boxes, keypoints = mp_face_location(results, rgb_image) # 获取人脸识别的输出数据并将归一化数据转为像素坐标
        o_h, o_w = rgb_image.shape[:2]

        if len(boxes) > 0:
            self.detected_face += 1 
            self.detected_face = min(self.detected_face, 20) # 让计数总是不大于20

            # 连续 5 帧识别到了人脸就开始追踪, 避免误识别
            if self.detected_face >= 5:
                center = [box_center(box) for box in boxes] # 计算所有人脸的中心坐标
                dist = [distance(c, (o_w / 2, o_h / 2)) for c in center] # 计算所有人脸中心坐标到画面中心的距离
                face = min(zip(boxes, center, dist), key=lambda k: k[2]) # 找出到画面中心距离最小的人脸

                # 计算要追踪的人脸距画面中心的x轴距离，并归一化(-1~+1)。
                c_x, c_y = face[1]
                dist_x = 1.0 - c_x / (o_w / 2) #由这个简化来 (self.detect_w / 2.0 - c_x) / (self.detect_w / 2)
                dist_y = 1.0 - c_y / (o_h / 2) #由这个简化来 (self.detect_h / 2.0 - c_y) / (self.detect_h / 2)

                if abs(dist_y) > 0.3:
                    self.pid_pitch.SetPoint = 0
                    self.pid_pitch.update(dist_y) # 更新俯仰角 pid 控制器
                    # 获取新的俯仰角并限制运动范围
                    self.pitch = min(max(self.pitch - self.pid_pitch.output, 0), 1000)

                if abs(dist_x) > 0.2:
                    self.pid_yaw.SetPoint = 0
                    self.pid_yaw.update(dist_x) # 更新偏航角 pid 控制器
                    # 获取新的偏航角并限制运动范围
                    self.yaw = min(max(self.yaw - self.pid_yaw.output, 0),  1000)

                serial_servo.set_position(19, int(self.yaw), 20)

        else: # 这里是没有识别到人脸的处理
            if self.detected_face > 0:
                self.detected_face -= 1
            else:
                self.pid_pitch.clear()
                self.pid_yaw.clear()

        result_image = show_faces(rgb_image, result_image, boxes, keypoints) # 在画面中显示识别到的人脸和脸部关键点
        self.fps.update() # 更新 fsp 统计器
        result_image = self.fps.show_fps(result_image) # 画面中显示fps
        cv2.imshow('image', cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR))
        cv2.waitKey(1)
        if time.time() - self.timestamp > 0.3:
            self.timestamp = time.time()
            gc.collect()


if __name__ == "__main__":
    try:
        face_tracking_node = FaceTrackingNode()
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))

