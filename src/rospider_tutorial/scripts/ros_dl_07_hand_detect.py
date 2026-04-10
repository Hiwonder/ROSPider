#!/usr/bin/env python3
import cv2
import rospy
import numpy as np
import mediapipe as mp
from sensor_msgs.msg import Image
from vision_utils import fps


class HandDetectNode:
    def __init__(self):
        rospy.init_node('hand_detect_node')

        # 实例话一个手部识别器
        self.hand_detector = mp.solutions.hands.Hands( 
            static_image_mode=False,
            max_num_hands=1,
            # model_complexity=0,
            min_tracking_confidence=0.2,
            min_detection_confidence=0.7
        )
        self.drawing = mp.solutions.drawing_utils 

        self.fps = fps.FPS()  # fps计算器

        # 订阅相机画面
        self.camera_rgb_prefix = rospy.get_param('/camera_rgb_prefix', 'camera/rgb')
        self.image_sub = rospy.Subscriber(self.camera_rgb_prefix + '/image_raw', Image, self.image_callback, queue_size=1)

        rospy.loginfo("hand detect node created")

    def image_callback(self, ros_image: Image):
        # rospy.loginfo('Received an image! ')
        # ros 图像转为 opencv 格式
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data)
        rgb_image = cv2.flip(rgb_image, 1) # 翻转一下方便在屏幕上观察，要不然是镜像
        result_image = np.copy(rgb_image) # 结果图像
        try:
            results = self.hand_detector.process(rgb_image) # 进行手部识别
            if results is not None and results.multi_hand_landmarks: # 识别到手部
                for hand_landmarks in results.multi_hand_landmarks: # 将手部画出来
                    self.drawing.draw_landmarks( 
                        result_image,
                        hand_landmarks,
                        mp.solutions.hands.HAND_CONNECTIONS)
        except Exception as e:
            rospy.logerr(str(e))

        self.fps.update() # 更新帧率计数器
        result_image = self.fps.show_fps(result_image) # 在画面上显示帧率
        result_image = cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR)
        cv2.imshow('image', result_image)
        cv2.waitKey(1)


if __name__ == "__main__":
    try:
        hand_gesture_node = HandDetectNode()
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))
