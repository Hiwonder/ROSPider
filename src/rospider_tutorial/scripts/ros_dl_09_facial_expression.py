#!/usr/bin/env python3
import gc
import queue
import cv2
import rospy
import numpy as np
import mediapipe as mp
from sensor_msgs.msg import Image
from vision_utils import fps, vgg, warp_affine
from utils import show_faces, mp_face_location

EXPRESSIONS = ['Angry', 'Disgust', 'Fear', 'Happy', 'Sad', 'Surprise', 'Neutral']


class FacialExpressionNode:
    def __init__(self):
        rospy.init_node('facial_expression_node')

        self.fps = fps.FPS() # 帧率统计器

        # 人脸检测器
        self.face_detector = mp.solutions.face_detection.FaceDetection(
            # model_selection=0,
            min_detection_confidence=0.7,
        )

        # 表情分类器
        fer_model = rospy.get_param("~fer_model", "/home/hiwonder/weights/fer_vgg19_48.trt")
        self.fer = vgg.TrtVGG(fer_model, 48)

        # 订阅相机图像话题
        self.image_queue = queue.Queue(maxsize=2)
        self.camera_rgb_prefix = rospy.get_param('/camera_rgb_prefix', 'camera/rgb')
        self.image_sub = rospy.Subscriber(self.camera_rgb_prefix + '/image_raw', Image, self.image_callback, queue_size=1)
    
    def image_callback(self, ros_image):
        #rospy.logdebug('Received an image! ')
        self.image_queue.put(ros_image, block=True) # 将图片压入队列
        # 因为 pycuda 要求上下文建立的及执行要在同一个线程中, 如果不能在话题回调中执行识别， 要放到队列里在主线程执行识别

    def image_process(self):
        ros_image = self.image_queue.get(block=True)
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data) # 原始 RGB 画面

        rgb_image = cv2.resize(rgb_image, (480, 360)) # 缩放图像
        rgb_image = cv2.flip(rgb_image, 1)
        result_image = np.copy(rgb_image)
        results = self.face_detector.process(rgb_image) # 识别图像中的人脸及人脸关键点

        boxes, keypoints = mp_face_location(results, rgb_image) # 获取人脸识别的输出数据并将归一化数据转为像素坐标

        # 遍历所有识别到的人脸对这些人脸做表情识别
        for i, (box, landmark) in enumerate(zip(boxes, keypoints)):
            x1, y1, x2, y2 = np.array(box).astype(dtype=np.int)[:4] # 获取人脸框的坐标
            face = rgb_image[y1:y2, x1:x2] # 从画面中截取出人脸
            face = cv2.cvtColor(face, cv2.COLOR_RGB2GRAY)
            face = cv2.cvtColor(face, cv2.COLOR_GRAY2RGB)
            face = warp_affine(face, landmark) # 人脸对齐, 提高表情分类的准确度
            cv2.imshow('face', face)
            rospy.sleep(0.01)
            output = self.fer.execute(face) # 对人类进行表情分类

            # 打印这张脸每种表情的概率
            for j, i in enumerate(output): 
                print(EXPRESSIONS[j] + ":{:0.2f}".format(i))
            print("")

            # 画面中人脸左上角写上识别到的表情名称
            idx = np.argmax(output) # 找出概率最大的表情下标
            s = EXPRESSIONS[idx] + ' {:0.2f}'.format(output[idx])
            cv2.putText(result_image, s, (x1 + 5, y1 + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        result_image = show_faces(rgb_image, result_image, boxes, keypoints) # 在画面中显示识别到的人脸和脸部关键点

        self.fps.update() # 更新 fps 统计器
        result_image = self.fps.show_fps(result_image) # 画面中显示 fps
        result_image = cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR) 

        cv2.imshow('image', result_image)
        cv2.waitKey(1)
        gc.collect()


def main():
    try:
        facial_expression_node = FacialExpressionNode()
        while not rospy.is_shutdown():
            facial_expression_node.image_process()
    except Exception as e:
        rospy.logerr(str(e))

if __name__ == "__main__":
    main()

