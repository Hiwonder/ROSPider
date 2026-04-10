#!/usr/bin/env python3
import os
import sys
import cv2
import time
import rospy
import queue
import numpy as np
from sensor_msgs.msg import Image
from vision_utils import yolov5, fps


TRT_INPUT_SIZE = 160
TRT_NUM_CLASSES = 2
FACEMASK_LABELS = ("nomask", "mask")
COLORS = ((255, 0, 0), (0, 0, 255))


class FacemaskNode:
    def __init__(self):
        rospy.init_node('facemask_node')

        # 建立Yolo实例
        weights = rospy.get_param("~weights", "/home/hiwonder/weights/facemask_v5_160.trt") # 获取参数， 权重文件路径
        print(weights)
        self.yolov5 = yolov5.TrtYolov5(weights, TRT_INPUT_SIZE, TRT_NUM_CLASSES)

        self.fps = fps.FPS() # 帧率统计器

        # 订阅相机图像话题
        self.image_queue = queue.Queue(maxsize=2)
        self.camera_rgb_prefix = rospy.get_param('/camera_rgb_prefix', 'camera/rgb')
        self.image_sub = rospy.Subscriber(self.camera_rgb_prefix + '/image_raw', Image, self.image_callback, queue_size=1)
    
    def image_callback(self, ros_image):
        #rospy.logdebug('Received an image! ')
        self.image_queue.put(ros_image, block=True) # 将图片压入队列
        # 因为 pycuda 要求上下文建立的及执行要在同一个线程中, 如果不能在话题回调中执行识别， 要放到队列里在主线程执行识别

    def image_process(self):
        ros_image = self.image_queue.get(block=True) # 从队列里面取出画面

        # 将画面转为 opencv 格式
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data)
        result_image = np.copy(rgb_image)

        try:
            outputs = self.yolov5.detect(rgb_image) # 对画面进行识别
            # 后处理, 将原始输出转换为边界框,进行 NMS 阈值处理等
            boxes, confs, classes = self.yolov5.post_process(rgb_image, outputs, 0.6, 0.2) 
            height, width = rgb_image.shape[:2]

            for box, cls_id, cls_conf in zip(boxes, classes, confs):
                x1 = box[0] / TRT_INPUT_SIZE * width
                y1 = box[1] / TRT_INPUT_SIZE * height
                x2 = box[2] / TRT_INPUT_SIZE * width
                y2 = box[3] / TRT_INPUT_SIZE * height


                # 结果画面中显示是否戴口罩
                cv2.putText(result_image, 
                            FACEMASK_LABELS[cls_id] + " " + str(float(cls_conf))[:4],
                            (int(x1), int(y1) - 5), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, COLORS[cls_id], 2)
                # 结果画面中框出口罩
                cv2.rectangle(result_image, 
                              (int(x1), int(y1)), (int(x2), int(y2)), 
                              COLORS[cls_id], 3)

                rospy.loginfo((cls_id, float(cls_conf), x1, x2, y1, y2))
        except Exception as e:
            rospy.logerr(str(e))
        self.fps.update()
        self.fps.show_fps(result_image)
        result_image = cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR)
        cv2.imshow("image", result_image)
        cv2.waitKey(1)


if __name__ == '__main__':
    try:
        facemask_node = FacemaskNode()
        while not rospy.is_shutdown():
            facemask_node.image_process()
    except Exception as e:
        rospy.logerr(str(e))
