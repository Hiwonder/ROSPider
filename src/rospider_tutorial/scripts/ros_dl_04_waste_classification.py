#!/usr/bin/env python3

import cv2
import rospy
import queue
import numpy as np
from sensor_msgs.msg import Image
from vision_utils import yolov5, fps


TRT_INPUT_SIZE = 160
TRT_NUM_CLASSES = 12
TRT_CLASS_NAMES = ('Banana Peel', 'Broken Bones', 'Cigarette End', 'Disposable Chopsticks',
                   'Ketchup', 'Marker', 'Oral Liquid Bottle', 'Plate',
                   'Plastic Bottle', 'Storage Battery', 'Toothbrush', 'Umbrella')

WASTE_CLASSES = {
    'food_waste': ('Banana Peel', 'Broken Bones', 'Ketchup'),
    'hazardous_waste': ('Marker', 'Oral Liquid Bottle', 'Storage Battery'),
    'recyclable_waste': ('Plastic Bottle', 'Toothbrush', 'Umbrella'),
    'residual_waste': ('Plate', 'Cigarette End', 'Disposable Chopsticks'),
}

COLORS = {
    'recyclable_waste': (0, 0, 255),
    'hazardous_waste': (255, 0, 0),
    'food_waste': (0, 255, 0),
    'residual_waste': (80, 80, 80)
}

class WasteClassificationNode:
    def __init__(self):
        rospy.init_node('waste_classification_node')

        # 建立Yolo实例
        weights = rospy.get_param("~weights", "/home/hiwonder/waste_v5_160.trt") # 获取参数， 权重文件路径
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
            boxes, confs, classes = self.yolov5.post_process(rgb_image, outputs, 0.65) 
            width = rgb_image.shape[1]
            height = rgb_image.shape[0]
            cards = []

            for box, cls_conf, cls_id in zip(boxes, confs, classes):
                x1 = int(box[0] / TRT_INPUT_SIZE * width)
                y1 = int(box[1] / TRT_INPUT_SIZE * height)
                x2 = int(box[2] / TRT_INPUT_SIZE * width)
                y2 = int(box[3] / TRT_INPUT_SIZE * height)
                waste_name = TRT_CLASS_NAMES[cls_id]
                waste_class_name = ''
                for k, v in WASTE_CLASSES.items():
                    if waste_name in v:
                        waste_class_name = k
                        break
                cards.append((cls_conf, x1, y1, x2, y2, waste_class_name))
                result_image = cv2.putText(result_image, waste_name + " " + str(float(cls_conf))[:4], (x1, y1 - 5),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, COLORS[waste_class_name], 2)
                result_image = cv2.rectangle(result_image, (x1, y1), (x2, y2), COLORS[waste_class_name], 3)
        except Exception as e:
            rospy.logerr(str(e))

        self.fps.update() # 更新 fps 统计器
        result_image = self.fps.show_fps(result_image) # 画面中显示 fps
        result_image = cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR)
        cv2.imshow('result', result_image)
        cv2.waitKey(1)


if __name__ == '__main__':
    try:
        waste_classification_node = WasteClassificationNode()
        while not rospy.is_shutdown():
            waste_classification_node.image_process()
    except Exception as e:
        rospy.logerr(str(e))
