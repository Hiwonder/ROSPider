#!/usr/bin/env python3
# encoding: utf-8
# @data:2025/03/21
# @author:gcusms
# yolov11目标检测(yolov11 target detection)

import os
import cv2
import time
import queue
import rclpy
import signal
import threading
import numpy as np
import sdk.fps as fps
from sdk import common
from rclpy.node import Node
from ultralytics import YOLO
from cv_bridge import CvBridge
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
from interfaces.msg import ObjectInfo, ObjectsInfo
from example.yolo_detect.utils import Colors,plot_one_box

import logging
logging.getLogger('ultralytics').setLevel(logging.ERROR)

MODE_PATH = os.path.split(os.path.realpath(__file__))[0] + '/models'

class YoloNode(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        
        self.bgr_image = None
        self.start = False
        self.running = True

        self.bridge = CvBridge()
        self.image_queue = queue.Queue(maxsize=2)
        signal.signal(signal.SIGINT, self.shutdown)
        
        self.fps = 0.0

        engine = self.get_parameter('engine').value
        conf_thresh = self.get_parameter('conf').value
        self.get_logger().info('\033[1;32m%s\033[0m' % str(engine))
        self.get_logger().info('\033[1;32m%s\033[0m' % str(conf_thresh))

        self.classes = self.get_parameter('classes').value

        self.conf = float(self.get_parameter('conf').value)
        self.display = bool(self.get_parameter('display').value)
        self.task = self.get_parameter('task').get_parameter_value().string_value
        self.yolo_detect = YOLO(( MODE_PATH + "/" + engine), task=self.task, verbose=False)

        self.colors = Colors()
        self.create_service(Trigger, '/yolo/start', self.start_srv_callback)  # 进入玩法(enter the game)
        self.create_service(Trigger, '/yolo/stop', self.stop_srv_callback)  # 退出玩法(exit the game)

        self.image_sub = self.create_subscription(Image, '/depth_cam/rgb/image_raw', self.image_callback, 1)  # 摄像头订阅(subscribe to the camera)

        self.yolo_detet_flag = False
        self.object_pub = self.create_publisher(ObjectsInfo, '~/object_detect', 1)
        self.result_image_pub = self.create_publisher(Image, '~/object_image', 1)
        threading.Thread(target=self.image_proc, daemon=True).start()
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

    def get_node_state(self, request, response):
        response.success = True
        return response

    def yolo_detect_start(self, request, response):
        response.success = True
        return response
    
    def start_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "start yolo detect")

        self.start = True
        response.success = True
        response.message = "start"
        return response

    def stop_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "stop yolo detect")

        self.start = False
        response.success = True
        response.message = "start"
        return response

    def image_callback(self, ros_image):
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        bgr_image = np.array(cv_image, dtype=np.uint8)
        if self.image_queue.full():
            self.image_queue.get()
        self.image_queue.put(bgr_image)

    def shutdown(self, signum, frame):
        self.running = False
        self.get_logger().info('\033[1;32m%s\033[0m' % "shutdown")

    def image_proc(self):
            while self.running:
                try:
                    t_start = time.time()
                    image = self.image_queue.get(block=True, timeout=1)
                except queue.Empty:
                    if not self.running:
                        break
                    else:
                        continue
                
                try:
                    if self.start:
                        
                        objects_info = []
                        h, w = image.shape[:2]
                        results = self.yolo_detect(image, conf=self.conf, task=self.task)
                        
                        if not self.yolo_detet_flag:
                            self.create_service(Trigger, '~/yolo_start_detect', self.yolo_detect_start)
                            self.yolo_detet_flag = True
                        
                        for result in results:
                            self.task = 'obb'

                            is_obb_task = (self.task == 'obb' and result.obb is not None)
                            items = result.obb if is_obb_task else result.boxes
                            
                            if items is not None and len(items) > 0:
                                for i in range(len(items)):
                                    confidence = items.conf[i].item()
                                    class_id = int(items.cls[i].item())
                                    class_name = self.classes[class_id] if class_id < len(self.classes) else f"ID:{class_id}"
                                    color = self.colors(class_id, True)
                                    
                                    object_info = ObjectInfo()
                                    object_info.class_name = class_name
                                    object_info.score = float(confidence)
                                    object_info.width = w
                                    object_info.height = h

                                    if is_obb_task:
                                        # obb
                                        obb_data = items.xywhr[i].cpu().numpy()
                                        object_info.box = obb_data[:4].astype(int).tolist()
                                        object_info.angle = int(np.degrees(obb_data[4]))

                                        points = items.xyxyxyxy[i].cpu().numpy().astype(int)
                                        cv2.polylines(image, [points], isClosed=True, color=color, thickness=2)
                                        cv2.putText(image, f"{class_name} {object_info.angle}deg", tuple(points[0]),
                                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                                    else:
                                        # detect
                                        box_coords = items.xyxy[i].cpu().numpy().astype(int).tolist()
                                        object_info.box = box_coords
                                        object_info.angle = 0
                                        
                                        plot_one_box(box_coords, image, color=color, 
                                                    label=f"{class_name} {confidence:.2f}")

                                    objects_info.append(object_info)

                        object_msg = ObjectsInfo()
                        object_msg.objects = objects_info
                        self.object_pub.publish(object_msg)
                    else:
                        time.sleep(0.01)
                except BaseException as e:
                    print('error', e)
                t_end = time.time()
                time_delta = t_end - t_start
                if time_delta > 0:
                    current_fps = 1.0 / time_delta
                    self.fps = (self.fps * 0.9) + (current_fps * 0.1)

                cv2.putText(image, f"FPS: {self.fps:.1f}", (20, 40), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 255), 2)
                if self.display:
                    cv2.imshow("yolo_detect", image)
                    if cv2.waitKey(1) == 27: break
                self.result_image_pub.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))

def main():
    node = YoloNode('yolo')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
