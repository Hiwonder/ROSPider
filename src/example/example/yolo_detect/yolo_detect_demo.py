#!/usr/bin/python3
# coding=utf8

import cv2
import time
import queue
import rclpy
import signal
import threading
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
from interfaces.msg import ObjectInfo, ObjectsInfo
from ultralytics import YOLO

class Colors:
    def __init__(self):
        hex = ('FF3838', 'FF9D97', 'FF701F', 'FFB21D', 'CFD231', '48F90A', '92CC17', '3DDB86', '1A9334', '00D4BB',
               '2C99A8', '00C2FF', '344593', '6473FF', '0018EC', '8438FF', '520085', 'CB38FF', 'FF95C8', 'FF37C7')
        self.palette = [self.hex2rgb('#' + c) for c in hex]
        self.n = len(self.palette)

    def __call__(self, i, bgr=False):
        c = self.palette[int(i) % self.n]
        return (c[2], c[1], c[0]) if bgr else c

    @staticmethod
    def hex2rgb(h):
        return tuple(int(h[1 + i:1 + i + 2], 16) for i in (0, 2, 4))

colors = Colors()
MODAL_PATH = '/home/ubuntu/third_party/yolo/yolo26/'

class yoloNode(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)

        self.fps = 0.0
        # Get parameters from Launch (从 Launch 获取参数)
        self.start = self.get_parameter('start').value
        self.model_name = self.get_parameter('model_name').value
        self.image_topic = self.get_parameter('image_topic').value
        self.conf_threshold = float(self.get_parameter('conf_threshold').value)
        self.model_size = self.get_parameter('model_size').value
        
        # Get classes list from Launch (从 Launch 获取类别列表)
        self.classes = self.get_parameter('classes').value 
        self.get_logger().info(f"Loaded {len(self.classes)} classes from launch.")

        self.nms_threshold = 0.5

        # Load model (加载模型)
        model_path = MODAL_PATH + f'{self.model_name}.pt'
        self.model = YOLO(model_path)
        self.get_logger().info(f"Using YOLO model: {model_path}")

        self.bridge = CvBridge()
        self.image_queue = queue.Queue(maxsize=2)
        self.running = True
        self.prev_time = time.time()

        signal.signal(signal.SIGINT, self.shutdown)

        # Services (服务)
        self.create_service(Trigger, '~/start', self.start_srv_callback)
        self.create_service(Trigger, '~/stop', self.stop_srv_callback)
        self.create_service(Trigger, '~/init_finish', self.get_node_state)

        # Subscription (订阅)
        self.image_sub = self.create_subscription(Image, self.image_topic, self.image_callback, 1)
        self.object_pub = self.create_publisher(ObjectsInfo, '~/object_detect', 1)
        self.result_image_pub = self.create_publisher(Image, '~/object_image', 1)

        threading.Thread(target=self.image_proc, daemon=True).start()

    def get_node_state(self, request, response):
        response.success = True
        return response

    def start_srv_callback(self, request, response):
        self.start = True
        response.success = True
        return response

    def stop_srv_callback(self, request, response):
        self.start = False
        response.success = True
        return response

    def image_callback(self, ros_image):
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        if self.image_queue.full():
            self.image_queue.get()
        self.image_queue.put(cv_image)

    def shutdown(self, signum, frame):
        self.running = False

    def image_proc(self):
        while self.running:
            try:
                result_image = self.image_queue.get(timeout=1)
            except queue.Empty:
                continue

            if self.start and result_image is not None:
                t_start = time.time()
                try:
                    objects_info = ObjectsInfo()
                    h, w = result_image.shape[:2]

                    results = self.model(result_image, imgsz=self.model_size, conf=self.conf_threshold, iou=self.nms_threshold)[0]

                    is_obb = results.obb is not None
                    boxes_data = results.obb if is_obb else results.boxes

                    for box in boxes_data:
                        cls_id = int(box.cls[0])
                        score = float(box.conf[0])
                        cls_name = self.classes[cls_id] if cls_id < len(self.classes) else f"id_{cls_id}"
                        color = colors(cls_id, True)

                        object_info = ObjectInfo()
                        object_info.class_name = cls_name
                        object_info.score = score
                        object_info.width = w
                        object_info.height = h

                        if is_obb:
                            obb_data = box.xywhr[0].cpu().numpy()
                            cx, cy, bw, bh, rad = obb_data
                            object_info.box = [int(cx), int(cy), int(bw), int(bh)]
                            object_info.angle = int(np.degrees(rad))
                            
                            points = box.xyxyxyxy[0].cpu().numpy().astype(int)
                            cv2.polylines(result_image, [points], isClosed=True, color=color, thickness=2)
                            cv2.putText(result_image, f"{cls_name} {object_info.angle}deg", tuple(points[0]),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                        else:
                            x1, y1, x2, y2 = map(int, box.xyxy[0])
                            object_info.box = [x1, y1, x2, y2]
                            object_info.angle = 0
                            cv2.rectangle(result_image, (x1, y1), (x2, y2), color, 2)
                            cv2.putText(result_image, f"{cls_name}:{score:.2f}", (x1, y1 - 10),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                        objects_info.objects.append(object_info)
                    t_end = time.time()
                    time_delta = t_end - t_start
                    if time_delta > 0:
                        current_fps = 1.0 / time_delta
                        self.fps = (self.fps * 0.9) + (current_fps * 0.1)

                    cv2.putText(result_image, f"FPS: {self.fps:.1f}", (20, 40), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 255), 2)

                    cv2.imshow("yolo_detect", result_image)
                    if cv2.waitKey(1) == 27: 
                        break
                    self.object_pub.publish(objects_info)
                    self.result_image_pub.publish(self.bridge.cv2_to_imgmsg(result_image, "bgr8"))

                except Exception as e:
                    self.get_logger().error(f"Detection error: {e}")
            else:
                if result_image is not None:
                    self.result_image_pub.publish(self.bridge.cv2_to_imgmsg(result_image, "bgr8"))

def main():
    node = yoloNode('yolo_node')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()