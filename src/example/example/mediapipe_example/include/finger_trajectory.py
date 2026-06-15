#!/usr/bin/env python3
# coding: utf8
#指尖轨迹
import cv2
import time
import enum
import rclpy
import queue
import threading
import numpy as np
import mediapipe as mp
from rclpy.node import Node
from cv_bridge import CvBridge
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
from rclpy.executors import MultiThreadedExecutor
from sdk.common import  distance, vector_2d_angle, get_area_max_contour
from servo_controller_msgs.msg import ServosPosition
from rclpy.callback_groups import ReentrantCallbackGroup
from servo_controller.bus_servo_control import set_servo_position


def get_hand_landmarks(img, landmarks):
    """
    将landmarks从medipipe的归一化输出转为像素坐标(convert landmarks from the normalized output of mediapipe to pixel coordinates)
    :param img: 像素坐标对应的图片(pixel coordinates corresponding image)
    :param landmarks: 归一化的关键点(normalized key points)
    :return:
    """
    h, w, _ = img.shape
    landmarks = [(lm.x * w, lm.y * h) for lm in landmarks]
    return np.array(landmarks)


def hand_angle(landmarks):
    """
    计算各个手指的弯曲角度(calculate the blending angle of each finger)
    :param landmarks: 手部关键点(hand key point)
    :return: 各个手指的角度(the angle of each finger)
    """
    angle_list = []
    # thumb 大拇指
    angle_ = vector_2d_angle(landmarks[3] - landmarks[4], landmarks[0] - landmarks[2])
    angle_list.append(angle_)
    # index 食指
    angle_ = vector_2d_angle(landmarks[0] - landmarks[6], landmarks[7] - landmarks[8])
    angle_list.append(angle_)
    # middle 中指
    angle_ = vector_2d_angle(landmarks[0] - landmarks[10], landmarks[11] - landmarks[12])
    angle_list.append(angle_)
    # ring 无名指
    angle_ = vector_2d_angle(landmarks[0] - landmarks[14], landmarks[15] - landmarks[16])
    angle_list.append(angle_)
    # pink 小拇指
    angle_ = vector_2d_angle(landmarks[0] - landmarks[18], landmarks[19] - landmarks[20])
    angle_list.append(angle_)
    angle_list = [abs(a) for a in angle_list]
    return angle_list


def h_gesture(angle_list):
    """
    通过二维特征确定手指所摆出的手势(determine the gesture formed by the fingers through two-dimensional features)
    :param angle_list: 各个手指弯曲的角度(the blending angle of each finger)
    :return : 手势名称字符串(gesture name string)
    """
    thr_angle, thr_angle_thumb, thr_angle_s = 65.0, 53.0, 49.0
    if (angle_list[0] < thr_angle_s) and (angle_list[1] < thr_angle_s) and (angle_list[2] < thr_angle_s) and (
            angle_list[3] < thr_angle_s) and (angle_list[4] < thr_angle_s):
        gesture_str = "five"
    elif (angle_list[0] > 5) and (angle_list[1] < thr_angle_s) and (angle_list[2] > thr_angle) and (
            angle_list[3] > thr_angle) and (angle_list[4] > thr_angle):
        gesture_str = "one"
    else:
        gesture_str = "none"
    return gesture_str


class State(enum.Enum):
    NULL = 0
    TRACKING = 1
    RUNNING = 2


def draw_points(img, points, tickness=4, color=(255, 0, 0)):
    """
    将记录的点连线画在画面上(draw lines connecting the recorded points on the screen)
    """
    points = np.array(points).astype(dtype=np.int32)
    if len(points) > 2:
        for i, p in enumerate(points):
            if i + 1 >= len(points):
                break
            cv2.line(img, p, points[i + 1], color, tickness)


def get_track_img(points):
    """
    用记录的点生成一张黑底白线的轨迹图(generate a trajectory image with a black background and white lines using the recorded points)
    """
    points = np.array(points).astype(dtype=np.int32)
    x_min, y_min = np.min(points, axis=0).tolist()
    x_max, y_max = np.max(points, axis=0).tolist()
    track_img = np.full([y_max - y_min + 100, x_max - x_min + 100, 1], 0, dtype=np.uint8)
    points = points - [x_min, y_min]
    points = points + [50, 50]
    draw_points(track_img, points, 1, (255, 255, 255))
    return track_img


class FingerTrajectorykNode(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.drawing = mp.solutions.drawing_utils
        self.timer = time.time()

        self.hand_detector = mp.solutions.hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_tracking_confidence=0.05,
            min_detection_confidence=0.6
        )


        self.bridge = CvBridge()  # 用于ROS Image消息与OpenCV图像之间的转换
        self.image_queue = queue.Queue(maxsize=2)
        self.state = State.NULL
        self.running = True
        self.points = []
        self.start_count = 0
        self.no_finger_timestamp = time.time()

        timer_cb_group = ReentrantCallbackGroup()
        self.image_sub = self.create_subscription(Image, '/depth_cam/rgb/image_raw', self.image_callback, 1)  # 摄像头订阅(subscribe to the camera)
        self.joints_pub = self.create_publisher(ServosPosition, '/servo_controller', 1) # 舵机控制

        #等待服务启动
        # self.client = self.create_client(Trigger, '/controller_manager/init_finish')
        # self.client.wait_for_service()
        # set_servo_position(self.joints_pub, 1.0, ((10, 500), (5, 500), (4, 330), (3, 100), (2, 700), (1, 500)))
        threading.Thread(target=self.main, daemon=True).start()
        self.get_logger().info('\033[1;32m%s\033[0m' % '按空格清空已经记录的轨迹')  
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
            bgr_image = cv2.flip(bgr_image, 1) # 镜像画面(mirrored image)
            result_image = np.copy(bgr_image) # 拷贝一份用作结果显示，以防处理过程中修改了图像(make a copy for result display to prevent modification of the image during processing)
            if self.timer <= time.time() and self.state == State.RUNNING:
                self.state = State.NULL
            try:
                results = self.hand_detector.process(bgr_image) if self.state != State.RUNNING else None
                if results is not None and results.multi_hand_landmarks:
                    gesture = "none"
                    index_finger_tip = [0, 0]
                    self.no_finger_timestamp = time.time()  # 记下当期时间，以便超时处理(note the current time for timeout handling)
                    for hand_landmarks in results.multi_hand_landmarks:
                        self.drawing.draw_landmarks(
                            result_image,
                            hand_landmarks,
                            mp.solutions.hands.HAND_CONNECTIONS)
                        landmarks = get_hand_landmarks(bgr_image, hand_landmarks.landmark)
                        angle_list = (hand_angle(landmarks))
                        gesture = (h_gesture(angle_list))
                        index_finger_tip = landmarks[8].tolist()

                    if self.state == State.NULL:
                        if gesture == "one":  # 检测到单独伸出食指，其他手指握拳(detect the index finger extended while the other fingers are clenched into a fist)
                            self.start_count += 1
                            if self.start_count > 20:
                                self.state = State.TRACKING
                                self.points = []
                        else:
                            self.start_count = 0

                    elif self.state == State.TRACKING:
                        if gesture == "five": # 伸开五指结束画图(finish drawing when all five fingers are spread out)
                            self.state = State.NULL

                            # 生成黑白轨迹图(generate a black and white trajectory image)
                            track_img = get_track_img(self.points)
                            contours = cv2.findContours(track_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
                            contour, area = get_area_max_contour(contours, 300)

                            # 按轨迹图识别所画图形(identify the drawn shape based on the trajectory image)
                            track_img = cv2.fillPoly(track_img, [contour, ], (255, 255, 255))
                            for _ in range(3):
                                track_img = cv2.erode(track_img, cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5)))
                                track_img = cv2.dilate(track_img, cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5)))
                            contours = cv2.findContours(track_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
                            contour, area = get_area_max_contour(contours, 300)
                            h, w = track_img.shape[:2]

                            track_img = np.full([h, w, 3], 0, dtype=np.uint8)
                            track_img = cv2.drawContours(track_img, [contour, ], -1, (0, 255, 0), 2)
                            approx = cv2.approxPolyDP(contour, 0.026 * cv2.arcLength(contour, True), True)
                            track_img = cv2.drawContours(track_img, [approx, ], -1, (0, 0, 255), 2)

                            print(len(approx))
                            # 根据轮廓包络的顶点数确定图形(determine the shape based on the number of vertices in the contour envelope)
                            if len(approx) == 3:
                                cv2.putText(track_img, 'Triangle', (10, 40),cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 0), 2)
                            if len(approx) == 4 or len(approx) == 5:
                                cv2.putText(track_img, 'Square', (10, 40),cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 0), 2)
                            if 5 < len(approx) < 10:
                                cv2.putText(track_img, 'Circle', (10, 40),cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 0), 2)
                            if len(approx) == 10:
                                cv2.putText(track_img, 'Star', (10, 40),cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 0), 2)

                            cv2.imshow('track', track_img)

                        else:
                            if len(self.points) > 0:
                                if distance(self.points[-1], index_finger_tip) > 5:
                                    self.points.append(index_finger_tip)
                            else:
                                self.points.append(index_finger_tip)

                        draw_points(result_image, self.points)
                    else:
                        pass
                else:
                    if self.state == State.TRACKING:
                        if time.time() - self.no_finger_timestamp > 2:
                            self.state = State.NULL
                            self.points = []

            except Exception as e:
                self.get_logger().error(str(e))

            cv2.imshow('image', result_image)
            key = cv2.waitKey(1) 

            if key == ord(' '): # 按空格清空已经记录的轨迹(press the space key to clear the recorded trajectory)
                self.points = []


def main():
    node = FingerTrajectorykNode('finger_trajectory')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
 
if __name__ == "__main__":
    main()
