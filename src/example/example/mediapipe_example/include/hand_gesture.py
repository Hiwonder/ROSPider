#!/usr/bin/env python3
# coding: utf8
#手势控制
import cv2
import time
import enum
import rclpy
import queue
import threading
import numpy as np
import sdk.fps as fps
import mediapipe as mp
from rclpy.node import Node
from app.common import Heart
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from controller import step_controller
from sdk.common import  vector_2d_angle
from std_srvs.srv import Trigger, SetBool
from rclpy.executors import MultiThreadedExecutor
from servo_controller_msgs.msg import ServosPosition
from ros_robot_controller_msgs.msg import BuzzerState
from rclpy.callback_groups import ReentrantCallbackGroup
from servo_controller.bus_servo_control import set_servo_position
from servo_controller.action_group_controller import ActionGroupController

def get_hand_landmarks(img_size, landmarks):
    """
    将landmarks从medipipe的归一化输出转为像素坐标(convert landmarks from the normalized output of mediapipe to pixel coordinates)
    :param img: 像素坐标对应的图片(pixel coordinates corresponding image)
    :param landmarks: 归一化的关键点(normalized key points)
    :return:
    """
    w, h = img_size
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
    if (angle_list[0] > thr_angle_thumb) and (angle_list[1] > thr_angle) and (angle_list[2] > thr_angle) and (
            angle_list[3] > thr_angle) and (angle_list[4] > thr_angle):
        gesture_str = "fist"
    elif (angle_list[0] < thr_angle_s) and (angle_list[1] < thr_angle_s) and (angle_list[2] > thr_angle) and (
            angle_list[3] > thr_angle) and (angle_list[4] > thr_angle):
        gesture_str = "gun"
    elif (angle_list[0] < thr_angle_s) and (angle_list[1] > thr_angle) and (angle_list[2] > thr_angle) and (
            angle_list[3] > thr_angle) and (angle_list[4] > thr_angle):
        gesture_str = "hand_heart"
    elif (angle_list[0] > 5) and (angle_list[1] < thr_angle_s) and (angle_list[2] > thr_angle) and (
            angle_list[3] > thr_angle) and (angle_list[4] > thr_angle):
        gesture_str = "one"
    elif (angle_list[0] > thr_angle_thumb) and (angle_list[1] < thr_angle_s) and (angle_list[2] < thr_angle_s) and (
            angle_list[3] > thr_angle) and (angle_list[4] > thr_angle):
        gesture_str = "two"
    elif (angle_list[0] > thr_angle_thumb) and (angle_list[1] < thr_angle_s) and (angle_list[2] < thr_angle_s) and (
            angle_list[3] < thr_angle_s) and (angle_list[4] > thr_angle):
        gesture_str = "three"
    elif (angle_list[0] > thr_angle_thumb) and (angle_list[1] > thr_angle) and (angle_list[2] < thr_angle_s) and (
            angle_list[3] < thr_angle_s) and (angle_list[4] < thr_angle_s):
        gesture_str = "OK"
    elif (angle_list[0] > thr_angle_thumb) and (angle_list[1] < thr_angle_s) and (angle_list[2] < thr_angle_s) and (
            angle_list[3] < thr_angle_s) and (angle_list[4] < thr_angle_s):
        gesture_str = "four"
    elif (angle_list[0] < thr_angle_s) and (angle_list[1] < thr_angle_s) and (angle_list[2] < thr_angle_s) and (
            angle_list[3] < thr_angle_s) and (angle_list[4] < thr_angle_s):
        gesture_str = "five"
    elif (angle_list[0] < thr_angle_s) and (angle_list[1] > thr_angle) and (angle_list[2] > thr_angle) and (
            angle_list[3] > thr_angle) and (angle_list[4] < thr_angle_s):
        gesture_str = "six"
    else:
        gesture_str = "none"

    return gesture_str

class State(enum.Enum):
    NULL = 0
    TRACKING = 1
    RUNNING = 2

def draw_points(img, points, tickness=4, color=(255, 0, 0)):
    points = np.array(points).astype(dtype=np.int64)
    if len(points) > 2:
        for i, p in enumerate(points):
            if i + 1 >= len(points):
                break
            cv2.line(img, p, points[i + 1], color, tickness)

class HandGestureNode(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.drawing = mp.solutions.drawing_utils
        self.hand_detector = mp.solutions.hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_tracking_confidence=0.4,
            min_detection_confidence=0.4
        )

        self.lock = threading.RLock()
        self.fps = fps.FPS()  # fps计算器(fps calculator)
        self.bridge = CvBridge()  # 用于ROS Image消息与OpenCV图像之间的转换
        self.image_queue = queue.Queue(maxsize=2)
        self.running = True
        self.draw = True
        self.state = State.NULL
        self.points = [[0, 0], ]
        self.no_finger_timestamp = time.time()
        self.one_count = 0
        self.count = 0
        self.direction = ""
        self.last_gesture = "none"
        self.gesture = None
        self.debug = self.get_parameter('debug').value
        self.controller = step_controller.StepController()
        self.gesture_list = ('gun', 'hand_heart', 'OK', 'fist')

        self.timer_cb_group = ReentrantCallbackGroup()
        self.joints_pub = self.create_publisher(ServosPosition, '/servo_controller', 1) # 舵机控制
        self.buzzer_pub = self.create_publisher(BuzzerState, '/ros_robot_controller/set_buzzer', 1) # 蜂鸣器控制

        self.agc_controller = ActionGroupController(self.create_publisher(ServosPosition, 'servo_controller', 1), '/home/ubuntu/software/actionset_editor/ActionGroups')
        self.client = self.create_client(Trigger, '/controller_manager/init_finish')
        self.client.wait_for_service()
       
        self.image_sub = self.create_subscription(Image, '/depth_cam/rgb/image_raw', self.image_callback, 1)  # 摄像头订阅(subscribe to the camera)
        self.timer = self.create_timer(0.0, self.init_process, callback_group=self.timer_cb_group)

    def init_process(self):
        self.timer.cancel()
        self.init_action()

        threading.Thread(target=self.main, daemon=True).start()
        threading.Thread(target=self.do_act, daemon=True).start()
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

    def get_node_state(self, request, response):
        response.success = True
        return response

    def shutdown(self, signum, frame):
        self.running = False

    def init_action(self):
        self.controller.set_build_in_pose('DEFAULT_POSE', 1)
        time.sleep(1)
        joint_angle = [500, 750, 200, 150, 500, 700]
        set_servo_position(self.joints_pub, 1, ((19, joint_angle[0]), (20, joint_angle[1]), (21, joint_angle[2]), (22, joint_angle[3]), (23, joint_angle[4]), (24, joint_angle[5])))
        time.sleep(1)

    def send_request(self, client, msg):
        future = client.call_async(msg)
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()

    def do_act(self):
        ACTION = {  'gun' : 'attack',
                    'hand_heart' : 'twist_l',
                    'OK' : 'wave',
                    'fist' : 'forward_flutter'}
        
        LOCK_SERVOS={'19':500, '20':750, '21':200, '22':150,  '23':500,  '24':700}

        while self.running:
            with self.lock:
                if self.gesture is not None and self.gesture in ACTION:
                    self.set_buzzer(0.2, 0.01, 1)
                    self.agc_controller.run_action(ACTION[self.gesture],lock_servos=LOCK_SERVOS)
                    self.count = 0
                    self.last_gesture = "none"
                    self.state = State.NULL
                    self.draw = True
                    self.gesture = None
            time.sleep(0.01)
    
    def set_buzzer(self, on_time, off_time, repeat):
        # 设置蜂鸣器
        msg = BuzzerState()
        msg.freq = 1900
        msg.on_time = on_time
        msg.off_time = off_time
        msg.repeat = repeat
        self.buzzer_pub.publish(msg)

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

            if time.time() - self.no_finger_timestamp > 2:
                self.direction = ""
            else:
                if self.direction != "":
                    cv2.putText(result_image, self.direction, (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
            try:
                results = self.hand_detector.process(bgr_image) # 手部、关键点识别(hand and key points recognition)
                if results.multi_hand_landmarks and self.draw :
                    gesture = "none"
                    index_finger_tip = [0, 0]
                    for hand_landmarks in results.multi_hand_landmarks: 
                        self.drawing.draw_landmarks(
                            result_image,
                            hand_landmarks,
                            mp.solutions.hands.HAND_CONNECTIONS)
                        h, w = bgr_image.shape[:2]
                        landmarks = get_hand_landmarks((w, h), hand_landmarks.landmark)
                        angle_list = hand_angle(landmarks)
                        gesture = h_gesture(angle_list) # 根据关键点位置判断手势(judge gesture based on key points position)
                        index_finger_tip = landmarks[8].tolist()

                    cv2.putText(result_image, gesture.upper(), (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 0), 5)
                    cv2.putText(result_image, gesture.upper(), (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 0), 2)
                    draw_points(result_image, self.points[1:])
                    if self.state != State.RUNNING :
                        if gesture in self.gesture_list :
                            if gesture == self.last_gesture :
                                self.count += 1
                            else:
                                self.count = 0
                            if self.count > 20:
                                self.count = 0
                                self.state = State.RUNNING
                                self.draw = False
                                self.gesture = gesture
                    else:
                        self.count = 0
                    self.last_gesture = gesture

                else:
                    if self.state != State.NULL:
                        if time.time() - self.no_finger_timestamp > 2:
                            self.one_count = 0
                            self.points = [[0, 0],]
                            self.state = State.NULL

            except Exception as e:
                self.get_logger().error(str(e))

            if result_image is not None :
                cv2.imshow('result_image', result_image)
                key = cv2.waitKey(1)
        self.init_action()
        rclpy.shutdown()
        
def main():
    node = HandGestureNode('hand_gesture')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
 
if __name__ == "__main__":
    main()