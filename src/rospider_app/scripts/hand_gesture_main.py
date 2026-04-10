#!/usr/bin/env python3
import enum
import math
import threading
import rospy
import cv2
import time
import numpy as np
import mediapipe as mp
import std_srvs.srv
from sensor_msgs.msg import Image
from vision_utils import fps, distance, vector_2d_angle
from rospider_controller import client
from rospider_sdk import buzzer
from rospider_app import Heart


def get_hand_landmarks(img_size, landmarks):
    """
    将landmarks从medipipe的归一化输出转为像素坐标
    :param img: 像素坐标对应的图片
    :param landmarks: 归一化的关键点
    :return:
    """
    w, h = img_size
    landmarks = [(lm.x * w, lm.y * h) for lm in landmarks]
    return np.array(landmarks)


def hand_angle(landmarks):
    """
    计算各个手指的弯曲角度
    :param landmarks: 手部关键点
    :return: 各个手指的角度
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
    通过二维特征确定手指所摆出的手势
    :param angle_list: 各个手指弯曲的角度
    :return : 手势名称字符串
    """
    thr_angle = 65.
    thr_angle_thumb = 53.
    thr_angle_s = 49.
    gesture_str = "none"
    if (angle_list[0] > thr_angle_thumb) and (angle_list[1] > thr_angle) and (angle_list[2] > thr_angle) and (
            angle_list[3] > thr_angle) and (angle_list[4] > thr_angle):
        gesture_str = "fist"
    elif (angle_list[0] < thr_angle_s) and (angle_list[1] < thr_angle_s) and (angle_list[2] > thr_angle) and (
            angle_list[3] > thr_angle) and (angle_list[4] > thr_angle):
        gesture_str = "hand_heart"
    elif (angle_list[0] < thr_angle_s) and (angle_list[1] > thr_angle) and (angle_list[2] > thr_angle) and (
            angle_list[3] > thr_angle) and (angle_list[4] > thr_angle):
        gesture_str = "hand_heart"
    elif (angle_list[0] > thr_angle_thumb) and (angle_list[1] < thr_angle_s) and (angle_list[2] > thr_angle) and (
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
        gesture_str = "ok"
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
        "none"
    return gesture_str


class State(enum.Enum):
    NULL = 0
    START = 1
    TRACKING = 2


def di_di(repeat):
    for _ in range(repeat):
        buzzer.on()
        rospy.sleep(0.1)
        buzzer.off()
        rospy.sleep(0.1)


def draw_points(img, points, tickness=4, color=(255, 0, 0)):
    # points = np.expand_dims(np.array(points).astype(dtype=np.int), axis=0)
    points = np.array(points).astype(dtype=np.int)
    if len(points) > 2:
        for i, p in enumerate(points):
            if i + 1 >= len(points):
                break
            cv2.line(img, p, points[i + 1], color, tickness)


def get_track_img(points):
    points = np.array(points).astype(dtype=np.int)
    x_min, y_min = np.min(points, axis=0).tolist()
    x_max, y_max = np.max(points, axis=0).tolist()
    track_img = np.full([y_max - y_min + 1, x_max - x_min + 1, 1], 0, dtype=np.uint8)
    points = points - [x_min, y_min]
    draw_points(track_img, points, 1, (255, 255, 255))
    return track_img


class HandGestureNode:
    def __init__(self, name, log_level=rospy.INFO):
        rospy.init_node(name, log_level=log_level)
        self.node_name = name
        self.drawing = mp.solutions.drawing_utils
        self.hand_detector = mp.solutions.hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            #model_complexity=0,
            min_tracking_confidence=0.1,
            min_detection_confidence=0.7
        )

        self.lock = threading.RLock()
        self.robot = client.Client(self)
        self.fps = fps.FPS()  # fps计算器
        self.state = State.NULL
        self.points = []
        self.timestamp = 0
        self.timestamp_1 = 0
        self.count = 0
        self.gesture_count = 0
        self.last_gesture = ""
        self.is_running = False

        self.image_sub = None
        self.camera_rgb_prefix = rospy.get_param('/camera_rgb_prefix', 'camera/rgb')
        self.result_publisher = rospy.Publisher(self.node_name + '/image_result', Image, queue_size=2)
        self.enter_srv = rospy.Service(self.node_name + "/enter", std_srvs.srv.Trigger, self.enter_srv_callback)
        self.exit_srv = rospy.Service(self.node_name + "/exit", std_srvs.srv.Trigger, self.exit_srv_callback)
        self.set_running_srv = rospy.Service(self.node_name + "/set_running", std_srvs.srv.SetBool, self.set_running_srv_callback)
        self.heart = Heart(self.node_name + "/heartbeat", 5, lambda _: self.exit_srv_callback(None))

        rospy.loginfo("hand gesture node created")

    def reset_values(self):
        self.state = State.NULL
        self.is_running = False
        self.count = 0
        self.gesture_count = 0
        self.last_gesture = ""
        self.timestamp = 0
        self.timestamp_1 = 0
        self.points = []

    def enter_srv_callback(self, _):
        rospy.loginfo("enter")
        rsp = std_srvs.srv.TriggerResponse(success=True)
        with self.lock:
            try:
                self.image_sub.unregister()
            except Exception as e:
                rospy.logerr(str(e))
            try:
                self.camera_rgb_prefix = rospy.get_param('/camera_rgb_prefix', self.camera_rgb_prefix)
                self.image_sub = rospy.Subscriber(self.camera_rgb_prefix + '/image_raw', Image, self.image_callback, queue_size=2)
            except Exception as e:
                rospy.logerr(str(e))
                rsp.success = False
                rsp.message = str(e)
        return rsp

    def exit_srv_callback(self, _):
        rsp = std_srvs.srv.TriggerResponse(success=True)
        with self.lock:
            try:
                self.reset_values()
                self.image_sub.unregister()
            except Exception as e:
                rospy.logerr(str(e))
                rsp.success = False
                rsp.message = str(e)
        return rsp

    def set_running_srv_callback(self, req: std_srvs.srv.SetBoolRequest):
        rsp = std_srvs.srv.SetBoolResponse(success=True)
        rsp.success = True
        with self.lock:
            if req.data:
                self.is_running = True
            else:
                self.reset_values()
        return rsp

    def image_callback(self, ros_image):
        #rospy.loginfo('Received an image! ')
        # 将ros格式图像转换为opencv格式
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data) # 原始 RGB 画面
        rgb_image = cv2.flip(rgb_image, 1)
        o_h, o_w = rgb_image.shape[:2]
        result_image = np.copy(rgb_image) # 拷贝一份用作结果显示，以防处理过程中修改了图像

        try:
            if self.is_running and time.time() - self.timestamp_1 > 5:
                results = self.hand_detector.process(cv2.resize(rgb_image, (int(o_w/4), int(o_h/4))))
                if results.multi_hand_landmarks:
                    gesture = "none"
                    index_finger_tip = [0, 0]
                    for hand_landmarks in results.multi_hand_landmarks:
                        self.drawing.draw_landmarks(
                            result_image,
                            hand_landmarks,
                            mp.solutions.hands.HAND_CONNECTIONS)
                        landmarks = get_hand_landmarks((o_w, o_h), hand_landmarks.landmark)
                        angle_list = hand_angle(landmarks)
                        gesture = h_gesture(angle_list)
                        index_finger_tip = landmarks[8].tolist()

                    if gesture != "none":
                        cv2.putText(result_image, gesture, (50, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
                    if self.state == State.NULL:
                        if gesture == "five":  # 检测张开手掌开始识别食指指尖
                            self.count += 1
                            if self.count > 5:
                                di_di(2)
                                self.count = 0
                                self.state = State.START
                                self.timestamp = time.time()  # 记下当期时间，以便超时处理
                        else:
                            self.count = 0
                            if self.last_gesture == gesture:
                                self.gesture_count += 1
                            else:
                                self.last_gesture = gesture
                                self.gesture_count = 0

                            if self.gesture_count > 10:
                                if gesture == "fist":
                                    self.robot.run_actionset("forward_flutter.d6a", 1)
                                if gesture == "ok":
                                    self.robot.run_actionset("twist_f.d6a", 1)
                                if gesture == "hand_heart":
                                    self.robot.run_actionset("wave.d6a", 1)
                                self.gesture_count = 0
                                self.last_gesture = "none"
                                self.timestamp_1 = time.time()

                    elif self.state == State.START:
                        if time.time() - self.timestamp > 5:
                            self.state = State.NULL
                            buzzer.on()
                            rospy.sleep(0.5)
                            buzzer.off()
                        else:
                            if gesture == "one":  # 检测食指手势， 开始指尖追踪
                                self.count += 1
                                if self.count > 5:
                                    di_di(1)
                                    self.count = 0
                                    self.state = State.TRACKING
                                    self.points = []
                            else:
                                self.count = 0
                    elif self.state == State.TRACKING:
                        self.timestamp = time.time()
                        if gesture == "five" or self.count > 20:
                            self.state = State.NULL
                            buzzer.on()
                            rospy.sleep(0.5)
                            buzzer.off()
                            start_x, start_y = self.points[0]
                            end_x, end_y = self.points[-1]
                            dx = start_x - end_x
                            dy = start_y - end_x
                            if abs(dx) > 50:
                                if start_x < end_x:
                                    self.robot.traveling(gait=1, direction=math.radians(90), time=1.0, steps=5)
                                else:
                                    self.robot.traveling(gait=1, direction=math.radians(270), time=1.0, steps=5)
                                self.timestamp_1 = time.time()
                            elif abs(dy) > 30:
                                if start_y < end_y:
                                    self.robot.traveling(gait=1, direction=0, time=1.0, steps=5)
                                else:
                                    self.robot.traveling(gait=1, direction=math.radians(180), time=1.0, steps=5)
                                self.timestamp_1 = time.time()
                            else:
                                pass
                            #track_img = get_track_img(self.points)
                            #cv2.imwrite('data/{}.jpg'.format(int(time.time())), cv2.cvtColor(track_img, cv2.COLOR_GRAY2BGR))
                        else:
                            if gesture != "two":
                                if len(self.points) > 0:
                                    last_point = self.points[-1]
                                    if distance(last_point, index_finger_tip) < 5:
                                        self.count += 1
                                    else:
                                        self.count = 0
                                        self.points.append(index_finger_tip)
                                else:
                                    self.points.append(index_finger_tip)
                        draw_points(result_image, self.points)
                        #track_img = get_track_img(self.points)
                        #cv2.imshow('trck', cv2.cvtColor(track_img, cv2.COLOR_GRAY2BGR))
                    else:
                        pass
                else:
                    if self.state != State.NULL:
                        if time.time() - self.timestamp > 1:
                            self.state = State.NULL
                            buzzer.on()
                            rospy.sleep(0.5)
                            buzzer.off()

            self.fps.update()
            result_image = self.fps.show_fps(result_image)
            ros_image.data = result_image.tostring()
            self.result_publisher.publish(ros_image) # 发布新的结果图像
            #cv2.imshow('img', result_image)
            #cv2.waitKey(1)

        except Exception as e:
            rospy.logerr(str(e))


def main(args=None):
    try:
        hand_gesture_node = HandGestureNode('hand_gesture', rospy.INFO)
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))
        

if __name__ == "__main__":
    main()
