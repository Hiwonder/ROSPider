#!/usr/bin/env python3
# encoding: utf-8
# 轨迹点发布(publish trajectory points)
import cv2
import enum
import time
import rclpy
import queue
import threading
import numpy as np
import mediapipe as mp
from rclpy.node import Node
from cv_bridge import CvBridge
from std_msgs.msg import String 
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from sdk.common import vector_2d_angle, distance
from interfaces.msg import Points, PixelPosition
from mediapipe.framework.formats import landmark_pb2
from ros_robot_controller_msgs.msg import BuzzerState

MARGIN = 10  # pixels
FONT_SIZE = 1
FONT_THICKNESS = 1
HANDEDNESS_TEXT_COLOR = (255, 255, 0) # yellow

def hand_angle(landmarks):
    """
    计算各个手指的弯曲角度(calculate the bending angles of each finger)
    :param landmarks: 手部关键点(hand key points)
    :return: 各个手指的角度(the angles of each finger)
    """
    angle_list = []
    # thumb 大拇指
    angle_ = vector_2d_angle(landmarks[4] - landmarks[0], landmarks[2] - landmarks[0])
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
    通过二维特征确定手指所摆出的手势(determine the hand gesture that the fingers are making based on the two-dimensional features)
    :param angle_list: 各个手指弯曲的角度(the bending angles of each finger)
    :return : 手势名称字符串(a string represents the name of the gesture)
    """
    thr_angle = 65.
    thr_angle_thumb = 10.
    thr_angle_s = 49.
    gesture_str = "None"
    if (angle_list[0] > thr_angle_thumb) and (angle_list[1] > thr_angle) and (angle_list[2] > thr_angle) and (
            angle_list[3] > thr_angle) and (angle_list[4] > thr_angle):
        gesture_str = "fist"
    elif (angle_list[0] < thr_angle_s) and (angle_list[1] < thr_angle_s) and (angle_list[2] > thr_angle) and (
            angle_list[3] > thr_angle) and (angle_list[4] > thr_angle):
        gesture_str = "hand_heart"
    elif (angle_list[0] < thr_angle_s) and (angle_list[1] < thr_angle_s) and (angle_list[2] > thr_angle) and (
            angle_list[3] > thr_angle) and (angle_list[4] < thr_angle_s):
        gesture_str = "rock"
    elif (angle_list[0] < thr_angle_s) and (angle_list[1] > thr_angle) and (angle_list[2] > thr_angle) and (
            angle_list[3] > thr_angle) and (angle_list[4] > thr_angle):
        gesture_str = "thumbs_up"
    elif (angle_list[0] > 5) and (angle_list[1] < thr_angle_s) and (angle_list[2] > thr_angle) and (
            angle_list[3] > thr_angle) and (angle_list[4] > thr_angle):
        gesture_str = "one"
    elif (angle_list[0] > thr_angle_thumb) and (angle_list[1] < thr_angle_s) and (angle_list[2] < thr_angle_s) and (
            angle_list[3] > thr_angle) and (angle_list[4] > thr_angle):
        gesture_str = "yes"
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
        "none"
    return gesture_str

def draw_points(img, points, thickness=4, color=(200, 200, 0)):
    points = np.array(points).astype(dtype=int)
    if len(points) > 2:
        for i, p in enumerate(points):
            if i + 1 >= len(points):
                break
            cv2.line(img, p, points[i + 1], color, thickness)

class State(enum.Enum):
    NULL = 0
    START = 1
    TRACKING = 2
    RUNNING = 3

class HandTrajectoryNode(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)

        self.name = name
        self.start = False
        self.running = True
        self.image = None
        self.state = State.NULL
        self.points = []
        self.count = 0
        self.image_sub = None
        self.count_miss = 0
        self.last_point = [0, 0]
        self.final_gesture  = "None"
        self.last_gesture  = "None"
        model_path = '/home/ubuntu/ros2_ws/src/example/example/mediapipe_example/include/model/hand_landmarker.task'
        base_options = python.BaseOptions(model_asset_path=model_path)
        options = vision.HandLandmarkerOptions(base_options=base_options, min_hand_detection_confidence=0.2, num_hands=2)
        self.detector = vision.HandLandmarker.create_from_options(options)

        self.bridge = CvBridge()
        self.image_queue = queue.Queue(maxsize=2)

        self.camera = 'depth_cam'
        self.buzzer_pub = self.create_publisher(BuzzerState, '/ros_robot_controller/set_buzzer', 1)
        self.point_publisher = self.create_publisher(Points, '~/points', 1)  # 使用~可以自动加上前缀名称(use ~ to automatically add prefix name)
        self.gesture_publisher = self.create_publisher(String, '~/gesture', 1)  # 使用~可以自动加上前缀名称(use ~ to automatically add prefix name)
        self.result_publisher = self.create_publisher(Image, '~/image_result', 1)  # 图像处理结果发布(publish the image processing result)
        self.create_service(Trigger, '~/enter', self.enter_srv_callback)  # 进入玩法(enter the game)
        self.create_service(Trigger, '~/exit', self.exit_srv_callback)  # 退出玩法(exit the game)
        self.create_service(Trigger, '~/start', self.start_srv_callback)  # 进入玩法(enter the game)
        self.create_service(Trigger, '~/stop', self.stop_srv_callback)  # 退出玩法(exit the game)
        
        self.debug = self.get_parameter('debug').value
        threading.Thread(target=self.image_proc, daemon=True).start()
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')


    def get_node_state(self, request, response):
        response.success = True
        return response

    def enter_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "enter")
        if self.image_sub is None:
            self.image_sub = self.create_subscription(Image, '/%s/rgb/image_raw' % self.camera, self.image_callback, 1)  # 摄像头订阅(subscribe to camera)
        response.success = True
        response.message = "enter"
        return response

    def exit_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "exit")
        self.start = False
        try:
            if self.image_sub is not None:
                self.destroy_subscription(self.image_sub)
                self.image_sub = None
        except Exception as e:
            self.get_logger().error(str(e))


        response.success = True
        response.message = "enter"
        return response

    def start_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "start hand trajectory")

        self.state = State.NULL
        self.points = []
        self.count = 0
        self.count_miss = 0
        self.start = True
        self.last_point = [0, 0]

        response.success = True
        response.message = "start"
        return response

    def stop_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "stop hand trajectory")

        self.start = False

        response.success = True
        response.message = "stop"
        return response

    def buzzer_warn(self):
        msg = BuzzerState()
        msg.freq = 1900
        msg.on_time = 0.2
        msg.off_time = 0.01
        msg.repeat = 1
        self.buzzer_pub.publish(msg)

    def image_proc(self):
        points_list = []
        gesture_list = ['None', 'fist', 'five', 'yes', 'rock', 'OK', 'thumbs_up']
        while self.running:
            try:
                image = self.image_queue.get(block=True, timeout=1)
            except queue.Empty:
                if not self.running:
                    break
                else:
                    continue
            image = cv2.flip(image, 1)
            annotated_image = image.copy()
            if self.start:
                try:
                    mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=image)
                    detection_result = self.detector.detect(mp_image)
                    hand_landmarks_list = detection_result.hand_landmarks
                    handedness_list = detection_result.handedness

                    if len(hand_landmarks_list) > 0:
                        self.count_miss = 0
                        gesture = "None"
                        hand_center = [0, 0]
                        for idx in range(len(hand_landmarks_list)):
                            hand_landmarks = hand_landmarks_list[idx]
                            handedness = handedness_list[idx]

                            # Draw the hand landmarks.
                            hand_landmarks_proto = landmark_pb2.NormalizedLandmarkList()
                            hand_landmarks_proto.landmark.extend([
                              landmark_pb2.NormalizedLandmark(x=landmark.x, y=landmark.y, z=landmark.z) for landmark in hand_landmarks
                            ])
                            mp.solutions.drawing_utils.draw_landmarks(
                              annotated_image,
                              hand_landmarks_proto,
                              mp.solutions.hands.HAND_CONNECTIONS,
                              mp.solutions.drawing_styles.get_default_hand_landmarks_style(),
                              mp.solutions.drawing_styles.get_default_hand_connections_style())

                            # Get the top left corner of the detected hand's bounding box.
                            height, width, _ = annotated_image.shape
                            x_coordinates = [landmark.x for landmark in hand_landmarks]
                            y_coordinates = [landmark.y for landmark in hand_landmarks]
                            text_x = int(min(x_coordinates) * width)
                            text_y = int(min(y_coordinates) * height) - MARGIN
                            
                            if handedness[0].category_name == "Left":
                                handedness[0].category_name = 'Right'
                            elif handedness[0].category_name == "Right":
                                handedness[0].category_name = 'Left'

                            # Draw handedness (left or right hand) on the image.
                            cv2.putText(annotated_image, f"{handedness[0].category_name}",
                                        (text_x, text_y), cv2.FONT_HERSHEY_DUPLEX,
                                        FONT_SIZE, HANDEDNESS_TEXT_COLOR, FONT_THICKNESS, cv2.LINE_AA)
                            if handedness[0].category_name == 'Right':
                                p1 = hand_landmarks[0]
                                p2 = hand_landmarks[2]
                                p3 = hand_landmarks[9]
                                p4 = hand_landmarks[17]
                                hand_center = [int((p1.x + p2.x + p3.x + p4.x)/4*width), int((p1.y + p2.y + p3.y + p4.y)/4*height)]
                                cv2.circle(annotated_image, tuple(hand_center), 10, (255, 255, 0), -1)

                                landmarks = np.array([[landmark.x, landmark.y] for landmark in hand_landmarks])
                                angle_list = (hand_angle(landmarks))
                                gesture = (h_gesture(angle_list))

                                if gesture == self.last_gesture:
                                    count += 1
                                else :
                                    count = 0
                                    self.last_gesture = gesture

                                if count >= 5 and gesture in gesture_list:
                                    self.final_gesture = gesture

                                    string = String()
                                    string.data = self.final_gesture
                                    self.gesture_publisher.publish(string)

                                cv2.putText(annotated_image, self.final_gesture, (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 0), 5)
                                cv2.putText(annotated_image, self.final_gesture, (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 0), 2)
                        if self.state != State.TRACKING:
                            if self.final_gesture == "five":  
                                self.count += 1
                                if self.count > 5:
                                    self.count = 0
                                    if self.state != State.TRACKING:
                                        self.buzzer_warn()

                                    self.state = State.TRACKING
                                    self.points = []
                                    points_list = []

                            else:
                                self.count = 0
                        elif self.state == State.TRACKING:
                            if self.final_gesture != "fist":
                                self.count += 1
                                if self.count > 1:
                                    self.count = 1
                                    if 100 > distance(self.last_point, hand_center) > 2:
                                        pixels = PixelPosition()
                                        pixels.x = int(hand_center[0])
                                        pixels.y = int(hand_center[1])
                                        points_list.append(pixels)
                                        self.points.append(hand_center)
                                    draw_points(annotated_image, self.points)
                                    self.last_point = hand_center
                        if self.final_gesture == "fist":
                            if self.state != State.NULL:
                                self.buzzer_warn()
                            self.state = State.NULL
                            if points_list:
                                points = Points()
                                points.points = points_list
                                self.point_publisher.publish(points)
                            self.count = 0
                            self.last_point = [0, 0]
                            self.points = []
                            points_list = []
                            draw_points(annotated_image, self.points)
                        
                    else:
                        self.count_miss += 1
                        if self.count_miss > 5:
                            self.state = State.NULL
                            self.count_miss = 5
                            self.count = 0
                            self.last_point = [0, 0]
                            self.points = []
                    




                except Exception as e:
                    print(e)
            else:
                time.sleep(0.01)
            
            if self.debug:
                cv2.imshow('annotated_image', cv2.cvtColor(annotated_image, cv2.COLOR_RGB2BGR))
                cv2.waitKey(1)
            self.result_publisher.publish(self.bridge.cv2_to_imgmsg(annotated_image, "rgb8"))


    def image_callback(self, ros_image):
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, "rgb8")
        rgb_image = np.array(cv_image, dtype=np.uint8)
        if self.image_queue.full():
            # 如果队列已满，丢弃最旧的图像(if the queue is full, discard the oldest image)
            self.image_queue.get()
        # 将图像放入队列(put the image into the queue)
        self.image_queue.put(rgb_image)

def main():
    node = HandTrajectoryNode('hand_trajectory')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
