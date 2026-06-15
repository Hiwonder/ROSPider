#!/usr/bin/env python3
# coding: utf8
#肢体体感控制

import cv2
import enum
import time
import rclpy
import numpy as np
import mediapipe as mp
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from servo_controller_msgs.msg import ServosPosition
from ros_robot_controller_msgs.msg import BuzzerState
from rclpy.callback_groups import ReentrantCallbackGroup
from servo_controller.bus_servo_control import set_servo_position
from sdk.common import  distance, vector_2d_angle
from servo_controller.action_group_controller import ActionGroupController

display_size = [int(640*8/4), int(400*8/4)]

PULSE_PER_DEGREE = 1000 / 240
RIGHT_SERVO_I_DEFAULT = 340
RIGHT_SERVO_M_DEFAULT = 425
RIGHT_SERVO_O_DEFAULT = 235
LEFT_SERVO_I_DEFAULT = 660
LEFT_SERVO_M_DEFAULT = 575
LEFT_SERVO_O_DEFAULT = 775


SERVO_LIMITS = {
    "right_servo_1": (300, 600),
    "right_servo_2": (100, 600),
    "right_servo_3": (100, 500),
    "left_servo_1": (400, 700),
    "left_servo_2": (400, 900),
    "left_servo_3": (500, 900),
}

SERVO_IDS = {
    "right_servo_1": 5,
    "right_servo_2": 3,
    "right_servo_3": 1,
    "left_servo_1": 6,
    "left_servo_2": 4,
    "left_servo_3": 2,
}

class State(enum.Enum):
    NULL = 0
    INTO_IMITATION_1 = 1
    IMITATION = 2

def get_angle(p1, p2, p3):
    """
    获取三个点形成的夹角(get the angle formed by three points)
    :param p1: 第一个点(the first point)
    :param p2: 第二个点(the second point)
    :param p3: 第三个点(the third point)
    :return: 角度(the angle in degrees)
    """
    angle = vector_2d_angle(p2 - p1, p3 - p2)
    return angle


def is_pentagon(landmarks):
    """
    通过手工2d几何特征判断两手是否举过头顶, 并且手臂与肩部形成五边形(determine using manual 2D geometric features whether both hands are raised above the head and whether the arms and shoulders form a pentagon)
    :param landmarks: 肢体的各个关键点(the various landmarks of the body)
    :return: True or False 符合或不符合(True or False, indicating whether the pose conforms to the criteria)
    """
    shoulder_width = distance(landmarks[12], landmarks[11])  # 肩宽(shoulder width)
    hand_dist = distance(landmarks[16], landmarks[15])  # 两手腕的距离(the distance between the two wrists)
    if hand_dist > shoulder_width:  # 手腕距离要比肩宽小(the hand distance is smaller than the shoulder width)
        return False
    for p in landmarks[:7]:  # 两手腕要举过头，比眼鼻都高(both wrists are above the head, higher than the eyes and nose)
        if landmarks[15][1] > p[1] or landmarks[16][1] > p[1]:
            return False
    if get_angle(landmarks[11], landmarks[12], landmarks[14]) < 40:
        return False  # 左上臂要举起超过肩部40度以上(the left upper arms are raised more than 40 degrees above the shoulder)
    if get_angle(landmarks[13], landmarks[11], landmarks[12]) < 40:
        return False  # 右上臂要举起超过肩部40度以上(the right upper arms are raised more than 40 degrees above the shoulder)
    return True


def is_level(landmarks, angle_threshold=15):
    """
    通过手工2d几何特征判断肩部是否和画面水平(determine manually using 2D geometric features whether the shoulders are horizontal to the screen)
    :param landmarks:
    :param angle_threshold:
    :return:
    """
    p0 = landmarks[12].copy()
    p0[0] = 0
    if abs(get_angle(p0, landmarks[12], landmarks[11])) > angle_threshold:
        return False
    return True


def is_flat(landmarks, angle_threshold):
    """
    通过手工2d几何特征判断双臂是否展开(determine manually using 2D geometric features whether both arms are stretched out)
    :param landmarks:
    :param angle_threshold:
    :return: True or False 符合或不符合(True or False, indicating whether the pose conforms to the criteria)
    """
    arm_marks = [15, 13, 11, 12, 14, 16]
    for i in range(3):
        angle = get_angle(landmarks[arm_marks[i]], landmarks[arm_marks[i + 1]], landmarks[arm_marks[i + 2]])
        if abs(angle) > angle_threshold:
            return False
    return is_level(landmarks, angle_threshold)


def is_cross(landmarks):
    """
    通过手工2d几何特征判断双臂是否居高并交叉(determine whether the two arms are raised and crossed by manual 2D geometric features)
    :param landmarks:
    :return: True of False
    """
    if landmarks[16][0] <= landmarks[15][0]:
        return False
    return is_pentagon(landmarks)


def mp_pose_landmarks(results, img_rgb, draw=True):
    lm_list = []
    if results and results.pose_landmarks:
        h, w, = img_rgb.shape[:2]
        for idx, lm in enumerate(results.pose_landmarks.landmark):
            cx, cy = int(lm.x * w), int(lm.y * h)
            lm_list.append([cx, cy])
            if draw:
                cv2.circle(img_rgb, (cx, cy), 5, (255, 0, 0), cv2.FILLED)

        cv2.circle(img_rgb, lm_list[11], 5, (255, 255, 0), cv2.FILLED)
        cv2.circle(img_rgb, lm_list[12], 5, (0, 255, 255), cv2.FILLED)
        cv2.circle(img_rgb, lm_list[14], 5, (0, 255, 0), cv2.FILLED)

    if len(lm_list) > 0:
        return np.array(lm_list)
    else:
        return None


class MankindPoseNode(Node):
    def __init__(self):
        super().__init__('pose_control_node')
                
        # 初始化MediaPipe姿势检测
        self.pose = mp.solutions.pose.Pose(
            static_image_mode=False,
            model_complexity=1,
            min_detection_confidence=0.6,
            min_tracking_confidence=0.7
        )
        self.drawing = mp.solutions.drawing_utils
        self.bridge = CvBridge()
        self.timer_cb_group = ReentrantCallbackGroup()

        self.image_sub = self.create_subscription(Image, '/depth_cam/rgb/image_raw', self.image_callback, 1)  # 摄像头订阅(subscribe to the camera)
        self.joints_pub = self.create_publisher(ServosPosition, '/servo_controller', 1) # 舵机控制
        self.buzzer_pub = self.create_publisher(BuzzerState, '/ros_robot_controller/set_buzzer', 1) # 蜂鸣器控制

        self.agc_controller = ActionGroupController(self.create_publisher(ServosPosition, 'servo_controller', 1), '/home/ubuntu/software/actionset_editor/ActionGroups')
        self.client = self.create_client(Trigger, '/controller_manager/init_finish')
        self.client.wait_for_service()
        
        # 状态初始化
        self.state = State.NULL
        self.timestamp = 0
        self.r_x_dist = 0
        self.l_x_dist = 0
        self.count = 0

        self.timer = self.create_timer(0.0, self.init_process, callback_group=self.timer_cb_group)

    def init_process(self):
        self.timer.cancel()
        LOCK_SERVOS={'19':500, '20':750, '21':100, '22':240,  '23':500,  '24':700}
        self.agc_controller.run_action('body_1', lock_servos=LOCK_SERVOS)

        self.reset_servos()
        # threading.Thread(target=self.main, daemon=True).start()
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

    def get_node_state(self, request, response):
        response.success = True
        return response
    
    def reset_servos(self):
        self.get_logger().info("Resetting servos...")
        set_servo_position(self.joints_pub, 1, ((5, RIGHT_SERVO_I_DEFAULT), (3, RIGHT_SERVO_M_DEFAULT), (1, RIGHT_SERVO_O_DEFAULT), (6, LEFT_SERVO_I_DEFAULT), (4, LEFT_SERVO_M_DEFAULT), (2, LEFT_SERVO_O_DEFAULT)))
        time.sleep(1.2)

    def stop_imitation(self):
        self.state = State.NULL
        self.reset_servos()
        self.set_buzzer(0.2, 0.01, 1)
        self.get_logger().info("Imitation mode exited")

    def set_buzzer(self, on_time, off_time, repeat):
        # 设置蜂鸣器
        msg = BuzzerState()
        msg.freq = 1900
        msg.on_time = on_time
        msg.off_time = off_time
        msg.repeat = repeat
        self.buzzer_pub.publish(msg)

    def image_callback(self, msg):
        try:
            rgb_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            rgb_image = cv2.flip(rgb_image, 1)
            result_image = np.copy(rgb_image)

            results = self.pose.process(rgb_image)
            if results:
                self.drawing.draw_landmarks(
                    result_image, 
                    results.pose_landmarks, 
                    mp.solutions.pose.POSE_CONNECTIONS
                )
                landmarks = mp_pose_landmarks(results, result_image, True)
                
                if landmarks is not None:
                    self.process_landmarks(landmarks)
                else:
                    # self.stop_imitation()
                    time.sleep(0.01)
            self.update_display(result_image)
            
        except Exception as e:
            self.get_logger().error(f"Image processing error: {str(e)}")

    def process_landmarks(self, landmarks):
        if self.state == State.NULL:
            if time.time() - self.timestamp > 5:
                if is_pentagon(landmarks):
                    self.enter_imitation_mode()
        elif self.state == State.INTO_IMITATION_1:
            self.handle_imitation_transition(landmarks)
        else:
            self.handle_imitation(landmarks)

    def enter_imitation_mode(self):
        self.state = State.INTO_IMITATION_1
        self.timestamp = time.time()
        self.set_buzzer(0.2, 0.01, 1)
        self.get_logger().info("Entering imitation mode...")

    def handle_imitation_transition(self, landmarks):
        if is_flat(landmarks, 30):
            self.count += 1
            if self.count > 2:
                self.activate_imitation(landmarks)
        else:
            pass

    def activate_imitation(self, landmarks):
        self.state = State.IMITATION
        self.r_x_dist = distance(landmarks[13], landmarks[11])
        self.l_x_dist = distance(landmarks[12], landmarks[14])
        self.set_buzzer(0.2, 0.01, 1)
        self.get_logger().info("Imitation mode activated")

    def handle_imitation(self, landmarks):
        self.timestamp = time.time()
        if not is_cross(landmarks):
            self.adjust_servos(landmarks)
        else:
            self.stop_imitation()


    def clamp(self, value, min_value, max_value):

        if value is None:
            return min_value 
        return max(min_value, min(value, max_value))
    def adjust_servos(self, landmarks):

        left_angle_2 = get_angle(landmarks[11], landmarks[12], landmarks[14])
        left_angle_3 = get_angle(landmarks[12], landmarks[14], landmarks[16])
        right_angle_2 = get_angle(landmarks[12], landmarks[11], landmarks[13])
        right_angle_3 = get_angle(landmarks[11], landmarks[13], landmarks[15])

        raw_right_servo_2 = int(RIGHT_SERVO_M_DEFAULT + PULSE_PER_DEGREE * right_angle_2)
        raw_right_servo_3 = int(RIGHT_SERVO_O_DEFAULT + PULSE_PER_DEGREE * right_angle_3)
        raw_left_servo_2 = int(LEFT_SERVO_M_DEFAULT + PULSE_PER_DEGREE * left_angle_2)
        raw_left_servo_3 = int(LEFT_SERVO_O_DEFAULT + PULSE_PER_DEGREE * left_angle_3)

        r_x_dist = distance(landmarks[13], landmarks[11])
        l_x_dist = distance(landmarks[12], landmarks[14])

        left_angle_1 = self.clamp(90.0 - l_x_dist / self.l_x_dist * 90.0, 0, 120)
        right_angle_1 = self.clamp(90.0 - r_x_dist / self.r_x_dist * 90.0, 0, 120)

        raw_left_servo_1 = int(LEFT_SERVO_I_DEFAULT - PULSE_PER_DEGREE * left_angle_1)
        raw_right_servo_1 = int(RIGHT_SERVO_I_DEFAULT + PULSE_PER_DEGREE * right_angle_1)

        current_servo_values = {
            "right_servo_1": raw_right_servo_1,
            "right_servo_2": raw_right_servo_2,
            "right_servo_3": raw_right_servo_3,
            "left_servo_1": raw_left_servo_1,
            "left_servo_2": raw_left_servo_2,
            "left_servo_3": raw_left_servo_3,
        }

        servo_positions_list = []
        for servo_name, limits in SERVO_LIMITS.items():
            current_value = current_servo_values.get(servo_name)
            servo_id = SERVO_IDS.get(servo_name)

            if current_value is not None and servo_id is not None:
                min_val, max_val = limits
                clamped_value = self.clamp(current_value, min_val, max_val)
                servo_positions_list.append((servo_id, clamped_value))
            else:
                print(f"Warning: Missing value or ID for servo '{servo_name}'")

        servo_positions = tuple(servo_positions_list)

        set_servo_position(self.joints_pub, 0.02, servo_positions)
        time.sleep(0.02)
       


    def update_display(self, image):

        cv2.imshow("image", cv2.resize(image, (display_size[0], display_size[1])))
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = MankindPoseNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()