#!/usr/bin/env python3
# encoding: utf-8
# @Author: Aiden
# @Date: 2024/11/18
import os
import cv2
import json
import time
import queue
import rclpy
import threading
import PIL.Image
import numpy as np
import sdk.fps as fps
import message_filters
from sdk import common
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32, Bool
from std_srvs.srv import Trigger, SetBool, Empty
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from speech import speech
from large_models.config import *
from large_models_msgs.srv import SetString, SetModel, SetInt32
from large_models_examples.track_anything import ObjectTracker
from servo_controller.bus_servo_control import set_servo_position
from servo_controller_msgs.msg import ServosPosition, ServoPosition
from large_models_examples.tracker import Tracker
import pycuda.driver as cuda
cuda.init()  # 确保CUDA已经初始化

language = os.environ["ASR_LANGUAGE"]
if language == 'Chinese':
    PROMPT = '''
你作为智能六足机器人，善于图像识别，你的能力是将用户发来的图片进行目标检测精准定位，并按「输出格式」进行最后结果的输出，然后进行跟随。
## 1. 理解用户指令
我会给你一句话，你需要根据我的话中提取「物体名称」。 **object对应的name要用英文表示**, **不要输出没有提及到的物体**
## 2. 理解图片
我会给你一张图, 从这张图中找到「物体名称」对应物体的左上角和右下角的像素坐标; 如果没有找到，那xyxy为[]。**不要输出没有提及到的物体**
【特别注意】： 要深刻理解物体的方位关系, response需要结合用户指令和检测的结果进行回答
## 输出格式（请仅输出以下内容，不要说任何多余的话)
{
    "object": "name", 
    "xyxy": [xmin, ymin, xmax, ymax],
    "response": "5到30字的中文回答"
}
    '''
else:
    PROMPT = '''
**Role
You are a smart car with advanced visual recognition capabilities. Your task is to analyze an image sent by the user, perform object detection, and follow the detected object. Finally, return the result strictly following the specified output format.

Step 1: Understand User Instructions
You will receive a sentence. From this sentence, extract the object name to be detected.
Note: Use English for the object value, do not include any objects not explicitly mentioned in the instruction.

Step 2: Understand the Image
You will also receive an image. Locate the target object in the image and return its coordinates as the top-left and bottom-right pixel positions in the form [xmin, ymin, xmax, ymax].
Note: If the object is not found, then "xyxy" should be an empty list: [], only detect and report objects mentioned in the user instruction.The coordinates (xmin, ymin, xmax, ymax) must be normalized to the range [0, 1]

**Important: Accurately understand the spatial position of the object. The "response" must reflect both the user’s instruction and the detection result.

**Output Format (strictly follow this format, do not output anything else.The coordinates (xmin, ymin, xmax, ymax) must be normalized to the range [0, 1])
{
    "object": "name", 
    "xyxy": [xmin, ymin, xmax, ymax],
    "response": "reflect both the user’s instruction and the detection result (5–30 characters)"
}

**Example
Input: track the person
Output:
{
    "object": "person",
    "xyxy": [0.1, 0.3, 0.4, 0.6],
    "response": "I have detected a person in a white T-shirt and will track him now."
}
    '''

display_size = [int(640*6/4), int(400*6/4)]
class VLLMTrack(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)
        self.fps = fps.FPS() # 帧率统计器(frame rate counter)
        self.image_queue = queue.Queue(maxsize=2)
        self.vllm_result = ''
        # self.vllm_result = '''json{"object":"红色方块", "xyxy":[521, 508, 637, 683]}'''
        self.set_above = False
        self.running = True
        self.data = []
        self.box = []
        self.stop = True
        self.start_track = False
        self.action_finish = False
        self.play_audio_finish = False
        self.track = ObjectTracker(use_mouse=True, automatic=True, log=self.get_logger())
        self.camera = os.environ['DEPTH_CAMERA_TYPE']

        timer_cb_group = ReentrantCallbackGroup()
        self.client = speech.OpenAIAPI(api_key, base_url)
        self.joints_pub = self.create_publisher(ServosPosition, 'servo_controller', 1)
        self.cmv_pub = self.create_publisher(Twist, '/controller/cmd_vel', 1)  # 底盘控制
        self.tts_text_pub = self.create_publisher(String, 'tts_node/tts_text', 1)
        self.create_subscription(Bool, 'tts_node/play_finish', self.play_audio_finish_callback, 1, callback_group=timer_cb_group)
        self.create_subscription(String, 'agent_process/result', self.vllm_result_callback, 1)
        self.create_subscription(Bool, 'vocal_detect/wakeup', self.wakeup_callback, 1)
        
        self.awake_client = self.create_client(SetBool, 'vocal_detect/enable_wakeup')
        self.awake_client.wait_for_service()
        self.set_model_client = self.create_client(SetModel, 'agent_process/set_model')
        self.set_model_client.wait_for_service()
        self.set_mode_client = self.create_client(SetInt32, 'vocal_detect/set_mode')
        self.set_mode_client.wait_for_service()
        self.set_prompt_client = self.create_client(SetString, 'agent_process/set_prompt')
        self.set_prompt_client.wait_for_service()
        
        if self.camera == 'aurora':
            image_sub = message_filters.Subscriber(self, Image, 'depth_cam/rgb/image_raw')
            depth_sub = message_filters.Subscriber(self, Image, 'depth_cam/depth/image_raw')

            # 同步时间戳, 时间允许有误差在0.03s
            sync = message_filters.ApproximateTimeSynchronizer([depth_sub, image_sub], 3, 0.02)
            sync.registerCallback(self.multi_callback)
        else:
            self.image_sub = self.create_subscription(Image, '/depth_cam/rgb/image_raw', self.image_callback, 1)  # 摄像头订阅(subscribe to the camera)

        # 定义 PID 参数
        # 0.07, 0, 0.001
        self.pid_params = {
            'kp1': 0.1, 'ki1': 0.0, 'kd1': 0.00,
            'kp2': 0.002, 'ki2': 0.0, 'kd2': 0.0,
        }

        # 动态声明参数
        for param_name, default_value in self.pid_params.items():
            self.declare_parameter(param_name, default_value)
            self.pid_params[param_name] = self.get_parameter(param_name).value

        self.track.update_pid([self.pid_params['kp1'], self.pid_params['ki1'], self.pid_params['kd1']],
                      [self.pid_params['kp2'], self.pid_params['ki2'], self.pid_params['kd2']])

        # 动态更新时的回调函数
        self.add_on_set_parameters_callback(self.on_parameter_update)

        self.timer = self.create_timer(0.0, self.init_process, callback_group=timer_cb_group)

    def on_parameter_update(self, params):
        """参数更新回调"""
        for param in params:
            if param.name in self.pid_params.keys():
                self.pid_params[param.name] = param.value
        # self.get_logger().info(f'PID parameters updated: {self.pid_params}')
        # 更新 PID 参数
        self.track.update_pid([self.pid_params['kp1'], self.pid_params['ki1'], self.pid_params['kd1']],
                      [self.pid_params['kp2'], self.pid_params['ki2'], self.pid_params['kd2']])

        return SetParametersResult(successful=True)

    def create_update_callback(self, param_name):
        """生成动态更新回调"""
        def update_param(msg):
            new_value = msg.data
            self.pid_params[param_name] = new_value
            self.set_parameters([Parameter(param_name, Parameter.Type.DOUBLE, new_value)])
            self.get_logger().info(f'Updated {param_name}: {new_value}')
            # 更新 PID 参数

        return update_param

    def get_node_state(self, request, response):
        return response

    def init_process(self):
        self.timer.cancel()
        
        msg = SetModel.Request()
        msg.model_type = 'vllm'
        if language == 'Chinese':
            msg.model = stepfun_vllm_model
            msg.api_key = stepfun_api_key
            msg.base_url = stepfun_base_url
        else:
            msg.api_key = vllm_api_key
            msg.base_url = vllm_base_url
            msg.model = vllm_model
        self.send_request(self.set_model_client, msg)

        msg = SetString.Request()
        msg.data = PROMPT
        self.send_request(self.set_prompt_client, msg)
        
        set_servo_position(self.joints_pub, 1.5, ((24, 700), (23, 500), (22, 100), (21, 100), (20, 750), (19, 500)))
        # self.cmv_pub.publish(Twist())
        time.sleep(1.8)
        speech.play_audio(start_audio_path)
        threading.Thread(target=self.process, daemon=True).start()
        threading.Thread(target=self.display_thread, daemon=True).start()
        self.create_service(Empty, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')
        self.get_logger().info('\033[1;32m%s\033[0m' % PROMPT)

    def send_request(self, client, msg):
        future = client.call_async(msg)
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()

    def wakeup_callback(self, msg):
        if msg.data and self.vllm_result:
            self.get_logger().info('wakeup interrupt')
            self.track.stop()
            self.stop = True
        elif msg.data and not self.stop:
            self.get_logger().info('wakeup interrupt')
            self.track.stop()
            self.stop = True

    def vllm_result_callback(self, msg):
        self.vllm_result = msg.data

    def play_audio_finish_callback(self, msg):
        self.play_audio_finish = msg.data

    def process(self):
        box = ''
        while self.running:
            if self.vllm_result:
                try:
                    # self.get_logger().info('vllm_result: %s' % self.vllm_result)
                    if self.vllm_result.startswith("```") and self.vllm_result.endswith("```"):
                        self.vllm_result = self.vllm_result.strip("```").replace("json\n", "").strip()
                    self.vllm_result = json.loads(self.vllm_result)
                    response = self.vllm_result['response']
                    msg = String()
                    msg.data = response
                    self.tts_text_pub.publish(msg)
                    box = self.vllm_result['xyxy']
                    if box:
                        if language == 'Chinese':
                            box = self.client.data_process(box, 640, 400)
                            self.get_logger().info('box: %s' % str(box))
                        else:
                            box = [int(box[0] * 640), int(box[1] * 400), int(box[2] * 640), int(box[3] * 400)]
                        box = [box[0], box[1], box[2] - box[0], box[3] - box[1]]
                        box[0] = int(box[0] / 640 * display_size[0])
                        box[1] = int(box[1] / 400 * display_size[1])
                        box[2] = int(box[2] / 640 * display_size[0])
                        box[3] = int(box[3] / 400 * display_size[1])
                        self.get_logger().info('box: %s' % str(box))
                        self.box = box
                except (ValueError, TypeError):
                    self.box = []
                    msg = String()
                    msg.data = self.vllm_result
                    self.tts_text_pub.publish(msg)
                self.vllm_result = ''
                self.action_finish = True
            else:
                time.sleep(0.02)
            if self.play_audio_finish and self.action_finish:
                self.play_audio_finish = False
                self.action_finish = False
                msg = SetBool.Request()
                msg.data = True
                self.send_request(self.awake_client, msg)
                self.stop = False

    def display_thread(self):
        # 在当前线程创建一个新的 CUDA 上下文
        dev = cuda.Device(0)
        ctx = dev.make_context()
        try:
            model_path = os.path.split(os.path.realpath(__file__))[0]

            back_exam_engine_path = os.path.join(model_path, "resources/models/nanotrack_backbone_exam.engine")
            back_temp_engine_path = os.path.join(model_path, "resources/models/nanotrack_backbone_temp.engine")
            head_engine_path = os.path.join(model_path, "resources/models/nanotrack_head.engine")
            tracker = Tracker(back_exam_engine_path, back_temp_engine_path, head_engine_path)
            while self.running:
                if self.camera == 'aurora':
                    image, depth_image = self.image_queue.get(block=True)
                else:
                    image= self.image_queue.get(block=True)
                image = cv2.resize(image, tuple(display_size))
                if self.box:
                    self.track.set_track_target(tracker, self.box, image)
                    self.start_track = True
                    self.box = []
                if self.start_track:
                    if self.camera == 'aurora':
                        self.data = self.track.track(tracker, image, depth_image)
                    else:
                        self.data = self.track.track_usb(tracker, image)
                    image = self.data[-1]
                    twist = Twist()
                    twist.linear.x, twist.angular.z = self.data[0], self.data[1]
                    self.cmv_pub.publish(twist)
                self.fps.update()
                self.fps.show_fps(image)
                cv2.imshow('image', image)
                key = cv2.waitKey(1)
                if key == ord('q') or key == 27:  # 按q或者esc退出
                    self.cmv_pub.publish(Twist())
                    self.running = False
                if not self.set_above:
                    cv2.moveWindow('image', 1920 - display_size[0], 0)
                    os.system("wmctrl -r image -b add,above")
                    self.set_above = True

            cv2.destroyAllWindows()
        finally:
            # 确保上下文被正确释放
            ctx.pop()

    def multi_callback(self, depth_image, ros_image):
        depth_frame = np.ndarray(shape=(depth_image.height, depth_image.width), dtype=np.uint16, buffer=depth_image.data)
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data)  # 将自定义图像消息转化为图像(convert the custom image message into image)  into)
        if self.camera == "usb_cam":
            rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
        if self.image_queue.full():
            # 如果队列已满，丢弃最旧的图像
            self.image_queue.get()
        # 将图像放入队列
        self.image_queue.put([rgb_image, depth_frame])
    
    def image_callback(self,ros_image):
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data)  # 将自定义图像消息转化为图像(convert the custom image message into image)  into)
        if self.camera == "usb_cam":
            rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
        if self.image_queue.full():
            # 如果队列已满，丢弃最旧的图像
            self.image_queue.get()
        # 将图像放入队列
        self.image_queue.put(rgb_image)

def main():
    node = VLLMTrack('vllm_track')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()

if __name__ == "__main__":
    main()


