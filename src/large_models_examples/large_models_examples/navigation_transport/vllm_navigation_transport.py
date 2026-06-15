#!/usr/bin/env python3
# encoding: utf-8
# @Author: Aiden
# @Date: 2024/11/18
import re
import cv2
import time
import json
import rclpy
import queue
import threading
import numpy as np
from speech import speech
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger, SetBool, Empty

from large_models.config import *
from large_models_msgs.srv import SetModel, SetContent, SetString, SetInt32

from interfaces.srv import SetPose2D, SetPoint, SetBox
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

language = os.environ["ASR_LANGUAGE"]
if language == 'Chinese':
    position_dict = {
                     "红色盒子": [0.1, -0.1, 0.0, 0.0, 0.0],
                     "红色方块": [0.2, 0.3, 0.0, 0.0, 90.0],
                     "绿色方块": [0.0, 0.45, 0.0, 0.0, 90.0],
                     "绿色盒子": [1.2, -0.9, 0.0, 0.0, -62.0],
                     "蓝色方块": [0.42, -0.08, 0.0, 0.0, -0.0],
                     "蓝色盒子": [-0.2, -0.75, 0.0, 0.0, -90.0],
                     "原点": [0.0, 0.0, 0.0, 0.0, 0.0]}
    LLM_PROMPT = '''
# 角色任务
作为语言专家，你能深刻的洞悉用户的指令。

## 技能细则
* 分析用户指令，将一次抓取和一次放置作为一个动作
* 有较强的逻辑能力, 且能理解物体的修饰
* 能准确分解出抓取和放置的物体

 ##要求与限制
1.根据输入的内容，在函数库中找到对应的指令，并输出对应的指令。
2.多个动作需要放到列表里
3.为动作序列编织一句精炼（10至30字）、风趣且变化无穷的反馈信息，让交流过程妙趣横生。
4.搬运分为运动到目的地点拿起和再运动到目标点放下两个阶段。
5.抓取和放置间需要移动，如果没有指定要移动到哪，默认不执行move移动。
6.地点只能是下面这些：蓝色方块，蓝色盒子，红色方块，红色盒子，绿色方块，绿色盒子，原点。
7.直接输出json格式的数据，不要分析，不要输出多余内容。
8.有看到带有红、绿、蓝三种颜色的立方体的才算是方块, 不要识别地面，然后 response不要输出任务完成等术语。
9.格式：{"action":"xx", "response":"xx"}

## 可用函数与操作：
* 拿起物体: pick('红色方块')
* 放到指定物体上：place('红色盒子')
* 移动到指定位置：move('原点')

## 示例：
输入：将红色方块放到对应颜色的盒子里
输出：{"action": ["move('红色方块')","pick('红色方块')", "move('红色盒子')", "place('红色盒子')"], "response": "好的，马上就是办"}
输入：将蓝色方块放到蓝色的盒子里
输出：{"action": ["move('蓝色方块')","pick('蓝色方块')", "move('蓝色盒子')", "place('蓝色盒子')"], "response": "好的，开始执行任务"}
    '''

    VLLM_PROMPT = '''
你作为图像识别专家，你的能力是将用户发来的图片进行目标检测精准定位，并按「输出格式」进行最后结果的输出。
## 1. 理解用户指令
我会给你一句话，你需要根据我的话做出最佳决策，从做出的决策中提取「物体名称」, **object对应的name要用英文表示**, **不要输出没有提及到的物体**
## 2. 理解图片
我会给你一张图, 从这张图中找到「物体名称」对应物体是否有可以抓取的地方，如果有输出他的左上角和右下角的像素坐标, **不要输出没有提及到的物体**
【特别注意】： 要深刻理解物体间的方位关系
## 输出格式（请仅输出以下内容，不要说任何多余的话)
{
    "object": name, 
    "xyxy": [xmin, ymin, xmax, ymax]
}
    '''
else:
    position_dict = {
            "red box": [1.2, 0.55, 0.0, 0.0, 0.0],
            "red cube": [0.25, 0.35, 0.0, 0.0, 90.0],
            "green cube": [0.0, 0.45, 0.0, 0.0, 90.0],
            "green box": [1.2, -0.9, 0.0, 0.0, -62.0],
            "blue cube": [0.0, 0.5, 0.0, 0.0, -180.0],
            "blue box": [-0.2, -0.75, 0.0, 0.0, -90.0],
            "origin": [0.0, 0.0, 0.0, 0.0, 0.0]}

    LLM_PROMPT = '''
You are a language expert who precisely interprets user commands.

**Skills
- Treat each pick-and-place as one composite action.
- Strong logical reasoning; understand object descriptors.
- Accurately decompose which objects to pick and which to place.

**Requirements & Constraints
- Map the user’s request to the available functions and output those calls.
- Bundle multiple actions into a JSON array.
- Add a concise (10–30 characters), witty, ever-changing feedback message for the sequence.
- Each transport has two phases: move to the pick location, then move to the place location.
- If no move target is specified, omit the move call.
- Valid locations: blue cube, blue box, red cube, red box, green cube, green box, origin.
- Only cubes with red, green, or blue faces count as "cubes"; ignore the ground.
- Don’t include any extra text (e.g. “task complete”).
- Output only this JSON structure:
{"action":"xx", "response":"xx"}

**Available Functions
- pick('red cube')
- place('red box')
- move('origin')

**Example
Input: Put the red cube into its matching box
Output:
{"action": ["move('red cube')", "pick('red cube')", "move('red box')", "place('red box')"], "response": "Right away—on the move!"}
Input: Put the blue cube into the blue box
Output: {"action": ["move('blue cube')","pick('blue cube')", "move('blue box')", "place('blue box')"], "response": "OK, start the task"}
    '''

    VLLM_PROMPT = '''
You are an image‐recognition specialist. Your task is to detect and precisely localize specified objects in user‐provided images, then produce the final result in the exact format below.

1. Parse User Instruction
I will give you a sentence describing which object to find.
Extract and output only the object name mentioned (in English). Do not include any unmentioned objects.
2. Analyze the Image
I will provide an image. For the specified object, determine if there is a graspable region.
If found, output its bounding box as pixel coordinates: top-left (xmin, ymin) and bottom-right (xmax, ymax).
Ignore any objects not mentioned.
Pay close attention to spatial relationships between objects.

**Output Format (strictly follow this format, do not output anything else.The coordinates (xmin, ymin, xmax, ymax) must be normalized to the range [0, 1])
{
  "object": "name", 
  "xyxy": [xmin, ymin, xmax, ymax]
}

**Example
Input: red box 
Output:
{
    "object": "red box",
    "xyxy": [0.1, 0.3, 0.4, 0.6]
}
    '''

class VLLMNavigation(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)
        
        self.action = []
        self.response_text = ''
        self.llm_result = ''
        self.action_finish = False
        self.transport_action_finish = False
        self.play_audio_finish = False
        # self.llm_result = '{\'action\':[\'move(\"前台\")\', \'vision(\"大门有没有关\")\', \'move(\"原点\")\', \'play_audio()\'], \'response\':\'马上！\'}'
        self.running = True
        self.reach_goal = False
        self.interrupt = False
        self.bridge = CvBridge()
        self.image_queue = queue.Queue(maxsize=2)
        timer_cb_group = ReentrantCallbackGroup()
        self.client = speech.OpenAIAPI(api_key, base_url)
        self.tts_text_pub = self.create_publisher(String, 'tts_node/tts_text', 1)
        self.create_subscription(Image, 'automatic_pick/image_result', self.image_callback, 1)
        self.create_subscription(String, 'agent_process/result', self.llm_result_callback, 1)
        self.create_subscription(Bool, 'vocal_detect/wakeup', self.wakeup_callback, 1)
        self.create_subscription(Bool, 'tts_node/play_finish', self.play_audio_finish_callback, 1, callback_group=timer_cb_group)
        self.create_subscription(Bool, 'navigation_controller/reach_goal', self.reach_goal_callback, 1)
        self.create_subscription(Bool, 'automatic_pick/action_finish', self.action_finish_callback, 1)
        self.awake_client = self.create_client(SetBool, 'vocal_detect/enable_wakeup')
        self.awake_client.wait_for_service()
        self.set_mode_client = self.create_client(SetInt32, 'vocal_detect/set_mode')
        self.set_mode_client.wait_for_service()
        self.set_model_client = self.create_client(SetModel, 'agent_process/set_model')
        self.set_model_client.wait_for_service()
        self.set_prompt_client = self.create_client(SetString, 'agent_process/set_prompt')
        self.set_prompt_client.wait_for_service()
        self.set_vllm_content_client = self.create_client(SetContent, 'agent_process/set_vllm_content')
        self.set_vllm_content_client.wait_for_service()
        self.set_pose_client = self.create_client(SetPose2D, 'navigation_controller/set_pose')
        self.set_pose_client.wait_for_service()
        # self.set_target_client = self.create_client(SetPoint, 'automatic_pick/set_target_color')
        # self.set_target_client.wait_for_service()
        self.set_box_client = self.create_client(SetBox, 'automatic_pick/set_box')
        self.set_box_client.wait_for_service()
        self.set_pick_client = self.create_client(Trigger, 'automatic_pick/pick')
        self.set_pick_client.wait_for_service()
        self.set_place_client = self.create_client(Trigger, 'automatic_pick/place')
        self.set_place_client.wait_for_service()
        self.timer = self.create_timer(0.0, self.init_process, callback_group=timer_cb_group)

    def get_node_state(self, request, response):
        return response

    def init_process(self):
        self.timer.cancel()
        
        msg = SetModel.Request()
        msg.model = llm_model
        msg.model_type = 'llm'
        msg.api_key = api_key 
        msg.base_url = base_url
        self.send_request(self.set_model_client, msg)

        msg = SetString.Request()
        msg.data = LLM_PROMPT
        self.send_request(self.set_prompt_client, msg)
        
        init_finish = self.create_client(Empty, 'navigation_controller/init_finish')
        init_finish.wait_for_service()
        speech.play_audio(start_audio_path)
        threading.Thread(target=self.process, daemon=True).start()
        self.create_service(Empty, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

    def send_request(self, client, msg):
        future = client.call_async(msg)
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()

    def wakeup_callback(self, msg):
        self.interrupt = msg.data

    def llm_result_callback(self, msg):
        self.llm_result = msg.data

    def move(self, position):
        self.get_logger().info('position: %s' % str(position))
        msg = SetPose2D.Request()
        if position in position_dict:
            p = position_dict[position]
            msg.data.x = float(p[0])
            msg.data.y = float(p[1])
            msg.data.roll = p[2]
            msg.data.pitch = p[3]
            msg.data.yaw = p[4]
            self.send_request(self.set_pose_client, msg)
            return True
        else:
            return False

    def reach_goal_callback(self, msg):
        self.get_logger().info('reach goal')
        self.reach_goal = msg.data

    def action_finish_callback(self, msg):
        self.get_logger().info('action finish')
        self.transport_action_finish = msg.data

    def get_object_position(self, query, image):
        msg = SetContent.Request()
        if language == 'Chinese':
            msg.model = stepfun_vllm_model
            msg.api_key = stepfun_api_key
            msg.base_url = stepfun_base_url
        else:
            msg.api_key = vllm_api_key
            msg.base_url = vllm_base_url
            msg.model = vllm_model
        msg.prompt = VLLM_PROMPT
        msg.query = query
        msg.image = self.bridge.cv2_to_imgmsg(image, "bgr8")
         
        self.get_logger().info('vision: %s' % query)
        self.get_logger().info('send image')
        res = self.send_request(self.set_vllm_content_client, msg)
        vllm_result = res.message
        self.get_logger().info('vllm_result: %s' % vllm_result)
        if 'object' in vllm_result: 
            if vllm_result.startswith("```") and vllm_result.endswith("```"):
                vllm_result = vllm_result.strip("```").replace("json\n", "").strip()
            # self.get_logger().info('vllm_result: %s' % vllm_result)
            vllm_result = json.loads(vllm_result[vllm_result.find('{'):vllm_result.find('}')+1])
            box = vllm_result['xyxy']
            h, w = image.shape[:2]
            if language == 'Chinese':
                box = self.client.data_process(box, w, h)
                self.get_logger().info('box: %s' % str(box))
            else:
                box = [int(box[0] * w), int(box[1] * h), int(box[2] * w), int(box[3] * h)]
            cv2.rectangle(image, (box[0], box[1]), (box[2], box[3]), (255, 0, 0), 2, 1)
            return box
        else:
            msg = String()
            msg.data = str(vllm_result)
            self.tts_text_pub.publish(msg)
            return []

    def pick(self, query):
        self.send_request(self.set_pick_client, Trigger.Request())
        image = self.image_queue.get(block=True)
        box = self.get_object_position(query, image)
        if box:
            msg = SetBox.Request()
            msg.x_min = box[0]
            msg.y_min = box[1]
            msg.x_max = box[2]
            msg.y_max = box[3]
            self.send_request(self.set_box_client, msg)

    def place(self, query):
        self.get_logger().info('place: %s' % query)
        self.send_request(self.set_place_client, Trigger.Request())
        image = self.image_queue.get(block=True)
        h, _ = image.shape[:2]
        image = image[:int(h*0.7), :] #0:300
        box = self.get_object_position(query, image)
        if box:
            msg = SetBox.Request()
            msg.x_min = box[0]
            msg.y_min = box[1]
            msg.x_max = box[2]
            msg.y_max = box[3]
            self.send_request(self.set_box_client, msg)

    def play_audio_finish_callback(self, msg):
        self.play_audio_finish = msg.data

    def process(self):
        while self.running:
            if self.llm_result:
                self.interrupt = False
                msg = String()
                if 'action' in self.llm_result: # 如果有对应的行为返回那么就提取处理
                    result = eval(self.llm_result[self.llm_result.find('{'):self.llm_result.find('}')+1])
                    if 'response' in result:
                        msg.data = result['response']
                        self.tts_text_pub.publish(msg)
                    if 'action' in result:
                        action = result['action']
                        self.get_logger().info(f'vllm action: {action}')
                        for a in action:
                            if 'move' in a: 
                                self.reach_goal = False
                                res = eval(f'self.{a}')
                                if res:
                                    while not self.reach_goal:
                                        # if self.interrupt:
                                            # self.get_logger().info('interrupt')
                                            # break
                                        # self.get_logger().info('waiting for reach goal')
                                        time.sleep(0.01)
                                else:
                                    self.get_logger().info('cannot move to %s' % a)
                                    break
                            elif 'pick' in a or 'place' in a:
                                time.sleep(3.0)
                                eval(f'self.{a}')
                                self.transport_action_finish = False
                                while not self.transport_action_finish:
                                    # if self.interrupt:
                                        # self.get_logger().info('interrupt')
                                        # break
                                    time.sleep(0.01)
                            # if self.interrupt:
                                # self.get_logger().info('interrupt')
                                # break
                else: # 没有对应的行为，只回答
                    msg.data = self.llm_result
                    self.tts_text_pub.publish(msg)
                self.action_finish = True
                self.llm_result = ''
            else:
                time.sleep(0.01)
            if self.play_audio_finish and self.action_finish:
                self.play_audio_finish = False
                self.action_finish = False
                msg = SetBool.Request()
                msg.data = True
                self.send_request(self.awake_client, msg)
                # msg = SetInt32.Request()
                # msg.data = 2
                # self.send_request(self.set_mode_client, msg)
        rclpy.shutdown()

    def image_callback(self, ros_image):
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        rgb_image = np.array(cv_image, dtype=np.uint8)

        if self.image_queue.full():
            # 如果队列已满，丢弃最旧的图像
            self.image_queue.get()
            # 将图像放入队列
        self.image_queue.put(rgb_image)

def main():
    node = VLLMNavigation('vllm_navigation')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()

if __name__ == "__main__":
    main()
