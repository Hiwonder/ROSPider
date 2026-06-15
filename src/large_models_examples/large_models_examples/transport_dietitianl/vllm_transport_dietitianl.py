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

'''
客厅   厨房

卧室   卫生间

'''

if os.environ["ASR_LANGUAGE"] == 'Chinese':
    position_dict = {"客厅的遥控器": [0.9, 1.0, 0.0, 0.0, 0.0],
                     "客厅的药箱": [0.9, 0.5, 0.0, 0.0, 0.0],
                     "客厅": [0.85, 1.0, 0.0, 0.0, 0.0],
                     "卧室": [-0.35, 0.7, 0.0, 0.0, 90.0],
                     "卧室的洗浴用品": [-0.9, 0.7, 0.0, 0.0, 90.0],
                     "浴室": [-0.62, -0.75, 0.0, 0.0, -90.0],
                     "厨房的早餐": [0.86, -0.8, 0.0, 0.0, -90.0]}


    LLM_PROMPT = '''
# 角色任务
作为智能管家，你能深刻的洞悉用户的指令。

## 技能细则
* 分析用户指令，将一次抓取和一次放置作为一个动作
* 有较强的逻辑能力, 且能理解物体的修饰
* 能准确分解出抓取和放置

## 要求与限制
1.根据输入的内容，在函数库中找到对应的指令，并输出对应的指令。
2.多个动作需要放到列表里
3.为动作序列编织一句精炼（10至30字）、风趣且变化无穷的反馈信息，让交流过程妙趣横生。
4.地点可能是下面这些：客厅的遥控器，客厅的药箱，客厅，卧室，卧室的洗浴用品，浴室，厨房，厨房的早餐。
5.直接输出json格式的数据，不要分析，不要输出多余内容。
6.放置前一定要有抓取
7.格式：{"action":"xx", "response":"xx"}

## 可用函数与操作：
* 拿起物体: pick()
* 放下物体：place()
* 移动到指定位置：move('卧室')

## 示例：
输入：我在卧室，将客厅的药箱拿给我
输出：{"action": ["move('客厅的药箱')", "pick()", "move('卧室')", "place()"], "response": "好的，我这就去把药箱拿过来"}
输入：将药箱放回客厅
输出：{"action": ["pick()", "move('客厅的药箱')", "place()"], "response": "明白，把药箱放回客厅"}
输入：我在卧室，把药箱放回客厅，把客厅药箱旁边的遥控器拿给我
输出：{"action": ["pick()", move("客厅的药箱"), "place()", "move('客厅的遥控器')", "pick()", "move('卧室')", "place()"], "response": "明白，把药箱放回客厅, 把遥控器拿给你"}
    '''
else:
    position_dict = {"living room remote control": [0.9, 1.0, 0.0, 0.0, 0.0],
                     "living room medicine box": [0.9, 0.5, 0.0, 0.0, 0.0],
                     "living room": [0.85, 1.0, 0.0, 0.0, 0.0],
                     "bedroom": [-0.35, 0.7, 0.0, 0.0, 90.0],
                     "bedroom toiletries": [-0.9, 0.7, 0.0, 0.0, 90.0],
                     "bathroom": [-0.62, -0.75, 0.0, 0.0, -90.0],
                     "kitchen breakfast": [0.86, -0.8, 0.0, 0.0, -90.0]}
    LLM_PROMPT = '''
You are an intelligent butler who deeply understands user instructions.

**Skills
- Parse each command, treating one pick-and-place as a composite action.
- Strong logical reasoning and mastery of object modifiers.
- Accurately identify which object to pick and which to place.

**Requirements & Constraints
- Translate the user’s request into the available library functions and output only those calls.
- Bundle multiple steps into a JSON array under "action".
- Provide a concise (10–30 characters), witty, ever-changing feedback string under "response".
- Valid locations are exactly:living room remote control, living room medicine box, living room, bedroom, bedroom toiletries, bathroom, kitchen breakfast

**Output only the final JSON—no analysis or extra text.
Every placement must be preceded by a pick.
JSON format (exactly):
{
  "action": [ /* function calls */ ],
  "response": "/* witty feedback */"
}

**Available Functions
- pick()
- place()
- move("location")

**Examples
Input: "I’m in the bedroom; bring me the medicine box from the living room."
Output:
{
  "action": [
    "move('living room medicine box')",
    "pick()",
    "move('bedroom')",
    "place()"
  ],
  "response": "On my way with the medicine box!"
}
Input: "Put the medicine box back in the living room."
Output:
{
  "action": [
    "pick()",
    "move('living room medicine box')",
    "place()"
  ],
  "response": "Got it—returning the medicine box now."
}
Input: "I’m in the bedroom, put the medicine box back in the living room, then bring me the remote control next to that box."
Output:
{
  "action": [
    "pick()",
    "move('living room medicine box')",
    "place()",
    "move('living room remote control')",
    "pick()",
    "move('bedroom')",
    "place()"
  ],
  "response": "Done—box returned, now fetching the remote!"
    '''

class VLLMTransportDietitianl(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)
        
        self.action = []
        self.response_text = ''
        self.llm_result = ''
        self.action_finish = False
        self.transport_action_finish = False
        self.play_audio_finish = False
        self.running = True
        self.reach_goal = False
        self.interrupt = False
        self.bridge = CvBridge()
        self.image_queue = queue.Queue(maxsize=2)
        timer_cb_group = ReentrantCallbackGroup()
        self.client = speech.OpenAIAPI(api_key, base_url)
        self.tts_text_pub = self.create_publisher(String, 'tts_node/tts_text', 1)
        self.create_subscription(Image, 'automatic_transport/image_result', self.image_callback, 1)
        self.create_subscription(String, 'agent_process/result', self.llm_result_callback, 1)
        self.create_subscription(Bool, 'vocal_detect/wakeup', self.wakeup_callback, 1)
        self.create_subscription(Bool, 'tts_node/play_finish', self.play_audio_finish_callback, 1, callback_group=timer_cb_group)
        self.create_subscription(Bool, 'navigation_controller/reach_goal', self.reach_goal_callback, 1)
        self.create_subscription(Bool, 'automatic_transport/action_finish', self.action_finish_callback, 1)
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
        self.set_pick_client = self.create_client(Trigger, 'automatic_transport/pick')
        self.set_pick_client.wait_for_service()
        self.set_place_client = self.create_client(Trigger, 'automatic_transport/place')
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
        if position not in position_dict:
            return False
        p = position_dict[position]
        msg.data.x = float(p[0])
        msg.data.y = float(p[1])
        msg.data.roll = p[2]
        msg.data.pitch = p[3]
        msg.data.yaw = p[4]
        self.send_request(self.set_pose_client, msg)
        return True

    def reach_goal_callback(self, msg):
        self.get_logger().info('reach goal')
        self.reach_goal = msg.data

    def action_finish_callback(self, msg):
        self.get_logger().info('action finish')
        self.transport_action_finish = msg.data

    def pick(self):
        self.send_request(self.set_pick_client, Trigger.Request())

    def place(self):
        self.send_request(self.set_place_client, Trigger.Request())

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
                                        time.sleep(0.01)
                                else:
                                    self.get_logger().info('cannot move to %s' % a)
                                    break
                            elif 'pick' in a or 'place' in a:
                                time.sleep(3.0)
                                eval(f'self.{a}')
                                self.transport_action_finish = False
                                while not self.transport_action_finish:
                                    time.sleep(0.01)
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
    node = VLLMTransportDietitianl('vllm_transport_dietitianl')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()

if __name__ == "__main__":
    main()
