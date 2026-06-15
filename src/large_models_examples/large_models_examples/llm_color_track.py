#!/usr/bin/env python3
# encoding: utf-8
# @Author: Aiden
# @Date: 2025/03/06
import os
import re
import time
import rclpy
import threading
from rclpy.node import Node
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger, SetBool, Empty

from speech import speech
from interfaces.srv import SetString as SetColor
from large_models.config import *
from large_models_msgs.srv import SetModel, SetString, SetInt32

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

if os.environ["ASR_LANGUAGE"] == 'Chinese':
    PROMPT = '''
#角色任务
你是一款智能陪伴机器人，需要根据输入的内容，生成对应的json指令。

##要求
1.用户输入的任何内容，都需要在动作函数库中寻找对应的指令，并输出对应的json指令。
2.为每个动作序列编织一句精炼（10至30字）、风趣且变化无穷的反馈信息，让交流过程妙趣横生。
3.直接输出json结果，不要分析，不要输出多余内容。
4.格式：{"action": ["xx", "xx"], "response": "xx"}

##特别注意:
- "action"键下承载一个按执行顺序排列的函数名称字符串数组，当找不到对应动作函数时action输出[]。 
- "response"键则配以精心构思的简短回复，完美贴合上述字数与风格要求。  
 
##动作函数库
- 追踪不同颜色的物体：color_track('red') 

##任务示例
输入：跟踪绿色物体
输出：{"action": ["color_track('green')"], "response": "收到"}
    '''
else:
    PROMPT = '''
**Role
You are an intelligent companion robot. Your job is to generate corresponding JSON commands based on the user’s input.

**Requirements
- For every user input, search the action function library for matching commands and return the corresponding JSON instruction.
- Craft a witty, ever-changing, and concise response (between 10 to 30 characters) for each action sequence to make interactions lively and fun.
- Output only the JSON result — do not include explanations or any extra text.
- Output format:{"action": ["xx", "xx"], "response": "xx"}

**Special Notes
The "action" key holds an array of function name strings arranged in execution order. If no match is found, return an empty array [].
The "response" key must contain a cleverly worded, short reply (10–30 characters), adhering to the tone and length guidelines above.

**Action Function Library
Track an object of a specific color: color_track('red')

**Example
Input: Track a green object
Output:
{"action": ["color_track('green')"], "response": "Got it!"}
    '''

class LLMColorTrack(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)
        
        self.action = []
        self.stop = True
        self.llm_result = ''
        # self.llm_result = '{"action": ["color_track(\'blue\')"], "response": "ok！"}'
        self.running = True
        self.action_finish = False
        self.play_audio_finish = False
        
        self.declare_parameter('interruption', False)
        self.interruption = self.get_parameter('interruption').value
        self.asr_mode = os.environ.get("ASR_MODE", "online").lower()

        timer_cb_group = ReentrantCallbackGroup()
        self.tts_text_pub = self.create_publisher(String, 'tts_node/tts_text', 1)
        self.create_subscription(String, 'agent_process/result', self.llm_result_callback, 1)
        self.create_subscription(Bool, 'vocal_detect/wakeup', self.wakeup_callback, 1, callback_group=timer_cb_group)
        self.create_subscription(Bool, 'tts_node/play_finish', self.play_audio_finish_callback, 1, callback_group=timer_cb_group)
        self.awake_client = self.create_client(SetBool, 'vocal_detect/enable_wakeup')
        self.awake_client.wait_for_service()
        self.set_mode_client = self.create_client(SetInt32, 'vocal_detect/set_mode')
        self.set_mode_client.wait_for_service()
        
        self.set_model_client = self.create_client(SetModel, 'agent_process/set_model')
        self.set_model_client.wait_for_service()
        self.set_prompt_client = self.create_client(SetString, 'agent_process/set_prompt')
        self.set_prompt_client.wait_for_service()
        self.enter_client = self.create_client(Trigger, 'object_tracking/enter')
        self.enter_client.wait_for_service()
        self.start_client = self.create_client(SetBool, 'object_tracking/set_running')
        self.start_client.wait_for_service()
        self.set_target_client = self.create_client(SetColor, 'object_tracking/set_color')
        self.set_target_client.wait_for_service()
        
        self.timer = self.create_timer(0.0, self.init_process, callback_group=timer_cb_group)

    def get_node_state(self, request, response):
        return response

    def init_process(self):
        self.timer.cancel()
        
        msg = SetModel.Request()
        msg.model_type = 'llm'
        if self.asr_mode == "offline":
            msg.model = 'qwen3:0.6b'
            msg.base_url = ollama_host
        else:
            msg.model = llm_model
            msg.api_key = api_key 
            msg.base_url = base_url
        self.send_request(self.set_model_client, msg)

        msg = SetString.Request()
        msg.data = PROMPT
        self.send_request(self.set_prompt_client, msg)
        
        init_finish = self.create_client(Trigger, 'object_tracking/init_finish')
        init_finish.wait_for_service()
        self.send_request(self.enter_client, Trigger.Request())
        speech.play_audio(start_audio_path)
        threading.Thread(target=self.process, daemon=True).start()
        self.create_service(Empty, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')
        self.get_logger().info('\033[1;32m%s\033[0m' % PROMPT)


    def send_request(self, client, msg):
        future = client.call_async(msg)
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()

    def wakeup_callback(self, msg):
        if msg.data and self.llm_result:
            self.get_logger().info('wakeup interrupt')
            self.send_request(self.enter_client, Trigger.Request())
            self.stop = True
        elif msg.data and not self.stop:
            self.get_logger().info('wakeup interrupt')
            self.send_request(self.enter_client, Trigger.Request())
            self.stop = True

    def llm_result_callback(self, msg):
        self.llm_result = msg.data

    def play_audio_finish_callback(self, msg):
        self.play_audio_finish = msg.data

    def process(self):
        while self.running:
            if self.llm_result:
                msg = String()
                if 'action' in self.llm_result: # 如果有对应的行为返回那么就提取处理
                    result = eval(self.llm_result[self.llm_result.find('{'):self.llm_result.find('}')+1])
                    if 'action' in result:
                        text = result['action']
                        # 使用正则表达式提取括号中的所有字符串
                        pattern = r"color_track\('([^']+)'\)"
                        # 使用re.search找到匹配的结果
                        for i in text:
                            match = re.search(pattern, i)
                            # 提取结果
                            if match:
                                # 获取所有的参数部分（括号内的内容）
                                color = match.group(1)
                                self.get_logger().info(str(color))
                                color_msg = SetColor.Request()
                                color_msg.data = color
                                self.send_request(self.set_target_client, color_msg)
                                # 开启分拣
                                start_msg = SetBool.Request()
                                start_msg.data = True 
                                self.send_request(self.start_client, start_msg)
                    if 'response' in result:
                        msg.data = result['response']
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
                if self.interruption:
                    msg = SetInt32.Request()
                    msg.data = 2
                    self.send_request(self.set_mode_client, msg)
                msg = SetBool.Request()
                msg.data = True
                self.send_request(self.awake_client, msg)
                self.stop = False
        rclpy.shutdown()

def main():
    node = LLMColorTrack('llm_color_track')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()

if __name__ == "__main__":
    main()
