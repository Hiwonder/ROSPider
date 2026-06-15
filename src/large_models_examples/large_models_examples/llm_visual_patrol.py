#!/usr/bin/env python3
# encoding: utf-8
# @Author: Aiden
# @Date: 2024/11/18
import os
import re
import time
import rclpy
import threading
from speech import speech
from rclpy.node import Node
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger, SetBool, Empty

from interfaces.srv import SetString as SetColor
from large_models.config import *
from large_models_msgs.srv import SetModel, SetString, SetInt32

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

if os.environ["ASR_LANGUAGE"] == 'Chinese':
    PROMPT = '''
##角色任务
你是一款智能机器人，需要根据输入的内容，生成对应的json指令。

##要求
1.用户输入的任何内容，都需要在动作函数库中寻找对应的指令，并输出对应的json指令。
2.为每个动作序列编织一句精炼（10至30字）、风趣且变化无穷的反馈信息，让交流过程妙趣横生。
3.直接输出json结果，不要分析，不要输出多余内容。
4.一共有四种颜色作为目标：红色(red)、绿色(green) 、蓝色(blue)、黑色(black)。
5.格式：{"action": ["xx", "xx"], "response": "xx"}

##特别注意
- "action"键下承载一个按执行顺序排列的函数名称字符串数组，当找不到对应动作函数时action输出[]。 
- "response"键则配以精心构思的简短回复，完美贴合上述字数与风格要求。  
 
##动作函数库
- 巡不同颜色的线：line_following('black') 

##任务示例
输入：沿着黑线走
输出：{"action": ["line_following('black')"], "response": "收到"}
'''
else:
    PROMPT = '''
**Role
You are a smart robot that generates corresponding JSON commands based on user input.

**Requirements
- For every user input, search for matching commands in the action function library and output the corresponding JSON instruction.
- For each action sequence, craft a witty and creative response (10 to 30 characters) to make interactions delightful.
- Directly return the JSON result — do not include any explanations or extra text.
- There are four target colors: red, green, blue, and black.
- Format:
{
  "action": ["xx", "xx"],
  "response": "xx"
}

**Special Notes
The "action" field should contain a list of function names as strings, ordered by execution. If no matching action is found, output an empty list: [].
The "response" field should provide a concise and charming reply, staying within the word and tone guidelines.

**Action Function Library
Follow a line of a given color: line_following('black')

**Example
Input: Follow the red line
Output:
{
  "action": ["line_following('red')"],
  "response": "Roger that!"
}
    '''

class LLMColorTrack(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)
        
        self.action = []
        self.stop = True
        self.llm_result = ''
        # self.llm_result = '{"action": ["line_following(\'black\')"], "response": "ok！"}'
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
        self.enter_client = self.create_client(Trigger, 'line_following/enter')
        self.enter_client.wait_for_service()
        self.start_client = self.create_client(SetBool, 'line_following/set_running')
        self.start_client.wait_for_service()
        self.set_target_client = self.create_client(SetColor, 'line_following/set_color')
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
      
        init_finish = self.create_client(Trigger, 'line_following/init_finish')
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
                        pattern = r"line_following\('([^']+)'\)"
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
                msg = SetBool.Request()
                msg.data = True
                self.send_request(self.awake_client, msg)
                if self.interruption:
                    msg = SetInt32.Request()
                    msg.data = 2
                    self.send_request(self.set_mode_client, msg)
                self.stop = False
        rclpy.shutdown()

def main():
    node = LLMColorTrack('llm_line_following')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()

if __name__ == "__main__":
    main()
