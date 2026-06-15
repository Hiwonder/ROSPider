#!/usr/bin/env python3
# encoding: utf-8
# @Author: Aiden
# @Date: 2025/03/06
import os
import time
import rclpy
import threading
from speech import speech
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger, SetBool, Empty

from large_models.config import *
from large_models_msgs.srv import SetModel, SetString, SetInt32

from controller.controller_client import ControllerClient
from servo_controller_msgs.msg import ServosPosition, ServoPosition
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from servo_controller.bus_servo_control import set_servo_position

if os.environ["ASR_LANGUAGE"] == 'Chinese': 
    PROMPT = '''
##角色任务
你是一个智能六足机器人，可以通过 direction 控制移动方向，单位为rad/s，通过 rotation 控制旋转方向,单位rad/s，通过 step 控制移动的步数。需要根据输入的内容，生成对应的指令。

##要求
1.确保速度范围正确：
方向：direction ∈ [0, 3.14]，（0为前进，逆时针方向递增）
旋转角度：rotation ∈ [-0.2, 0.2]（逆时针为正, 顺时针为负）
移动步数：step
2.顺序执行多个动作，输出一个 包含多个移动指令的 action 列表。
3.旋转时step默认为4
4.为每个动作序列编织一句精炼（5至10字）、风趣且变化无穷的反馈信息，让交流过程妙趣横生。
5.直接输出json结果，不要分析，不要输出多余内容。
6.格式：
{  
  "action": [[direction_1, rotation_1, step_1], [direction_2, rotation_2, step_2], ...],  
  "response": "xx"  
}  
7.很强的数学计算能力

##特别注意
- "action"键下承载一个按执行顺序排列的函数名称字符串数组，当找不到对应动作函数时action输出[]。 
- "response"键则配以精心构思的简短回复，完美贴合上述字数与风格要求。 

##任务示例
输入：后退两步
输出：{"action": [[3.14, 0.0, 2],], "response": "向后走两步，出发！"}
输入：向前走两步，然后顺时针旋转 
输出：{"action": [[0.0, 0.0, 2], [0.0, -0.2, 4]], "response": "向前走两步，然后顺时针旋转，出发！"}
输入：向后走两步，向左平移两步，然后逆时针旋转
输出：{"action": [[3.14, 0.0, 2], [1.57, 0.0, 2], [0.0, 0.2, 4]], "response": "好嘞"}
    '''
else:
    PROMPT = '''
**Role
You are an intelligent hexapod robot that can control the direction of movement in rad/s through direction, control the direction of rotation in rad/s through rotation, and control the number of steps taken through step. Corresponding instructions need to be generated based on the input content.

##Requirement
1. Ensure that the speed range is correct:
Direction: direction ∈ [0, 3.14], (0 represents forward, increasing counterclockwise)
Rotation angle: Rotation ∈ [-0.2, 0.2] (counterclockwise is positive, clockwise is negative)
Number of moving steps: step
2. Execute multiple actions in sequence and output an action list containing multiple movement instructions.
3. When rotating, step defaults to 4
4. Weave a concise (5 to 10 words), witty, and constantly changing feedback message for each action sequence, making the communication process full of fun.
5. Directly output JSON results without analysis or unnecessary content.
6. Format:
{  
"action": [[direction_1, rotation_1, step_1], [direction_2, rotation_2, step_2], ...],  
"response": "xx"  
}  
7. Strong mathematical calculation ability

##Special attention
-The 'action' key carries an array of function name strings arranged in order of execution. When the corresponding action function cannot be found, the action outputs [].  
-The 'response' button is paired with carefully crafted short replies, perfectly fitting the word count and style requirements mentioned above.  

##Task Example
Input: Take two steps forward and then rotate clockwise
Output: {"action": [0.0, 0.0, 2], [0.0, -0.2, 4]], "response": "Take two steps forward, then rotate clockwise, start! "}
Input: Take two steps backward, move two steps left, and then rotate counterclockwise
Output: {"action": [3.14, 0.0, 2], [1.57, 0.0, 2], [0.0, 0.2, 4], "response": "Okay"}
    '''

class LLMControlMove(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)
        
        self.action = []
        self.llm_result = ''
        self.running = True
        self.interrupt = False
        self.action_finish = False
        self.play_audio_finish = False
        
        self.declare_parameter('interruption', False)
        self.interruption = self.get_parameter('interruption').value
        self.asr_mode = os.environ.get("ASR_MODE", "online").lower()
        
        self.controller = ControllerClient()
        timer_cb_group = ReentrantCallbackGroup()
        self.joints_pub = self.create_publisher(ServosPosition, 'servo_controller', 1)
        self.tts_text_pub = self.create_publisher(String, 'tts_node/tts_text', 1)
        self.create_subscription(String, 'agent_process/result', self.llm_result_callback, 1)
        self.create_subscription(Bool, 'vocal_detect/wakeup', self.wakeup_callback, 1, callback_group=timer_cb_group)
        self.create_subscription(Bool, 'tts_node/play_finish', self.play_audio_finish_callback, 1, callback_group=timer_cb_group)
        self.set_model_client = self.create_client(SetModel, 'agent_process/set_model')
        self.set_model_client.wait_for_service()

        self.awake_client = self.create_client(SetBool, 'vocal_detect/enable_wakeup')
        self.awake_client.wait_for_service()
        self.set_mode_client = self.create_client(SetInt32, 'vocal_detect/set_mode')
        self.set_mode_client.wait_for_service()
        self.set_prompt_client = self.create_client(SetString, 'agent_process/set_prompt')
        self.set_prompt_client.wait_for_service()

        self.timer = self.create_timer(0.0, self.init_process, callback_group=timer_cb_group)

    def get_node_state(self, request, response):
        return response

    def init_process(self):
        self.timer.cancel()
        set_servo_position(self.joints_pub, 1, ((24, 500), (23, 500), (22, 150), (21, 130), (20, 720), (19, 500)))
        self.controller.traveling(gait=-2, time=1, steps=0)
        time.sleep(1)

        msg = SetModel.Request()
        msg.model_type = 'llm'
        if self.asr_mode == "offline":
            msg.model = 'qwen3:1.7b'
            msg.base_url = ollama_host
        else:
            msg.model = llm_model
            msg.api_key = api_key 
            msg.base_url = base_url
        self.send_request(self.set_model_client, msg)

        msg = SetString.Request()
        msg.data = PROMPT
        self.send_request(self.set_prompt_client, msg)

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
        if self.llm_result:
            self.get_logger().info('wakeup interrupt')
            self.interrupt = msg.data

    def llm_result_callback(self, msg):
        self.llm_result = msg.data

    def play_audio_finish_callback(self, msg):
        msg = SetBool.Request()
        msg.data = True
        self.send_request(self.awake_client, msg)

        self.play_audio_finish = msg.data

    def process(self):
        while self.running:
            if self.llm_result:
                msg = String()
                if 'action' in self.llm_result:  # 如果有对应的行为返回那么就提取处理
                    result = eval(self.llm_result[self.llm_result.find('{'):self.llm_result.find('}') + 1])
                    self.get_logger().info(str(result))
                    action_list = []
                    if 'action' in result:
                        action_list = result['action']
                    if 'response' in result:
                        response = result['response']
                    msg.data = response
                    self.tts_text_pub.publish(msg)
                    for i in action_list:
                        if float(i[1]) != 0.0:
                            self.controller.traveling(gait=2, stride=0.0, height=30.0, direction=i[0], rotation=i[1], time=1.0, steps=i[2], relative_height=True, interrupt=True )
                            time.sleep(1)
                        else:
                            self.controller.traveling(gait=2, stride=45.0, height=30.0, direction=i[0], rotation=i[1], time=1.0, steps=i[2], relative_height=True, interrupt=True )
                            time.sleep(1)

                        if self.interrupt:
                            self.interrupt = False
                            self.controller.traveling(gait=-2, time=1, steps=0)
                            break
                else:  # 没有对应的行为，只回答
                    response = self.llm_result
                    msg.data = response
                    self.tts_text_pub.publish(msg)
                self.action_finish = True 
                self.llm_result = ''
            else:
                time.sleep(0.01)
            if self.play_audio_finish and self.action_finish:
                self.play_audio_finish = False
                self.action_finish = False

        rclpy.shutdown()

def main():
    node = LLMControlMove('llm_control_move')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
 
if __name__ == "__main__":
    main()
