#!/usr/bin/env python3
# encoding: utf-8
# @Author: Aiden
# @Date: 2025/03/06
import os
import re
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
    你是一个智能六足机器人，负责解析用户的移动指令，提取动作和参数。

    ##动作函数库
    - "move(direction, distance, duration)"
    - direction (方向): 'forward', 'backward',  'turn_left', 'turn_right', 'shift_left', 'shift_right','stop'
    - step (距离, 单位步): 一个数字, 如果用户没说距离则为2

    ##要求
    1. 解析用户意图，生成包含一个或多个 move 函数调用的 action 列表。
	    2. **严格遵守**：当用户指令中包含**时间**（如“前进3步”），则**step参数必须为3**。
    4. 直接输出json结果，不要分析。
    5. 格式: {"action": ["move('forward', 3)"], "response": "好的, 向前走3步"}
    

    ##任务示例
    输入：向前走 1 步，然后向左转，
    输出：{"action": ["move('forward', 1)", "move('turn_left', 2)"], "response": "收到，马上执行！"}
	    输入：后退 2 步, 然后向左平移 3 步, 然后再右转
    输出：{"action": ["move('backward', 2)","move('shift_left', 3)","move('turn_right', 2)"], "response": "好嘞，出发！"}
    '''
else:
    PROMPT = '''
    ## Role and Task
    You are an intelligent hexapod robot, responsible for parsing user movement commands and extracting actions and parameters.

    ## Action Function Library
    - "move(direction, step)"
    - direction (Direction): 'forward', 'backward', 'turn_left', 'turn_right', 'shift_left', 'shift_right', 'stop'
    - step (Distance, unit: steps): A number, default is 2 if user doesn't specify distance

    ## Requirements
    1. Parse user intent and generate an action list containing one or more move function calls.
    2. **Strict Compliance**: When user command includes **distance** (e.g., "move forward 3 steps"), the **step parameter must be 3**.
    3. Output JSON result directly, no analysis.
    4. Format: {"action": ["move('forward', 3)"], "response": "OK, moving forward 3 steps"}

    ## Task Examples
    Input: Move forward 1 step, then turn left
    Output: {"action": ["move('forward', 1)", "move('turn_left', 2)"], "response": "Received, executing immediately!"}
    Input: Move backward 2 steps, then shift left 3 steps, then turn right
    Output: {"action": ["move('backward', 2)", "move('shift_left', 3)", "move('turn_right', 2)"], "response": "Alright, let's go!"}
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
        
    def parse_action(self, action_str):
        """
        辅助函数，用于解析 "move('direction', )" 格式的字符串。
        使用正则表达式提取参数，更健壮。
        """
        # 匹配 move(...) 中的内容
        match = re.search(r"move\((.*)\)", action_str)
        if not match:
            return None, 0, 0

        # 提取参数字符串，例如 "'forward', 5, 0"
        params_str = match.group(1)
        
        # 分割参数
        params = [p.strip() for p in params_str.split(',')]
        
        # 提取并转换类型
        direction = params[0].strip("'\"") # 去掉字符串参数的引号
        step = int(params[1])
        
        return direction, step


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
                    # 循环执行动作列表
                    for action_str in action_list:
                        direction, step = self.parse_action(action_str)
                        
                        if direction is None:
                            continue # 解析失败，跳过这个动作
                        
                        # 根据方向设置角度
                        if direction =='forward':
                            direction = 0.0
                            rotation = 0.0
                        elif direction == 'backward':
                            direction = 3.14
                            rotation = 0.0
                        elif direction == 'shift_left':
                            direction = 1.57
                            rotation = 0.0
                        elif direction == 'shift_right':
                            direction = 2.355
                            rotation = 0.0
                        elif direction == 'turn_left':
                            direction = 0.0
                            rotation = 0.2
                        elif direction == 'turn_right':
                            direction = 0.0
                            rotation = -0.2

                        if rotation != 0.0:
                            self.controller.traveling(gait=2, stride=0.0, height=30.0, direction=direction, rotation=rotation, time=1.0, steps=step, relative_height=True, interrupt=True )
                            time.sleep(1)
                        else:
                            self.controller.traveling(gait=2, stride=45.0, height=30.0, direction=direction, rotation=rotation, time=1.0, steps=step, relative_height=True, interrupt=True )
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
