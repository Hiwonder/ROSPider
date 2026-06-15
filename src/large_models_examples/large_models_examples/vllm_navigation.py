
#!/usr/bin/env python3
# encoding: utf-8
# @Author: Aiden
# @Date: 2024/11/18
import os
import re
import time
import json
import rclpy
import threading
from speech import speech
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger, SetBool, Empty

from large_models.config import *
from large_models_msgs.srv import SetModel, SetContent, SetString, SetInt32

from interfaces.srv import SetPose2D
from rclpy.executors import MultiThreadedExecutor
from servo_controller_msgs.msg import ServosPosition
from rclpy.callback_groups import ReentrantCallbackGroup
from servo_controller.action_group_controller import ActionGroupController

language = os.environ["ASR_LANGUAGE"]
if language == 'Chinese':
    # position_dict = {"厨房": [2.8, -2.8, 0.0, 0.0, 0.0], #xyzrpy, m,deg
    #                  "前台": [2.8, -2.7, 0.0, 0.0, 0.0],
    #                  "卧室": [0.0, 0.0, 0.0, 0.0, 0.0],
    #                  "动物园": [1.3, 0.37, 0.0, 0.0, 0.0],
    #                  "航天基地": [1.58, -0.74, 0.0, 0.0, -48.0],
    #                  "足球场": [0.32, -0.65, 0.0, 0.0, -90.0],
    #                  "原点": [0.0, 0.0, 0.0, 0.0, 0.0],
    #                  "家": [0.1, 0.4, 0.0, 0.0, 0.0]
    #                  }

    position_dict = {"动物园": [0.58, -0.42, 0.0, 0.0, 0.0], #xyzrpy, m,deg
                     "停车场": [2.8, -2.7, 0.0, 0.0, 0.0],
                     "原点": [0.0, 0.0, 0.0, 0.0, 0.0],
                     }
    LLM_PROMPT = '''
##角色任务
你是一款智能六足机器人，带有摄像头和扬声器，能通过播放音频回答问题，需要根据输入的内容，生成对应的json指令。

##要求
1.用户输入的任何内容，都需要在动作函数库中寻找对应的指令，并输出对应的json指令。
2.为每个动作序列编织一句精炼（5至20字）、风趣且变化无穷的反馈信息，让交流过程妙趣横生。
3.直接输出json结果，不要分析，不要输出多余内容。
4.格式：{"action": ["xx", "xx"], "response": "xx"}

##特别注意
- "action"键下承载一个按执行顺序排列的函数名称字符串数组，当找不到对应动作函数时action输出[]。 
- "response"键则配以精心构思的简短回复，完美贴合上述字数与风格要求。 
 
##动作函数库
- 移动指定地点：move('厨房') 
- 回到出发点：move('原点') 
- 检测画面：vision('你看到了什么') 
- 播放音频：play_audio() 

##任务示例
输入：前往动物园看看有哪些动物，然后回来告诉我
输出：{"action": ["move('动物园')"，"vision('有哪些动物')", "move("原点")", "play_audio()"], "response": "收到"}
    '''

    VLLM_PROMPT = '''
你作为我的机器人管家，能认真视察周围的情况，对于给出的指令，要做出贴心且人性化的回答, 不要进行反问，字数在20到40字间。
    '''
else:
    position_dict = {"kitchen": [2.8, -2.8, 0.0, 0.0, 0.0], #xyzrpy, m,deg
                     "front desk": [2.8, -2.7, 0.0, 0.0, 0.0],
                     "bedroom": [0.0, 0.0, 0.0, 0.0, 0.0],
                     "zoo": [1.3, 0.37, 0.0, 0.0, 0.0],
                     "space base": [1.58, -0.74, 0.0, 0.0, -48.0],
                     "football field": [0.32, -0.65, 0.0, 0.0, -90.0],
                     "origin": [0.0, 0.0, 0.0, 0.0, 0.0],
                     "home": [0.1, 0.4, 0.0, 0.0, 0.0]
                     }

    LLM_PROMPT = '''
**Role
You are a smart navigation vehicle equipped with a camera and speaker. You can move to different places, analyze visual input, and respond by playing audio. Based on user input, you need to generate the corresponding JSON command.

**Requirements
- For any user input, look up corresponding functions from the Action Function Library, and generate the proper JSON output.
- For each action sequence, include a concise (5–20 characters) and witty, varied response to make the interaction lively and engaging.
- Output only the JSON result, no analysis or extra text.
- Output format:
{
  "action": ["xx", "xx"],
  "response": "xx"
}

**Special Notes
The "action" field contains an ordered list of function names to be executed in sequence. If no matching function is found, return: "action": [].
The "response" field should contain a carefully crafted, short, humorous, and varied message (5–20 characters).

**Action Function Library
Move to a specified place: move('kitchen')
Return to starting point: move('origin')
Analyze current view: vision('What do you see?')
Play audio response: play_audio()

**Example
Input: Go to the front desk to see if the door is closed, and then come back and tell me
Output:
{
  "action": ["move('front desk')", "vision('Is the door closed?')", "move("origin")", "play_audio()"],
  "response": "On it, reporting soon!"
}
    '''

    VLLM_PROMPT = '''
As my robot butler, you should carefully observe the surrounding situation and give considerate and humane responses to the instructions given. Do not ask questions in return. The number of words should be between 20 and 40.
    '''

class VLLMNavigation(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)
        
        self.action = []
        self.response_text = ''
        self.llm_result = ''
        self.play_audio_finish = False
        # self.llm_result = '{\'action\':[\'move(\"前台\")\', \'vision(\"大门有没有关\")\', \'move(\"原点\")\', \'play_audio()\'], \'response\':\'马上！\'}'
        self.running = True
        self.play_delay = False
        self.reach_goal = False
        self.interrupt = False
        
        timer_cb_group = ReentrantCallbackGroup()
        self.tts_text_pub = self.create_publisher(String, 'tts_node/tts_text', 1)
        # self.create_subscription(Image, 'depth_cam/rgb/image_raw', self.image_callback, 1)
        self.create_subscription(String, 'agent_process/result', self.llm_result_callback, 1)
        self.create_subscription(Bool, 'vocal_detect/wakeup', self.wakeup_callback, 1)
        self.create_subscription(Bool, 'tts_node/play_finish', self.play_audio_finish_callback, 1, callback_group=timer_cb_group)
        self.create_subscription(Bool, 'navigation_controller/reach_goal', self.reach_goal_callback, 1)
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

        self.agc = ActionGroupController(self.create_publisher(ServosPosition, 'servo_controller', 1), '/home/ubuntu/software/actionset_editor/ActionGroups')

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
        self.get_logger().info('\033[1;32m%s\033[0m' % LLM_PROMPT)

    def send_request(self, client, msg):
        future = client.call_async(msg)
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()

    def wakeup_callback(self, msg):
        self.get_logger().info('wakeup interrupt')
        self.interrupt = msg.data

    def llm_result_callback(self, msg):
        self.llm_result = msg.data

    def move(self, position):
        self.get_logger().info('position: %s' % str(position))
        msg = SetPose2D.Request()
        p = position_dict[position]
        msg.data.x = float(p[0])
        msg.data.y = float(p[1])
        msg.data.roll = p[2]
        msg.data.pitch = p[3]
        msg.data.yaw = p[4]
        self.send_request(self.set_pose_client, msg)

    def play_audio(self):
        msg = String()
        msg.data = self.response_text
        self.get_logger().info(f'{self.response_text}')
        while not self.play_audio_finish:
            time.sleep(0.1)
        self.play_audio_finish = False
        self.tts_text_pub.publish(msg)
        self.response_text = ''
        # time.sleep(20)

    def reach_goal_callback(self, msg):
        self.get_logger().info('reach goal')
        self.reach_goal = msg.data

    def vision(self, query):
        # self.agc.run_action('init_horizontal')
        msg = SetContent.Request()
        if language == 'Chinese':
            msg.api_key = stepfun_api_key
            msg.base_url = stepfun_base_url
            msg.model = stepfun_vllm_model
        else:
            msg.api_key = vllm_api_key
            msg.base_url = vllm_base_url
            msg.model = vllm_model
        msg.prompt = VLLM_PROMPT
        msg.query = query
        self.get_logger().info('vision: %s' % query)
        res = self.send_request(self.set_vllm_content_client, msg)
        # self.agc.run_action('init')
        return res.message

    def play_audio_finish_callback(self, msg):
        self.play_audio_finish = msg.data

    def process(self):
        first = True
        while self.running:
            if self.llm_result:
                self.interrupt = False
                msg = String()
                if 'action' in self.llm_result: # 如果有对应的行为返回那么就提取处理
                    result = json.loads(self.llm_result[self.llm_result.find('{'):self.llm_result.find('}')+1])
                    if 'response' in result:
                        msg.data = result['response']
                        self.tts_text_pub.publish(msg)
                    if 'action' in result:
                        action = result['action']
                        self.get_logger().info(f'vllm action: {action}')
                        for a in action:
                            if 'move' in a:
                                self.reach_goal = False
                                eval(f'self.{a}')
                                while not self.reach_goal:
                                    if self.interrupt:
                                        self.get_logger().info('interrupt')
                                        break
                                    # self.get_logger().info('waiting for reach goal')
                                    time.sleep(0.01)
                            elif 'vision' in a:
                                res = eval(f'self.{a}')
                                self.response_text = res
                                self.get_logger().info(f'vllm response: {res}')
                            elif 'play_audio' in a:
                                eval(f'self.{a}')
                                while not self.play_audio_finish:
                                    time.sleep(1)
                            if self.interrupt:
                                self.get_logger().info('interrupt')
                                break
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

def main():
    node = VLLMNavigation('vllm_navigation')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()

if __name__ == "__main__":
    main()
