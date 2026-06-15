#!/usr/bin/env python3
# encoding: utf-8
# @Author: Aiden
# @Date: 2024/11/18
import os
import json
import queue
import rclpy
import time
import threading
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger, SetBool, Empty

from speech import speech
from large_models.config import *
from large_models_msgs.msg import Tools
from large_models_msgs.srv import SetString, SetModel, SetContent, SetTools

class AgentProcess(Node):
    """
        Agent processing node class, responsible for processing speech recognition results and calling large models to generate answers(智能体处理节点类，负责处理语音识别结果并调用大模型生成回答)
    """
    def __init__(self, name):
        """
                Initialize the agent processing node(初始化智能体处理节点)
        
        Args:
            name:  Node name(节点名称)
        """
        rclpy.init()
        super().__init__(name)
        
        self.declare_parameter('camera_topic', 'depth_cam/rgb/image_raw')  #  Declare camera topic parameter(声明相机话题参数)
        camera_topic = self.get_parameter('camera_topic').value  #  Get camera topic(获取相机话题)

        self.prompt = ''  #  Prompt(提示词)
        self.model = llm_model  #  Large language model(大语言模型)
        self.chat_text = ''  # Chat text(聊天文本)
        self.model_type = 'llm'  # Model type(模型类型)
        self.asr_result = ''  # ASR result(语音识别结果)
        self.enable_search = False  # Enable search(是否启用搜索)
        self.enable_think = False
        self.tools = []  # Tools(工具)
        self.tools_result = ''  # Tools result(工具结果)
        self.wait_for_tools_result = False  # Wait for tools result(等待工具结果)
        self.start_record_chat = False  # Start recording chat flag(开始记录聊天标志)
        self.bridge = CvBridge()  # Image conversion bridge(图像转换桥接器)
        self.image_queue = queue.Queue(maxsize=2)  # Image queue with max capacity of 2(图像队列，最大容量为2)
        
        if os.environ["ASR_MODE"] == 'offline':
            self.offline = True
        else:
            self.offline = False
        if self.offline:
            self.model = offline_llm
            self.enable_search = None
            self.client = speech.OllamaAPI(ollama_host, timeout=60)
            if os.environ["ASR_LANGUAGE"] != 'Chinese':
                self.enable_think = None
                self.enable_search = None
            self.get_logger().info(f"\033[1;32mOllama {self.client.llm('请回复ready', '', model=self.model, enable_search=self.enable_search, enable_think=self.enable_think)}\033[0m")
        else:
            self.client = speech.OpenAIAPI(api_key, base_url)  # Initialize OpenAI API client(初始化OpenAI API客户端)
        # Create publisher and subscribers(创建发布者和订阅者)
        self.result_pub = self.create_publisher(String, '~/result', 1)  # Result publisher(结果发布者)
        self.tools_pub = self.create_publisher(Tools, '~/tools', 1)  # Tools publisher(工具发布者)
        self.create_subscription(Tools, '~/tools_result', self.tools_process_callback, 1)  # Tools subscription(工具订阅)
        self.create_subscription(String, 'vocal_detect/asr_result', self.asr_callback, 1)  #  ASR result subscription(语音识别结果订阅)
        self.create_subscription(Image, camera_topic, self.image_callback, 1)  #  Subscribe to the camera(摄像头订阅)
        
        #  Create services(创建服务)
        self.create_service(SetModel, '~/set_model', self.set_model_srv)  #  Set model service(设置模型服务)
        self.create_service(SetString, '~/set_prompt', self.set_prompt_srv)  #  Set prompt service(设置提示词服务)
        self.create_service(SetContent, '~/set_llm_content', self.set_llm_content_srv)  # Set LLM content service (设置LLM内容服务)
        self.create_service(SetContent, '~/set_vllm_content', self.set_vllm_content_srv)  #  Set VLLM content service(设置VLLM内容服务)
        self.create_service(SetTools, '~/set_tool', self.set_tools_srv)  #  Set VLLM content service(设置VLLM内容服务)

        self.create_service(SetBool, '~/record_chat', self.record_chat)  #  Record chat service(记录聊天服务)
        self.create_service(Trigger, '~/get_chat', self.get_chat)  #  Get chat service(获取聊天服务)
        self.create_service(Empty, '~/clear_chat', self.clear_chat)  #  Clear chat service(清除聊天服务)
        self.timer = self.create_timer(0.0, self.init_process)
        # self.create_service(Empty, '~/init_finish', self.get_node_state)  # Initialization complete service(初始化完成服务)
        # self.get_logger().info('\033[1;32m%s\033[0m' % 'start')  #  Print start information(打印启动信息)

    def get_node_state(self, request, response):
        """
        Obtain node state service callback(获取节点状态服务回调)

        
        Args:
            request: Request object(请求对象)
            response:  Response object(响应对象)
            
        Returns:
            response:  Service response(服务响应)
        """
        return response

    def init_process(self):
        self.timer.cancel()

        threading.Thread(target=self.tools_process, daemon=True).start()
        self.create_service(Empty, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

    def record_chat(self, request, response):
        """
        Record chat service callback(记录聊天服务回调)
        
        Args:
            request:  Request object containing flag for recording chat(请求对象，包含是否记录聊天的标志)
            response:  Response object(响应对象)
            
        Returns:
            response:  Service response(服务响应)
        """
        self.get_logger().info('\033[1;32m%s\033[0m' % 'record chat')
        self.start_record_chat = request.data  #  Set recording chat flag(设置记录聊天标志)
        response.success = True
        return response

    def get_chat(self, request, response):
        """
        Get chat service callback(获取聊天服务回调)

        
        Args:
            request:  Request object(请求对象)
            response:  Response object(响应对象)
            
        Returns:
            response: Service response containing chat text(服务响应，包含聊天文本)
        """
        self.get_logger().info('\033[1;32m%s\033[0m' % 'get chat')
        response.message = self.chat_text.rstrip(",")  #  Return chat text with trailing comma removed(返回去除末尾逗号的聊天文本)
        response.success = True
        return response

    def clear_chat(self, request, response):
        """
        Clear chat service callback(清除聊天服务回调)
        
        Args:
            request:  Request object(请求对象)
            response: Response object(响应对象)
            
        Returns:
            response: Service response(服务响应)
        """
        self.get_logger().info('\033[1;32m%s\033[0m' % 'clear chat')
        self.chat_text = ''  #  Clear chat text(清空聊天文本)
        self.record_chat = False  #  Stop recording chat(停止记录聊天)
        return response

    def asr_callback(self, msg):
        """
        ASR result callback function(语音识别结果回调函数)
        
        Args:
            msg:  Message containing ASR result(包含语音识别结果的消息)
        """
        # self.get_logger().info(msg.data)
        #  Pass recognition result to agent for answering(将识别结果传给智能体让他来回答)
        if msg.data != '':
            self.get_logger().info('\033[1;32m%s\033[0m' % 'thinking...')
            if self.start_record_chat:
                self.chat_text += msg.data + ','  #  Add to chat record(添加到聊天记录)
                self.get_logger().info('\033[1;32m%s\033[0m' % 'record chat:' + self.chat_text)
            res = ''
            if self.model_type == 'llm':
                if os.environ["ASR_LANGUAGE"] != 'Chinese':
                    self.enable_think = None
                    self.enable_search = None
                res = self.client.llm(msg.data, self.prompt, model=self.model, enable_search=self.enable_search, enable_think=self.enable_think)  #  Call language model(调用语言模型)
                self.get_logger().info('\033[1;32m%s\033[0m' % 'publish llm result:' + str(res))
            elif self.model_type == 'llm_tools':
                self.asr_result = msg
            elif self.model_type == 'vllm':
                image = self.image_queue.get(block=True)  #  Get image from queue(从队列获取图像)
                res = self.client.vllm(msg.data, image, prompt=self.prompt, model=self.model)  #  Call vision language model(调用视觉语言模型)
                self.get_logger().info('\033[1;32m%s\033[0m' % 'publish vllm result:' + str(res))
            if self.model_type != 'llm_tools':
                msg = String()
                msg.data = res
                self.result_pub.publish(msg)  #  Publish result(发布结果)
        else:
            self.get_logger().info('\033[1;32m%s\033[0m' % 'asr result none')

    def image_callback(self, ros_image):
        """

        Image callback function (图像回调函数)
        
        Args:
            ros_image: ROS image message (ROS图像消息)
        """
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")  #  Convert to OpenCV image(转换为OpenCV图像)
        bgr_image = np.array(cv_image, dtype=np.uint8)  #  Convert to NumPy array(转换为NumPy数组)
        if self.image_queue.full():
            # If the queue is full, remove the oldest image(如果队列已满，丢弃最旧的图像)
            self.image_queue.get()
        # Put the image into the queue(将图像放入队列)
        self.image_queue.put(bgr_image)

    def set_model_srv(self, request, response):
        """

        Set model service callback(设置模型服务回调)
        
        Args:
            request:  Request object containing model information(请求对象，包含模型信息)
            response:  Response object(响应对象)
            
        Returns:
            response: Service response (服务响应)
        """
        #  Set which model to call(设置调用哪个模型)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'set model')
        #self.get_logger().info(f'{request}')
        self.model = request.model  #  Set model(设置模型)
        self.model_type = request.model_type  #  Set model type(设置模型类型)
        self.enable_think = request.enable_think
        if self.offline:
            self.enable_search = None
            self.client = speech.OllamaAPI(request.base_url)
            if os.environ["ASR_LANGUAGE"] != 'Chinese':
                self.enable_think = None
                self.enable_search = None
            self.get_logger().info(f"loading finish: {self.client.llm('请回复ready', '', model=self.model, enable_search=self.enable_search, enable_think=self.enable_think)}")
        else:
            self.enable_search = request.enable_search  #  Set enable search(设置是否启用搜索)
            self.client = speech.OpenAIAPI(request.api_key, request.base_url)  # Update API client(更新API客户端 )
        response.success = True
        return response

    def set_prompt_srv(self, request, response):
        """
        Set prompt service callback(设置提示词服务回调)
        
        Args:
            request:  Request object containing prompt(请求对象，包含提示词)
            response:  Response object(响应对象)
            
        Returns:
            response:  Service response(服务响应)
        """
        #  Set prompt for large model(设置大模型的prompt)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'set prompt')
        self.prompt = request.data  #  Update prompt(更新提示词)
        response.success = True
        return response

    def set_tools_srv(self, request, response):
        """
        Set tools service callback(设置工具服务回调)
        
        Args:
            request:  Request object containing tools(请求对象，包含工具)
            response:  Response object(响应对象)
            
        Returns:
            response: Service response(服务响应)
        """
        #  Set tools for large model(设置大模型的工具)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'set tools')
        tools_list = []
            
        # 将JSON字符串转换为Python对象
        for tool_json in request.tools:
            tool = json.loads(tool_json)
            tools_list.append(tool)
        self.tools = tools_list.copy()
        #self.get_logger().info(f'{self.tools}')
        response.success = True
        return response

    def set_llm_content_srv(self, request, response):
        """

        Set LLM content service callback(设置LLM内容服务回调)
        
        Args:
            request:  Request object containing query and prompt(请求对象，包含查询和提示词)
            response:  Response object(响应对象)
            
        Returns:
            response:  Service response containing LLM answer(服务响应，包含LLM回答)
        """
        #  Input text is passed to agent for answering(输入文本传给智能体让他来回答)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'thinking...')
        if self.offline:
            client = speech.OllamaAPI(request.base_url)
        else:
            client = speech.OpenAIAPI(request.api_key, request.base_url)  #  Create temporary client(创建临时客户端)
        response.message = client.llm(request.query, request.prompt, model=request.model)  #  Call language model(调用语言模型)
        response.success = True
        return response

    def set_vllm_content_srv(self, request, response):
        """
        Set VLLM content service callback(设置VLLM内容服务回调)
        
        Args:
            request: Request object containing query, image and prompt (请求对象，包含查询、图像和提示词)
            response:  Response object(响应对象)
            
        Returns:
            response: Service response containing VLLM answer (服务响应，包含VLLM回答)
        """
        # Input prompt and text, vision agent returns answer (输入提示词和文本，视觉智能体返回回答)
        if request.image.data:
            self.get_logger().info(f'receive image')  # Received image (接收到图像)
            image = self.bridge.imgmsg_to_cv2(request.image, desired_encoding="bgr8")  # Convert image (转换图像)
        else:
            image = self.image_queue.get(block=True)  # Get image from queue (从队列获取图像)
        if self.offline:
            client = speech.OllamaAPI(request.base_url)
        else:
            client = speech.OpenAIAPI(request.api_key, request.base_url)  #  Create temporary client(创建临时客户端)
        res = client.vllm(request.query, image, prompt=request.prompt, model=request.model)  # Call vision language model (调用视觉语言模型)
        response.message = res
        response.success = True
        return response

    def tools_process(self):
        """
        Tools process(工具处理)
        """
        while rclpy.ok():
            if not self.wait_for_tools_result:
                if self.asr_result:              
                    #self.get_logger().info(f'1:{self.asr_result} {self.enable_search} {self.model}')
                    #  Get tool calls from large model(从大模型获取工具调用)
                    if os.environ["ASR_LANGUAGE"] != 'Chinese':
                        self.enable_think = None
                        self.enable_search = None
                    res = self.client.llm_tools(self.asr_result.data, self.prompt, self.model, self.tools, enable_search=self.enable_search, enable_think=self.enable_think)
                    self.asr_result = ''
                    #self.get_logger().info(f'2:{res}')
                    # contenr, [tool_name, tool_args]
                    if res[1]:
                        #  Process tool calls(处理工具调用)
                        tools_msg = Tools()
                        tools_msg.id = res[1][0]
                        tools_msg.name = res[1][1]
                        data = json.dumps(res[1][2], ensure_ascii=False, indent=2)
                        if data:
                            tools_msg.data = data
                        self.tools_pub.publish(tools_msg)
                        self.wait_for_tools_result = True
                    
                    if res[0]:
                        msg = String()
                        msg.data = res[0]
                        self.result_pub.publish(msg)  #  Publish result(发布结果)
                        # self.wait_for_tools_result = True
                else:
                    time.sleep(0.02)
            else:
                if self.tools_result:
                    #  Process tool result(处理工具结果)
                    #self.get_logger().info(f'tools result: {self.tools_result}')
                    #  Get tool result from large model(从大模型获取工具结果)
                    res = self.client.llm_tools_result(self.tools_result.id, self.tools_result.data)
                    #self.get_logger().info(f'3: {res}')
                    self.tools_result = ''
                    if res[1]:
                        #  Process tool calls(处理工具调用)
                        tools_msg = Tools()
                        tools_msg.id = res[1][0]
                        tools_msg.name = res[1][1]
                        data = json.dumps(res[1][2], ensure_ascii=False, indent=2)
                        if data:
                            tools_msg.data = data
                        #self.get_logger().info(f'4:{tools_msg}')
                        self.tools_pub.publish(tools_msg)
                    else:
                        self.wait_for_tools_result = False
                        self.client.reset_tools()
                        
                    if res[0]:
                        msg = String()
                        msg.data = res[0]
                        self.result_pub.publish(msg)  #  Publish result(发布结果)
                else:
                    time.sleep(0.02)

    def tools_process_callback(self, msg):
        """
        Tools process callback(工具处理回调)
        """
        #self.get_logger().info(f'tools name: {msg.name}, arguments: {msg.data}')
        self.tools_result = msg

def main():
    """
    Main function(主函数)
    """
    node = AgentProcess('agent_process')  # Create agent process node (创建智能体处理节点)
    try:
        rclpy.spin(node)  # Run node (运行节点)
    except KeyboardInterrupt:
        print('shutdown')
    finally:
        rclpy.shutdown()  # Shutdown ROS2 (关闭ROS2)

if __name__ == "__main__":
    main()
