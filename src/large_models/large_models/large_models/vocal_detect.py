#!/usr/bin/env python3
# encoding: utf-8
# @Author: Aiden
# @Date: 2024/11/18
import time
import rclpy
import threading
from rclpy.node import Node
from std_msgs.msg import Int32, String, Bool
from std_srvs.srv import SetBool, Trigger, Empty

from speech import awake
from speech import speech
from large_models.config import *
from large_models_msgs.srv import SetInt32

class VocalDetect(Node):
    """

    Voice detection node class, responsible for voice wake-up and speech recognition(语音检测节点类，负责语音唤醒和语音识别)
    """
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)

        self.running = True  # Running flag (运行标志)

        # Declare parameters(声明参数)
        self.declare_parameter('awake_method', 'xf')
        self.declare_parameter('mic_type', 'mic6_circle')
        self.declare_parameter('port', '/dev/ring_mic')
        self.declare_parameter('enable_wakeup', True)
        self.declare_parameter('enable_setting', False)
        self.declare_parameter('awake_word', 'hello hi wonder')
        self.declare_parameter('mode', 1)
        self.declare_parameter('offline', 'false')
        self.declare_parameter('punct_model', '')

        self.awake_method = self.get_parameter('awake_method').value  # Wake-up method (唤醒方法)
        mic_type = self.get_parameter('mic_type').value  # Microphone type (麦克风类型)
        port = self.get_parameter('port').value  # Device port (设备端口)
        awake_word = self.get_parameter('awake_word').value  # Wake-up word (唤醒词)
        enable_setting = self.get_parameter('enable_setting').value  # Enable settings (启用设置)
        self.enable_wakeup = self.get_parameter('enable_wakeup').value  # Enable wake-up (启用唤醒)
        self.mode = int(self.get_parameter('mode').value)  # Working mode (工作模式)
        self.punct_model = self.get_parameter('punct_model').value
        self.sherpa_onnx = None
        if os.environ["ASR_MODE"] == 'offline':
            import sherpa_onnx
            self.offline = True
            self.sherpa_onnx = sherpa_onnx
        else:
            self.offline = False

        # Initialize wake-up device (初始化唤醒设备)
        if self.awake_method == 'xf':
            self.kws = awake.CircleMic(port, awake_word, mic_type, enable_setting)
        else:
            self.kws = awake.WonderEchoPro(port) 
        
        self.language = os.environ["ASR_LANGUAGE"]  # Get language environment (获取语言环境)
        # Initialize ASR based on language and wake-up method (根据语言和唤醒方法初始化ASR)
        if self.awake_method == 'xf':
            if self.offline:
                self.asr = speech.OfflineRealTimeASR(log=self.get_logger(), timeout=180)
            else:
                if self.language == 'Chinese':
                    self.asr = speech.RealTimeASR(log=self.get_logger())
                else:
                    self.asr = speech.RealTimeOpenAIASR(log=self.get_logger())
                    self.asr.update_session(model=asr_model, language='en')
        else:
            if self.offline:
                self.asr = speech.OfflineRealTimeASR(log=self.get_logger(), timeout=180)
            else:
                if self.language == 'Chinese':
                    self.asr = speech.RealTimeASR(log=self.get_logger())
                else:
                    self.asr = speech.RealTimeOpenAIASR(log=self.get_logger())
                    self.asr.update_session(model=asr_model, language='en')
        
        # Create publishers and services (创建发布者和服务)
        self.asr_pub = self.create_publisher(String, '~/asr_result', 1)  # ASR result publisher (语音识别结果发布者)
        self.wakeup_pub = self.create_publisher(Bool, '~/wakeup', 1)  # Wake-up status publisher (唤醒状态发布者)
        self.awake_angle_pub = self.create_publisher(Int32, '~/angle', 1)  # Wake-up angle publisher (唤醒角度发布者)
        self.create_service(SetInt32, '~/set_mode', self.set_mode_srv)  # Set mode service (设置模式服务)
        self.create_service(SetBool, '~/enable_wakeup', self.enable_wakeup_srv)  # Enable wake-up service (启用唤醒服务)

        # Start publisher callback thread (启动发布回调线程)
        threading.Thread(target=self.pub_callback, daemon=True).start()
        self.create_service(Empty, '~/init_finish', self.get_node_state)  # Initialization complete service (初始化完成服务)
        
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

    def get_node_state(self, request, response):
        """

        Node state service callback(获取节点状态服务回调)
        """
        return response

    def record(self, mode, angle=None):
        """

        Record and recognize speech(录音并识别语音)
        
        Args:
            mode (int):  Working mode(工作模式)
            angle (int, optional): Wake-up angle (唤醒角度)
        """
        self.get_logger().info('\033[1;32m%s\033[0m' % 'asr...')
        if self.offline:
            asr_result = self.asr.asr()
            # asr_result = self.asr.punctuation(asr_result, self.punct_model, self.sherpa_onnx)
            if self.language != 'Chinese':
                asr_result = asr_result.lower()
        else:
            if self.language == 'Chinese':
                asr_result = self.asr.asr(model=asr_model)  # Start recording and recognition (开启录音并识别)
            else:
                asr_result = self.asr.asr()
        if asr_result: 
            speech.play_audio(dong_audio_path)  # Play prompt sound (播放提示音)
            if self.awake_method == 'xf' and self.mode == 1: 
                msg = Int32()
                msg.data = int(angle)
                self.awake_angle_pub.publish(msg)  # Publish wake-up angle (发布唤醒角度)
            asr_msg = String()
            asr_msg.data = asr_result
            self.asr_pub.publish(asr_msg)  # Publish ASR result (发布语音识别结果)
            self.enable_wakeup = False
            self.get_logger().info('\033[1;32m%s\033[0m' % 'publish asr result:' + asr_result)
        else:
            self.get_logger().info('\033[1;32m%s\033[0m' % 'no voice detect')
            speech.play_audio(dong_audio_path)
            if mode == 1:
                speech.play_audio(no_voice_audio_path)  # Play no voice prompt (播放无语音提示)

    def pub_callback(self):
        """

        Publisher callback thread function, handling wake-up and recording(发布回调线程函数，处理唤醒和录音)
        """
        self.kws.start()  # Start wake-up device (启动唤醒设备)
        while self.running:
            if self.enable_wakeup:
                if self.mode == 1:  # Wake-up mode (唤醒模式)
                    result = self.kws.wakeup()
                    if result:
                        self.wakeup_pub.publish(Bool(data=True))  # Publish wake-up status (发布唤醒状态)
                        speech.play_audio(wakeup_audio_path)  # Play wake-up prompt (唤醒播放提示音)
                        self.record(self.mode, result)  # Record and recognize (录音并识别)
                    else:
                        time.sleep(0.02)
                elif self.mode == 2:  # Direct recording mode (直接录音模式)
                    self.record(self.mode)
                    self.mode = 1
                elif self.mode == 3:  #  Another recording mode(另一种录音模式)
                    self.record(self.mode)
                else:
                    time.sleep(0.02)
            else:
                time.sleep(0.02)
        rclpy.shutdown()

    def enable_wakeup_srv(self, request, response):
        """

        Enable wake-up service callback(启用唤醒服务回调)
        
        Args:
            request: Request object (请求对象)
            response: Response object (响应对象)
            
        Returns:
            response: Service response (服务响应)
        """
        self.get_logger().info('\033[1;32m%s\033[0m' % ('enable_wakeup'))
        self.kws.start()  # Start wake-up device (启动唤醒设备)
        self.enable_wakeup = request.data  # Set wake-up status (设置唤醒状态)
        response.success = True
        return response 

    def set_mode_srv(self, request, response):
        """

        Set mode service callback(设置模式服务回调)
        
        Args:
            request: Request object (请求对象)
            response:  Response object(响应对象)
            
        Returns:
            response: Service response(服务响应)
        """
        self.get_logger().info(f'\033[1;32mset mode: {request.data}\033[0m')
        self.kws.start()  # Start wake-up device (启动唤醒设备)
        self.mode = int(request.data)  # Set working mode (设置工作模式)
        if self.mode != 1:
            self.enable_wakeup = True  # Enable wake-up in non-wake-up mode (非唤醒模式下启用唤醒)
        response.success = True
        return response 

def main():
    """

    Main function(主函数)
    """
    node = VocalDetect('vocal_detect')  # Create voice detection node (创建语音检测节点)
    try:
        rclpy.spin(node)  # Run node (运行节点)
    except KeyboardInterrupt:
        print('shutdown')
    finally:
        rclpy.shutdown()  # Shutdown ROS2 (关闭ROS2)

if __name__ == "__main__":
    main()
