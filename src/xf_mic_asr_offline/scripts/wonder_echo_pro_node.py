#!/usr/bin/env python3
# encoding: utf-8
#!/usr/bin/env python3
# encoding: utf-8
# @Author: Aiden
# @Date: 2024/11/18
import os
import time
import rclpy
import binascii
import threading
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from speech import awake

zh_cmd = ['前进', 
        '后退', 
        '左转', 
        '右转', 
        '左平移', 
        '右平移',
        '跳个舞吧',
        '过来']

zh_asr_dict = {"aa550300fb": '唤醒成功(wake-up-success)',
               "aa550200fb": '休眠(Sleep)'}

start = 0
frame = "aa550001fb"
for i in zh_cmd:
    start += 1
    frame = frame[:6] + str(format(start, '02x')) + frame[8:]
    zh_asr_dict[frame] = i

en_cmd = ['go forward',
        'go backward',
        'turn left',
        'turn right',
        'move left',
        'move right',
        'dance',
        'come here']

en_asr_dict = {'aa550300fb': '唤醒成功(wake-up-success)',
               'aa550200fb': '休眠(Sleep)'}

start = 0
frame = "aa550001fb"
for i in en_cmd:
    start += 1
    frame = frame[:6] + str(format(start, '02x')) + frame[8:]
    en_asr_dict[frame] = i

class ASRNode(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)

        self.running = True

        # 声明参数
        self.declare_parameter('port', '/dev/ring_mic')
        port = self.get_parameter('port').value
        self.kws = awake.WonderEchoPro(port) 
        self.language = os.environ['ASR_LANGUAGE']
        self.asr_pub = self.create_publisher(String, '~/voice_words', 1)
        threading.Thread(target=self.pub_callback, daemon=True).start()
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

    def get_node_state(self, request, response):
        return response

    def pub_callback(self):
        while self.running:
            self.kws.start()
            result = self.kws.detect()
            result = binascii.hexlify(result).decode('utf-8')
            if result:
                print(result)
                asr_msg = String()
                asr_result = result
                if self.language == 'Chinese':
                    if result in zh_asr_dict:
                        asr_result = zh_asr_dict[result]
                else:
                    if result in en_asr_dict:
                        asr_result = en_asr_dict[result]
                asr_msg.data = asr_result
                self.asr_pub.publish(asr_msg)
            else:
                time.sleep(0.02)
        rclpy.shutdown()

def main():
    node = ASRNode('asr_node')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('shutdown')
    finally:
        rclpy.shutdown() 

if __name__ == "__main__":
    main()

