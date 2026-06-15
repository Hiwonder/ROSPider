#!/usr/bin/env python3
# encoding: utf-8
import os
import json
import math
import time
import rclpy
import numpy as np
from rclpy.node import Node
import sdk.common as common
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from interfaces.srv import SetString
from rclpy.duration import Duration
from interfaces.srv import SetPose2D
from xf_mic_asr_offline import voice_play
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.srv import GetParameters
from rclpy.executors import MultiThreadedExecutor
from ros_robot_controller_msgs.msg import BuzzerState
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.callback_groups import ReentrantCallbackGroup

class NavigationTransport(Node):
    markerArray = MarkerArray()
    
    def __init__(self, name):
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        
        self.words = None
        self.find = True
        self.pick = True
        self.place = False
        self.running = True
        self.narrow = False
        self.received = True
        self.cross_bridge = False
        self.goal_pose = PoseStamped()
        self.haved_publish_goal = False
        self.language = os.environ['ASR_LANGUAGE']

        timer_cb_group = ReentrantCallbackGroup()

        self.target_pub=self.create_publisher(String , '~/locked_target_class', 10)

        self.create_subscription(Bool, '/narrow_slit_traversal/finish', self.narrow_finish_callback, 1, callback_group=timer_cb_group)
        self.create_subscription(Bool, '/cross_bridge/finish', self.cross_bridge_finish_callback, 1, callback_group=timer_cb_group)
        self.create_subscription(Bool, '/automatic_pick/finish', self.automatic_pick_finish_callback, 1, callback_group=timer_cb_group)
        self.create_subscription(String, '/asr_node/voice_words', self.words_callback, 1)

        self.create_service(SetPose2D, '~/place', self.start_place_srv_callback)

        self.narrow_slit_traversal_client = self.create_client(Trigger, '/narrow_slit_traversal/enter', callback_group=timer_cb_group)
        self.cross_bridge_client = self.create_client(Trigger, '/cross_bridge/enter', callback_group=timer_cb_group)
        self.cross_bridge_set_mode_client = self.create_client(SetString, '/cross_bridge/set_mode', callback_group=timer_cb_group)

        self.automatic_pick_client = self.create_client(Trigger, '/automatic_pick/lidar_enable', callback_group=timer_cb_group)


        self.narrow_slit_traversal_client.wait_for_service()

        self.cross_bridge_client.wait_for_service()
        self.cross_bridge_set_mode_client.wait_for_service()


        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

        self.get_param_client = self.create_client(GetParameters, '/automatic_pick/get_parameters', callback_group=timer_cb_group)
        self.get_param_client.wait_for_service()
        
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')


        self.get_logger().info('唤醒口令: 小幻小幻(Wake up word: hello hiwonder)')
        self.get_logger().info('唤醒后15秒内可以不用再唤醒(No need to wake up within 15 seconds after waking up)')
        self.get_logger().info('控制指令: 本次任务需要排除易燃物/易爆物/有毒物')
        self.play('running')

        if self.narrow:
            res = self.send_request(self.cross_bridge_client, Trigger.Request())
            if res.success:
                self.get_logger().info('start cross bridge')

    def play(self, name):
        voice_play.play(name, language=self.language)

    def words_callback(self, msg):
        self.words = json.dumps(msg.data, ensure_ascii=False)[1:-1]
        self.words = self.words.replace(' ', '')
        self.get_logger().info(str(self.words))

        if self.words is not None and self.received  and self.words not in ['唤醒成功(wake-up-success)', '休眠(Sleep)', '失败5次(Fail-5-times)',
                                                         '失败10次(Fail-10-times']:
            if self.words == '排除易燃物':
                target = 'flammable'
                self.target_pub.publish(String(data=target))

            elif self.words == '排除易爆物':
                target = 'explosives'
                self.target_pub.publish(String(data=target))

            elif self.words == '排除有毒物':
                target = 'poison'
                self.target_pub.publish(String(data=target))
            else:
                target = 'none'
                
            if target != 'none':
                self.target_pub.publish(String(data=target))
                self.play('received')
                res = self.send_request(self.narrow_slit_traversal_client, Trigger.Request())
                if res.success:
                    self.get_logger().info('start narrow slit traversal')
                    self.received = False
            
            
        elif self.words == '唤醒成功(wake-up-success)':
            self.play('awake')
        elif self.words == '休眠(Sleep)':
            msg = BuzzerState()
            msg.freq = 1000
            msg.on_time = 0.1

            msg.off_time = 0.01
            msg.repeat = 1
            self.buzzer_pub.publish(msg)
        
    def get_node_state(self, request, response):
        response.success = True
        return response

    def send_request(self, client, msg):
        future = client.call_async(msg)
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()
	    

    def narrow_finish_callback(self, msg):
        self.narrow = msg.data 
        self.get_logger().info(' norrow = msg.data '+str( self.narrow))

    def cross_bridge_finish_callback(self, msg):
        res = self.send_request(self.automatic_pick_client, Trigger.Request())
        if res.success:
            self.get_logger().info('start automatic pick')


    def automatic_pick_finish_callback(self, msg):
        res = self.send_request(self.cross_bridge_set_mode_client, SetString.Request(data="hole"))
        if res.success:
            self.get_logger().info('set cross bridge mode')


    def start_place_srv_callback(self, request, response):
        self.get_logger().info('start place')
       
        response.success = True
        response.message = "place"
        return response

def main():
    rclpy.init()
    node = NavigationTransport('competition')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()
    try:
        node.stop_nav_launch()
    except Exception:
        pass
    node.destroy_node()
 
if __name__ == "__main__":
    main()


