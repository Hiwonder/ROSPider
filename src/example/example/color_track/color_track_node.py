#!/usr/bin/env python3
# encoding: utf-8

# 颜色跟踪(color tracking)
import time
import rclpy
import signal
import threading
import sdk.pid as pid
from rclpy.node import Node
from arm_kinematics import transform
from std_srvs.srv import Trigger
from controller import step_controller
from arm_kinematics_msgs.srv import SetRobotPose
from rclpy.executors import MultiThreadedExecutor
from interfaces.msg import ColorsInfo, ColorDetect
from servo_controller_msgs.msg import ServosPosition
from rclpy.callback_groups import ReentrantCallbackGroup
from interfaces.srv import SetColorDetectParam, SetString
from arm_kinematics.kinematics_control import set_pose_target
from servo_controller.bus_servo_control import set_servo_position

class ColorTrackNode(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.z_dis = 0.30
        self.y_dis = 500
        self.x_init = transform.link3 + transform.tool_link
        self.center = None
        self.running = True
        self.start = False
        self.name = name
        self.pid_z = pid.PID(0.00002, 0.0, 0.000001)
        self.pid_y = pid.PID(0.030, 0.0001, 0.000001)
        signal.signal(signal.SIGINT, self.shutdown)
        self.controller = step_controller.StepController()

        self.joints_pub = self.create_publisher(ServosPosition, '/servo_controller', 1) # 舵机控制(servo control)

        self.create_subscription(ColorsInfo, '/color_detect/color_info', self.get_color_callback, 1)

        timer_cb_group = ReentrantCallbackGroup()
        self.create_service(Trigger, '~/start', self.start_srv_callback) # 进入玩法(enter the game)
        self.create_service(Trigger, '~/stop', self.stop_srv_callback, callback_group=timer_cb_group) # 退出玩法(exit the game)
        self.create_service(SetString, '~/set_color', self.set_color_srv_callback, callback_group=timer_cb_group) # 设置颜色(set color)
        self.client = self.create_client(Trigger, '/controller_manager/init_finish')
        self.client.wait_for_service()
        self.client = self.create_client(Trigger, '/arm_kinematics/init_finish')
        self.client.wait_for_service()
        self.set_color_client = self.create_client(SetColorDetectParam, '/color_detect/set_param', callback_group=timer_cb_group)
        self.set_color_client.wait_for_service()

        self.arm_kinematics_client = self.create_client(SetRobotPose, '/arm_kinematics/set_pose_target')
        self.arm_kinematics_client.wait_for_service()

        self.timer = self.create_timer(0.0, self.init_process, callback_group=timer_cb_group)

    def init_process(self):
        self.timer.cancel()

        self.init_action()
        if self.get_parameter('start').value:
            self.start_srv_callback(Trigger.Request(), Trigger.Response())
            request = SetString.Request()
            request.data = 'red'
            self.set_color_srv_callback(request, SetString.Response())

        threading.Thread(target=self.main, daemon=True).start()
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

    def get_node_state(self, request, response):
        response.success = True
        return response

    def shutdown(self, signum, frame):
        self.running = False

    def init_action(self):
        # self.controller.set_build_in_pose('DEFAULT_POSE', 1)
        msg = set_pose_target([self.x_init, 0.0, self.z_dis], 0.0, [-180.0, 180.0], 1.0)
        res = self.send_request(self.arm_kinematics_client, msg)
        if res.pulse:
            servo_data = res.pulse
            set_servo_position(self.joints_pub, 1.5, ((24, 500), (23, 500), (22, servo_data[3]), (21, servo_data[2]), (20, servo_data[1]), (19, servo_data[0])))
            time.sleep(1.8)

    def send_request(self, client, msg):
        future = client.call_async(msg)
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()

    def set_color_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "set_color")
        msg = SetColorDetectParam.Request()
        msg_red = ColorDetect()
        msg_red.color_name = request.data
        msg_red.detect_type = 'circle'
        msg.data = [msg_red]
        res = self.send_request(self.set_color_client, msg)
        if res.success:
            self.get_logger().info('\033[1;32m%s\033[0m' % 'start_track_%s'%msg_red.color_name)
        else:
            self.get_logger().info('\033[1;32m%s\033[0m' % 'track_fail')
        response.success = True
        response.message = "set_color"
        return response

    def start_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "start color track")
        self.start = True
        response.success = True
        response.message = "start"
        return response

    def stop_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "stop color track")
        self.start = False
        res = self.send_request(ColorDetect.Request())
        if res.success:
            self.get_logger().info('\033[1;32m%s\033[0m' % 'set color success')
        else:
            self.get_logger().info('\033[1;32m%s\033[0m' % 'set color fail')
        response.success = True
        response.message = "stop"
        return response

    def get_color_callback(self, msg):
        if msg.data != []:
            if msg.data[0].radius > 10:
                self.center = msg.data[0]
            else:
                self.center = None 
        else:
            self.center = None

    def main(self):
        while self.running:
            if self.center is not None and self.start:
                t1 = time.time()
                center = self.center

                self.pid_y.SetPoint = center.width/2 
                self.pid_y.update(center.x)
                self.y_dis += self.pid_y.output
                
                if self.y_dis < 200:
                    self.y_dis = 200
                if self.y_dis > 800:
                    self.y_dis = 800

                self.pid_z.SetPoint = center.height/2 
                self.pid_z.update(center.y)
                self.z_dis += self.pid_z.output
                if self.z_dis > 0.37:
                    self.z_dis = 0.37
                if self.z_dis < 0.30:
                    self.z_dis = 0.30
                msg = set_pose_target([self.x_init, 0.0, self.z_dis], 0.0, [-180.0, 180.0], 1.0)
                res = self.send_request(self.arm_kinematics_client, msg)
                t2 = time.time()
                t = t2 - t1
                if t < 0.02:
                    time.sleep(0.02 - t)
                if res.pulse:
                    servo_data = res.pulse
                    set_servo_position(self.joints_pub, 0.02, ((24, 500), (23, 500), (22, servo_data[3]), (21, servo_data[2]), (20, servo_data[1]),(19, int(self.y_dis))))
                else:
                    set_servo_position(self.joints_pub, 0.02, ((19, int(self.y_dis)), ))
            else:
                time.sleep(0.01)

        self.init_action()
        rclpy.shutdown()


def main():
    node = ColorTrackNode('color_track')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
 
if __name__ == "__main__":
    main()
