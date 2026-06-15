#!/usr/bin/env python3
# encoding: utf-8
# ROS 机器人姿态自平衡

import time
import rclpy
import queue
import threading
from sdk import pid
from rclpy.node import Node
from app.common import Heart
from sensor_msgs.msg import Imu
from std_srvs.srv import SetBool, Trigger
from kinematics import kinematics_calculate
from scipy.spatial.transform import Rotation as R
from servo_controller_msgs.msg import ServosPosition
from controller import step_controller, build_in_pose
from rclpy.callback_groups import ReentrantCallbackGroup
from servo_controller.bus_servo_control import set_servo_position


class SelfBalancing(Node):
    def __init__(self,name):
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.name = name
        self.pid_pitch = pid.PID(0.05, 0.0005, 0.00001)
        self.pid_roll = pid.PID(0.032, 0.0005, 0.00001)
        self.pitch = 0
        self.roll = 0
        self.running = False
        self.imu_queue = queue.Queue(maxsize=2)

        self.step_controller = step_controller.StepController()
        self.client = self.create_client(Trigger, '/controller_manager/init_finish')
        self.client.wait_for_service()
        self.create_subscription(Imu, '/ros_robot_controller/imu_raw', self.imu_callback, 1)

        timer_cb_group = ReentrantCallbackGroup()
        self.create_service(Trigger, '~/enter', self.enter_srv_callback, callback_group=timer_cb_group)  # 进入玩法(enter the game)
        self.create_service(Trigger, '~/exit', self.exit_srv_callback, callback_group=timer_cb_group)  # 退出玩法(exit the game)
        self.create_service(SetBool, '~/set_running', self.set_running_srv_callback)  # 开启玩法(start the game)
        self.joints_pub = self.create_publisher(ServosPosition, 'servo_controller', 1)

        Heart(self, self.name + '/heartbeat', 5, lambda _: self.exit_srv_callback(request=Trigger.Request(), response=Trigger.Response()))  # 心跳包(heartbeat package)
        threading.Thread(target=self.main, daemon=True).start()
        self.debug = self.get_parameter('debug').value

        if self.debug: 
            threading.Thread(target=self.enter_srv_callback,args=(Trigger.Request(), Trigger.Response()), daemon=True).start()
            time.sleep(2)
            threading.Thread(target=self.set_running_srv_callback,args=(SetBool.Request(data=True), SetBool.Response()), daemon=True).start()

        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')
          
    def get_node_state(self, request, response):
        response.success = True
        return response

    def send_request(self, client, msg):
        future = client.call_async(msg)
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()

    def enter_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "self balancing enter")
        self.step_controller.set_build_in_pose('DEFAULT_POSE', 1)
        set_servo_position(self.joints_pub, 1, ((24, 500), (23, 500), (22, 150), (21, 130), (20, 720), (19, 500)))

        response.success = True
        response.message = "enter"
        return response

    def exit_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "self balancingl exit")
        self.step_controller.set_build_in_pose('DEFAULT_POSE', 1)
        self.running = False

        response.success = True
        response.message = "exit"
        return response
   
    def set_running_srv_callback(self, request, response):
        
        if request.data:
            self.running = True
            self.get_logger().info('\033[1;32m%s\033[0m' % "set_running")
        else:
            self.running = False

        response.success = True
        response.message = "set_running"
        return response

    def imu_callback(self, imu_msg):
        if self.imu_queue.full():
            # 如果队列已满，丢弃最旧的数据(if the queue is full, discard the oldest image)
            self.imu_queue.get()
            # 将IMU数据放入队列(put the image into the queue)
        self.imu_queue.put(imu_msg)

    def main(self):
        while rclpy.ok():
            if self.running :
                try:
                    imu = self.imu_queue.get(block=True, timeout=1)
                    q = imu.linear_acceleration
                    x = q.x
                    y = q.y
                    self.pid_pitch.update(x)
                    self.pid_roll.update(y)
                except queue.Empty:
                    if not self.running:
                        break
                    else:
                        continue

                try:
                    pitch = self.pid_pitch.output
                    roll =  self.pid_roll.output

                    new_pose = kinematics_calculate.transform_euler(build_in_pose.DEFAULT_POSE, (0, 0, 0), 'xyz', (pitch, roll, 0), degrees=False)
                    self.step_controller.set_pose_base(new_pose, 0.02)
                except Exception as e:
                    self.get_logger().error(str(e))
                    self.pitch = 0
                    self.roll = 0

            else:
                time.sleep(0.02)


def main():
    node = SelfBalancing('self_balancing')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()
        node.get_logger().info('shutdown')
    finally:
        node.get_logger().info('shutdown finish')

if __name__ == '__main__':
    main()
