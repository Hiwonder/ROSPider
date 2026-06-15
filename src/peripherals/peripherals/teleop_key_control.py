#!/usr/bin/env python3
# encoding: utf-8
import rclpy
import time
import tty, termios
import sys, select, os
from rclpy.node import Node
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist
from interfaces.msg import CmdParam
from rclpy.callback_groups import ReentrantCallbackGroup


settings = termios.tcgetattr(sys.stdin)

LIN_VEL_X = 0.06
LIN_VEL_Y = 0.03
ANG_VEL = 0.2

msg = """
Control Your Robot!
--------------------------------------
Long press:

w : Move Forward  |  s : Move Backward
a : Trun Left     |  d : Turn Right

CTRL-C to quit
--------------------------------------
"""

def getKey(settings):

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class TeleopControl(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)

        self.cmd_vel = self.create_publisher(Twist,"controller/cmd_vel", 1)
        self.cmd_param_pub = self.create_publisher(CmdParam, '/step_controller/cmd_param', 1) # 行走姿态控制  
        self.timer_cb_group = ReentrantCallbackGroup()
        self.client = self.create_client(Trigger, '/controller_manager/init_finish',  callback_group=self.timer_cb_group)
        self.client.wait_for_service()

        cmd_param = CmdParam()
        cmd_param.pose = 'DEFAULT_POSE_M'
        cmd_param.gait = 2
        cmd_param.height = 20
        cmd_param.period = 1.0

        self.cmd_param_pub.publish(cmd_param)
        control_linear_vel_x = 0.0
        control_linear_vel_y = 0.0
        control_angular_vel = 0.0
        last_x = 0.0
        last_y = 0.0
        last_z = 0.0

        try:
            print(msg)
            while rclpy.ok():
                key = getKey(settings)
                if key == 'w':
                    control_linear_vel_x = LIN_VEL_X
                elif key == 's':
                    control_linear_vel_x = -LIN_VEL_X

                elif key == 'a':
                    control_angular_vel = ANG_VEL
                elif key == 'd':
                    control_angular_vel = -ANG_VEL

                elif key == '':
                    control_linear_vel_x = 0.0
                    control_linear_vel_y = 0.0
                    control_angular_vel = 0.0

                else:
                    if (key == '\x03'):
                        break

                twist = Twist()

                twist.linear.x = control_linear_vel_x
                twist.linear.y = control_linear_vel_y
                twist.linear.z = 0.0

                twist.angular.x = 0.0
                twist.angular.y = 0.0
                twist.angular.z = control_angular_vel

                if last_x != control_linear_vel_x or last_y != control_linear_vel_y or last_z != control_angular_vel or control_angular_vel != 0.0:
                    self.cmd_vel.publish(twist)
                    time.sleep(0.5)
                
                last_x = control_linear_vel_x
                last_y = control_linear_vel_y
                last_z = control_angular_vel



        except BaseException as e:
            print(e)

        finally:
            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0
            self.cmd_vel.publish(twist)

        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

def main():
    node = TeleopControl('teleop_control')
    rclpy.spin(node)  # 循环等待ROS2退出(loop waiting for ROS2 to exit)

if __name__ == "__main__":
    main()
