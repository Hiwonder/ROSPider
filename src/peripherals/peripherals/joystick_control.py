#!/usr/bin/env python3
# encoding: utf-8
import time
import math
import rclpy
import numpy as np
from enum import Enum
from rclpy.node import Node
from sdk.common import val_map
from std_srvs.srv import Trigger
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from interfaces.msg import CmdParam
import arm_kinematics.transform as transform
from servo_controller import bus_servo_control
from rclpy.executors import MultiThreadedExecutor
from ros_robot_controller_msgs.msg import BuzzerState
from rclpy.callback_groups import ReentrantCallbackGroup
from arm_kinematics.kinematics_control import set_pose_target
from arm_kinematics_msgs.srv import SetRobotPose, GetRobotPose
from servo_controller_msgs.msg import ServosPosition, ServoPosition, ServoStateList
from controller.controller_client import ControllerClient

AXES_MAP = 'lx', 'ly', 'rx', 'ry', 'r2', 'l2', 'hat_x', 'hat_y'
BUTTON_MAP = 'cross', 'circle', '', 'square', 'triangle', '', 'l1', 'r1', 'l2', 'r2', 'select', 'start', '', 'l3', 'r3', '', 'hat_xl', 'hat_xr', 'hat_yu', 'hat_yd', ''

BIG_STEP = 0.15
BIG_ROTATE = math.radians(25)
SMALL_STEP = 0.08
SMALL_ROTATE = math.radians(15)

class ButtonState(Enum):
    Normal = 0
    Pressed = 1
    Holding = 2
    Released = 3

class JoystickController(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)


        self.min_value = 0.1

        self.transform = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
        self.direction, self.rotate, self.stride = 0, 0, 0
        self.period = 1.0
        self.step_height = 10
        self.gait = 1
        self.count = 0
        self.current_servo_position = None
        self.do_transform = False
        self.do_movement = False
        self.force_stopped = False
        self.set_parameters = False
        self.slam = self.get_parameter('slam').value
        self.rtabmap = self.get_parameter('rtabmap').value

        self.joints_pub = self.create_publisher(ServosPosition, '/servo_controller', 1)
        self.joy_sub = self.create_subscription(Joy, 'ros_robot_controller/joy', self.joy_callback, 1)
        self.buzzer_pub = self.create_publisher(BuzzerState, 'ros_robot_controller/set_buzzer', 1)
        self.servo_states_sub = self.create_subscription(ServoStateList, '/controller_manager/servo_states', self.servo_states_callback, 10)
        self.cmd_param_pub = self.create_publisher(CmdParam, '/step_controller/cmd_param', 1) 
        timer_cb_group = ReentrantCallbackGroup()
        self.client = self.create_client(Trigger, '/controller_manager/init_finish',  callback_group=timer_cb_group)
        self.client.wait_for_service()
        self.client = self.create_client(Trigger, '/arm_kinematics/init_finish', callback_group=timer_cb_group)
        self.client.wait_for_service()
        self.kinematics_client = self.create_client(SetRobotPose, '/arm_kinematics/set_pose_target', callback_group=timer_cb_group)
        self.kinematics_client.wait_for_service()
        self.get_current_pose_client = self.create_client(GetRobotPose, '/arm_kinematics/get_current_pose', callback_group=timer_cb_group)
        self.get_current_pose_client.wait_for_service()

        self.last_axes = dict(zip(AXES_MAP, [0.0, ] * len(AXES_MAP)))
        self.last_buttons = dict(zip(BUTTON_MAP, [0.0, ] * len(BUTTON_MAP)))
        self.mode = 0
        self.controller = ControllerClient()
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'joystick start')
        if self.slam:
            cmd_param = CmdParam()
            cmd_param.pose = 'DEFAULT_POSE_M'
            cmd_param.gait = 2
            cmd_param.height = 20
            cmd_param.period = 1.0

            self.cmd_param_pub.publish(cmd_param)

    def get_node_state(self, request, response):
        response.success = True
        return response
    
    def send_request(self, client, msg):
        future = client.call_async(msg)
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()
    
    def buzzer_warn(self, repeat):
        msg = BuzzerState()
        msg.freq = 2100
        msg.on_time = 0.15
        msg.off_time = 0.01
        msg.repeat = repeat
        self.buzzer_pub.publish(msg)

    def servo_states_callback(self,msg):
        servo_positions = []
        for i in msg.servo_state:
            servo_positions.append(i.position)
        self.current_servo_position = np.array(servo_positions)

    def relative_move(self, x, y, z, pitch):

        endpoint = self.send_request(self.get_current_pose_client, GetRobotPose.Request())

        pose = endpoint.pose
        x += pose.position.x
        y += pose.position.y
        z += pose.position.z
        rpy = transform.qua2rpy(endpoint.pose.orientation)
        self.pitch = pitch + rpy[1]
        msg = set_pose_target([x, y, z], self.pitch, pitch_range=[-50, 50], resolution=1)

        res = self.send_request(self.kinematics_client, msg)
        if res.pulse : # 可以达到
            servo_data = res.pulse  
            # # 驱动舵机
            bus_servo_control.set_servo_position(self.joints_pub, 0.02, ((19, servo_data[0]), (20, servo_data[1]), (21, servo_data[2]), (22, servo_data[3])))

    def axes_callback(self, axes):
        lx, ly, rx, ry = axes['lx'], axes['ly'], axes['rx'], axes['ry']

        if abs(lx) < self.min_value: lx = 0
        if abs(ly) < self.min_value: ly = 0
        if abs(rx) < self.min_value: rx = 0
        if abs(ry) < self.min_value: ry = 0

        if axes['l2'] == 1.0:
            self.l2_callback(ButtonState.Pressed)
        
        if axes['r2'] == 1.0:
            self.r2_callback(ButtonState.Pressed)
            
        # 处理平移和旋转的叠加逻辑
        if lx != 0 or ly != 0 or rx != 0:
            self.force_stopped = False
            self.gait = 2
            
            # 计算平移方向和步幅
            if lx != 0 or ly != 0:
                self.stride = BIG_STEP
                self.direction = math.atan2(lx, ly)
            else:
                self.stride = 0

            # 计算旋转
            if rx != 0:
                self.rotate = BIG_ROTATE if rx > 0 else -BIG_ROTATE
            else:
                self.rotate = 0
            
            self.do_movement = True
        else:
            # 停止逻辑
            if not self.force_stopped and (self.stride != 0 or self.rotate != 0):
                self.direction, self.rotate, self.stride = 0, 0, 0
                self.do_movement = True

        # 处理高度/姿态变换 (ry)
        if ry > 0:
            new_state = ButtonState.Pressed if self.last_axes['ry'] <= self.min_value else ButtonState.Holding
            self.raxis_u_callback(new_state)
        elif ry < 0:
            new_state = ButtonState.Pressed if self.last_axes['ry'] >= -self.min_value else ButtonState.Holding
            self.raxis_d_callback(new_state)

    def raxis_u_callback(self, new_state):
        self.force_stopped = False
        if new_state == ButtonState.Pressed or new_state == ButtonState.Holding:
            self.transform[0][2] = 5.0
            self.do_transform = True

    def raxis_d_callback(self, new_state):
        self.force_stopped = False
        if new_state == ButtonState.Pressed or new_state == ButtonState.Holding:
            self.transform[0][2] = -5.0
            self.do_transform = True

    def raxis_l_callback(self, new_state):
        self.force_stopped = False
        if new_state == ButtonState.Pressed:
            self.gait = 2
            self.rotate = BIG_ROTATE
            self.do_movement = True
        elif new_state == ButtonState.Released:
            self.rotate = 0
            self.do_movement = True
        else:
            pass

    def raxis_r_callback(self, new_state):
        self.force_stopped = False
        if new_state == ButtonState.Pressed:
            self.gait = 2
            self.rotate = -BIG_ROTATE
            self.do_movement = True
        elif new_state == ButtonState.Released:
            self.rotate = 0
            self.do_movement = True
        else:
            pass

    def laxis_u_callback(self, new_state):
        self.force_stopped = False
        if new_state == ButtonState.Pressed:
            self.gait = 2
            self.direction, self.rotate, self.stride = 0, 0, BIG_STEP
            self.do_movement = True
        elif new_state == ButtonState.Released:
            self.direction, self.rotate, self.stride = 0, 0, 0
            self.do_movement = True
        else:
            pass

    def laxis_d_callback(self, new_state):
        self.force_stopped = False
        if new_state == ButtonState.Pressed:
            self.gait = 2
            self.direction, self.rotate, self.stride = math.radians(180), 0, BIG_STEP
            self.do_movement = True
        elif new_state == ButtonState.Released:
            self.direction, self.rotate, self.stride = 0, 0, 0
            self.do_movement = True
        else:
            pass

    def laxis_l_callback(self, new_state):
        self.force_stopped = False
        if new_state == ButtonState.Pressed:
            self.gait = 2
            self.direction, self.rotate, self.stride = math.radians(90), 0, BIG_STEP
            self.do_movement = True
        elif new_state == ButtonState.Released:
            self.direction, self.rotate, self.stride = 0, 0, 0
            self.do_movement = True
        else:
            pass

    def laxis_r_callback(self, new_state):
        self.force_stopped = False
        if new_state == ButtonState.Pressed:
            self.gait = 2
            self.direction, self.rotate, self.stride = math.radians(270), 0, BIG_STEP
            self.do_movement = True
        elif new_state == ButtonState.Released:
            self.direction, self.rotate, self.stride = 0, 0, 0
            self.do_movement = True
        else:
            pass

    def l1_callback(self, new_state):
        if self.mode == 0 and (new_state == ButtonState.Pressed or new_state == ButtonState.Holding):
            period = self.period - 0.1
            self.period = max(period, 0.8)
            self.set_parameters = True
            self.do_movement = True
        if self.mode == 1 and (new_state == ButtonState.Pressed or new_state == ButtonState.Holding):
            bus_servo_control.set_servo_position(self.joints_pub, 0.050, ((21, int(self.current_servo_position[20] - 10)), ))   

    def l2_callback(self, new_state):
        if self.mode == 0 and (new_state == ButtonState.Pressed or new_state == ButtonState.Holding):
            period = self.period + 0.1
            self.period = min(period, 3.0)
            self.set_parameters = True
            self.do_movement = True
        if self.mode == 1 and (new_state == ButtonState.Pressed or new_state == ButtonState.Holding):
            bus_servo_control.set_servo_position(self.joints_pub, 0.050, ((21, int(self.current_servo_position[20] + 10)), ))

    def r1_callback(self, new_state):
        if self.mode == 0 and (new_state == ButtonState.Pressed or new_state == ButtonState.Holding):
            step_height = self.step_height + 2
            self.step_height = min(step_height, 80)
            self.set_parameters = True
            self.do_movement = True
        if self.mode == 1 and (new_state == ButtonState.Pressed or new_state == ButtonState.Holding):
            bus_servo_control.set_servo_position(self.joints_pub, 0.050, ((22, int(self.current_servo_position[21] + 10)), ))


    def r2_callback(self, new_state):
        if self.mode == 0 and (new_state == ButtonState.Pressed or new_state == ButtonState.Holding):
            step_height = self.step_height - 2
            self.step_height = max(step_height, 14)
            self.set_parameters = True
            self.do_movement = True
        if self.mode == 1 and (new_state == ButtonState.Pressed or new_state == ButtonState.Holding):
            bus_servo_control.set_servo_position(self.joints_pub, 0.050, ((22, int(self.current_servo_position[21] - 10)), ))


    def cross_callback(self, new_state):
        
        if self.mode == 0 and (new_state == ButtonState.Pressed or new_state == ButtonState.Holding):
            self.force_stopped = False
            self.transform[1][1] = math.radians(-2)
            self.do_transform = True
        if (self.mode == 1) and (new_state == ButtonState.Pressed or new_state == ButtonState.Holding):
            bus_servo_control.set_servo_position(self.joints_pub, 0.050, ((24, int(self.current_servo_position[23] - 10)), ))


    def triangle_callback(self, new_state):
        
        if self.mode == 0 and (new_state == ButtonState.Pressed or new_state == ButtonState.Holding):
            self.force_stopped = False
            self.transform[1][1] = math.radians(2)
            self.do_transform = True
        if (self.mode == 1) and (new_state == ButtonState.Pressed or new_state == ButtonState.Holding):
            bus_servo_control.set_servo_position(self.joints_pub, 0.050, ((24, int(self.current_servo_position[23] + 10)), ))

    def circle_callback(self, new_state):
        
        if self.mode == 0 and (new_state == ButtonState.Pressed or new_state == ButtonState.Holding):
            self.force_stopped = False
            self.gait = 1
            self.rotate = -SMALL_ROTATE
            self.do_movement = True
        if self.mode == 0 and new_state == ButtonState.Released:
            self.rotate = 0
            self.do_movement = True
        if (self.mode == 1) and (new_state == ButtonState.Pressed or new_state == ButtonState.Holding):
            bus_servo_control.set_servo_position(self.joints_pub, 0.010, ((23, int(self.current_servo_position[22] + 10)), ))

    def square_callback(self, new_state):
        
        if self.mode == 0 and (new_state == ButtonState.Pressed or new_state == ButtonState.Holding):
            self.force_stopped = False
            self.gait = 1
            self.rotate = SMALL_ROTATE
            self.do_movement = True
        if self.mode == 0 and  new_state == ButtonState.Released:
            self.rotate = 0
            self.do_movement = True
        if (self.mode == 1) and (new_state == ButtonState.Pressed or new_state == ButtonState.Holding):
            bus_servo_control.set_servo_position(self.joints_pub, 0.050, ((23, int(self.current_servo_position[22] - 10)), ))


    def hat_yu_callback(self, new_state):
        
        if self.mode == 0 and (new_state == ButtonState.Pressed or new_state == ButtonState.Holding):
            self.force_stopped = False
            self.gait = 1
            self.direction, self.stride = 0, SMALL_STEP
            self.do_movement = True
        if self.mode == 0 and  new_state == ButtonState.Released:
            self.stride = 0
            self.do_movement = True
        if self.mode == 1 and (new_state == ButtonState.Pressed or new_state == ButtonState.Holding):
            bus_servo_control.set_servo_position(self.joints_pub, 0.050, ((20, int(self.current_servo_position[19] - 10)), ))


    def hat_yd_callback(self, new_state):
        
        if self.mode == 0 and (new_state == ButtonState.Pressed or new_state == ButtonState.Holding):
            self.force_stopped = False
            self.gait = 1
            self.direction, self.stride = math.radians(180), SMALL_STEP
            self.do_movement = True
        if self.mode == 0 and  new_state == ButtonState.Released:
            self.stride = 0
            self.do_movement = True
        if self.mode == 1 and (new_state == ButtonState.Pressed or new_state == ButtonState.Holding):
            bus_servo_control.set_servo_position(self.joints_pub, 0.050, ((20, int(self.current_servo_position[19] + 10)), ))


    def hat_xl_callback(self, new_state):
        
        if self.mode == 0 and (new_state == ButtonState.Pressed or new_state == ButtonState.Holding):
            self.force_stopped = False
            self.gait = 1
            self.direction, self.stride = math.radians(90), SMALL_STEP
            self.do_movement = True
        if self.mode == 0 and  new_state == ButtonState.Released:
            self.stride = 0
            self.do_movement = True
        if self.mode == 1 and (new_state == ButtonState.Pressed or new_state == ButtonState.Holding):
            bus_servo_control.set_servo_position(self.joints_pub, 0.050, ((19, int(self.current_servo_position[18] + 10)), ))


    def hat_xr_callback(self, new_state):
        
        if self.mode == 0 and (new_state == ButtonState.Pressed or new_state == ButtonState.Holding):
            self.force_stopped = False
            self.gait = 1
            self.direction, self.stride = math.radians(270), SMALL_STEP
            self.do_movement = True
        if self.mode == 0 and new_state == ButtonState.Released:
            self.stride = 0
            self.do_movement = True
        if self.mode == 1 and (new_state == ButtonState.Pressed or new_state == ButtonState.Holding):
            bus_servo_control.set_servo_position(self.joints_pub, 0.050, ((19, int(self.current_servo_position[18] - 10)), ))


    def start_callback(self, new_state):
        try:
            if new_state == ButtonState.Pressed and not self.last_buttons['select']:
                repeat = 1
                if self.slam:
                    self.controller.traveling(gait=-1, time=1, steps=0)
                    if self.rtabmap:
                        bus_servo_control.set_servo_position(self.joints_pub, 1.0, ((19, 500), (20, 810), (21, 180), (22, 150), (23, 500), (24, 500)))
                    else:
                        bus_servo_control.set_servo_position(self.joints_pub, 1.0, ((19, 500), (20, 720), (21, 130), (22, 150), (23, 500), (24, 500)))
                else:
                    self.controller.traveling(gait=-2, time=1, steps=0)
                    bus_servo_control.set_servo_position(self.joints_pub, 1.0, ((19, 500), (20, 720), (21, 130), (22, 150), (23, 500), (24, 500)))


                self.buzzer_warn(repeat)
                self.force_stopped = True


            if new_state == ButtonState.Pressed and self.last_buttons['select']:
                if self.mode == 0:
                    self.mode = 1
                    repeat = 2
                elif self.mode == 1:
                    self.mode = 0
                    repeat = 1

                self.buzzer_warn(repeat)


        except Exception as e:
            self.get_logger().error(str(e))

    def joy_callback(self, joy_msg):
        # self.get_logger().info('value'+str(joy_msg))

        axes = dict(zip(AXES_MAP, joy_msg.axes))
        axes_changed = False
        hat_x, hat_y = axes['hat_x'], axes['hat_y']
        hat_xl, hat_xr = 1 if hat_x > 0.5 else 0, 1 if hat_x < -0.5 else 0
        hat_yu, hat_yd = 1 if hat_y > 0.5 else 0, 1 if hat_y < -0.5 else 0
        buttons = list(joy_msg.buttons)
        buttons.extend([hat_xl, hat_xr, hat_yu, hat_yd, 0])
        buttons = dict(zip(BUTTON_MAP, buttons))

        for key, value in axes.items(): # 轴的值被改变(the value of axes is changed)
            # if abs(self.last_axes[key] - value) > 0.01:
            if self.last_axes[key] != value:
                axes_changed = True

        # 检查是否有任何轴处于激活状态（不在死区内）
        any_axes_active = any(abs(v) > self.min_value for v in axes.values())
        
        # 只要轴数值发生了变化，或者当前有轴正被推向某个方向，就调用 axes_callback
        if axes_changed or any_axes_active:
            try:
                self.axes_callback(axes)
            except Exception as e:
                self.get_logger().error(str(e))
        
        if axes_changed:
            axes_changed = False
        for key, value in buttons.items():
            if value != self.last_buttons[key]:
                new_state = ButtonState.Pressed if value > 0 else ButtonState.Released
                
            else:
                new_state = ButtonState.Holding if value > 0 else ButtonState.Normal

            callback = "".join([key, '_callback'])

            if new_state != ButtonState.Normal:
                if  hasattr(self, callback):
                    try:
                        getattr(self, callback)(new_state)
                    except Exception as e:
                        self.get_logger().error(str(e))

        if self.do_movement and not self.force_stopped:
            self.controller.traveling(10 + self.gait, height=self.step_height, time=self.period)
            speed = self.stride * (1.0 / self.period)/2
            rotate = self.rotate * (1.0 / self.period)/2
            vx = math.cos(self.direction) * speed
            vy = math.sin(self.direction) * speed
            self.controller.cmd_vel(vx, vy, rotate)
            self.do_movement = False


        if self.do_transform:
            self.controller.pose_transform_euler(self.transform[0], self.transform[1], 0.1)
            self.transform = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
            self.do_transform = False

        self.last_buttons = buttons
        self.last_axes = axes

def main():
    node = JoystickController('joystick_control')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
if __name__ == "__main__":
    main()

