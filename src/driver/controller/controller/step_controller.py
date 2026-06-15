#!/usr/bin/env python3
# encoding: utf-8
import sys
import os
import time
import yaml
import math
import rclpy
import threading
import itertools
import numpy as np
from scipy.spatial.transform import Rotation as R

from rclpy.node import Node
from geometry_msgs.msg import Twist
from interfaces.msg import CmdParam 
from rclpy.executors import MultiThreadedExecutor

from controller import build_in_pose
from kinematics import kinematics, config, kinematics_calculate
from kinematics.x_joint_control import JointControl
from servo_controller.action_group_controller import ActionGroupController
from controller.pose_transformer import PoseTransformer, PoseTransformerParams
from controller.move import MovingGenerator, MovingParams, CmdVelGenerator, CmdVelParams
from servo_controller_msgs.msg import ServosPosition
class StepController(Node):
    
    RIPPLE_GAIT = 1
    TRIPOD_GAIT = 2
    
    def __init__(self):
        
        
        super().__init__('step_controller', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        
        self.joints_state = {}

        for value in config.SERVOS.values():
            self.joints_state[value['name']] = 0.0

        self.lock = threading.RLock()

        self.cur_moving_generator = None
        self.new_moving_generator = None
        self.cur_pose_transformer = None
        self.new_pose_transformer = None
        self.cur_actionset_runner = None
        self.new_actionset_runner = None
        self.cur_pose_setter = None
        self.new_pose_setter = None
        self.last_twist = None 

        self.stop = False
        self.pose = build_in_pose.DEFAULT_POSE
        self.org_transform = ((0, 0, 120), (0, 0, 0))
        self.transform = ((0, 0, 120), (0, 0, 0)) #xyz 平移 mm, xyz 欧拉角 rad(Translation in millimeters along the XYZ axes and Euler angles in radians along the XYZ axes)
        self.servo_controller_pub = self.create_publisher(ServosPosition, 'servo_controller', 1)
        self.acg = ActionGroupController(self.servo_controller_pub, '/home/ubuntu/software/actionset_editor/ActionGroups')
        self.pose_yaw = 0 # 原始积分偏航角(the raw integrated yaw angle)
        self.real_pose_yaw = None  # 多传感器融合后偏航角(the fused yaw angle from multiple sensors)
        self.position = (0, 0, 0)
        self.angular_z = 0 # z轴上的角速度(the angle velocity along the Z-axis)
        self.voltage = 0.0
        self.linear_x, self.linear_y, self.linear_z = 0, 0, 0 # 三轴上的线性速度(the linear velocity along the three axes (X, Y, and Z))
        self.stopped = threading.Event()  # 正在运行的动作已经被停止标志(a sign indicating that the currently running action has been stopped)
        self.stopping = False  # 停止当前正在运行的任务标志(a sign to stop the currently running task)

        self.cmd_gait = 2
        self.cmd_height = 20
        self.cmd_period = 1.0

        self.loop_thread = threading.Thread(target=self.loop, daemon=True)
        self.loop_enable = True
        self.loop_thread.start()
        self.joint_control = JointControl()

        self.create_subscription(CmdParam, '/step_controller/cmd_param', self.cmd_param_callback, 1)
        
    def cmd_param_callback(self, msg):
        with self.lock:
            self.pose = msg.pose
            self.cmd_gait = msg.gait
            self.cmd_height = msg.height
            self.cmd_period = msg.period
            pose =  getattr(build_in_pose, self.pose)
            transform = getattr(build_in_pose, self.pose + '_TRANSFORM')
            self.new_pose_setter = (pose, transform, 2)


    def reset_all_new_gen(self):
        self.new_pose_setter = None
        self.new_actionset_runner = None
        self.new_pose_transformer = None
        self.new_moving_generator = None

    def reset_all_cur_gen(self):
        self.cur_pose_setter = None
        self.cur_actionset_runner = None
        self.cur_pose_transformer = None
        self.cur_moving_generator = None

    def loop(self):
        """
        实际执行具体操作的线程循环(the thread loop that actually performs the specific operation)
        """
        os.system("sudo renice -n -19 -p " + str(os.getpid()))
        last_status = 'start'
        send_status = 'first'
        temp = None
        while self.loop_enable:
            # 设置姿态(set posture)
            try:
                if self.new_pose_setter is not None:
                    self.cur_pose_setter = self.new_pose_setter
                    self.new_pose_setter = None
                if self.cur_pose_setter is not None:
                    pose, transform, duration = self.cur_pose_setter
                    if pose is None or transform is None:
                        self.set_pose_base(self.pose, duration, update_pose=True)
                    else:
                        self.set_pose_base(pose, duration, update_pose=True)
                        self.transform = transform
                    self.reset_all_cur_gen()
                    self.reset_all_new_gen()
            except Exception as e:
                self.get_logger().error("SET POSE " + str(e))
                self.reset_all_cur_gen()
                self.reset_all_new_gen()
                continue
            pose = self.pose
            transform = self.transform
            moving_pose = None

            # 进行姿态变换(perform posture transformation)
            try:
                if self.new_pose_transformer is not None:
                    self.cur_pose_transformer = self.new_pose_transformer # 如果没有新的变换任务， 那么new_pose_transformer 就是 None(If there is no new transformation task, then new_pose_transformer is None)
                    self.new_pose_transformer = None

                if self.cur_pose_transformer is not None:
                    pose, transform, last_part = self.cur_pose_transformer.send((pose, transform))
                    if last_part:
                        self.cur_pose_transformer = None
            except Exception as e:
                self.get_logger().error("TRANSFORM " + str(e))
                self.cur_pose_transformer = None

            # 进行步态处理(perform gait processing)
            # 走路有点特殊， 停止前总是要先走当前的完一整步，这样可以简化逻辑。(Walking is a bit special in that the robot always needs to finish the current step before stopping, which simplifies the logic)
            # 当然可以总是重新计算并实时更新步态路径,但是这样会带来些别的问题。(Of course, we could always recalculate and update the gait path in real-time, but that would introduce other issues)
            # 所以我们让机器人在开始一步之后总是要走完完整一步再停止(So we make the robot always finish a complete step before stopping after starting a new one)
            params = None
            last_part = False
            
            if self.cur_moving_generator is None:
                if self.new_moving_generator is not None:
                    self.cur_moving_generator = self.new_moving_generator

            if self.cur_moving_generator is not None:
                try:
                    moving_pose, last_part, params, slow = self.cur_moving_generator.send((pose, send_status))
                    
                except StopIteration as e:
                    # 生成器已自然结束
                    self.cur_moving_generator = None
                except Exception as e:
                    # 其他异常（如类型错误、数值错误等）
                    self.get_logger().error("GENERATE ERROR: " + repr(e))
                    self.cur_moving_generator = None

                if last_part:
                    if slow == 'move':
                        if self.cur_moving_generator is not self.new_moving_generator:
                            self.cur_moving_generator = self.new_moving_generator
                        # 如果没有移动生成器，重置线性和角速度
                        if self.cur_moving_generator is None:
                            self.linear_x, self.linear_y, self.angular_z = 0, 0, 0
                    else:
                        send_status = 'running'
                        if self.cur_moving_generator is not self.new_moving_generator:
                            if last_status != 'finish':
                                last_status = 'stop'
                                send_status = 'finish'
                                temp = self.new_moving_generator
                        if self.cur_moving_generator is None:
                            self.linear_x, self.linear_y, self.angular_z = 0, 0, -1
                        if last_status == 'finish':
                            send_status = 'first'
                            last_status = 'start'
                            self.cur_moving_generator = temp
            else:
                self.linear_x, self.linear_y, self.angular_z = 0, 0, 0
                        
            # 应用新的姿态(apply new posture)
            if pose is not self.pose:
                try:
                    self.set_pose_base(pose, 0.02, pseudo=(moving_pose is not None), update_pose=True)
                    self.transform = transform
                except Exception as e:
                    self.get_logger().info("POSE " + str(e))
                    self.cur_pose_transformer = None

            if moving_pose is not None:
                try: 
                    if send_status == 'first' and slow == 'cmd_true':
                        self.set_pose_base(moving_pose, 0.05, pseudo=False, update_pose=False)
                        # time.sleep(0.02)
                    else:
                        self.set_pose_base(moving_pose, 0.02, pseudo=False, update_pose=False)
                    if last_status == 'stop':
                        last_status = 'finish'
                    self.transform = transform
                    if isinstance(params, CmdVelParams):
                        self.linear_x = params.velocity_x / 1000.0
                        self.linear_y = params.velocity_y / 1000.0
                        self.angular_z = params.angular_z
                        yaw = self.real_pose_yaw if self.real_pose_yaw else self.pose_yaw
                        x = math.cos(yaw) * (self.linear_x) * 0.02 - math.sin(yaw) * self.linear_y * 0.02
                        y = math.sin(yaw) * (self.linear_x) * 0.02 + math.cos(yaw) * self.linear_y * 0.02
                        self.position = self.position[0] + x, self.position[1] + y, self.position[2]
                        self.pose_yaw += self.angular_z * 0.02


                except Exception as e:
                    self.get_logger().info("MOVING_POSE " + str(e))
                    self.cur_moving_generator = None
                    self.linear_x, self.linear_y, self.angular_z = 0, 0, 0
            else:
                self.linear_x, self.linear_y, self.angular_z = 0, 0, 0
                
            if self.cur_moving_generator is None and self.cur_pose_setter is None and self.cur_pose_transformer is None and self.cur_actionset_runner is None:
                self.stopped.set()
            time.sleep(0.02)

    def stop_running(self, timeout=0, callback=None):
        """
        停止当前正在执行的任务(stop the currently executing task)
        :param timeout: 超时实际, 超过这个实际还没停止的话直接返回(Timeout limit. If the task is not stopped after this time limit, return directly)
        """
        with self.lock:
            self.reset_all_new_gen() # 将所有现有的新指令清空(clear all existing new commands)
            self.stopping = True
            self.stopped.clear()
            if timeout is None:
                self.stopped.wait()
            elif timeout > 0:
                self.stopped.wait(timeout)
        if callable(callback):
            callback()

    def set_leg_position(self, leg_id, position, duration, pseudo=False, update_pose=False):
        """
        根据输入的指定的腿及末端位置， 计算、设置舵机角度(calculate and set the servo angles based on the specified leg and end effector position)
        此方法将可能更新类成员pose(this method may update the class member 'pose')
        :param leg: 腿的号数(the number of the leg)
        :param position: 末端位置(the end effector position)
        :param duration: 完成此次移动所用时间(the time required to complete this movement)
        :param pseudo: 是否真的执行移动， 若True则只返回计算得到的对应舵机角度而不真正发送控制指令给舵机(Whether to actually execute the movement. If True, only the corresponding servo angles calculated will be returned without actually sending control commands to the servo)
        :param update_pose: 是否更新类成员pose, 此成员记录了机器人的当前姿态(Whether to update the class member 'pose', which records the current posture of the robot)
        :return: 末端位置对应的舵机角度（里(id, 角度）， 中(id, 角度）， 外）, 角度为0-1000的数值(The servo angles corresponding to the end effector position (inside (id, angle), middle (id, angle), outside), with angle values ranging from 0 to 1000)
        """
        self.get_logger().info(f'{leg_id}: {position}')
        joints = kinematics.set_leg_position(leg_id, position)  # 计算新末端位置对应的各个舵机的角度(calculate the angles of each servo corresponding to the new end effector position)
        joints_id_radians = zip([(leg_id - 1) * 3 + i + 1 for i, s in enumerate(joints)], joints)

        if not pseudo:

            new_joints_state = self.joint_control.set_multi_joints(duration, joints_id_radians, self.joints_state)
            self.joints_state = new_joints_state


        if update_pose:
            pose = list(self.pose)
            pose[leg_id - 1] = tuple(position)
            self.pose = tuple(pose)
        return joints

    def set_joint(self, joint_id, radians, duration):
        """
        设置关节角度(set joint angle)
        :param joint_id: 关节id(joint ID)
        :param radians: 关节角度， 单位为弧度(joint angle in the unit of radian)
        :param duration: 完成此动作的用时(the time required to complete this movement)
        """
        new_joints_state = self.joint_control.set_joint(joint_id, radians, duration, self.joints_state)
        self.joints_state = new_joints_state



    def set_leg_relatively(self, leg_id, offset, duration):
        cur_pos = list(self.pose[leg_id - 1])
        new_pos = cur_pos[0] + offset[0], cur_pos[1] + offset[1], cur_pos[2] + offset[2]
        self.set_leg_position(leg_id, new_pos, duration)

    def set_pose_base(self, new_pose, duration, pseudo=False, update_pose=False):
        """
        设置机器人的姿态的基础调用，其他 function 都会调用它(Basic call to set the posture of the robot. It will be called for other functions)
        此方法将更新类成员pose(this method will update the class member 'pose')
        :param new_pose:  机器人的新姿态，六条腿的末端坐标,形如（(x1, y1, z1), (x2, y2, z2),...)(The new posture of the robot and the end effector coordinates of the six legs, in the form of ((x1, y1, z1), (x2, y2, z2), ...))
        :param duration: 完成这次动作所用时间(the time required to complete this movement)
        :param pseudo: 是否真的控制舵机运动， 若为True则只计算并设置相应变量而不真正发送控制指令给舵机(whether to actually control the servo movement. If True, only calculate and set the corresponding variables without actually sending control commands to the servo)
        :return: None
        """
        joints = [kinematics.set_leg_position(i + 1, position) for i, position in enumerate(new_pose)]
        joints = list(itertools.chain.from_iterable(joints))
        joints_data = [[j, r] for j, r in zip(list(range(1, 19)), joints)]
        


        if not pseudo:
            self.joints_state = self.joint_control.set_multi_joints(duration, joints_data, self.joints_state)
        if update_pose:
            self.pose = tuple(map(tuple, new_pose))

    def set_pose(self, pose, transform, duration, interrupt=True):
        """
        设置机器人的姿态(set the posture of the robot)
        :param pose: 新姿态(new posture)
        :param duration:  完成这次动作所用时间(the time required to complete this movement)
        :return:
        """
        with self.lock:
            if self.pose is None:
                self.new_pose_setter = (None, None, duration)
            else:  
                self.new_pose_setter = (pose, transform, duration)

    
    def set_build_in_pose(self, pose_name, duration, interrupt=True):
        """
        设置机器人的姿态(set the posture of the robot)
        :param pose: 新姿态(new posture)
        :param duration:  完成这次动作所用时间(the time required to complete this movement)
        :return: None
        """

        with self.lock:
            pose =  getattr(build_in_pose, pose_name)
            transform = getattr(build_in_pose, pose_name + '_TRANSFORM')
            self.new_pose_setter = (pose, transform, duration)

    def transform_pose_quaternion(self, translate, quaternion, duration):
        """
        使用平移变换加四元数改变机器人的姿态(change the posture of the robot using translation transformation and quaternion)
        :param translate: 机体中心偏移 (x, y, z)(offset of the center of the robot body (x, y, z))
        :param quaternion: 机体的旋转变换四元数 (x, y, z, w)(quaternion for the rotation and transformation of the robot body)
        :param duration: 完成这个变换的用时(the time required to complete this transformation)
        :return: None
        """
        # 设置新的机器人姿态(set new posture for the robot)
        self.set_pose_base(kinematics_calculate.transform_quat(translate, quaternion), duration) 
    
    def transform_absolutely(self, translate, euler, duration):
        generator = PoseTransformer(PoseTransformerParams(translation=translate, rotation=euler, absolutely=True, duration=duration))
        if generator:
            with self.lock:
                generator.send(None)
                self.new_pose_transformer = generator

    def transform_pose_euler(self, translate, axis, euler, duration, degrees=True):
        """
        使用平移变换加欧拉角改变机器人的姿态(change the posture of the robot using translation transformation and Euler angles)
        :param translate: 机体中心偏移的平移变换 (x, y, z)(the translation transformation for the center offset of the robot (x, y, z))
        :param axis: 欧拉角三个轴的顺序 如 'xyz' 或者 'yzx'(the order of the three axes of Euler angles, such as 'xyz' or 'yzx')
        :param euler: 欧拉角的元组, 顺序要与axis一致(the tuple of Euler angles, in the same order as the axis)
        :param duration: 完成这个变换的用时(the time required to complete this transformation)
        :param degrees: 欧拉角单位是否为角度, True为角度, False为弧度(Whether the unit of Euler angles is degrees. True for degrees, False for radians)
        """
        rotate = R.from_euler(axis, euler, degrees=degrees)  # 用欧拉角建立转换器(use Euler anglr to create transformer)
        r = rotate.as_euler('xyz', degrees=False) # 转换成固定顺序的欧拉角(convert Euler angles to fixed order)
        generator = PoseTransformer(PoseTransformerParams(translation=translate, rotation=r, duration=duration))

        if generator:
            with self.lock:
                generator.send(None)
                self.new_pose_transformer = generator
      
    def cmd_vel(self, twist: Twist):
        # 1. 定义速度变化的阈值
        # LINEAR_VEL_THRESHOLD = 0.01  # m/s, 线性速度变化阈值，例如1cm/s
        # ANGULAR_VEL_THRESHOLD = 0.1 # rad/s, 角速度变化阈值，例如约3度/s

        # # 2. 如果上一次的速度存在，则进行判断
        # if self.last_twist:
        #     # 计算线速度和角速度的变化量
        #     linear_x_diff = abs(twist.linear.x - self.last_twist.linear.x)
        #     linear_y_diff = abs(twist.linear.y - self.last_twist.linear.y)
        #     angular_z_diff = abs(twist.angular.z - self.last_twist.angular.z)

        #     # 如果所有变化量都在阈值之内，则认为速度未变，直接返回
        #     if (linear_x_diff < LINEAR_VEL_THRESHOLD and
        #         linear_y_diff < LINEAR_VEL_THRESHOLD and
        #         angular_z_diff < ANGULAR_VEL_THRESHOLD):
        #         return
        
        # # 3. 如果是第一次接收指令，或者速度变化超过阈值，则更新last_twist
        # # 如果twist是 (0, 0, 0)，表示停止，将last_twist设为None，以便下次任何非零速度都能启动
        # if twist.linear.x == 0 and twist.linear.y == 0 and twist.angular.z == 0:
        #     self.last_twist = None
        # else:
        #     self.last_twist = twist
        
        linear_x = twist.linear.x*1000  # linear_x 单位为 毫米每秒(linear_x is measured in meters per second)
        linear_y = twist.linear.y*1000  # linear_y 单位为 毫米每秒(linear_y is measured in meters per second)
        angular_z = twist.angular.z # 旋转角速度 rad/sec(the speed of rotation angle is rad/sec)


        generator = CmdVelGenerator(CmdVelParams(
            gait =  self.cmd_gait,
            velocity_x = linear_x,
            velocity_y = linear_y,
            angular_z = angular_z,
            height = self.cmd_height,
            relative_h = False,
            period = self.cmd_period,
            linear_factor = 1.0,
            rotate_factor = 1.0 # 转向时的转向系数， 即给定的值(the turning coefficient during turning, which is the given value)
        ), self.get_logger())
        if generator:
            with self.lock:
                generator.send(None)
                self.new_moving_generator = generator

                self.stop = False


    def set_step_mode(self,
                      gait,
                      amplitude,
                      height,
                      direction,
                      rotation,
                      duration,
                      repeat=1,
                      relative_height=False,
                      rectify=True,
                      integral=True,
                      interrupt=True,
                      feedback_cb=None):

        if gait == 11 or gait == 12 or gait == 13:
            self.cmd_period = duration
            self.cmd_gait = gait - 10
            self.cmd_height = height

        else:
            self.set_step_mode_base(gait, amplitude, height, direction, rotation, duration, repeat, relative_height, rectify, integral, feedback_cb)


    def set_step_mode_base(self,
                      gait,
                      amplitude,
                      height,
                      direction,
                      rotation,
                      duration,
                      repeat=1,
                      relative_height=False,
                      rectify=True,
                      integral=True,
                      interrupt = True,
                      feedback_cb=None):
        """
        设置机器人的运动步态(set the robot's walking gait)
        :param gait: 步态(walking gait)
        :param amplitude: 步幅(step length)
        :param height: 步高, 即走路时脚尖的抬起高度(The step height, i.e. the height at which the toe is lifted when walking)
        :param direction: 运动方向(direction of motion)
        :param rotation: 机器人绕机体中心的旋转角速度(the angular velocity of the robot around its center of body during walking)
        :param period: 每步用时(the time taken for each step)
        :param repeat: 要走的步数, 0会一直走下去(the number of steps to take. If set to 0, the robot will keep walking)
        :param relative_height: 步高参数是否为相对高度(whether the step height parameter is a relative height)
        :param rectify: 对实际行走距离的校正参数(the correction parameter for the actual walking distance)
        :param integral: 是否对行走距离进行积分实现里程计(whether to integrate the walking distance to implement odom)
        :param feedback_cb: 运行中状态报告的回调，不建议使用(a callback for reporting the status during operation, which is not recommended to use)
        """

        generator = MovingGenerator(MovingParams(
                gait=gait,
                stride = amplitude,
                height = height,
                direction = direction,
                rotation = rotation,
                period = duration,
                repeat = repeat,
                forever = True if repeat == 0 else False,

                relative_h = relative_height,
                ), self.get_logger())

        if generator:
            with self.lock:
                generator.send(None)
                self.new_moving_generator = generator
