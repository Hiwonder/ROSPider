#!/usr/bin/env python3
# encoding: utf-8
# @Author: Aiden
# @Date: 2023/11/10

import time
from rclpy.duration import Duration
from rclpy.time import Time
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer
from servo_controller_msgs.msg import ServoPosition

class Segment:
    def __init__(self, num_joints):
        self.start_time = Time()
        self.duration = 0.0
        self.positions = [0.0] * num_joints
        self.velocities = [0.0] * num_joints

class JointTrajectoryActionController:
    def __init__(self, node, servo_manager, controller_namespace, controllers):
        self.node = node
        self.servo_manager = servo_manager
        
        self.joint_names = []
        self.joint_to_controller = {}
        for c in controllers:
            self.joint_names.append(c.joint_name)
            self.joint_to_controller[c.joint_name] = c

        self.num_joints = len(self.joint_names)

        self.goal_constraints = []
        self.trajectory_constraints = []
        ns = controller_namespace + '/joint_trajectory_action_node/constraints'
        for joint in self.joint_names:
            self.goal_constraints.append(-1)
            self.trajectory_constraints.append(-1)

        # Message containing current state for all controlled joints
        self.feedback_msg = FollowJointTrajectory.Feedback()
        self.feedback_msg.joint_names = self.joint_names
        self.feedback_msg.desired.positions = [0.0] * self.num_joints
        self.feedback_msg.desired.velocities = [0.0] * self.num_joints
        self.feedback_msg.desired.accelerations = [0.0] * self.num_joints
        self.feedback_msg.actual.positions = [0.0] * self.num_joints
        self.feedback_msg.actual.velocities = [0.0] * self.num_joints
        self.feedback_msg.error.positions = [0.0] * self.num_joints
        self.feedback_msg.error.velocities = [0.0] * self.num_joints

        self.action_server = ActionServer(
            self.node,
            FollowJointTrajectory,
            controller_namespace + '/follow_joint_trajectory',
            self.follow_trajectory_callback)

    def follow_trajectory_callback(self, goal_handle):
        self.node.get_logger().info('Received a new goal.')
        goal = goal_handle.request
        traj = goal.trajectory
        num_points = len(traj.points)  # 计算总的轨迹点数

        if num_points == 0:  # 如果没有轨迹点则立刻返回
            msg = 'Incoming trajectory is empty'
            self.node.get_logger().error(msg)
            goal_handle.abort()
            return FollowJointTrajectory.Result()

        lookup = []
        for joint in self.joint_names:
            try:
                index = traj.joint_names.index(joint)
            except ValueError:
                self.node.get_logger().error(f'Joint {joint} not found in trajectory joint names.')
                goal_handle.abort()
                return FollowJointTrajectory.Result()
            lookup.append(index)

        durations = [0.0] * num_points

        # find out the duration of each segment in the trajectory
        durations[0] = traj.points[0].time_from_start.sec + traj.points[0].time_from_start.nanosec / 1e9

        for i in range(1, num_points):
            current_time_sec = traj.points[i].time_from_start.sec + traj.points[i].time_from_start.nanosec / 1e9
            previous_time_sec = traj.points[i - 1].time_from_start.sec + traj.points[i - 1].time_from_start.nanosec / 1e9
            durations[i] = current_time_sec - previous_time_sec

        if not traj.points[0].positions:  # 如果为空
            res = FollowJointTrajectory.Result()
            res.error_code = res.INVALID_GOAL
            msg = 'First point of trajectory has no positions'
            self.node.get_logger().error(msg)
            goal_handle.abort()
            return res

        trajectory = []

        # 如果 traj.header.stamp 为零，则使用当前时间
        if traj.header.stamp.sec == 0 and traj.header.stamp.nanosec == 0:
            traj_header_stamp = self.node.get_clock().now()
        else:
            traj_header_stamp = Time.from_msg(traj.header.stamp)

        current_time = self.node.get_clock().now() + Duration(seconds=0.01)

        for i in range(num_points):
            seg = Segment(self.num_joints)

            time_from_start = Duration.from_msg(traj.points[i].time_from_start)
            durations_i_duration = Duration(seconds=durations[i])

            seg.start_time = traj_header_stamp + time_from_start - durations_i_duration

            seg.duration = durations[i]

            for j in range(self.num_joints):
                if traj.points[i].positions:
                    seg.positions[j] = traj.points[i].positions[lookup[j]]

            trajectory.append(seg)

        self.node.get_logger().info(f'Trajectory start requested at {traj_header_stamp.nanoseconds / 1e9:.3f}, waiting...')

        while traj_header_stamp > current_time:
            current_time = self.node.get_clock().now()
            time.sleep(0.001)

        total_duration = sum(durations)
        end_time = traj_header_stamp + Duration(seconds=total_duration)
        seg_end_times = [trajectory[seg].start_time + Duration(seconds=trajectory[seg].duration) for seg in range(len(trajectory))]

        self.node.get_logger().info(f'Trajectory start time is {current_time.nanoseconds / 1e9:.3f}, end time is {end_time.nanoseconds / 1e9:.3f}, total duration is {total_duration:.3f}')

        for seg in range(len(trajectory)):
            current_time = self.node.get_clock().now()
            time_left = durations[seg] - (current_time - trajectory[seg].start_time).nanoseconds / 1e9
            self.node.get_logger().debug(f'Current segment is {seg}, time left {time_left:.3f}, current time {current_time.nanoseconds / 1e9:.3f}')
            self.node.get_logger().debug(f'Goal positions are: {trajectory[seg].positions}')

            # Skip segments with duration of 0 seconds
            if durations[seg] == 0:
                self.node.get_logger().debug(f'Skipping segment {seg} with duration of 0 seconds')
                continue

            position = []
            for joint in self.joint_names:
                j = self.joint_names.index(joint)
                desired_position = trajectory[seg].positions[j]
                self.feedback_msg.desired.positions[j] = desired_position
                servo_id = self.joint_to_controller[joint].servo_id
                pos = self.joint_to_controller[joint].pos_rad_to_pulse(desired_position)
                position.append((servo_id, pos))

            # Send positions to servo manager
            self.servo_manager.set_position(durations[seg], [ServoPosition(id=id_, position=pos_) for id_, pos_ in position])

            # 在这里打印反馈消息表示程序是否接收到了反馈
            self.node.get_logger().info(f'Feedback received: desired positions: {self.feedback_msg.desired.positions}')

            while self.node.get_clock().now() < seg_end_times[seg]:
                if goal_handle.is_cancel_requested:
                    msg = 'Trajectory execution was canceled.'
                    self.node.get_logger().info(msg)
                    goal_handle.canceled()
                    return FollowJointTrajectory.Result()

                time.sleep(0.001)

        # Trajectory execution completed
        msg = 'Trajectory execution successfully completed'
        self.node.get_logger().info(msg)
        goal_handle.succeed()
        return FollowJointTrajectory.Result()

