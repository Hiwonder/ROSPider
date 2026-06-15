#!/usr/bin/env python3
# coding: utf-8

import time
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from controller import step_controller
from interfaces.msg import RunActionSet
from kinematics_msgs.srv import SetPose1
from rclpy.executors import MultiThreadedExecutor
from scipy.spatial.transform import Rotation as R
from servo_controller_msgs.msg import ServosPosition
from rclpy.callback_groups import ReentrantCallbackGroup

from servo_controller.action_group_controller import ActionGroupController
from kinematics_msgs.msg import Traveling, TransformEuler, LegPosition, Pose
from geometry_msgs.msg import Quaternion, Vector3, TransformStamped, TwistWithCovarianceStamped, Twist


class MoveController(Node):
    def __init__(self, node_name):
        rclpy.init()
        super().__init__(node_name)
        
        self.v = 0.0
        self.bian = 1
        self.last_time = 0.03
        self.action_complete = True
        # 初始化计数器
        self.current_trans = [0.0, 0.0, 0.0]  # x,y,z位移累计
        self.current_rot = [0.0, 0.0, 0.0]  # x,y,z旋转累计
    
        # 设置计数边界（根据实际需求调整）
        self.translation_limits = [0.60, 0.14, 40.0]  # 位移最大阈值
        self.rotation_limits = [0.20, 0.20, 0.40]  # 旋转最大阈值

        # 控制器初始化
        self.step_controller = step_controller.StepController()
        self.status_pub = self.create_publisher(Bool, 'action_complete', 1)
        self.agc = ActionGroupController(self.create_publisher(ServosPosition, 'servo_controller', 1), '/home/ubuntu/software/actionset_editor/ActionGroups',self.status_pub)

        # 启动完成信号
        self.declare_parameter('tf_prefix', '')
        self.declare_parameter('odom_enable', False)
        
        if self.get_parameter('odom_enable').value:
            self.odom_trans_pub = self.create_publisher(TransformStamped, 'middle_tf', 1)
            self.odometry_pub = self.create_publisher(Odometry, 'odom/raw', 1)
        timer_cb_group = ReentrantCallbackGroup()
        # 姿态控制订阅
        # 机器人机体姿态变换
        # 通过 平移和欧拉角旋转变换机器人的姿态， 相对姿态变换, 旋转顺序为 RPY
        self.create_subscription(TransformEuler, '~/pose_transform_euler', self.pose_transform_euler_callback, 1)  
        # 通过 平移和欧拉角旋转变换机器人的姿态， 绝对变换, 旋转顺序为 RPY
        self.create_subscription(Pose, '~/set_pose_euler', self.set_pose_euler_callback, 1)

        # 腿部控制订阅
        # 设置一条腿末端移动到指定位置, 绝对坐标
        self.create_subscription(LegPosition, '~/set_leg_absolute', self.set_leg_absolute_callback, 1)  
         # 设置一条腿末端移动当相对与当期位置的指定位置
        self.create_subscription(LegPosition, '~/set_leg_relatively', self.set_leg_relatively_callback,1)
        
        # 运动控制订阅
        # 通过步态参数控制机器人的移动
        self.create_subscription(Traveling, '~/traveling', self.set_traveling_callback, 1)
         # 通过线速度、角速度控制机器人的移动，其他参数由上一次执行的 gait大于0的traveling来指定
        self.create_subscription(Twist, '~/cmd_vel', self.cmd_vel_callback, 1)  
        # 机器人动作组运行服务
        self.create_subscription(RunActionSet, '~/run_actionset', self.run_actionset_callback, 1)
        # 机器人姿态设置服务
        self.create_service(SetPose1, '~/set_pose_1', self.set_pose1_callback, callback_group=timer_cb_group)
        # 机器人特殊动作组发布
        self.perfrom_actions_pub = self.create_publisher(RunActionSet, '/perform_actions/actions', 1)
        # 机器人动作组运行完成订阅
        self.create_subscription(Bool, '/action_complete', self.action_complete_callback, 1)

        if self.get_parameter('odom_enable').value:
            self.create_timer(0.02, self.odometry_publish)
        self.tf_prefix = self.get_parameter('tf_prefix').value
        self.tf_prefix = f"{self.tf_prefix}/" if self.tf_prefix else ''
        
    def action_complete_callback(self, msg):
        """获取动作组完成状态"""
        self.action_complete = msg.data

   
    def pose_transform_euler_callback(self, msg: TransformEuler):
        # 获取当前值
        current_trans = [msg.translation.x, msg.translation.y, msg.translation.z]
        current_rot = [msg.rotation.x, msg.rotation.y, msg.rotation.z]

        for i in range(3):
            self.current_trans[i] += current_trans[i]
            self.current_rot[i] += current_rot[i]

            if abs(self.current_trans[i]) >= self.translation_limits[i]:
                current_trans[i] = 0
                self.current_trans[i] = (self.translation_limits[i] if self.current_trans[i] > 0 else -self.translation_limits[i])

            if abs(self.current_rot[i]) >= self.rotation_limits[i]:
                current_rot[i] = 0
                self.current_rot[i] = (self.rotation_limits[i] if self.current_rot[i] > 0 else -self.rotation_limits[i])

        translation = current_trans
        rotation = current_rot

        try:
            self.step_controller.transform_pose_euler(
                translation, "xyz", rotation, msg.duration, degrees=False)   
        except Exception as e:
            self.get_logger().error(str(e))

    def set_pose_euler_callback(self, msg: Pose):
        """设置欧拉角姿态"""
        self.step_controller.transform_absolutely(
            (msg.position.x, msg.position.y, msg.position.z), 
            (msg.orientation.roll, msg.orientation.pitch, msg.orientation.yaw), 
            0.4
        )

    def set_leg_absolute_callback(self, msg: LegPosition):
        """设置腿部绝对位置"""
        self.step_controller.set_leg_position(
            msg.leg_id,
            (msg.position.x, msg.position.y, msg.position.z),
            msg.duration
        )

    def set_leg_relatively_callback(self, msg: LegPosition):
        """设置腿部相对位置"""
        leg_id = msg.leg_id
        duration = msg.duration
        leg_pos = msg.position.x, msg.position.y, msg.position.z
        cur_pos = list(self.step_controller.pose[leg_id - 1])
        new_pos = cur_pos[0] + leg_pos[0], cur_pos[1] + leg_pos[1], cur_pos[2] + leg_pos[2]
        self.step_controller.set_leg_position(leg_id, new_pos, duration)

    def set_traveling_callback(self, msg: Traveling):
        """运动控制回调"""

        try:
            if msg.gait > 0:
                self.last_time = msg.time
                self.step_controller.set_step_mode(
                    msg.gait,
                    msg.stride,
                    msg.height,
                    msg.direction,
                    msg.rotation,
                    msg.time,
                    msg.steps,
                    interrupt=msg.interrupt,
                    relative_height=msg.relative_height)

            else:
                if msg.gait == 0:     
                          
                    self.step_controller.stop_running(
                        timeout= None,
                        callback=lambda: self.step_controller.set_pose(None, None, msg.time)

                    )
                if msg.gait == -1 :
                    self.step_controller.set_build_in_pose('SLAM_POSE', msg.time)
                elif msg.gait == -2:
                    self.step_controller.set_build_in_pose('DEFAULT_POSE', msg.time)
                    self.current_trans = [0.0, 0.0, 0.0]  # x,y,z位移累计
                    self.current_rot = [0.0, 0.0, 0.0]     # x,y,z旋转累计

        except Exception as e:
            self.get_logger().error('error1'+str(e))

    def cmd_vel_callback(self, msg: Twist):
        # self.get_logger().info(str(msg))
        """速度控制回调"""
        msg.linear.x = max(min(msg.linear.x, 0.12), -0.12)
        msg.linear.y = max(min(msg.linear.y, 0.10), -0.10)
        msg.angular.z = max(min(msg.angular.z, 0.6), -0.6)

        self.step_controller.cmd_vel(msg)


    # 服务回调 
    def set_pose1_callback(self, request: SetPose1.Request, response: SetPose1.Response):
        """内置姿态服务回调"""
        try:
            self.step_controller.set_build_in_pose(request.pose, request.duration)
            
            response.result = 0
        except Exception as e:
            self.get_logger().error('error2'+str(e))
            response.result = -1
            response.msg = str(e)
        return response

    def run_actionset_callback(self, msg: RunActionSet):
        file_path = msg.action_path 
        
        msg = RunActionSet()
        msg.action_path = file_path
        msg.interrupt = True
        self.get_logger().info('\033[1;32m%s\033[0m' % str(file_path))

        if file_path == 'stop' :
            self.agc.stop_action_group()
            self.perfrom_actions_pub.publish(msg)
            time.sleep(0.5)
            self.agc.run_action('init_pose')
            self.get_logger().info('\033[1;32m%s\033[0m' % 'init_pose')
        elif file_path == 'dance_1' or file_path == 'dance_2' or file_path == 'dance_3':
            if self.action_complete == True:
                self.perfrom_actions_pub.publish(msg)
        else:
            if self.action_complete == True:
                self.agc.start_action_thread(file_path)

    # 定时发布任务 
    def odometry_publish(self):
        """发布里程计信息"""
        cur_time = self.get_clock().now().to_msg()
        cur_quat = R.from_euler('xyz', [
            -self.step_controller.transform[1][0],
            -self.step_controller.transform[1][1],
            self.step_controller.pose_yaw
        ]).as_quat()
        
        # 发布TF转换
        odom_trans = TransformStamped()
        odom_trans.header.stamp = cur_time
        odom_trans.header.frame_id = f"{self.tf_prefix}odom"
        odom_trans.child_frame_id = f"{self.tf_prefix}base_link"
        odom_trans.transform.translation = Vector3(
            x=self.step_controller.position[0],
            y=self.step_controller.position[1],
            z=self.step_controller.position[2]
        )
        odom_trans.transform.rotation = Quaternion(
            x=cur_quat[0], y=cur_quat[1], z=cur_quat[2], w=cur_quat[3]
        )
        self.odom_trans_pub.publish(odom_trans)

def main(args=None):
    
    node = MoveController('controller')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(node.step_controller) 
    executor.spin()
    node.destroy_node()
    node.step_controller.destroy_node()

if __name__ == '__main__':
    main()
