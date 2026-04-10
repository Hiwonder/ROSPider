#!/usr/bin/env python3
# encoding: utf-8
import os
import math
from re import L
import time
import rospy
import threading
import numpy as np
from rospider_app import Heart
import rospider_sdk.misc as misc
import rospider_sdk.pid as pid
import geometry_msgs.msg as geo_msg
import sensor_msgs.msg as sensor_msg
import sensor_msgs.point_cloud2 as pc2
from rospider_controller import client
from std_srvs.srv import Empty, Trigger, TriggerRequest, TriggerResponse
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from rospider_controller_interfaces.srv import SetInt64, SetInt64Request, SetInt64Response
from rospider_controller_interfaces.srv import SetFloat64List, SetFloat64ListRequest, SetFloat64ListResponse
from laser_geometry import LaserProjection
import ros_numpy

MAX_SCAN_ANGLE = 360 # 激光的扫描角度,去掉总是被遮挡的部分degree

class LidarController:
    def __init__(self, name):
        rospy.init_node(name, anonymous=True)
        self.name = name
        self.running_mode = 0
        self.threshold = 0.4 # meters
        self.scan_angle = math.radians(80)  # radians
        self.speed = 0.08
        self.last_act = 0
        self.timestamp = 0
        self.pid_yaw = pid.PID(0.8, 0, 0.05)
        self.pid_dist = pid.PID(0.6, 0, 0.05)
        self.lock = threading.RLock()
        self.lidar_sub = None
        self.robot = client.Client(self)
        self.lidar_type = ""
        self.start_scan = self.__empty
        self.stop_scan = self.__empty
        self.laser_projection = LaserProjection()
        self.pc_publish = rospy.Publisher("laserPointCloud2", sensor_msg.PointCloud2, queue_size=1)
        self.pc_publish1 = rospy.Publisher("laserPointCloud21", sensor_msg.PointCloud2, queue_size=1)

        if "LIDAR_TYPE" in os.environ:
            self.lidar_type = os.environ["LIDAR_TYPE"]
            if "YDLIDAR" in self.lidar_type:
                rospy.wait_for_service("/stop_scan")
                rospy.wait_for_service("/start_scan")
                self.stop_scan = rospy.ServiceProxy("/stop_scan", Empty);
                self.start_scan = rospy.ServiceProxy("/start_scan", Empty);
                self.stop_scan()
            elif "RPLIDAR" in self.lidar_type:
                rospy.wait_for_service("/stop_motor")
                rospy.wait_for_service("/start_motor")
                self.stop_scan = rospy.ServiceProxy("/stop_motor", Empty);
                self.start_scan = rospy.ServiceProxy("/start_motor", Empty);
                self.stop_scan()
            else:
                pass

        self.enter_srv = rospy.Service(self.name + "/enter", Trigger, self.enter_srv_callback)
        self.exit_srv = rospy.Service(self.name + "/exit", Trigger, self.exit_srv_callback)
        self.set_running_srv = rospy.Service(self.name + "/set_running", SetInt64, self.set_running_srv_callback)
        self.set_parameters_srv = rospy.Service(self.name + "/set_parameters", SetFloat64List, self.set_parameters_srv_callback)
        self.heart = Heart(self.name + "/heartbeat", 5, lambda _: self.exit_srv_callback(None))
        # self.client_pub.publish(geo_msg.Twist())

    def __empty(self):
        pass

    def reset_value(self):
        self.running_mode = 0
        self.threshold = 0.4
        self.speed = 0.08
        self.last_act = 0
        self.timestamp = 0
        self.scan_angle = math.radians(80)
        self.pid_yaw.clear()
        self.pid_dist.clear()
        try:
            if self.lidar_sub is not None:
                self.lidar_sub.unregister()
        except Exception as e:
            rospy.logerr(str(e))

    # 进入玩法
    def enter_srv_callback(self, _):
        rospy.loginfo("lidar enter")
        self.reset_value()
        self.start_scan()
        self.lidar_sub = rospy.Subscriber('scan', sensor_msg.LaserScan, self.lidar_callback) 
        return TriggerResponse(success=True)
    
    # 退出玩法
    def exit_srv_callback(self, _):
        rospy.loginfo('lidar exit')
        self.stop_scan()
        self.reset_value()
        self.robot.traveling(gait=-2)
        return TriggerResponse(success=True)

    # 设置运行模式
    def set_running_srv_callback(self, req: SetInt64Request):
        rsp =SetInt64Response(success=True)
        new_running_mode = req.data
        rospy.loginfo("set_running " + str(new_running_mode))
        if not 0 <= new_running_mode <= 3:
            rsp.success = False
            rsp.message = "Invalid running mode {}".format(new_running_mode)
        else:
            with self.lock:
                self.running_mode = new_running_mode
                if self.running_mode == 0:
                    self.robot.traveling(gait=-2)
        self.robot.cmd_vel_pub.publish(geo_msg.Twist())
        return rsp

    # 设置运行参数
    def set_parameters_srv_callback(self, req: SetFloat64ListRequest):
        rsp = SetFloat64ListResponse(success=True)
        new_parameters = req.data
        new_threshold, new_scan_angle, new_speed = new_parameters
        rospy.loginfo("n_t:{:2f}, n_a:{:2f}, n_s:{:2f}".format(new_threshold, new_scan_angle, new_speed))
        if not 0.3 <= new_threshold <= 1.5:
            rsp.success = False
            rsp.message = "New threshold ({:.2f}) is out of range (0.3 ~ 1.5)".format(new_threshold)
            return rsp
        """
        if not 0 <= new_scan_angle <= 90:
            rsp.success = False
            rsp.message = "New scan angle ({:.2f}) is out of range (0 ~ 90)"
            return rsp
        """
        if not new_speed > 0:
            rsp.success = False
            rsp.message = "Invalid speed"
            return rsp

        with self.lock:
            self.threshold = new_threshold
            #self.scan_angle = math.radians(new_scan_angle)
            self.speed = new_speed
        
        return rsp
    

    def lidar_callback(self, lidar_data:sensor_msg.LaserScan):
        cloud = self.laser_projection.projectLaser(lidar_data) # 将 Laserscan 转为点云
        points = np.array(list(pc2.read_points(cloud, skip_nans=True)), dtype=np.float32) # 将点云转为 array

        # 使用 RPLIDAR_A 系列雷达时 LaserScan 的坐标系X轴相对机身旋转了 180deg, 将雷达数据旋转至与机身方向一致
        if "RPLIDAR" in self.lidar_type:
            points = points * [-1.0, -1.0, 1.0, 1.0, 1.0] 

        twist = geo_msg.Twist()
        with self.lock:
            # 避障
            if self.running_mode == 1 and self.timestamp <= time.time():
                # 0.3 是机器人的宽度，单位是米。 这里的用途是去除不在机器人正前方的点, 这些点都是不会挡道的
                points = filter(lambda p: abs(p[1]) < 0.3, points)
                # 去除所有到雷达的x轴距离大于避障阈值的点, 这些点暂时还不会挡道
                points = filter(lambda p: p[0] <= self.threshold, points)
                # 去除与x轴夹角大于设置的扫描角度的点，这些点不在设定的扫描区域内。
                points = filter(lambda p: abs(math.atan2(p[1], p[0])) < self.scan_angle / 2, points)
                # 剩下的点就是会挡道的点了
                points = list(points)
                if len(points) > 0: # 有障碍
                    min_x, min_y, min_z, _, _ = min(points, key=lambda p: p[0])
                    if min_y >= 0: #左侧有障碍
                        twist.linear.x = self.speed / 6
                        max_angle = math.radians(90)
                        w = self.speed * 3
                        twist.angular.z = -w
                        self.robot.cmd_vel_pub.publish(twist)
                        self.timestamp = time.time() + (max_angle / w) * 0.5
                    else:  # 右侧有障碍
                        twist.linear.x = self.speed / 6
                        max_angle = math.radians(90)
                        w = self.speed * 3
                        twist.angular.z = w
                        self.robot.cmd_vel_pub.publish(twist)
                        self.timestamp = time.time() + (max_angle / w) * 0.5
                else: # 没有障碍
                    self.last_act = 0
                    twist.linear.x = self.speed
                    self.robot.cmd_vel_pub.publish(twist)

            # 追踪
            elif self.running_mode == 2:
                # 只追踪在雷达中心往前偏移4cm的前半球的点, 限制追踪区域避免误触发
                points = list(filter(lambda p: p[0] > 0.04, points))
                # 计算所有点到雷达的距离
                points = map(lambda p: (p[0], p[1], math.sqrt(p[0] * p[0] + p[1] * p[1])), points) 
                # 过滤掉距离太小的, 这些点可能是腿部遮挡的位置 
                points = filter(lambda p: p[2] > 0.45, points)
                # 找出距离雷达最近的点
                point_x, point_y, dist = min(points, key=lambda p: p[2])
                # 计算距离最近的点的角度
                angle = math.atan2(point_y, point_x)

                if dist < self.threshold and abs(0.45 - dist) > 0.04:
                    self.pid_dist.update((0.45 - dist)/2)
                    twist.linear.x = misc.set_range(self.pid_dist.output, -self.speed * 4, self.speed * 4)
                else:
                    self.pid_dist.clear()

                if dist < self.threshold and abs(math.degrees(angle)) > 5: # 控制左右
                    self.pid_yaw.update(-(angle/2))
                    if twist.linear.x != 0:
                        twist.angular.z = misc.set_range(self.pid_yaw.output, -self.speed * 4, self.speed * 4)
                    
                    else:
                      
                        twist.linear.x = misc.set_range(self.pid_dist.output, -self.speed , -self.speed )
                else:
                    self.pid_yaw.clear()
                 
                self.robot.cmd_vel_pub.publish(twist)

            # 追踪旋转, 报警
            elif self.running_mode == 3:
                # 计算所有点到雷达的距离
                points = map(lambda p: (p[0], p[1], math.sqrt(p[0] * p[0] + p[1] * p[1])), points)
                # 过滤掉距离太小的, 这些点可能是腿部遮挡的位置 
                points = filter(lambda p: p[2] > 0.45, points)
                # 找出距离雷达最近的点
                point_x, point_y, dist = min(points, key=lambda p: p[2])
                # 计算距离最近的点的角度
                angle = math.atan2(point_y,point_x)

                if dist < self.threshold and abs(math.degrees(angle)) <= 5 : # 控制左右
                    self.pid_yaw.update(-(angle/4))
                    z = misc.set_range(self.pid_yaw.output, -self.speed * 2  , self.speed * 2 )
                elif dist < self.threshold and abs(math.degrees(angle)) > 5 : # 控制左右
                    self.pid_yaw.update(-(angle/2))
                    z = misc.set_range(self.pid_yaw.output, -self.speed * 2  , self.speed * 2 )
                else:
                    z = 0
                    self.pid_yaw.clear()
                self.robot.cmd_vel(0, 0, z)


if __name__ == "__main__":
    node = LidarController('lidar_app')
    try:
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))
