#!/usr/bin/env python3
# coding: utf8
import rospy
import cv2
import threading
import numpy as np
import std_srvs.srv
from sensor_msgs.msg import Image
from color_tracker import ColorTracker
from apriltag_tracker import AprilTagTracker
from vision_utils import fps
from rospider_controller.client import Client
from rospider_app import Heart
import rospider_interfaces.srv as jetsrv 


class ObjectTrackingNode:
    def __init__(self, name, log_level=rospy.INFO):
        rospy.init_node(name, log_level=log_level)
        self.node_name = name

        self.lock = threading.RLock()
        self.robot = Client(self)
        self.fps = fps.FPS()  # 帧率统计器
        self.running_mode = 0  # 0 -> 未运行, 1 -> 颜色追踪模式, 2 -> apriltag追踪模式

        # running_mode 不为0时 target_color_range  target_apriltag_id 哪个不为None就启动哪个模式
        self.target_color_range = None # 目标颜色
        self.target_color_name = "" # 目标颜色名称
        self.target_apriltag_id = None # 目标id
        self.tracking_helper = None  # 跟踪识别器

        # 获取和发布图像的topic
        self.image_sub = None
        self.camera_rgb_prefix = rospy.get_param('/camera_rgb_prefix', 'camera/rgb')
        self.result_publisher = rospy.Publisher(self.node_name + '/image_result', Image, queue_size=2)

        # 控制运行的各种服务
        self.set_target_color_srv = rospy.Service(self.node_name + '/set_target_color', jetsrv.SetTargetColor, self.set_target_color_srv_callback)
        self.set_target_apriltag_srv = rospy.Service(self.node_name + '/set_target_apriltag', jetsrv.SetTargetAprilTag, self.set_target_apriltag_srv_callback)
        self.set_running_srv = rospy.Service(self.node_name + '/set_running', std_srvs.srv.SetBool, self.set_running_srv_callback)
        self.exit_srv = rospy.Service(self.node_name + "/exit", std_srvs.srv.Trigger, self.exit_srv_callback)
        self.enter_srv = rospy.Service(self.node_name + "/enter", std_srvs.srv.Trigger, self.enter_srv_callback)
        self.heart = Heart(self.node_name + '/heartbeat', 5, lambda e: self.exit_srv_callback(e))

        rospy.loginfo("object tracking node created")

    def reset_values(self):
        self.running_mode = 0  # 设置运行模式为未运行
        self.tracking_helper = None  # 关闭追踪器
        self.target_apriltag_id = None
        self.target_color_range = None
        self.target_color_name = ""

    def enter_srv_callback(self, _):
        rospy.loginfo("enter")
        rsp = std_srvs.srv.TriggerResponse(success=True)
        with self.lock:
            self.reset_values()
            self.robot.set_head_absolute(0, 0, 0.2)
            self.robot.traveling(-2)
            try:
                self.image_sub.unregister()  # 尝试停止订阅摄像头话题, 防止重复订阅
            except Exception as e:
                rospy.logerr(str(e))
        self.camera_rgb_prefix = rospy.get_param('/camera_rgb_prefix', self.camera_rgb_prefix)
        self.image_sub = rospy.Subscriber(self.camera_rgb_prefix + '/image_raw', Image, self.image_callback, queue_size=2)  # 订阅
        return rsp

    def exit_srv_callback(self, _):
        rospy.loginfo("exit")
        rsp = std_srvs.srv.TriggerResponse(success=False)
        with self.lock:
            self.reset_values()
            try:
                self.image_sub.unregister()  # 停止订阅 摄像头话题
                rsp.success = True
            except Exception as e:
                rospy.logerr(str(e))
                rsp.message = str(e)
        return rsp

    def set_target_color_srv_callback(self, req: jetsrv.SetTargetColorRequest):
        color_name = req.color_name
        rsp = jetsrv.SetTargetAprilTagResponse(success=True)
        try:
            color_ranges = rospy.get_param('/lab_config_manager/color_range_list', {})
            with self.lock:
                self.target_apriltag_id = None
                if color_name not in color_ranges:
                    rsp.success = False
                    rsp.message = "{} is not in color ranges".format(color_name)
                    rospy.logerr(rsp.message)
                    return rsp
                self.target_color_range = color_ranges[color_name]
                self.target_color_name = color_name
                rospy.logdebug("set target color range lab:{}".format(self.target_color_range))
                if self.running_mode != 0:
                    self.tracking_helper = ColorTracker(color=self.target_color_range,
                                                        color_name=self.target_color_name,
                                                        node=self)
        except Exception as e:
            rsp.success = False
            rsp.message = str(e)
            rospy.logerr(rsp.message)
        return rsp

    def set_target_apriltag_srv_callback(self, req: jetsrv.SetTargetAprilTagRequest):
        rospy.loginfo("set target apriltag id: " + str(req.tag_id))
        rsp = jetsrv.SetTargetAprilTagResponse(success=True)
        try:
            with self.lock:
                self.target_apriltag_id = req.tag_id
                self.target_color_range = None
                self.target_color_name = ""
                if self.running_mode != 0:
                    self.tracking_helper = AprilTagTracker(tag_id=self.target_apriltag_id, node=self)
        except Exception as e:
            rsp.success = False
            rsp.message = str(e)
        return rsp

    def set_running_srv_callback(self, req: std_srvs.srv.SetBoolRequest):
        rsp = std_srvs.srv.SetBoolResponse(success=True)
        is_running = req.data
        try:
            with self.lock:
                self.robot.set_head_absolute(0, 0, 0.2) # 将头恢复初始位置
                self.robot.traveling(-2) # 将身体恢复初始位置
                if is_running:
                    if self.target_color_range is not None:  # 颜色追踪范围不为 None, 启动追踪颜色
                        self.tracking_helper = ColorTracker(color=self.target_color_range,
                                                                   color_name=self.target_color_name, node=self)
                        self.running_mode = 1
                    elif self.target_apriltag_id is not None:  # AprilTag id 不为 None, 启动追踪 AprilTag
                        self.tracking_helper = AprilTagTracker(tag_id=self.target_apriltag_id, node=self)
                        self.running_mode = 2
                    else: # 如果还未设置有效的追踪目标， 就先标记已经set running 了
                        rospy.logdebug("set running 99")
                        self.running_mode = 99
                else:
                    self.reset_values()

        except Exception as e:
            rsp.message = str(e)
        return rsp

    def image_callback(self, ros_image: Image):
        # rospy.logdebug('Received an image! ')
        # 将ros格式图像转换为opencv格式
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data) # 原始 RGB 画面
        result_image = np.copy(rgb_image) # 拷贝一份用作结果显示，以防处理过程中修改了图像
        try:
            with self.lock:
                tracking_helper = self.tracking_helper # 获取追踪器
            if tracking_helper is not None:
                result_image, pitch, yaw = tracking_helper(rgb_image)  # 处理
                # 如果识别到了追踪目标，返回有效的俯仰偏航就控制机器人动起来
                if pitch is not None and yaw is not None:
                    rospy.logdebug("pitch:{:4f}, yaw:{:4f}".format(pitch, yaw))
                    self.robot.set_head_absolute(pitch, yaw, 0.033)
        except Exception as e:
            rospy.logerr(str(e))
        self.fps.update() # 刷新 fps 统计器
        result_image = self.fps.show_fps(result_image) # 画面上显示 fps
        ros_image.data = result_image.tostring()
        self.result_publisher.publish(ros_image) # 发布新的结果图像
        # cv2.imshow('image', image)
        # cv2.waitKey(1)


def main(args=None):
    object_tracking_node = ObjectTrackingNode('object_tracking', log_level=rospy.INFO)
    try:
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))


if __name__ == "__main__":
    main()
