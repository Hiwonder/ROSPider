#!/usr/bin/env python3

import time
import math
import threading
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from  rospider_app import Heart
from rospider_sdk import pid
from rospider_controller import client
from vision_utils import fps, get_area_max_contour
import rospider_interfaces.srv as jsp_srv
import std_srvs.srv


class LineFollower:
    """
    巡线器，识别并计算线的倾斜、位置等返回控制参数，控制机器人运动
    """

    def __init__(self, color, color_name, node):
        self.node = node
        self.__color = color
        self.__color_name = color_name
        # 将画面分为三段， 在三段中寻找线的颜色
        self.rois = ((330, 360, 0, 640, 0.7), (260, 290, 0, 640, 0.3), (180, 220, 0, 640, 0.1))
        self.weight_sum = 1.0

    def __call__(self, image):
        centroid_sum = 0
        h, w = image.shape[:2]
        # 遍历所有关注区
        for roi in self.rois:
            blob = image[roi[0]:roi[1], roi[2]:roi[3]] # 从大画面中截取关注的部分
            img_lab = cv2.cvtColor(blob, cv2.COLOR_RGB2LAB) # 转换到lab空间
            img_blur = cv2.GaussianBlur(img_lab, (3, 3), 3) # 高斯模糊
            mask = cv2.inRange(img_blur, tuple(self.__color['min']), tuple(self.__color['max'])) #根据目标颜色阈值范围二值化

            # 开闭操作，平滑边缘，去除过小的色块，合并靠近的相邻色块
            eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))) 
            dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
            # cv2.imshow('section:{}:{}'.format(roi[0], roi[1]), cv2.cvtColor(dilated, cv2.COLOR_GRAY2BGR))

            # 找出轮廓
            contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)[-2]
            max_contour_area = get_area_max_contour(contours, 30) # 找出最大的轮廓

            if max_contour_area is not None:
                rect = cv2.minAreaRect(max_contour_area[0]) # 获取最大轮廓的最小外接矩形
                box = np.int0(cv2.boxPoints(rect))
                for j in range(4):
                    box[j, 1] = box[j, 1] + roi[0]
                cv2.drawContours(image, [box], -1, (0, 255, 255), 2)  # 画出四个点组成的矩形

                # 获取矩形对角点
                pt1_x, pt1_y = box[0, 0], box[0, 1]
                pt3_x, pt3_y = box[2, 0], box[2, 1]
                # 线的中心点
                line_center_x, line_center_y = (pt1_x + pt3_x) / 2, (pt1_y + pt3_y) / 2
                cv2.circle(image, (int(line_center_x), int(line_center_y)), 5, (0, 0, 255), -1)
                centroid_sum += line_center_x * roi[-1]

        # 没找到线
        if centroid_sum == 0:
            return image, None
        
        # 计算倾斜角
        center_pos = centroid_sum / self.weight_sum
        deflection_angle = -math.atan((center_pos - (w / 2.0)) / (h / 2.0))
        return image, deflection_angle


class LineFollowingNode:

    def __init__(self, name, log_level=rospy.INFO):
        rospy.init_node(name, log_level=log_level) #初始化ros接点
        self.node_name = name

        self.fps = fps.FPS() # 帧数统计器
        self.pid = pid.PID(1.1, 0.0, 0.2) # pid 控制器
        self.robot = client.Client(self) # 控制机器人运动的ros 服务接口

        self.timestamp = time.time()
        self.lock = threading.RLock()
        self.is_running = False # 是否运动的标记
        self.follower = None # 巡线器
        self.target_color_range = None # 目标颜色阈值范围
        self.target_color_name = ""  # 目标颜色名称

        # 相机图像订阅及结果图像发布
        self.image_sub = None
        self.camera_rgb_prefix = rospy.get_param('/camera_rgb_prefix', 'camera/rgb')
        self.result_publisher = rospy.Publisher(self.node_name + '/image_result', Image, queue_size=2)

        # 进入退出心跳服务
        self.exit_srv = rospy.Service(self.node_name + "/exit", std_srvs.srv.Trigger, self.exit_srv_callback)
        self.enter_srv = rospy.Service(self.node_name + "/enter", std_srvs.srv.Trigger, self.enter_srv_callback)
        self.heart = Heart(self.node_name + '/heartbeat', 5, lambda e: self.exit_srv_callback(e))

        # 设置目标颜色及启停运动
        self.set_target_color_srv = rospy.Service(self.node_name + '/set_target_color', jsp_srv.SetTargetColor, self.set_target_color_srv_callback)
        self.set_running_srv = rospy.Service(self.node_name + '/set_running', std_srvs.srv.SetBool, self.set_running_srv_callback)

    def reset_values(self):
        self.is_running = False
        self.follower = None
        self.target_color_range = None
        self.target_color_name = ""

    def enter_srv_callback(self, _):
        """
        APP 进入功能
        """
        rospy.loginfo(self.node_name + " enter")
        rsp = std_srvs.srv.TriggerResponse(success=True)
        with self.lock:
            self.robot.traveling(-2) # 停止机器人的运动并恢复初始姿态
            self.robot.set_head_absolute(-0.5, 0, 0.2) # 将机器人的相机云台设置巡线要求的角度
            self.reset_values() # 复位相关变量
            try:
                self.image_sub.unregister() # 总是尝试取消对相机画面的订阅以保证不会重复订阅而导致些莫名其妙的问题
            except Exception as e:
                rospy.logerr(str(e))
            # 订阅相机画面
            self.camera_rgb_prefix = rospy.get_param('/camera_rgb_prefix', self.camera_rgb_prefix)
            self.image_sub = rospy.Subscriber(self.camera_rgb_prefix + '/image_raw', Image, self.image_callback, queue_size=2)
            return rsp

    def exit_srv_callback(self, _):
        """
        APP 退出功能
        """
        rospy.loginfo(self.node_name + " exit")
        rsp = std_srvs.srv.TriggerResponse(success=False)
        with self.lock:
            self.reset_values() # 服务相关变量
            try:
                self.image_sub.unregister() # 注销对相机画面的订阅
                rsp.success = True
            except Exception as e:
                rospy.logerr(str(e))
                rsp.message = str(e)
            return rsp

    def set_target_color_srv_callback(self, req: jsp_srv.SetTargetColorRequest):
        """
        设置目标颜色
        :param req: req.color_name 新的目标颜色的名称
        """
        color_name = req.color_name
        rsp = jsp_srv.SetTargetColorResponse(success=True)
        try:
            # 从参数服务器获取颜色阈值列表
            color_ranges = rospy.get_param('/lab_config_manager/color_range_list', {})
            with self.lock:
                self.target_color_name = color_name
                self.target_color_range = color_ranges[color_name]
                print(self.target_color_name, self.target_color_range)
                if self.is_running:  # 如果开启了识别
                    self.pid.clear()  # 服务 pid 控制器
                    # 用新的目标颜色建立新的巡线器
                    self.follower = LineFollower(color=self.target_color_range, 
                                                 color_name=self.target_color_name, node=self)
        except Exception as e:
            rsp.message = str(e)
        return rsp

    def set_running_srv_callback(self, req: std_srvs.srv.SetBoolRequest):
        """
        设置机器人是否运动
        :param req: req.data 为True则机器人开始识别
        """
        rospy.loginfo(self.node_name + " set running as " + str(req.data))
        rsp = std_srvs.srv.SetBoolResponse(success=True)
        try:
            with self.lock:
                self.robot.traveling(-2) # 让机器人先停下了
                self.robot.set_head_absolute(-0.5, 0, 0.2) # 控制机器人头部云台转到巡线要求的角度
                self.is_running = req.data
                if self.is_running: 
                    if self.target_color_range is not None: # 设置了开启识别，且目标颜色已经设置则建立一个巡线器
                        self.pid.clear() # 复位pid控制器
                        self.follower = LineFollower(color=self.target_color_range,
                                                   color_name=self.target_color_name, node=self)
                else:
                    self.reset_values() # 关闭识别, 复位相关变量
        except Exception as e:
            rsp.message = str(e)
        return rsp

    def image_callback(self, ros_image: Image):
        #rospy.loginfo('Received an image! ')
        # 将接收到的ros格式图像转为opencv格式
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data)
        result_image = np.copy(rgb_image) # 将原图拷贝一份做为结果输出的背景
        try:
            with self.lock:
                follower = self.follower # 获取巡线器

            if follower is not None: # 若巡线器可用
                result_image, deflection_angle = follower(rgb_image)  # 使用巡线器处理图像获得结果
                if deflection_angle is not None: # 巡线器找到了线返回了有效的结果，线的倾斜程度
                    self.pid.update(deflection_angle) # 更新 pid 控制器
                    pid_out = self.pid.output # 获取 pid 输出

                    rospy.logdebug("follower, deflection_angle:{:.4f}".format(deflection_angle))
                    rospy.logdebug("pid out: {:.4f}".format(pid_out))

                    pid_out = 1 if pid_out > 1 else -1 if pid_out < -1 else pid_out #pid 输出限幅

                    # 控制两次发送指令的间隔，保证能让机器人能完整的走完一步,
                    # 这个时实测出来的, 一般小于每步的用时且大于1/2每步用时
                    if time.time() - self.timestamp > 0.4:
                        # 发送控制指令到对应topic
                        self.robot.traveling(gait=1, stride=40.0, height=15.0, direction=0.0, rotation=-pid_out,
                                                 time=0.5, steps=1, interrupt=True, relative_height=False)
                        self.timestamp = time.time() #更新最后一次发送指令的时间
        except Exception as e:
            rospy.logerr(str(e))

        self.fps.update() # 刷新帧数统计器
        result_image = self.fps.show_fps(result_image) # 在结果画面中显示帧数
        ros_image.data = result_image.tostring() # 将结果画面转为字节串
        self.result_publisher.publish(ros_image) # 发布结果画面


def main():
    try:
        line_node = LineFollowingNode('line_following', log_level=rospy.INFO)
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))


if __name__ == "__main__":
    main()
