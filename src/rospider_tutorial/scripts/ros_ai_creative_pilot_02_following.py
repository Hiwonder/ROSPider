#!/usr/bin/env python3

import argparse
import sys
import time
import math
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from rospider_sdk import pid
from rospider_controller import client
from vision_utils import fps, get_area_max_contour


class LineFollowingNode:

    def __init__(self, target_color):
        rospy.init_node("pilot_02_fl") #初始化ros接点

        # 通过颜色名称从参数服务器获取颜色阈值范围
        color_ranges = rospy.get_param('/lab_config_manager/color_range_list', {}) 
        assert(target_color in color_ranges)  # 确保指定的颜色名称有相应的阈值
        self.target_color_range = color_ranges[target_color]
        self.target_color_name = target_color
        rospy.loginfo("linefollowing color_name: {}, range: {}".format(self.target_color_name, self.target_color_range))

        # 将画面分为三段， 在三段中寻找线的颜色
        self.rois = ((330, 360, 0, 640, 0.7), (260, 290, 0, 640, 0.3), (180, 220, 0, 640, 0.1))
        self.weight_sum = 1.0

        self.fps = fps.FPS() # 帧数统计器
        self.pid = pid.PID(1.1, 0.0, 0.2) # pid 控制器
        self.rospider = client.Client(self) # 控制机器人运动的ros 服务接口

        self.timestamp = time.time()
        rospy.sleep(5)
        self.rospider.traveling(-2)
        self.rospider.set_head_absolute(-0.5, 0, 0.2) # 将机器人的相机云台设置巡线要求的角度

        # 订阅相机画面
        self.camera_rgb_prefix = rospy.get_param('/camera_rgb_prefix', 'camera/rgb')
        self.image_sub = rospy.Subscriber(self.camera_rgb_prefix + '/image_raw', Image, self.image_callback, queue_size=1)


    def image_callback(self, ros_image: Image):
        #rospy.loginfo('Received an image! ')
        # 将接收到的 ros 格式图像转为 opencv 格式
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data)
        result_image = np.copy(rgb_image) # 将原图拷贝一份做为结果输出的背景

        centroid_sum = 0
        h, w = rgb_image.shape[:2]
        try:
            # 遍历所有关注区
            for roi in self.rois:
                blob = rgb_image[roi[0]:roi[1], roi[2]:roi[3]] # 从大画面中截取关注的部分
                img_lab = cv2.cvtColor(blob, cv2.COLOR_RGB2LAB) # 转换到lab空间
                img_blur = cv2.GaussianBlur(img_lab, (3, 3), 3) # 高斯模糊
                mask = cv2.inRange(img_blur, tuple(self.target_color_range['min']), tuple(self.target_color_range['max'])) #根据目标颜色阈值范围二值化

                # 开闭操作，平滑边缘，去除过小的色块，合并靠近的相邻色块
                eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))) 
                dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))

                # 找出轮廓
                contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)[-2]
                max_contour_area = get_area_max_contour(contours, 30) # 找出最大的轮廓

                if max_contour_area is not None:
                    rect = cv2.minAreaRect(max_contour_area[0]) # 获取最大轮廓的最小外接矩形
                    box = np.int0(cv2.boxPoints(rect))
                    for j in range(4):
                        box[j, 1] = box[j, 1] + roi[0]
                    cv2.drawContours(result_image, [box], -1, (0, 255, 255), 2)  # 画出四个点组成的矩形

                    # 获取矩形对角点
                    pt1_x, pt1_y = box[0, 0], box[0, 1]
                    pt3_x, pt3_y = box[2, 0], box[2, 1]
                    # 线的中心点
                    line_center_x, line_center_y = (pt1_x + pt3_x) / 2, (pt1_y + pt3_y) / 2
                    cv2.circle(result_image, (int(line_center_x), int(line_center_y)), 5, (0, 0, 255), -1)
                    centroid_sum += line_center_x * roi[-1]

            # 计算倾斜角
            if centroid_sum > 0:
                center_pos = centroid_sum / self.weight_sum
                deflection_angle = -math.atan((center_pos - (w / 2.0)) / (h / 2.0))

                self.pid.update(deflection_angle) # 更新 pid 控制器
                pid_out = self.pid.output # 获取 pid 输出

                rospy.logdebug("follower, deflection_angle:{:.4f}".format(deflection_angle))
                rospy.logdebug("pid out: {:.4f}".format(pid_out))

                pid_out = 1 if pid_out > 1 else -1 if pid_out < -1 else pid_out #pid 输出限幅

                # 控制两次发送指令的间隔，保证能让机器人能完整的走完一步,
                # 这个时实测出来的, 一般小于每步的用时且大于1/2每步用时
                if time.time() - self.timestamp > 0.4:
                    # 发送控制指令到对应topic
                    self.rospider.traveling(gait=1, stride=40.0, height=15.0, direction=0.0, rotation=-pid_out,
                                           time=0.5, steps=1, interrupt=True, relative_height=False)
                    self.timestamp = time.time() #更新最后一次发送指令的时间

        except Exception as e:
            rospy.logerr(str(e))

        self.fps.update() # 刷新帧数统计器
        self.fps.show_fps(result_image) # 在结果画面中显示帧数
        result_image = cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR)
        cv2.imshow('image', result_image)
        cv2.waitKey(1)


if __name__ == "__main__":
    argv = rospy.myargv(argv=sys.argv) # ros 会在启动 py 文件时传入自己的一些参数，myargv 会去掉这些，返回原始参数
    parser = argparse.ArgumentParser() # 参数解析器
    parser.add_argument('target_color', metavar='color', nargs='?', type=str, help="颜色名称如 red", default="red") # 添加要解析的参数
    argv = parser.parse_args(argv[1:]) # 对输入参数进行解析

    try:
        line_node = LineFollowingNode(argv.target_color)
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))
