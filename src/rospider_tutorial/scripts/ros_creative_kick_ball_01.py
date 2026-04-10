#!/usr/bin/env python3
# coding: utf8

import argparse
import sys
import rospy
import math
import cv2
import numpy as np
from sensor_msgs.msg import Image
from rospider_sdk import misc, pid, pwm_servo
from vision_utils import get_area_max_contour, colors, fps


class ColorKickNode:
    def __init__(self, target_color, log_level=rospy.INFO):
        rospy.init_node('color_tracing_node', log_level=log_level)

        # 从参数服务器获取各颜色的阈值范围
        color_ranges = rospy.get_param('/lab_config_manager/color_range_list', {})
        print(color_ranges)

        assert(target_color in color_ranges) # 确认有这种颜色记录
        self.target_color_name = target_color # 目标颜色名称
        self.target_color_range = color_ranges[target_color] # 目标颜色阈值范围
        rospy.loginfo("{}, {}".format(self.target_color_name, self.target_color_range))

        self.fps = fps.FPS()  # 帧率统计器

        self.pid_pitch = pid.PID(0.6, 0.01, 0.01) # 控制俯仰角的 pid 控制器
        self.pid_yaw = pid.PID(0.6, 0.01, 0.01) # 控制偏航角的 pid 控制器
        self.pitch = 1500 # 俯仰角舵机数值 500~2500 对应 0~180deg
        self.yaw = 1500 # 偏航角舵机数值
        self.yaw_inc = 10

        pwm_servo.pwm_servo1.start()
        pwm_servo.pwm_servo2.start()
        pwm_servo.pwm_servo1.set_position(1500, 1000)
        pwm_servo.pwm_servo2.set_position(1200, 1000)

        # 图像的topic
        self.camera_rgb_prefix = rospy.get_param('/camera_rgb_prefix', 'camera/rgb')
        self.image_sub = rospy.Subscriber(self.camera_rgb_prefix + '/image_raw', Image, self.image_callback, queue_size=1)
        rospy.loginfo("object tracking node created")

    def image_callback(self, ros_image: Image):
        # rospy.logdebug('Received an image! ')
        # 将ros格式图像转换为opencv格式
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data) # 原始 RGB 画面
        rgb_image = cv2.resize(rgb_image, (320, 180), cv2.INTER_NEAREST) # 缩放图片
        result_image = np.copy(rgb_image) # 拷贝一份用作结果显示，以防处理过程中修改了图像
        
        h, w = rgb_image.shape[:2]
        try:
            img_blur = cv2.GaussianBlur(rgb_image, (3, 3), 3) # 高斯模糊
            img_lab = cv2.cvtColor(img_blur, cv2.COLOR_RGB2LAB) # 转换到 LAB 空间
            mask = cv2.inRange(img_lab, tuple(self.target_color_range['min']), tuple(self.target_color_range['max'])) # 二值化

            # 平滑边缘，去除小块，合并靠近的块
            eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
            dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))

            # 找出最大轮廓
            contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
            max_contour_area = get_area_max_contour(contours, 50)

            # 如果有符合要求的轮廓
            if max_contour_area is not None:
                (center_x, center_y), radius = cv2.minEnclosingCircle(max_contour_area[0]) # 最小外接圆

                # 圈出识别的的要追踪的色块
                circle_color = colors.rgb[self.target_color_name] if self.target_color_name in colors.rgb else (0x55, 0x55, 0x55)
                cv2.circle(result_image, (int(center_x), int(center_y)), int(radius), circle_color, 2)

                # 画面 y 轴, 控制俯仰
                if abs(center_y - (h / 2)) > 30: # 相差范围小于一定值就不用再动了
                    self.pid_pitch.SetPoint = h / 2 # 我们的目标是要让色块在画面的中心, 就是整个画面的像素宽度的 1/2 位置
                    self.pid_pitch.update(center_y) # 更新 pid 控制器
                    self.pitch += self.pid_pitch.output # 获得 pid 输出
                else:
                    self.pid_pitch.clear() # 如果已经到达中心了就复位一下 pid 控制器

                # 画面 x 轴, 控制偏航
                if abs(center_x - (w / 2)) > 20:
                    self.pid_yaw.SetPoint = w / 2
                    self.pid_yaw.update(center_x)
                    self.yaw += self.pid_yaw.output
                else:
                    self.pid_pitch.clear()
                rospy.loginfo("pitch:{:.2f}\tyaw:{:.2f}".format(self.pitch , self.yaw))

                # 限制幅度，两个舵机的运动范围有物理限制，这里做限制保护它们
                self.yaw = misc.set_range(self.yaw, 500, 2500)
                self.pitch = misc.set_range(self.pitch, 1150, 2500)

                # 设置舵机角度， 画面大概为30fps， 将舵机运动时间设为相近
                pwm_servo.pwm_servo1.set_position(self.yaw, 33)
                pwm_servo.pwm_servo2.set_position(self.pitch, 33)
            else:
                self.pid_yaw.clear()
                self.pid_pitch.clear()
                self.pitch = 1150
                self.yaw += self.yaw_inc
                if self.yaw >= 2500 or self.yaw <= 500:
                    self.yaw_inc = -self.yaw_inc
                self.yaw = max(min(self.yaw, 2500), 500)
                pwm_servo.pwm_servo1.set_position(self.yaw, 33)
                pwm_servo.pwm_servo2.set_position(self.pitch, 33)



        except Exception as e:
            rospy.logerr(str(e))

        self.fps.update() # 刷新 fps 统计器
        self.fps.show_fps(result_image) # 画面上显示 fps
        result_image = cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR)
        cv2.imshow('image', result_image)
        cv2.waitKey(1)


if __name__ == '__main__':
    argv = rospy.myargv(argv=sys.argv)
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('target_color',  metavar="COLOR NAME", nargs='?', type=str, help="颜色名称", default='red') # 要追踪的颜色名称
    argv = parser.parse_args(argv[1:]) # 解析输入参数

    target_color = argv.target_color

    try:
        color_detect_node = ColorKickNode(target_color=target_color, log_level=rospy.INFO)
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))

