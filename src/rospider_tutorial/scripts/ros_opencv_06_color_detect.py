#!/usr/bin/env python3
# coding: utf8

import argparse
import sys
import rospy
import cv2
import math
import numpy as np
from sensor_msgs.msg import Image
from vision_utils import fps, get_area_max_contour, colors
from rospider_sdk import serial_servo, buzzer, misc

class ColorDetectNode:
    def __init__(self, target_color, log_level=rospy.INFO):
        rospy.init_node("color_detect", anonymous=True, log_level=log_level)

        #self.spi = board.SPI()
        #self.pixels = neopixel.NeoPixel_SPI(board.SPI(), 5, pixel_order=neopixel.GRB, auto_write=False)

        self.target_color_name = target_color
        self.target_color_range = None # 目标颜色
        self.detected_color = ""

        # 从参数服务器获取颜色阈值列表
        self.color_ranges = rospy.get_param('/lab_config_manager/color_range_list', None)
        assert(self.color_ranges is not None)
        self.target_color_range = self.color_ranges[self.target_color_name]
        rospy.loginfo("{}, {}".format(self.target_color_name, self.target_color_range))

        # 帧率统计器
        self.fps = fps.FPS()  

        # 启动 pwm 控制线程
        serial_servo.set_position(19, 500, 1000)

        # 获取图像的topic
        self.camera_rgb_prefix = rospy.get_param('/camera_rgb_prefix', 'camera/rgb')
        self.image_sub = rospy.Subscriber(self.camera_rgb_prefix + '/image_raw', Image, self.image_callback, queue_size=1)

        self.timer = rospy.Timer(rospy.Duration(2), self.timer_callback)
    
    def timer_callback(self, _):
        print(self.detected_color)
        # 根据是否是指定的颜色而做动作
        if self.detected_color != 'none':
            if self.detected_color != self.target_color_name:
                for i in range(2):
                    serial_servo.set_position(19, 300, 200)
                    rospy.sleep(0.2)
                    serial_servo.set_position(19, 700, 200)
                    rospy.sleep(0.2)
                serial_servo.set_position(19, 500, 200)
            else:
                for i in range(2):
                    buzzer.on()
                    rospy.sleep(0.2)
                    buzzer.off()
                    rospy.sleep(0.2)

        #color = colors.rgb[detected_color] if detected_color in colors.rgb else (0x55, 0x55, 0x55)
        #for i in range(5):
        #    self.pixels[i] = int(color[0] << 16 | color[1] << 8| color[2] )
        #self.pixels.show()

        self.detected_color = 'none'

    def image_callback(self, ros_image: Image):
        # rospy.logdebug('Received an image! ')
        # 将ros格式图像转换为opencv格式
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data) # 原始 RGB 画面
        self.color_ranges = rospy.get_param('/lab_config_manager/color_range_list', self.color_ranges)
        o_height, o_width = rgb_image.shape[:2]
        result_image = np.copy(rgb_image) # 拷贝一份用作结果显示，以防处理过程中修改了图像
        rgb_image = cv2.resize(rgb_image, (int(o_width/2), int(o_height/2))) # 缩放一下减少计算量
        detected_color = ""
        try:
            img_blur = cv2.GaussianBlur(rgb_image, (3, 3), 3) # 高斯模糊
            img_lab = cv2.cvtColor(img_blur, cv2.COLOR_RGB2LAB) # 转换到 LAB 空间
            
            max_contours = []
            for color_name in ('red', 'green', 'blue'): # 遍历指定颜色范围
                color_range = self.color_ranges[color_name]
                # 二值化
                mask = cv2.inRange(img_lab, tuple(color_range['min']), tuple(color_range['max'])) 

                # 平滑边缘，去除小块，合并靠近的块
                eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
                dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))

                # 找出最大轮廓
                contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
                # 返回值是 (面积最大的轮廓，轮廓面积)
                max_contour_area = get_area_max_contour(contours, 500)
                if max_contour_area is not None:
                    max_contours.append((color_name, max_contour_area[0], max_contour_area[1]))
            
            if max_contours:
                max_contour = max(max_contours, key=lambda x: x[2]) # 找出最大的轮廓

                (center_x, center_y), radius = cv2.minEnclosingCircle(max_contour[1]) # 最小外接圆
                # 将识别到的圆的参数恢复到原图的像素坐标下(因为是用缩放后的图片识别的)
                center_x = misc.val_map(center_x, 0, int(o_width / 2), 0, o_width)
                center_y = misc.val_map(center_y, 0, int(o_height / 2), 0, o_height)
                radius = misc.val_map(radius, 0, int(o_height / 2), 0, o_height)

                # 圈出识别的色块
                circle_color = colors.rgb[max_contour[0]] if max_contour[0] in colors.rgb else (0x55, 0x55, 0x55)
                cv2.circle(result_image, (int(center_x), int(center_y)), int(radius), circle_color, 2)
                if radius > o_height / 4:
                    detected_color = max_contour[0]
                else:
                    detected_color = "none"
            else:
                detected_color = "none"

        except Exception as e:
            rospy.logerr(str(e))
        self.detected_color = detected_color

        self.fps.update() # 刷新 fps 统计器
        result_image = self.fps.show_fps(result_image) # 画面上显示 fps
        result_image = cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR)
        cv2.imshow('image', result_image)
        cv2.waitKey(1)


if __name__ == "__main__":
    argv = rospy.myargv(argv=sys.argv)
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('target_color',  metavar="COLOR NAME", nargs='?', type=str, help="颜色名称", default='red') # 要追踪的颜色名称
    argv = parser.parse_args(argv[1:]) # 解析输入参数

    target_color = argv.target_color

    try:
        color_detect_node = ColorDetectNode(target_color=target_color, log_level=rospy.INFO)
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))


