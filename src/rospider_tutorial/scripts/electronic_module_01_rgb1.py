#!/usr/bin/env python3

import argparse
import sys
import time
import cv2
import numpy as np
import board
import neopixel_spi as neopixel
import rospy


if __name__ == "__main__":
    argv = rospy.myargv(argv=sys.argv)
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("color",  metavar="HEX_RGB_COLOR", nargs="?", type=str, help="十六进制RGB颜色", default="0xFFFF00") #添加颜色
    argv = parser.parse_args(argv[1:]) # 解析输入参数

    try:
        color = int(argv.color, 16) #将十六进制字符串转为数字
    except Exception as e:
        print(e)
        sys.exit(-1)

    spi = board.SPI()
    pixels_num = 5
    pixels = neopixel.NeoPixel_SPI(spi, pixels_num, pixel_order=neopixel.GRB, auto_write=False)
    value_inc = -7 # 亮度改变的增量值

    #  将rgb颜色值分离成独立的分量
    r, g, b = (color >> 16) & 0x0000FF, (color >> 8) & 0x0000FF, color & 0x0000FF # opencv rgb 范围 r: 0-255, g:0-255, b:0-255

    # 我们需要呼吸灯，就是色调不变，只改变亮度, 可以用 hsv 色彩空间实现
    # 用 rgb 值建立一个1个像素的 opencv 图像, 用 opencv 将图像从 rgb 转为 hsv , 实现色彩空间转换
    # opencv hsv 范围 h: 0-180， s: 0-255， v: 0-255
    h, s, v = cv2.cvtColor(np.array([[[r, g, b]]], dtype=np.uint8), cv2.COLOR_RGB2HSV).reshape(3)

    try:
        while True:
            v += value_inc # 改变亮度
            value_inc = -value_inc if v > 180 or v < 0 else value_inc # 当 v 值触顶或者触底之后翻转增量值
            v = max(min(v, 180), 0) # 将 v 的范围限制在 0-180 之间

            # 将 hsv 转换为 rgb
            r, g, b = cv2.cvtColor(np.array([[[h, s, v]]], dtype=np.uint8), cv2.COLOR_HSV2RGB).reshape(3)
            data = [int(r << 16 | g << 8 | b)] * pixels_num # 将 rgb 分量组合成 24bit 颜色值
            # 这里不能直接 pixels = data, 因为 pixels 不是简单的 list, 而是一个 rgb 灯带对象
            for i in range(pixels_num):
                pixels[i] = data[i] # 设置各个 rgb 灯的缓存
            pixels.show() # 将缓存写入灯泡
            time.sleep(0.03) # 0.03 秒，每秒刷新 30 次
    except Exception as e:
        print(e)
