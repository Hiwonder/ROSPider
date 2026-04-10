#!/usr/bin/env python3

import time
import cv2
import board
import numpy as np
import neopixel_spi as neopixel


if __name__ == "__main__":
    spi = board.SPI()
    pixels_num = 5
    pixels = neopixel.NeoPixel_SPI(spi, pixels_num, pixel_order=neopixel.GRB, auto_write=False)
    pixel_index = pixels_num - 1
    color_index = 0 # hsv 的色调值， 0-180
    color_inc = 5

    try:
        while True:
            color_index += 5 # 改变色调
            if color_index >= 180: # 色调之触顶
                color_index = color_index % 180 # 新的颜色值, 取模
            # 逐个计算灯的色调值，并设置灯泡颜色缓存，一圈布满整个0-180范围
            for i in range(pixels_num):
                # 每个灯泡的色调间隔就是 180 / 灯泡数, 灯泡间会因为散射混色而颜色连续
                c_index = (color_index + int(180 / pixels_num) * i) % 180
                # 建立一个用 颜色色调值， 饱和度和亮度都为255 的 hsv 色彩空间的 opencv 图片
                # 用 opencv 将 hsv 转换为 rgb
                r, g, b = cv2.cvtColor(np.array([[[c_index, 255, 255], ],], dtype=np.uint8), cv2.COLOR_HSV2RGB).reshape(3)
                pixels[i] = int(r << 16 | g << 8 | b) # 将 rgb 值组合成 24bit 的颜色值
            pixels.show()
            time.sleep(0.03) # 0.03 秒, 每秒刷新 30 次
    except Exception as e:
        print(e)
