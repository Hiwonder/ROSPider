#!/usr/bin/env python3

import sys
import time
from rospider_sdk import serial_servo


if __name__ == "__main__":

    serial_servo.set_position(1, 500, 1000) # 将 1 号舵机用 1000 毫秒转到 500 的位置, 舵机 0-1000 对应 0-240 度
    time.sleep(2)

    current_pos = 500
    current_angle = 0 # 设当前角度为 0 度
    new_angle = 30 # 新角度是 30 度
    new_pos = current_pos + (new_angle - current_angle) * (1000 / 240) # 计算新角度对应的舵机角度数值

    serial_servo.set_position(1, int(new_pos), 2000) # 用 2000 毫秒转过 30 度， 就是角速度为 15度/秒
    time.sleep(2)

    serial_servo.set_position(1, 500, 1000) # 将 1 号舵机用 1000 毫秒转到 500 的位置, 就是角速度 为 30度/秒
    time.sleep(2)
    # 可以将角度换算关系封装为函数直接控制舵机一定角速度转动
