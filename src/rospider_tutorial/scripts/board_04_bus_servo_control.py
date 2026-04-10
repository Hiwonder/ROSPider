#!/usr/bin/env python3

import sys
import time
from rospider_sdk import serial_servo


if __name__ == "__main__":

    serial_servo.set_position(1, 500, 1000) # 将 1 号舵机用 1000 毫秒转到 500 的位置, 舵机 0 - 1000 对应 0 - 240 度
    time.sleep(1)

    serial_servo.set_position(1, 300, 2000) # 将 1 号舵机用 2000 毫秒转到 500 的位置, 舵机 0 - 1000 对应 0 - 240 度
    time.sleep(2)

    serial_servo.set_position(1, 700, 2000) # 将 1 号舵机用 2000 毫秒转到 500 的位置, 舵机 0 - 1000 对应 0 - 240 度
    time.sleep(2)

    serial_servo.set_position(1, 500, 1000) # 将 1 号舵机用 1000 毫秒转到 500 的位置, 舵机 0 - 1000 对应 0 - 240 度
    time.sleep(1)

    sys.exit(0)