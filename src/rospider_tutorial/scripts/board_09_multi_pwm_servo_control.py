#!/usr/bin/env python3

import time
from rospider_sdk import pwm_servo

if __name__ == "__main__":
    pwm_servo.pwm_servo1.start()  # 启动 1 号 pwm 控制线程
    pwm_servo.pwm_servo2.start()  # 启动 2 号 pwm 控制线程

    pwm_servo.pwm_servo1.set_position(1500, 1000)  # 1 号 pwm 舵机用 1000 毫秒 转动到 1500 数值位置, 舵机数值 500~2500 对应 0 ~ 180 度
    pwm_servo.pwm_servo2.set_position(1500, 1000)  # 2 号 pwm 舵机用 1000 毫秒 转动到 1500 数值位置, 舵机数值 500~2500 对应 0 ~ 180 度
    time.sleep(1)

    pwm_servo.pwm_servo1.set_position(500, 1000)  # 1 号 pwm 舵机用 1000 毫秒 转动到 1500 数值位置, 舵机数值 500~2500 对应 0 ~ 180 度
    pwm_servo.pwm_servo2.set_position(2500, 1000)  # 2 号 pwm 舵机用 1000 毫秒 转动到 2500 数值位置, 舵机数值 500~2500 对应 0 ~ 180 度
    time.sleep(1)

    pwm_servo.pwm_servo1.set_position(2500, 2000)  # 1 号 pwm 舵机用 1000 毫秒 转动到 1500 数值位置, 舵机数值 500~2500 对应 0 ~ 180 度
    pwm_servo.pwm_servo2.set_position(500, 2000)  # 2 号 pwm 舵机用 1000 毫秒 转动到 1500 数值位置, 舵机数值 500~2500 对应 0 ~ 180 度
    time.sleep(2)

    pwm_servo.pwm_servo1.set_position(1500, 1000)  # 1 号 pwm 舵机用 1000 毫秒 转动到 1500 数值位置, 舵机数值 500~2500 对应 0 ~ 180 度
    pwm_servo.pwm_servo2.set_position(1500, 1000)  # 2 号 pwm 舵机用 1000 毫秒 转动到 1500 数值位置, 舵机数值 500~2500 对应 0 ~ 180 度
    time.sleep(1)