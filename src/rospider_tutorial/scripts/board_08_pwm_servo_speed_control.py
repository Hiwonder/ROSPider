#!/usr/bin/env python3

import sys
import time
from rospider_sdk import pwm_servo

if __name__ == "__main__":
    pwm_servo.pwm_servo1.start()  # 启动 1 号 pwm 控制线程
    pwm_servo.pwm_servo2.start()  # 启动 2 号 pwm 控制线程

    pwm_servo.pwm_servo1.set_position(1500, 1000)  # 1 号 pwm 舵机用 1000 毫秒 转动到 1500 数值位置, 舵机数值 500~2500 对应 0 ~ 180 度
    time.sleep(1)

    current_pos = 1500
    current_angle = 0 # 设当前角度为 0 度
    new_angle = 90 # 新角度是 90 度
    new_pos = current_pos + (new_angle - current_angle) * ((2500 - 1500) / 180) # 计算新角度对应的舵机角度数值 

    pwm_servo.pwm_servo1.set_position(int(new_pos), 2000) # 用2000毫秒转过90度， 就是 角速度为 45度/秒
    time.sleep(2)


    pwm_servo.pwm_servo1.set_position(1500, 1000) # 将 1 号舵机用 1000 毫秒转到 1500 的位置, 就是角速度 为 90度/秒
    # 可以将角度换算关系封装为函数直接控制舵机一定角速度转动
    time.sleep(2)

        # jetson nano 的pwm分辨率过低所以动作会不连续
