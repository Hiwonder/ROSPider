#!/usr/bin/env python3

import time
from rospider_sdk import buzzer


if __name__ == "__main__":
    buzzer.on()     # 蜂鸣器响
    time.sleep(0.1) 
    buzzer.off()    # 蜂鸣器停
    time.sleep(0.5)

    buzzer.on()  # 蜂鸣器响
    time.sleep(0.5)
    buzzer.off() # 蜂鸣器停
    time.sleep(1)

    for i in range(5):
        buzzer.on()     # 蜂鸣器响
        time.sleep(0.1) 
        buzzer.off()    # 蜂鸣器停
        time.sleep(0.1)





