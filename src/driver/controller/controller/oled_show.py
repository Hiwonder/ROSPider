#!/usr/bin/env python3
# encoding: utf-8
# OLED 显示
import time
import rclpy
import psutil
from std_srvs.srv import Trigger
from ros_robot_controller_msgs.msg import OLEDState

def get_cpu_serial_number():
    device_serial_number = open("/proc/device-tree/serial-number")
    serial_num = device_serial_number.readlines()[0][-10:-1]

    sn = (serial_num + "00000000000000000000000000")[:32]
    HW_WIFI_AP_SSID = ''.join(["WN-", sn[0:8]])

    return HW_WIFI_AP_SSID

def get_wlan():
    ip = ''
    info = psutil.net_if_addrs()
    for k, v in info.items():
        if 'wl' in k:
            for i in v:
                if i[2] is not None:

                    ip = i[1]
                    break
                else:
                    ip = None

    if ip != '' and ip is not None:
        return ip
    else:
        return '0.0.0.0'


def main():
    rclpy.init()
    node = rclpy.create_node('oled_show')
    oled_pub = node.create_publisher(OLEDState, '/ros_robot_controller/set_oled', 1)
    node.client = node.create_client(Trigger, 'controller_manager/init_finish')
    node.client.wait_for_service()

    msg = OLEDState()
    msg.index = 1
    msg.text = 'SSID:' + get_cpu_serial_number()
    oled_pub.publish(msg)
    time.sleep(0.2)
    msg = OLEDState()
    msg.index = 2
    msg.text = 'IP:' + get_wlan()
    oled_pub.publish(msg)
    

if __name__ == '__main__':
    main()
