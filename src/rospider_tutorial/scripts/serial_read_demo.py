#!/usr/bin/env python3
# encoding: utf-8
import time
import serial
import Jetson.GPIO as GPIO

rx_pin = 17
tx_pin = 27

def port_as_write():
    GPIO.output(tx_pin, 1)  # 拉高TX_CON 即 GPIO27
    GPIO.output(rx_pin, 0)  # 拉低RX_CON 即 GPIO17

def port_as_read():
    GPIO.output(rx_pin, 1)  # 拉高RX_CON 即 GPIO17
    GPIO.output(tx_pin, 0)  # 拉低TX_CON 即 GPIO27

def port_init():
    GPIO.setwarnings(False)
    mode = GPIO.getmode()
    if mode == 1 or mode is None:
        GPIO.setmode(GPIO.BCM)
    GPIO.setup(rx_pin, GPIO.OUT)  # 配置RX_CON 即 GPIO17 为输出
    GPIO.output(rx_pin, 0)
    GPIO.setup(tx_pin, GPIO.OUT)  # 配置TX_CON 即 GPIO27 为输出
    GPIO.output(tx_pin, 1)

port_init()
port_as_read()

data = []
my_serial = serial.Serial('/dev/ttyTHS1', 115200, timeout=0.01)

print('serial read data at 100hz')
data_str = ""

while True:
    STRING= str(my_serial.read())
    if STRING[2] != "'" and STRING[2] != "\\":
        data_str = data_str + STRING[2]
    time.sleep(0.01)
    if len(data_str)==5:
        print(data_str)
        data_str = ""
        time.sleep(0.01)
