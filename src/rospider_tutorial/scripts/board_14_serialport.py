#!/usr/bin/python3
# 测试串口发送接收，运行本程序然后短接 扩展板上的tx，rx能在命令行中看到输出"HELLO WORLD"
# 测试这个程序时不能接舵机，因为发送的数据可能会被舵机错误相应，无法正常处理
import serial
import time

if __name__ == "__main__":
    
    serialHandle = serial.Serial("/dev/ttyTHS1", 9600)
    while True:
        serialHandle.write(b"HELLO WORLD\r\n")
        time.sleep(0.1)
        count = serialHandle.in_waiting  # 获取串口缓冲区的数据长度
        print(count)
        if count != 0:  #如果接收到了数据
            recv = serialHandle.read(serialHandle.in_waiting)
            print(str(recv, encoding="utf8")) # 打印接收到的数据
        time.sleep(1)
    


