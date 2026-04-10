#!/usr/bin/env python3

# 此文件为 rospider 的一部分
# oled_display.py 实现oled屏幕显示系统状态, 传感器参数等

import sys
import os
import signal
import subprocess

import threading
from cv2 import invert
import rospy
import rospkg
import std_msgs.msg
import std_srvs.srv
import numpy as np
import queue
import psutil

from PIL import Image, ImageDraw, ImageFont

import time
import Adafruit_SSD1306
from vision_utils import fps
from wireless_utils import dev_state
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from std_msgs.msg import Float32
import Jetson.GPIO as GPIO
from button_helper import ButtonHelper


class Cutscens:
    def __init__(self, direction, increment, new_gram_name):
        self.direction = direction
        self.increment = increment
        self.new_gram_name = new_gram_name
        self.now_gram = None
        self.pixel_index = 0
    

class OledDisplayNode:
    def __init__(self, name, anonymous=True, log_level=rospy.INFO):
        # 按钮初始化, 按钮切换画面
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(4, GPIO.IN)
        self.btn = ButtonHelper(read_button=lambda : GPIO.input(4), clicked_cb=self.clicked_callback)

        rospy.init_node(name, log_level=log_level)
        self.node_name = name
        self.voltage = 0.0
        self.voltage_sub = rospy.Subscriber('/voltage', Float32, self.voltage_update, queue_size=1)

        # 初始化OLED和相关的资源
        self.screen = Adafruit_SSD1306.SSD1306_128_32(rst=None, i2c_bus=1, gpio=1, i2c_address=0x3C)
        self.screen.begin() # 启动屏幕
        self.fps = fps.FPS() # 屏幕 fps 统计器

        # 实现功能用到的变量资源
        self.lock = threading.RLock()
        self.on_off = True # 是否打开屏幕显示
        self.refresh_enable = True # 是否定时刷新屏幕

        self.last_gram = None # 最后一次输出到到屏幕上的数据
        self.current_gram_name = "clear" # 当前显示的缓存的名称
        self.current_cutscenes = None # 当前正在执行的过场动画
        self.cutscenes = queue.Queue(maxsize=1) # 过场动画队列
        self.font = ImageFont.load_default() # 加载字体

        # 定时反相显示,防止烧屏用
        self.invert = False # 是否反相显示
        self.invert_interval = 30 # 定时反相的实际间隔
        self.invert_timestamp = time.time() # 上次反相的时间戳

        # 建立各个画面的缓存
        self.grams = {"clear": np.zeros((32, 128), dtype=np.uint8), # 纯黑画面
                      "full": np.full((32, 128), 255, dtype=np.uint8)} # 纯白画面
        self.load_logo() # 加载logo
        self.sys_states_update(0) 
        self.wifi_iface = rospy.get_param('~wifi_iface', 'wlan0') # 获取要显示无线网卡的设备名称
        self.wireless_states_update(0)

        # 添加显示logo的过场, 让logo从下到上飞入oled
        self.cutscenes.put(Cutscens('bt', 1, 'logo')) 
        # 添加定时器, 3.5秒后让系统状态信息飞入oled
        rospy.Timer(rospy.Duration(3.5), lambda _: self.cutscenes.put(Cutscens('bt', 4, 'sys')), oneshot=True)

        # 开始扫描按钮状态
        rospy.Timer(rospy.Duration(0.05), lambda _:self.btn.update_button())

        # 控制显示的各种service、topic
        self.set_refresh_enable_srv = rospy.Service("set_refresh_enable", SetBool, self.set_refresh_enable_srv_callback)
        self.set_on_off_srv = rospy.Service("on_off", SetBool, self.set_on_off_srv_callback)
        self.timestamp = time.time()
        rospy.on_shutdown(self.display_off)

        # 让系统状态和wifi状态定时刷新
        rospy.Timer(rospy.Duration(1), self.sys_states_update)
        rospy.Timer(rospy.Duration(2), self.wireless_states_update)
        # 开始定时刷新屏幕
        rospy.Timer(rospy.Duration(0.06666), self.refresh)


    def clicked_callback(self, _):
        if self.timestamp > time.time():
            return
        self.timestamp = time.time() + 0.5
        if self.current_gram_name == 'sys':
            self.cutscenes.put_nowait(Cutscens('bt', 5, 'wireless'))
        if self.current_gram_name == 'wireless':
            self.cutscenes.put_nowait(Cutscens('bt', 5, 'sys'))

    def voltage_update(self, msg: Float32):
        self.voltage = msg.data

    def set_on_off_srv_callback(self, msg:SetBoolRequest):
        """
        设置是否开启显示
        """
        with self.lock:
            self.on_off = msg.data
            if not self.on_off: # 设置为关闭, 即不显示, 显示纯黑色画面
                self.screen.image(Image.fromarray(self.grams['clear']).convert('1'))
                self.screen.display()
            else:
                # 开启显示, 将最后显示的画面重新输出到屏幕
                try:
                    self.screen.image(Image.fromarray(self.last_gram).convert('1'))
                    self.screen.display()
                    self.last_gram = None
                except Exception as e:
                    rospy.logerr(str(e))
        return SetBoolResponse(success=True)
    
    def display_off(self):
        self.screen.image(Image.fromarray(self.grams['clear']).convert('1'))
        self.screen.display()

    def set_refresh_enable_srv_callback(self, msg: SetBoolRequest):
        """
        设置是否刷新屏幕
        有时候会禁用刷新,例如自平衡时刷新屏幕可能会导致读取imu延时使自平衡卡顿
        """
        self.refresh_enable = msg.data
        return SetBoolResponse(success=True)
        
    def load_logo(self):
        """
        加载logo图片并且转换未可以在oled上显示的缓存数据
        """
        logo = rospy.get_param('~logo', os.path.join(rospkg.RosPack().get_path('rospider_peripherals'), 'resources/hiwonder_logo.png'))
        gray_logo = Image.open(logo).resize((120, 14)).convert('L')
        logo_gram = np.array(gray_logo, dtype=np.uint8) // 128 * 255
        bg_gram = np.zeros((32, 128), dtype=np.uint8)
        bg_gram[9:9 + logo_gram.shape[0], 4: 4 + logo_gram.shape[1]] = logo_gram
        self.grams['logo'] = bg_gram
    
    def sys_states_update(self, timer_event):
        """
        更新系统状态信息并生成新的系统状态信息oled显示缓存
        """
        img = Image.new('1', (self.screen.width, self.screen.height))
        draw = ImageDraw.Draw(img)
        draw.rectangle((0, -1, self.screen.width, self.screen.height + 1), outline=0, fill=0)
        mem = psutil.virtual_memory()
        cpu = psutil.cpu_percent()
        disk = psutil.disk_usage('/')
        draw.rectangle((0, 0, self.screen.width - 1, 11), outline=0, fill=255)
        draw.text((27, 0),   "SYSTEM STATUS", font=self.font, fill=0)
        draw.text((4, 11),   "CPU:{}%".format(cpu), font=self.font, fill=255)
        draw.text((66, 11),  "MEM:{:0.1f}%".format(mem.used/mem.total * 100.0), font=self.font, fill=255)
        draw.text((4, 21),  "DISK:{}%".format(int(disk.percent)), font=self.font, fill=255)
        draw.text((66, 21), "BAT:{:.2f}v".format(self.voltage), font=self.font, fill=255)
        self.grams['sys'] = np.array(img, dtype=np.uint8) * 255


    def wireless_states_update(self, timer_event):
        """
        更新WIFI状态信息并生成新的WIFI状态信息oled显示缓存
        """
        img = Image.new('1', (self.screen.width, self.screen.height))
        draw = ImageDraw.Draw(img)
        draw.rectangle((0, -1, self.screen.width, self.screen.height + 1), outline=0, fill=0)
        draw.rectangle((0, 0, self.screen.width - 1, 11), outline=0, fill=255)
        wlan_ip = psutil.net_if_addrs()[self.wifi_iface][0].address
        wlan_state = dev_state('wlan0')
        draw.text((40, 0),   "WIRELESS", font=self.font, fill=0)
        draw.text((4, 11),   wlan_state['mode'] + ' SSID:' + wlan_state['ssid'], font=self.font, fill=255)
        draw.text((4, 21),   "IP:" + wlan_ip, font=self.font, fill=255)
        self.grams['wireless'] = np.array(img, dtype=np.uint8) * 255


    def imu_state_update(self, msg):
        """
        更新IMU状态信息并生成新的IMU状态信息oled显示缓存
        这个是被IMU相关包发布的topic触发的
        """
        img = Image.new('1', (self.screen.width, self.screen.height))
        draw.rectangle((0, -1, self.screen.width, self.screen.height + 1), outline=0, fill=0)
        draw = ImageDraw.Draw(img)
        draw.rectangle((0, 0, self.screen.width - 1, 11), outline=0, fill=255)
        draw.text((50, 0),   "IMU RAW", font=self.font, fill=0)
        draw.text((4, 11), 'AX:{:1.1f} AY:{:1.1f} AZ:{:1.1f}'.format(0.0, 0.0, 0.0), font=self.font, fill=255)
        draw.text((4, 21), 'GX:{:1.1f} GY:{:1.1f} GZ:{:1.1f}'.format(0.0, 0.0, 0.0), font=self.font, fill=255)
        self.grams['imu_raw'] = np.array(img, dtype=np.uint8) * 255

        img = Image.new('1', (self.screen.width, self.screen.height))
        draw = ImageDraw.Draw(img)
        draw.rectangle((0, 0, self.screen.width - 1, 11), outline=0, fill=255)
        draw.text((38, 0),   "IMU EULER", font=self.font, fill=0)
        draw.text((4, 11), 'EX:{:.2f}'.format(0.0), font=self.font, fill=255)
        draw.text((64, 11), 'EX:{:.2f}'.format(0.0), font=self.font, fill=255)
        draw.text((4, 21), 'EZ:{:.2f} '.format(0.0), font=self.font, fill=255)
        self.grams['imu_euler'] = np.array(img, dtype=np.uint8) * 255

        draw.rectangle((0, 0, self.screen.width - 1, 11), outline=0, fill=255)
        draw.text((20, 0),   "IMU QUARTERNION", font=self.font, fill=0)
        draw.text((4, 11), 'EX:{:.2f}'.format(0.0), font=self.font, fill=255)
        draw.text((64, 11), 'EY:{:.2f}'.format(0.0), font=self.font, fill=255)
        draw.text((4, 21), 'EZ:{:.2f} '.format(0.0), font=self.font, fill=255)
        draw.text((64, 21), 'EW:{:.2f} '.format(0.0), font=self.font, fill=255)
        self.grams['imu_quart'] = np.array(img, dtype=np.uint8) * 255

    def refresh(self, timer_event):
        self.fps.update()
        # 如果未开启屏幕或者未开启刷新不需要做后面操作
        if not self.on_off or not self.refresh_enable:
            return

        final_gram = None
        if self.current_cutscenes is None: # 如果过场动画为空那么就检测一下有没有过场动画要显示
            try: 
                self.current_cutscenes = self.cutscenes.get(block=False)
            except Exception as e:
                pass
            if self.current_cutscenes is not None:
                self.current_cutscenes.now_gram = self.grams[self.current_gram_name]
                self.current_cutscenes.new_gram = self.grams[self.current_cutscenes.new_gram_name]

        if self.current_cutscenes is not None: # 有过场
            # 从左往右飞入
            if self.current_cutscenes.direction == 'lr': 
                self.current_cutscenes.pixel_index += self.current_cutscenes.increment
                if self.current_cutscenes.pixel_index <= 128:
                    now = self.current_cutscenes.now_gram[:, :-self.current_cutscenes.pixel_index]
                    new = self.current_cutscenes.new_gram[:, -self.current_cutscenes.pixel_index:]
                    final_gram = np.hstack((new, now))
                else: # 超过了画面宽度, 就是已经完成了飞入
                    self.current_gram_name = self.current_cutscenes.new_gram_name # 将新的画面名称设为当前名称
                    self.current_cutscenes = None # 清除过场

            # 从右往左飞入
            elif self.current_cutscenes.direction == 'rl':
                self.current_cutscenes.pixel_index += self.current_cutscenes.increment
                if self.current_cutscenes.pixel_index <= 128:
                    now = self.current_cutscenes.now_gram[:, self.current_cutscenes.pixel_index:]
                    new = self.current_cutscenes.new_gram[:, :self.current_cutscenes.pixel_index ]
                    final_gram = np.hstack((now, new))
                else: # 超过了画面宽度, 就是已经完成了飞入
                    self.current_gram_name = self.current_cutscenes.new_gram_name # 将新的画面名称设为当前名称
                    self.current_cutscenes = None # 清除过场
            # 从上往下飞入
            elif self.current_cutscenes.direction == 'tb': 
                self.current_cutscenes.pixel_index += self.current_cutscenes.increment
                if self.current_cutscenes.pixel_index <= 32:
                    now = self.current_cutscenes.now_gram[:-self.current_cutscenes.pixel_index, :]
                    new = self.current_cutscenes.new_gram[-self.current_cutscenes.pixel_index:, :]
                    final_gram = np.vstack((new, now))
                else: # 超过了画面高度, 就是已经完成了飞入
                    self.current_gram_name = self.current_cutscenes.new_gram_name # 将新的画面名称设为当前名称
                    self.current_cutscenes = None # 清除过场

            # 从下往上飞入
            elif self.current_cutscenes.direction == 'bt':
                self.current_cutscenes.pixel_index += self.current_cutscenes.increment
                if self.current_cutscenes.pixel_index <= 32:
                    now = self.current_cutscenes.now_gram[self.current_cutscenes.pixel_index:, :]
                    new = self.current_cutscenes.new_gram[:self.current_cutscenes.pixel_index, :]
                    final_gram = np.vstack((now, new))
                else: # 超过了画面高度, 就是已经完成了飞入
                    self.current_gram_name = self.current_cutscenes.new_gram_name # 将新的画面名称设为当前名称
                    self.current_cutscenes = None # 清除过场

            else: # 要求转场, 但是未指定过场动画就直接显示新的画面
                final_gram = self.current_cutscenes.new_gram
                self.current_gram_name = self.current_cutscenes.new_gram_name
                self.current_cutscenes = None
        
        if final_gram is None: #如果上面过场动画没有处理出新的画面来
            """
            有要显示的gram且与上次刷新出来的不是同一个gram才将数据输出到oled屏幕
            注意判断的是内存地址. 若用[]索引修改,因数组在内存上的地址没有改变, 不会进行刷新
            必须要替换整个数组才会刷新
            """
            if self.current_gram_name in self.grams and not self.last_gram is self.grams[self.current_gram_name]:
                final_gram = self.grams[self.current_gram_name]
            
        if final_gram is not None :
            # 每经过 x 秒将画面显示颜色反相(黑白反转), 保护oled显示屏不烧屏
            # invert_interval 为反相的间隔时间
            if self.invert_interval > 0 and time.time() - self.invert_timestamp > self.invert_interval:
                self.invert_timestamp = time.time()
                self.invert = not self.invert 
            r_final_gram = final_gram
            if self.invert:
                r_final_gram = (final_gram + 1) * 255 # 因数组类型为 uint8, 所以 白=255, 黑=0, 255 + 1 = 0, 0+1 = 1, 再*255 就实现黑白反相了

            self.screen.image(Image.fromarray(r_final_gram).convert('1'))
            try:
                self.screen.display()
                self.last_gram = r_final_gram 
            except Exception as e:
                pass
    
def main(args=None):
    try:
        oled_display_node = OledDisplayNode('oled_display_node', log_level=rospy.INFO)
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))


if __name__ == "__main__":
    main()
