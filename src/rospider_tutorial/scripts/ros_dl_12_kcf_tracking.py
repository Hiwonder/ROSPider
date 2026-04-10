#!/usr/bin/env python3
import cv2
import rospy
import queue
import numpy as np
from sensor_msgs.msg import Image
from vision_utils import fps
from rospider_controller import client
from rospider_sdk import misc, pid, serial_servo


class KCFTrackingNode:
    def __init__(self):
        rospy.init_node('kcf_node')

        # 实例化 kcf 追踪器
        self.tracker = None
        self.enable_select = False
        self.fps = fps.FPS() # 帧率统计器

        self.pid_pitch = pid.PID(0.4, 0.01, 0.01) # 控制俯仰角的 pid 控制器
        self.pid_yaw = pid.PID(25.0, 0.02, 0.0005) # 控制偏航角的 pid 控制器
        self.pitch = 500 # 俯仰角舵机数值 0~1000 对应 0~240deg
        self.yaw = 500 # 偏航角舵机数值

        serial_servo.set_position(19, 500, 1000)


        # 订阅相机图像话题
        self.image_queue = queue.Queue(maxsize=2)
        self.camera_rgb_prefix = rospy.get_param('/camera_rgb_prefix', 'camera/rgb')
        self.image_sub = rospy.Subscriber(self.camera_rgb_prefix + '/image_raw', Image, self.image_callback, queue_size=1)



    def image_callback(self, ros_image):
        #rospy.logdebug('Received an image! ')
        # 将画面转为 opencv 格式
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data)
        result_image = np.copy(cv2.resize(rgb_image, (int(ros_image.width * 1), int(ros_image.height * 1))))
        rgb_image = cv2.resize(rgb_image, (int(ros_image.width / 4), int(ros_image.height / 4)))
        factor = result_image.shape[0] / rgb_image.shape[0]
        h, w = rgb_image.shape[:2]
        try:
            if self.tracker is None:
                if self.enable_select:
                    roi = cv2.selectROI("image", cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR), False)
                    roi =  tuple(int(i / factor)for i in roi)
                    if roi:
                        #self.tracker = cv2.TrackerCSRT_create()
                        self.tracker = cv2.TrackerKCF_create()
                        self.tracker.init(rgb_image, roi)
            else:
                status, box = self.tracker.update(rgb_image)
                if status:
                    rospy.loginfo(str(box))
                    p1 = int(box[0] * factor), int(box[1] * factor)
                    p2 = p1[0] + int(box[2] * factor), p1[1] + int(box[3] * factor)
                    cv2.rectangle(result_image, p1, p2, (255, 255, 0), 2)


                    center_x,center_y =  (box[0] + box[2] / 2) / w, (box[1] + box[3] / 2) / h
                    print('center:', center_x, center_y, w, h)
                    print(box)
                    # 画面 y 轴, 控制俯仰
                    #if abs(p1[1] - (h / 2)) > 30: # 相差范围小于一定值就不用再动了
                    #    self.pid_pitch.SetPoint = h / 2 # 我们的目标是要让色块在画面的中心, 就是整个画面的像素宽度的 1/2 位置
                    #    self.pid_pitch.update(p1[1]) # 更新 pid 控制器
                    #    self.pitch += self.pid_pitch.output # 获得 pid 输出
                    #else:
                    #    self.pid_pitch.clear() # 如果已经到达中心了就复位一下 pid 控制器
                        # 画面 x 轴, 控制偏航
                    if abs(center_x - 0.5) > 0.05:
                        self.pid_yaw.SetPoint = 0.5
                        self.pid_yaw.update(center_x)
                        self.yaw += self.pid_yaw.output
                        print(self.yaw,(w / 2))
                    else:
                        self.pid_yaw.clear()
                    # 限制幅度，两个舵机的运动范围有物理限制，这里做限制保护它们
                    self.yaw = misc.set_range(self.yaw, 0, 1000)
                    rospy.loginfo("pitch:{:.2f}\tyaw:{:.2f}".format(self.pitch , self.yaw))
                    # 设置舵机角度， 画面大概为30fps， 将舵机运动时间设为相近
                    serial_servo.set_position(19, int(self.yaw), 0)
                else:
                    self.pid_yaw.clear()
                    #self.pid_pitch.clear()

        except Exception as e:
            rospy.logerr(str(e))

        self.fps.update()
        self.fps.show_fps(result_image)
        result_image = cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR)
        cv2.imshow("image", result_image)

        key = cv2.waitKey(1)
        if key == ord('s'): # 按下s开始选择追踪目标
            self.tracker = None
            self.enable_select = True


if __name__ == '__main__':
    try:
        kcf_tracking = KCFTrackingNode()
        print("在画面窗口按下s开始选择追踪目标")
        print("在画面窗口按下s开始选择追踪目标")
        print("在画面窗口按下s开始选择追踪目标")
        print("在画面窗口按下s开始选择追踪目标")
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))
