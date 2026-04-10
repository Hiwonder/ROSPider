import cv2
import math
import numpy as np
import apriltag
from rospider_sdk import misc, pid
from vision_utils import distance


class AprilTagTracker:
    """
    Apriltag追踪器
    识别画面中的apriltag, 并且追踪id的标签
    如果有多个相同id标签则追踪最靠近画面中心的一个
    """
    def __init__(self, tag_id, node):
        """
        :param tag_id: 要识别的apriltag id
        :param node: 拥有这个追踪器的节点实例, 目前没啥用, 在ros2里获取logger等可能会用到
        """
        self.node = node
        self.tag_id = tag_id 
        self.tag_detector = apriltag.apriltag("tag36h11") # apriltag 识别器
        self.pid_pitch = pid.PID(0.0005, 0.0, 0.000005) # 俯仰角 pid 控制器
        self.pid_yaw = pid.PID(0.0006, 0.0, 0.00001) # 偏航角 pid 控制器
        self.pitch = 0 # 云台俯仰角
        self.yaw = 0 # 云台偏航角

    def __call__(self, image, proc_size=(320, 180)):
        """
        对画面做识别， 找出要追踪的apriltag, 返回新的俯仰角和偏航角
        :param image: 要识别处理的图像
        :param proc_size: 要图像处理中的尺寸, 先缩放到这个尺寸再处理识别
        """
        o_h, o_w = image.shape[:2]  # 原始尺寸
        n_w, n_h = proc_size # 新的尺寸， 图像处理中尺寸
        img_resize = cv2.resize(image, tuple(proc_size)) # 图像缩放
        img_gray = cv2.cvtColor(img_resize, cv2.COLOR_RGB2GRAY) # 转换成灰度图
        detections = self.tag_detector.detect(img_gray) # 识别图中 Apriltag
        if detections != ():
            target_tag = []
            for tag in detections:
                corners = tag['lb-rb-rt-lt'].reshape(-1, 2) # 识别到的 AprilTag 的四个角的点
                # 将缩放后的坐标转化为原图坐标
                org_corners = [[misc.val_map(c[0], 0, n_w, 0, o_w), misc.val_map(c[1], 0, n_h, 0, o_h)] for c in corners]
                cv2.drawContours(image, np.array(org_corners).reshape(1, -1, 2).astype(int), -1, (0, 255, 0), 2)  # 画出外框
                center = tag['center'] # AprilTag 中心点
                center = int(misc.val_map(center[0], 0, n_w, 0, o_w)), int(misc.val_map(center[1], 0, n_h, 0, o_h))
                cv2.circle(image, center, 4, (0, 255, 0), -1)  # 画出中心点
                #rospy.logdebug("tag id: {}, center: x:{} y:{}".format(tag.tag_id, center[0], center[1]))
                # 如果识别到的 tag 的 id 是我们要追踪的 id 就将这个tag的数据记下来
                # 因为可能有多个，所以用 list 保存
                if tag['id'] == self.tag_id:
                    target_tag.append(tag)

            # 识别到的要追踪的tag不为0个
            if len(target_tag) > 0:
                # 找出距离画面中心最近的一个 tag
                target_tag = min(target_tag, key=lambda tag: distance(tag['center'], (n_w / 2, n_h / 2)))
                corners = target_tag['lb-rb-rt-lt'].reshape(-1, 2).astype(int)  # 目标 tag 的四个角的点
                org_corners = [[misc.val_map(c[0], 0, n_w, 0, o_w), misc.val_map(c[1], 0, n_h, 0, o_h)] for c in corners]
                center = target_tag['center'] # 目标 tag 中心点
                center = misc.val_map(center[0], 0, n_w, 0, o_w), misc.val_map(center[1], 0, n_h, 0, o_h)
                # 追踪中的 tag 用红色标出来
                cv2.drawContours(image, np.array(org_corners).reshape(1, -1, 2).astype(int), -1, (255, 0, 0), 2)  # 画出外框
                cv2.circle(image, (int(center[0]), int(center[1])), 4, (255, 0, 0), -1)  # 画出中心点

                center_x, center_y = center
                if abs(center_y - (o_h / 2)) > 40: # 相差范围小于一定值就不用再动了
                    self.pid_pitch.SetPoint = o_h / 2 # 我们的目标是要让色块在画面的中心, 就是整个画面的像素宽度的 1/2 位置
                    self.pid_pitch.update(center_y) # 更新 pid 控制器
                    self.pitch += self.pid_pitch.output  # 获得 pid 输出
                else:
                    self.pid_pitch.clear()  # 如果已经到达中心了就复位一下 pid 控制器
                if abs(center_x - (o_w / 2)) > 40:
                    self.pid_yaw.SetPoint = o_w / 2
                    self.pid_yaw.update(center_x)
                    self.yaw += self.pid_yaw.output
                else:
                    self.pid_yaw.clear()

                # 限制幅度，两个舵机的运动范围有物理限制，这里做限制保护它们
                self.yaw = misc.set_range(self.yaw, -math.pi/2, math.pi/2)
                self.pitch = misc.set_range(self.pitch, -0.4, math.pi/2)
                return image, self.pitch, self.yaw
            else:
                self.pid_pitch.clear()
                self.pid_yaw.clear()
        # 没识别到，俯仰偏航均为None
        return image, None, None
