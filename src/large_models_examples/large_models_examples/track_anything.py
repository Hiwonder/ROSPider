#!/usr/bin/python3
# coding=utf8
import os
import cv2
import time
import numpy as np
import sdk.pid as pid

class ObjectTracker:
    def __init__(self, use_mouse=False, automatic=False, log=None): 
        self.log = log
        self.y_stop = 300
        self.stop_distance = 150
        self.start_track = False
        self.automatic = automatic
        self.use_mouse = use_mouse
        if self.use_mouse:
            name = 'image'
            # cv2.namedWindow(name, 1)
            cv2.setMouseCallback(name, self.onmouse)

        self.mouse_click = False
        self.selection = None  # 实时跟踪鼠标的跟踪区域
        self.track_window = None  # 要检测的物体所在区域
        self.drag_start = None  # 标记，是否开始拖动鼠标
        self.start_circle = True
        self.start_click = False
        self.stop_track = False
        
        self.linear_speed = 0
        self.linear_base_speed = 0.007
        self.angular_speed = 0
        self.angular_base_speed = 0.03
        
        self.linear_pid = pid.PID(0.0, 0.0, 0.0)#pid初始化(pid initialization)
        self.angular_pid  = pid.PID(0.0, 0.0, 0.0)

    def set_init_param(self, linear_pid, angular_pid): 
        self.linear_pid = linear_pid
        self.angular_pid = angular_pid

    def update_pid(self, p1, p2):
        self.linear_pid = pid.PID(p1[0], p1[1], p1[2])#pid初始化(pid initialization)
        self.angular_pid = pid.PID(p2[0], p2[1], p2[2])

    # 鼠标点击事件回调函数
    def onmouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:  # 鼠标左键按下
            self.mouse_click = True
            self.drag_start = (x, y)  # 鼠标起始位置
            self.track_window = None
        if self.drag_start:  # 是否开始拖动鼠标，记录鼠标位置
            xmin = min(x, self.drag_start[0])
            ymin = min(y, self.drag_start[1])
            xmax = max(x, self.drag_start[0])
            ymax = max(y, self.drag_start[1])
            self.selection = (xmin, ymin, xmax, ymax)
        if event == cv2.EVENT_LBUTTONUP:  # 鼠标左键松开
            self.mouse_click = False
            self.drag_start = None
            self.track_window = self.selection
            self.selection = None
        if event == cv2.EVENT_RBUTTONDOWN:
            self.mouse_click = False
            self.selection = None  # 实时跟踪鼠标的跟踪区域
            self.track_window = None  # 要检测的物体所在区域
            self.drag_start = None  # 标记，是否开始拖动鼠标
            self.start_circle = True
            self.start_click = False

    def set_track_target(self, tracker, target, image):
        self.stop_track = False
        self.start_circle = False
        self.start_track = True
        tracker.init(image, target)

    def stop(self):
        self.stop_track = True
        self.start_circle = False

    def get_target(self, tracker, image):
        if self.start_circle and self.use_mouse and not self.automatic:
            # 用鼠标拖拽一个框来指定区域
            h, w = image.shape[:2]
            if self.track_window:  # 跟踪目标的窗口画出后，实时标出跟踪目标
                cv2.rectangle(image, (self.track_window[0], self.track_window[1]),
                              (self.track_window[2], self.track_window[3]), (0, 0, 255), 2)
            elif self.selection:  # 跟踪目标的窗口随鼠标拖动实时显示
                cv2.rectangle(image, (self.selection[0], self.selection[1]), (self.selection[2], self.selection[3]),
                              (0, 255, 255), 2)
            if self.mouse_click:
                self.start_click = True
            if self.start_click:
                if not self.mouse_click:
                    self.start_circle = False
            if not self.start_circle:
                self.log.info('start trcking')
                bbox = (self.track_window[0], self.track_window[1], self.track_window[2] - self.track_window[0],
                        self.track_window[3] - self.track_window[1])
                # print(bbox)
                tracker.init(image, bbox)
                self.start_track = True
        else:
            if not self.start_circle:
                if not self.stop_track:
                    ok, box = tracker.track(image)
                    # print(ok, box)
                    # self.log.info(f'{ok} {box}')
                    if ok > 0.9:
                        return image, box
                    else:
                        # pass
                        # Tracking failure
                        cv2.putText(image, "Tracking failure detected !", (10, image.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                                    (0, 255, 255), 1)
        return image, None

    def get_object_distance(self, depth_image, x, y):
        h, w = depth_image.shape[:2]
        roi_h, roi_w = 5, 5
        w_1 = x - roi_w
        w_2 = x + roi_w
        if w_1 < 0:
            w_1 = 0
        if w_2 > w:
            w_2 = w
        h_1 = y - roi_h
        h_2 = y + roi_h
        if h_1 < 0:
            h_1 = 0
        if h_2 > h:
            h_2 = h
        
        # self.log.info(f'{w_1}, {w_2}, {h_1}, {h_2}') 
        # cv2.rectangle(bgr_image, (w_1, h_1), (w_2, h_2), (0, 255, 255), 2)
        w_1, w_2, h_1, h_2 = int(w_1), int(w_2), int(h_1), int(h_2)
        roi = depth_image[h_1:h_2, w_1:w_2]
        distances = roi[np.logical_and(roi > 0, roi < 40000)]
        if len(distances) > 0:
            distance = int(np.mean(distances)/10)
        else:
            distance = 0
            #print(distance)
        ################
        if distance > 600: 
            distance = 600
        # elif distance < 60:
            # distance = 60
        
        return  distance

    def track(self, tracker, image, depth_image):
        image, box = self.get_target(tracker, image)
        # print(box)
        if box is not None:
            img_h, img_w = image.shape[:2]
            p1 = (int(box[0]), int(box[1]))
            p2 = (int(p1[0] + box[2]), int(p1[1] + box[3]))

            cv2.rectangle(image, p1, p2, (0, 255, 0), 2, 1)
            center_x = (p1[0] + p2[0]) / 2
            center_y = (p1[1] + p2[1]) / 2
            cv2.circle(image, (int(center_x), int(center_y)), 5, (0, 255, 255), -1)
            
            depth_img_h, depth_img_w = depth_image.shape[:2]
            distance = self.get_object_distance(depth_image, int(center_x / img_h * depth_img_h), int(center_y / img_w * depth_img_w))
            # self.log.info(f'distance  {distance}')

            if self.start_track:
                self.start_track = False
                self.stop_distance = 40 
            self.linear_pid.SetPoint = self.stop_distance
            if abs(distance - self.stop_distance) < 5:
                distance = self.stop_distance
            self.linear_pid.update(distance)
            tmp = self.linear_base_speed - self.linear_pid.output
            # self.log.info(f'tmp  {tmp}')
            self.linear_speed = tmp/5
            if tmp > 0.1:
                self.linear_speed = 0.02
            if tmp < -0.1:
                self.linear_speed = -0.02
            if abs(tmp) <= 0.0075:
                self.linear_speed = 0
            
            if abs(center_x - img_w/2.0) < 25:
                center_x = img_w / 2.0
            self.angular_pid.SetPoint = img_w / 2.0
            self.angular_pid.update(center_x)

            tmp = self.angular_base_speed + self.angular_pid.output

            self.angular_speed = tmp/5
            if tmp > 1.0:
                self.angular_speed = 0.2
            if tmp < -1.0:
                self.angular_speed = -0.2
            if abs(tmp) <= 0.038:
                self.angular_speed = 0
            # self.log.info(f'{self.linear_speed}, {self.angular_speed}')
            return float(self.linear_speed), float(self.angular_speed), p1,p2,image
        else:
            return 0.0, 0.0, image

    def track_usb(self, tracker, image):
        image, box = self.get_target(tracker, image)
        if box is not None:
            img_h, img_w = image.shape[:2]
            # depth_img_h, depth_img_w = depth_image.shape[:2]
            p1 = (int(box[0]), int(box[1]))
            p2 = (int(p1[0] + box[2]), int(p1[1] + box[3]))

            cv2.rectangle(image, p1, p2, (0, 255, 0), 2, 1)
            center_x = (p1[0] + p2[0]) / 2
            center_y = (p1[1] + p2[1]) / 2
            cv2.circle(image, (int(center_x), int(center_y)), 5, (0, 255, 255), -1)

            if self.start_track:
                self.start_track = False
                # self.y_stop = 30 

            self.linear_pid.SetPoint = self.y_stop
            if abs(center_y - self.y_stop) < 20:
                center_y = self.y_stop
            self.linear_pid.update(center_y)
            tmp = self.linear_base_speed - self.linear_pid.output
            #self.log.info(f'{self.linear_pid.output}')
            self.linear_speed = tmp/5
            if tmp > 0.1:
                self.linear_speed = -0.03
            if tmp < -0.1:
                self.linear_speed = 0.03
            if abs(tmp) <= 0.0075:
                self.linear_speed = 0
            
            if abs(center_x - img_w/2.0) < 25:
                center_x = img_w / 2.0
            self.angular_pid.SetPoint = img_w / 2.0
            self.angular_pid.update(center_x)

            tmp = self.angular_base_speed + self.angular_pid.output

            self.angular_speed = tmp/5
            if tmp > 1.0:
                self.angular_speed = 0.3
            if tmp < -1.0:
                self.angular_speed = -0.3
            if abs(tmp) <= 0.038:
                self.angular_speed = 0
            # self.log.info(f'{self.linear_speed}, {self.angular_speed}')
            return float(self.linear_speed), float(self.angular_speed), image
        else:
            return 0.0, 0.0, image
if __name__ == '__main__':
    cap = cv2.VideoCapture(-1)
    track = ObjectTracker(True)
    while True:
        try:
            ret, image = cap.read()
            if ret:
                x, y, frame = track.track(image, None)
                cv2.imshow('image', frame)
                cv2.waitKey(1)
            else:
                time.sleep(0.01)
        except KeyboardInterrupt:
            break
    cap.release()
    cv2.destroyAllWindows()



