#!/usr/bin/env python3
# encoding: utf-8
# 标签定位

import cv2
import math
import rclpy
import threading
import numpy as np
from sdk import common
from rclpy.node import Node
from apriltag import apriltag
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from interfaces.msg import ApriltagInfo, ApriltagsInfo


OBJP = np.array([[-1, -1,  0],
                 [ 1, -1,  0],
                 [-1,  1,  0],
                 [ 1,  1,  0],
                 [ 0,  0,  0]], dtype=np.float32)

AXIS = np.float32([[0, 0, 0],
                   [1.5, 0, 0], 
                   [0, 1.5, 0], 
                   [0, 0, 1.5]])
CIRCLE = np.float32([[0.3 * math.cos(math.radians(i)), 0.3 * math.sin(math.radians(i)), 0] for i in range(360)])
AXIS = np.append(AXIS, CIRCLE, axis=0)


def draw(img, corners, imgpts):
    imgpts = np.int32(imgpts).reshape(-1,2)
    cv2.drawContours(img, [imgpts[4:]], -1, (255, 255, 0), -1)
    cv2.line(img, tuple(imgpts[0]), tuple(imgpts[1]), (255, 0, 0), 3)
    cv2.line(img, tuple(imgpts[0]), tuple(imgpts[2]), (0, 255, 0), 3)
    cv2.line(img, tuple(imgpts[0]), tuple(imgpts[3]), (0, 0, 255), 3)
    return img


class TagNode(Node):
    def __init__(self, name):
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.bridge = CvBridge()
        self.lock = threading.Lock()

        self.camera_intrinsic = np.array([[619.063979, 0, 302.560920],
                                          [0, 613.745352, 237.714934],
                                          [0, 0, 1]], dtype=np.float32)
        self.dist_coeffs = np.array([0.103085, -0.175586, -0.001190, -0.007046, 0.000000])
        self.tag_detector = apriltag("tag36h11")

        self.image_sub = self.create_subscription(Image, '/depth_cam/rgb/image_raw' , self.image_callback, 1)  # 相机画面订阅
        self.camera_info_sub = self.create_subscription(CameraInfo, '/depth_cam/rgb/camera_info' , self.camera_info_callback, 1)  # 相机信息订阅
        self.apriltag_info_pub = self.create_publisher(ApriltagsInfo, '~/apriltag_info', 1) # 标签信息发布
        self.result_publisher = self.create_publisher(Image, '~/image_result', 1)  # 图像处理结果发布
        # self.declare_parameter('enable_display', False)
        self.display = self.get_parameter('enable_display').value

    def camera_info_callback(self, msg):
        with self.lock:
            self.camera_intrinsic = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)

    def image_callback(self, msg):
        rgb_image = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
        result_image = np.copy(rgb_image)

        try:
            with self.lock:
                result_image = self.image_proc(rgb_image, result_image)
        except Exception as e:
            self.get_logger().error(str(e))
        if self.display:
            cv2.imshow('image', cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR))
            cv2.waitKey(1)
        self.result_publisher.publish(self.bridge.cv2_to_imgmsg(result_image, "rgb8"))

    def image_proc(self, rgb_image, result_image):
        gray = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2GRAY)
        h, w = gray.shape[:2]
        gray = cv2.resize(gray, (int(w/2), int(h/2)))
        detections = self.tag_detector.detect(gray)
        apriltags_info = ApriltagsInfo()
        apriltag_info_list = []
        if detections:
            for detection in detections:
                tag_id = detection['id']
                tag_center = detection['center']
                tag_corners = detection['lb-rb-rt-lt']
                corners = [common.point_remapped(p, (int(w/2), int(h/2)), (w, h)) for p in tag_corners]
                lb, rb, rt, lt = corners
                for pt in [lb, lt, rb, rt]:
                    cv2.circle(result_image, (int(pt[0]), int(pt[1])), 2, (0, 255, 255), -1)

                tag_center = common.point_remapped(tag_center, (int(w/2), int(h/2)), (w, h))
                corners = np.array([lb, rb, lt, rt, tag_center], dtype=np.float32).reshape(5, -1)
                ret, rvecs, tvecs = cv2.solvePnP(OBJP, corners, self.camera_intrinsic, self.dist_coeffs)
                imgpts, _ = cv2.projectPoints(AXIS, rvecs, tvecs, self.camera_intrinsic, self.dist_coeffs)

                text = 'id' + str(tag_id)
                text_size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0]
                text_x = int(tag_center[0] - text_size[0]/2 )
                text_y = int(tag_center[1] + text_size[1] + 25)
                cv2.putText(result_image, text, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX,  0.6, (255, 255, 0), 2)
                w =corners[1][0] - corners[0][0]
                d =tvecs[2][0]
                apriltag_info = ApriltagInfo()
                apriltag_info.id = tag_id
                apriltag_info.x = text_x
                apriltag_info.y = text_y
                apriltag_info.w = int(w)
                apriltag_info.d = int(d)
                apriltag_info_list.append(apriltag_info)
                result_image = draw(result_image, corners, imgpts)
        apriltags_info.data = apriltag_info_list
        self.apriltag_info_pub.publish(apriltags_info)

        return result_image


def main(args=None):
    rclpy.init(args=args)
    node = TagNode('apriltag_detect')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
