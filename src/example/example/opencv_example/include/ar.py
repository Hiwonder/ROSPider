#!/usr/bin/env python3
# encoding: utf-8
# AR视觉

import os
import cv2
import rclpy
import threading
import numpy as np
from rclpy.node import Node
from app.common import Heart
from apriltag import apriltag
from cv_bridge import CvBridge
from std_srvs.srv import Trigger
from example.opencv_example.include.obj_loader import OBJ as obj_load
from sensor_msgs.msg import CameraInfo, Image
from scipy.spatial.transform import Rotation as R

# 获取模型默认存放路径(acquire the default storage path for the model)
MODEL_PATH = os.path.join(os.path.abspath(os.path.join(os.path.split(os.path.realpath(__file__))[0])), '3d_model')

# 求解pnp的点，正方形的四个角和中心点(solve for the points of pnp, the four corners and the center point of the square)
OBJP = np.array([[-1, -1,  0],
                 [ 1, -1,  0],
                 [-1,  1,  0],
                 [ 1,  1,  0],
                 [ 0,  0,  0]], dtype=np.float32)

# 绘制立方体的坐标(draw the coordinate of the cube)
AXIS = np.float32([[-1, -1, 0], 
                   [-1,  1, 0], 
                   [ 1,  1, 0], 
                   [ 1, -1, 0],
                   [-1, -1, 2],
                   [-1,  1, 2],
                   [ 1,  1, 2],
                   [ 1, -1, 2]])

# 模型缩放比例(model scaling)
MODELS_SCALE = {
                'bicycle': 50, 
                'fox': 4, 
                'chair': 400, 
                'cow': 0.4,
                'wolf': 0.6,
                }

def draw_rectangle(img, imgpts):
    '''
    绘制立方体(draw the cube)
    :param img: 要绘制立方体的图像(the image to draw the cube)
    :param imgpts: 立方体的角点(angular point of the cube)
    :return: 要绘制立方体的图像(the image to draw the cube)
    '''
    imgpts = np.int32(imgpts).reshape(-1, 2)
    cv2.drawContours(img, [imgpts[:4]], -1, (0, 255, 0), -3)  # 绘制轮廓点，填充形式(draw contour points, filled)
    for i, j in zip(range(4), range(4, 8)):
        cv2.line(img, tuple(imgpts[i]), tuple(imgpts[j]), (255), 3)  # 绘制线连接点(draw points connected by lines)
    cv2.drawContours(img, [imgpts[4:]], -1, (0, 0, 255), 3)  # 绘制轮廓点，不填充(draw contour points, unfilled)
    
    return img

class ARNode(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.name = name
        self.target_model = self.get_parameter('model').value
        # 摄像头内参(camera intrinsic parameters)
        self.camera_intrinsic = np.matrix([[619.063979, 0,          302.560920],
                                           [0,          613.745352, 237.714934],
                                           [0,          0,          1]])
        self.dist_coeffs = np.array([0.103085, -0.175586, -0.001190, -0.007046, 0.000000])
        
        self.obj = None
        self.image_sub = None
        self.camera_info_sub = None
        self.bridge = CvBridge()
        self.tag_detector = apriltag("tag36h11")  # 实例化apriltag(instantiate apriltag)
        self.lock = threading.RLock()  # 线程锁(thread lock)
          
        self.result_publisher = self.create_publisher(Image, '~/image_result', 1)  # 发布最终图像(publish the final image)
        self.image_sub = self.create_subscription(Image, '/depth_cam/rgb/image_raw', self.image_callback, 1)  # 摄像头订阅(subscribe to the camera)
        self.camera_info_sub = self.create_subscription(CameraInfo, '/depth_cam/rgb/camera_info', self.camera_info_callback, 1) # 订阅摄像头信息(subscribe to the camera information)
        
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info(f'\033[1;32mmodel: {self.target_model}\033[0m')
        self.set_model()


    def get_node_state(self, request, response):
        response.success = True
        return response

    def set_model(self):
        # 设置模型(set model)
        with self.lock:
            if self.target_model != 'rectangle':  # 如果不是绘制立方体(if the cube is not being drawn)
                # 加载模型(load the model)
                obj = obj_load(os.path.join(MODEL_PATH, self.target_model + '.obj'), swapyz=True)
                obj.faces = obj.faces[::-1]
                new_faces = []
                # 对模型进行解析，获取点坐标(analyze the model and get the point coordinates)
                for face in obj.faces:
                    face_vertices = face[0]
                    points = []
                    colors = []
                    for vertex in face_vertices:
                        data = obj.vertices[vertex - 1]
                        points.append(data[:3])
                        if self.target_model != 'cow' and self.target_model != 'wolf':
                            colors.append(data[3:])
                    scale_matrix = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]) * MODELS_SCALE[self.target_model]  # 缩放(scale)
                    points = np.dot(np.array(points), scale_matrix)
                    if self.target_model == 'bicycle':
                        points = np.array([[p[0] - 670, p[1] - 350, p[2]] for p in points])
                        points = R.from_euler('xyz', (0, 0, 180), degrees=True).apply(points)
                    elif self.target_model == 'fox':
                        points = np.array([[p[0], p[1], p[2]] for p in points])
                        points = R.from_euler('xyz', (0, 0, -90), degrees=True).apply(points)
                    elif self.target_model == 'chair':
                        points = np.array([[p[0], p[1], p[2]] for p in points])
                        points = R.from_euler('xyz', (0, 0, -90), degrees=True).apply(points)
                    else:
                        points = np.array([[p[0], p[1], p[2]] for p in points])
                    if len(colors) > 0:
                        color = tuple(255 * np.array(colors[0]))
                    else:
                        color = None
                    new_faces.append((points, color))
                self.obj = new_faces

    def camera_info_callback(self, msg):
        # 摄像头内参信息获取回调(camera internal parameter callback)
        with self.lock:
            self.camera_intrinsic = np.array(msg.k).reshape(3, -1)
            self.dist_coeffs = np.array(msg.d)

    def image_callback(self, ros_image):
        # 图像回调(image callback)
        # 将ROS图像消息转化为numpy格式(convert ROS image into numpy format)
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, "rgb8")
        rgb_image = np.array(cv_image, dtype=np.uint8)
        result_image = np.copy(rgb_image)
        with self.lock:
            try:
                # 图像处理(process image)
                result_image = self.image_proc(rgb_image, result_image)
            except Exception as e:
                self.get_logger().info(str(e))
            cv2.imshow("result", cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR))
            cv2.waitKey(1)
        # opencv格式转为ros格式(convert opencv format into ros format)
        self.result_publisher.publish(self.bridge.cv2_to_imgmsg(result_image, "rgb8"))

    def image_proc(self, rgb_image, result_image):
        # 图像处理
        if self.target_model is not None: 
            gray = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2GRAY)  # 转为灰度图(convert into gray image)
            detections = self.tag_detector.detect(gray)  # aprilatg识别(aprilatg recognition)
            if detections != ():
                for detection in detections:  # 遍历(traverse)
                    # 获取四个角点和中心(acquire four angular points and center point)
                    tag_center = detection['center']
                    tag_corners = detection['lb-rb-rt-lt']
                    lb = tag_corners[0]
                    rb = tag_corners[1]
                    rt = tag_corners[2]
                    lt = tag_corners[3]
                    # 绘制四个角点(draw four angular points)
                    cv2.circle(result_image, (int(lb[0]), int(lb[1])), 2, (0, 255, 255), -1)
                    cv2.circle(result_image, (int(lt[0]), int(lt[1])), 2, (0, 255, 255), -1)
                    cv2.circle(result_image, (int(rb[0]), int(rb[1])), 2, (0, 255, 255), -1)
                    cv2.circle(result_image, (int(rt[0]), int(rt[1])), 2, (0, 255, 255), -1)
                    # cv2.circle(result_image, (int(tag_center[0]), int(tag_center[1])), 3, (255, 0, 0), -1)
                    corners = np.array([lb, rb, lt, rt, tag_center]).reshape(5, -1)
                    # 使用世界坐标系k个点坐标(OBJP)，对应图像坐标系2D的k个点坐标(corners)，以及相机内参camera_intrinsic和dist_coeffs进行反推图片的外参r, t
                    # (Use the world coordinate system k point coordinates (OBJP), the k point coordinates (corners) corresponding to the 2D image coordinate system,
                    # and the camera internal parameters camera_intrinsic and dist_coeffs to reverse the external parameters r, t of the picture)
                    ret, rvecs, tvecs = cv2.solvePnP(OBJP, corners, self.camera_intrinsic, self.dist_coeffs)
                    if self.target_model == 'rectangle':  # 如果要显示立方体则单独处理(if the cube needs to be displayed, process independently)
                        # 反向投影将世界坐标系点转换到图像点(backprojection converts world coordinate system points to image points)
                        imgpts, jac = cv2.projectPoints(AXIS, rvecs, tvecs, self.camera_intrinsic, self.dist_coeffs)
                        result_image = draw_rectangle(result_image, imgpts)
                    else:
                        for points, color in self.obj:
                             dst, jac = cv2.projectPoints(points.reshape(-1, 1, 3)/100.0, rvecs, tvecs, self.camera_intrinsic, self.dist_coeffs)
                             imgpts = dst.astype(int)
                             # 手动上色(manual coloring)
                             if self.target_model == 'cow':
                                 cv2.fillConvexPoly(result_image, imgpts, (0, 255, 255))
                             elif self.target_model == 'wolf':
                                 cv2.fillConvexPoly(result_image, imgpts, (255, 255, 0))
                             else:
                                 cv2.fillConvexPoly(result_image, imgpts, color)

        return result_image

def main():
    node = ARNode('ar')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
