import cv2
import math
from rospider_sdk import misc, pid
from vision_utils import get_area_max_contour, colors


class ColorTracker:
    """
    颜色追踪器，用于让机器人进行颜色追踪
    实例化后直接以实例名称调用, 返回云台俯仰角角和偏航角,以控制云台追踪
    """
    def __init__(self, color, color_name, node=None):
        """
        :param color: 要追踪的颜色阈值，一个字典形如{'min': (0, 0, 0), 'max': (255, 255, 255)}
        :param color: 要追踪的颜色名称， 如'red', 主要用于控制显示结果的颜色，不确定可以随便填
        :param node: 拥有这个追踪器的节点实例, 目前没啥用, 在ros2里获取logger等可能会用到
        """
        self.node = node # 
        self.color= color # 要追踪的目标颜色的阈值范围
        self.color_name = color_name # 要追踪的目标颜色的名称
        self.pid_pitch = pid.PID(0.0005, 0.0, 0.000005) # 控制俯仰角的 pid 控制器
        self.pid_yaw = pid.PID(0.0004, 0.0, 0.00002) # 控制偏航角的 pid 控制器
        self.pitch = 0 # 俯仰角
        self.yaw = 0 # 偏航角

    def __call__(self, image, proc_size=(320, 180)):
        """
        对画面进行颜色追踪
        :param image: 要识别处理的图像 opencv rgb 格式
        :param proc_size: 处理图像时的尺寸，输入图像会被缩放到这个尺寸
        :return image: 处理后的图像, 如果有识别到要追踪的目标会在上面圈出来
        :return pitch: 如果识别到了要追踪的目标为新的俯仰角， 如果没有识别到则为None
        :return yaw: 如果识别到了要追踪的目标为新的偏航角， 如果没有识别到则为None
        """
        o_h, o_w = image.shape[:2]  # 原始尺寸
        n_w, n_h = proc_size  # 新的尺寸, 可能缩小尺寸减小计算量
        img_resize = cv2.resize(image, tuple(proc_size), cv2.INTER_NEAREST) # 缩放图片
        img_blur = cv2.GaussianBlur(img_resize, (3, 3), 3) # 高斯模糊
        img_lab = cv2.cvtColor(img_blur, cv2.COLOR_RGB2LAB) # 转换到 LAB 空间
        mask = cv2.inRange(img_lab, tuple(self.color['min']), tuple(self.color['max'])) # 二值化
        # 平滑边缘，去除小块，合并靠近的块
        eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
        dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
        # 找出最大轮廓
        contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
        max_contour_area = get_area_max_contour(contours, 50)

        # 如果有符合要求的轮廓
        if max_contour_area is not None:
            (center_x, center_y), radius = cv2.minEnclosingCircle(max_contour_area[0]) # 最小外接圆
            # 将识别到的圆的参数恢复到原图的像素坐标下(因为是用缩放后的图片识别的)
            center_x = misc.val_map(center_x, 0, n_w, 0, o_w)
            center_y = misc.val_map(center_y, 0, n_h, 0, o_h)
            radius = misc.val_map(radius, 0, n_h, 0, o_h)

            # 圈出识别的的要追踪的色块
            circle_color = colors.rgb[self.color_name] if self.color_name in colors.rgb else (0x55, 0x55, 0x55)
            cv2.circle(image, (int(center_x), int(center_y)), int(radius), circle_color, 2)

            # 画面 y 轴, 控制上下方向
            if abs(center_y - (o_h / 2)) > 40: # 相差范围小于一定值就不用再动了
                self.pid_pitch.SetPoint = o_h / 2 # 我们的目标是要让色块在画面的中心, 就是整个画面的像素宽度的 1/2 位置
                self.pid_pitch.update(center_y) # 更新 pid 控制器
                self.pitch += self.pid_pitch.output # 获得 pid 输出
            else:
                self.pid_pitch.clear() # 如果已经到达中心了就复位一下 pid 控制器
            # 画面 x 轴, 控制左右方向
            if abs(center_x - (o_w / 2)) > 40:
                self.pid_yaw.SetPoint = o_w / 2
                self.pid_yaw.update(center_x)
                self.yaw += self.pid_yaw.output
            else:
                self.pid_pitch.clear()

            # 限制幅度，两个舵机的运动范围有物理限制，这里做限制保护它们
            self.yaw = misc.set_range(self.yaw, -math.pi/2, math.pi/2)
            self.pitch = misc.set_range(self.pitch, -0.4, math.pi/2)
            return image, self.pitch, self.yaw
        else:
            self.pid_yaw.clear()
            self.pid_pitch.clear()
        # 如果没找到符合要求的色块， 两个舵机的数据都为None
        return image, None, None
