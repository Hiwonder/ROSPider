import time
from copy import deepcopy
from rclpy.node import Node
from servo_controller_msgs.msg import ServosPosition
from servo_controller.bus_servo_control import set_servo_position
from .config import SERVOS, SIMULATE

class JointControl(Node):
    def __init__(self):
        super().__init__('joint_control')
        self.joints_pub = self.create_publisher(ServosPosition, 'servo_controller', 1)
    def set_joint(self, joint_id: int, radians: float, duration: float, joints_state: dict = None) -> dict:
        """
        设置物理舵机角度

        :param joint_id: 舵机id，这里不是真正的物理ID， 而是在SERVOS里面定义的id
        :param radians: 目标舵机角度
        :param duration: 到达目标角度的用时
        :param joints_state: 当前的关节状态字典
        :type servo_id: int
        :type radians: float
        :type duration: float
        :type joint_state: dict

        :return: 新的关节状态字典
        :rtype: dict
        """
        if not joint_id in SERVOS:
            raise ValueError("Invalid joint id %d" % joint_id)

        servo = SERVOS[joint_id]
        servo_name = servo['name']
        servo_id = servo['id']
        center = servo['center']
        ticks = servo['ticks']
        max_radians = servo['max_radians']
        direction = servo['direction']
        offset = servo['offset']

        # 舵机位置数值的最大最小值
        max_ticks = int(center + ticks / 2)
        min_ticks = int(center - ticks / 2)

        # 最终的舵机真实角度
        real_radians = direction * (offset + radians)
        if abs(real_radians) > abs(max_radians / 2):
            raise ValueError("Invalid radians {:.4f}".format(radians))

        # 计算舵机位置数值
        pos_a = (real_radians - (-max_radians / 2)) / max_radians * ticks
        pos_b = pos_a + min_ticks
        pos = int(pos_b)
        #限幅, 确保数值有效
        pos = pos if pos < max_ticks else max_ticks
        pos = pos if pos > min_ticks else min_ticks


        if not SIMULATE:
            t = time.time()
            set_servo_position(self.joints_pub, duration, ((servo_id, pos), ))

        if joints_state is not None:
            joints_state[servo_name] = radians
        return joints_state


    def set_multi_joints(self, duration, data, joints_state: dict = None) -> dict:
        """
        设置多个物理舵机角度

        :return: 新的关节状态字典
        :rtype: dict
        """
        servos_data = []
        new_joints_state = deepcopy(joints_state)
        for joint_id, radians in data:
            if not joint_id in SERVOS:
                raise ValueError("Invalid joint id %d" % joint_id)

            servo = SERVOS[joint_id]
            servo_name = servo['name']
            servo_id = servo['id']
            center = servo['center']
            ticks = servo['ticks']
            max_radians = servo['max_radians']
            direction = servo['direction']
            offset = servo['offset']

            # 舵机位置数值的最大最小值
            max_ticks = int(center + ticks / 2)
            min_ticks = int(center - ticks / 2)

            # 最终的舵机真实角度
            real_radians = direction * (offset + radians)
            if abs(real_radians) > abs(max_radians / 2):
                raise ValueError("Invalid radians {:.4f}".format(radians))

            # 计算舵机位置数值
            pos = int((real_radians - (-max_radians / 2)) / max_radians * ticks + min_ticks)
            if not SIMULATE:
                servos_data.append([servo_id, pos])

            else:
                raise ValueError("Invalid servo id %d", servo_id)
            if new_joints_state is not None:
                new_joints_state[servo_name] = radians

        if len(servos_data) > 0:    
            set_servo_position(self.joints_pub, duration, (servos_data))


        return new_joints_state
