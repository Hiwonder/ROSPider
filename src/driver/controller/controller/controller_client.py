import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from interfaces.msg import RunActionSet
from kinematics_msgs.srv import SetPose1
from kinematics_msgs.msg import Traveling, TransformEuler, LegPosition, Pose



class ControllerClient(Node):
    def __init__(self):
        super().__init__('controller_client')
        
        
        # 初始化发布器
        self._init_publishers()
        
        # 初始化服务客户端
        self.set_build_in_pose_client = self.create_client(SetPose1,  'controller/set_pose_1')
        self.set_build_in_pose_client.wait_for_service()

        while not self.set_build_in_pose_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('set_pose_1 service not available, waiting...')

    def _init_publishers(self, ):
        """初始化 ROS2 发布器"""
        self.traveling_pub = self.create_publisher(Traveling, 'controller/traveling', 1)
        self.leg_absolute_pub = self.create_publisher(LegPosition, 'controller/set_leg_absolute', 1)
        self.leg_relatively_pub = self.create_publisher(LegPosition, 'controller/set_leg_relatively', 1)
        self.transform_euler_pub = self.create_publisher(TransformEuler, 'controller/pose_transform_euler', 1)
        self.cmd_vel_pub = self.create_publisher(Twist, 'controller/cmd_vel', 1)
        self.run_actionset_pub = self.create_publisher(RunActionSet, 'controller/run_actionset', 1)
        self.set_pose_euler_pub = self.create_publisher(Pose, 'controller/set_pose_euler', 1)

    def send_request(self, client, msg):
        future = client.call_async(msg)
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()

    def pose_transform_euler(self, translation, euler, duration):
        msg = TransformEuler()
        msg.translation.x, msg.translation.y, msg.translation.z = translation
        msg.rotation.x, msg.rotation.y, msg.rotation.z = euler
        msg.duration = duration
        self.transform_euler_pub.publish(msg)


    def set_leg_absolute(self, leg_id, x, y, z, duration):
        msg = LegPosition()
        msg.leg_id = leg_id
        msg.position.x = float(x)
        msg.position.y = float(y)
        msg.position.z = float(z)
        msg.duration = duration
        self.leg_absolute_pub.publish(msg)

    def set_leg_relatively(self, leg_id, x, y, z, duration):
        msg = LegPosition()
        msg.leg_id = leg_id
        msg.position.x = float(x)
        msg.position.y = float(y)
        msg.position.z = float(z)
        msg.duration = duration
        self.leg_relatively_pub.publish(msg)

    def traveling(self,
                  gait=2,
                  stride=30.0,
                  height=15.0,
                  direction=0.0,
                  rotation=0.0,
                  time=0.6,
                  steps=1,
                  interrupt=True,
                  relative_height=False):
        msg = Traveling()
        msg.gait = gait
        msg.stride = float(stride)
        msg.height = float(height)
        msg.direction = float(direction)
        msg.rotation = float(rotation)
        msg.time = float(time)
        msg.steps = steps
        msg.interrupt = interrupt
        msg.relative_height = relative_height

        self.traveling_pub.publish(msg)


    def cmd_vel(self, v_x, v_y, a_z):
        msg = Twist()
        msg.linear.x = float(v_x)
        msg.linear.y = float(v_y)
        msg.angular.z = float(a_z)
        self.cmd_vel_pub.publish(msg)

    def set_build_in_pose(self, pose_name, duration):
        req = SetPose1.Request()
        req.pose = pose_name
        req.duration = duration
        self.send_request(self.set_build_in_pose_client, req)

    def run_actionset(self, action_name, repeat, default_path=True):
        msg = RunActionSet()
        msg.action_path = action_name
        msg.repeat = repeat
        msg.default_path = default_path
        self.run_actionset_pub.publish(msg)

    def set_pose_euler(self, trans, rotate):
        msg = Pose()
        msg.position.x, msg.position.y, msg.position.z = map(float, trans)
        msg.orientation.roll, msg.orientation.pitch, msg.orientation.yaw = map(float, rotate)
        self.set_pose_euler_pub.publish(msg)
