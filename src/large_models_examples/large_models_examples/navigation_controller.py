#!/usr/bin/env python3
# encoding: utf-8
# @data:2022/11/18
# @author:aiden
# 导航搬运(navigation transport)
import math
import rclpy
import numpy as np
from rclpy.node import Node
import sdk.common as common
from std_msgs.msg import Bool
from std_srvs.srv import Trigger, Empty
from rclpy.duration import Duration
from interfaces.srv import SetPose2D
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.srv import GetParameters
from rclpy.executors import MultiThreadedExecutor
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.callback_groups import ReentrantCallbackGroup
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

class NavigationController(Node):
    markerArray = MarkerArray()
    
    def __init__(self, name):
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        
        self.goal_pose = PoseStamped()
        self.navigator = BasicNavigator()

        self.map_frame = 'map'#self.get_parameter('map_frame').value
        self.nav_goal = '/nav_goal'#self.get_parameter('nav_goal').value

        timer_cb_group = ReentrantCallbackGroup()
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 1)
        self.nav_pub = self.create_publisher(PoseStamped, self.nav_goal, 1)
        self.mark_pub = self.create_publisher(MarkerArray, 'path_point', 1)
        self.reach_pub = self.create_publisher(Bool, '~/reach_goal', 1)

        self.create_subscription(PoseStamped, self.nav_goal, self.goal_callback, 1, callback_group=timer_cb_group)

        self.create_service(SetPose2D, '~/set_pose', self.move_srv_callback)
       
        self.navigator.waitUntilNav2Active()
        self.create_service(Empty, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

    def get_node_state(self, request, response):
        return response

    def send_request(self, client, msg):
        future = client.call_async(msg)
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()

    def move_srv_callback(self, request, response):
        self.get_logger().info('start navigaiton pick')

        marker_Array = MarkerArray()
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.action = Marker.DELETEALL
        marker_Array.markers.append(marker)

        self.mark_pub.publish(marker_Array)

        markerArray = MarkerArray()
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        data = request.data
        q = common.rpy2qua(math.radians(data.roll), math.radians(data.pitch), math.radians(data.yaw))
        pose.pose.position.x = data.x
        pose.pose.position.y = data.y
        pose.pose.orientation = q

        # 用数字标记来显示点(mark the point with number to display)
        marker = Marker()
        marker.header.frame_id = self.map_frame

        marker.type = marker.MESH_RESOURCE
        marker.mesh_resource = "package://example/resource/flag.dae"
        marker.action = marker.ADD
        # 大小(size)
        marker.scale.x = 0.08
        marker.scale.y = 0.08
        marker.scale.z = 0.2
        # 颜色(color)
        color = list(np.random.choice(range(256), size=3))
        marker.color.a = 1.0
        marker.color.r = color[0] / 255.0
        marker.color.g = color[1] / 255.0
        marker.color.b = color[2] / 255.0
        # marker.lifetime = rospy.Duration(10)  # 显示时间，没有设置默认一直保留(display time. If not set, it will be kept by default)
        # 位置姿态(position posture)
        marker.pose.position.x = pose.pose.position.x
        marker.pose.position.y = pose.pose.position.y
        marker.pose.orientation = pose.pose.orientation
        markerArray.markers.append(marker)

        self.mark_pub.publish(markerArray)
        self.nav_pub.publish(pose)
        
        response.success = True
        response.message = "navigation pick"
        return response

    def goal_callback(self, msg):
        # 获取要发布的导航点(obtain the navigation point to be published)
        self.get_logger().info('\033[1;32m%s\033[0m' % str(msg))

        self.navigator.goToPose(msg)
        i = 0
        # feedback = self.navigator.getFeedback()
        # total_time = Duration.from_msg(feedback.estimated_time_remaining).nanosecond / 1e9
        # self.get_logger().info('\033[1;32m%s\033[0m' % 'total_time')
        while not self.navigator.isTaskComplete():
            i = i + 1
            feedback = self.navigator.getFeedback()
            # self.get_logger().info(f'{feedback.navigation_time} {feedback.estimated_time_remaining}')
            if feedback and i % 5 == 0:
                # self.get_logger().info(
                    # 'Estimated time of arrival: '
                    # + '{0:.0f}'.format(
                        # Duration.from_msg(feedback.estimated_time_remaining).nanoseconds
                        # / 1e9
                    # )
                    # + ' seconds.'
                # )

                # Some navigation timeout to demo cancellation
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                    self.get_logger().info('\033[1;32m%s\033[0m' % 'timeout...')
                    self.navigator.cancelTask()

            # self.get_logger().info('\033[1;32m%s\033[0m' % 'feedback')
        # Do something depending on the return code
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Goal succeeded!')
            msg = Bool()
            msg.data = True
            self.reach_pub.publish(msg)
        elif result == TaskResult.CANCELED:
            self.get_logger().info('Goal was canceled!')
        elif result == TaskResult.FAILED:
            self.get_logger().info('Goal failed!')
        else:
            self.get_logger().info('Goal has an invalid return status!')

def main():
    node = NavigationController('navigation_controller')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
 
if __name__ == "__main__":
    main()

