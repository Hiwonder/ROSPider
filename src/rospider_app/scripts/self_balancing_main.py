#!/usr/bin/env python3

import time
import rospy
import std_srvs.srv
from rospider_sdk import pid
from scipy.spatial.transform import Rotation as R
from rospider_controller import build_in_pose, rospider, kinematics_api
from rospider_app import Heart
import sensor_msgs.msg

class SelfBalancingNode:

    def __init__(self, name):
        rospy.init_node(name)
        self.node_name = name
        self.pid_pitch = pid.PID(0.2, 0.1, 0.02)
        self.pid_roll = pid.PID(0.2, 0.1, 0.02)
        self.pitch = 0
        self.roll = 0
        self.robot = None
        self.timestamp = time.time()
        self.duration = 0.5
        self.imu_topic = rospy.get_param("~imu_topic", "/imu/filtered")

        self.imu_sub = None
        self.heart = Heart(self.node_name + '/heartbeat', 5, self.exit_srv_callback)
        self.set_running_srv = rospy.Service(self.node_name + "/set_running", std_srvs.srv.SetBool, self.set_running_srv_callback)
        self.enter_srv = rospy.Service(self.node_name + "/enter", std_srvs.srv.Trigger, lambda _: std_srvs.srv.TriggerResponse(success=True))
        self.exit_srv = rospy.Service(self.node_name + "/exit", std_srvs.srv.Trigger, self.exit_srv_callback)
        self.start_imu = rospy.ServiceProxy("start_imu", std_srvs.srv.Empty)
        self.stop_imu = rospy.ServiceProxy("stop_imu", std_srvs.srv.Empty)
        self.oled_on_off = rospy.ServiceProxy("/oled_display/on_off", std_srvs.srv.SetBool)
        rospy.loginfo("self balancing node created")

    def exit_srv_callback(self, _):
        rospy.loginfo("exit")
        self.set_running_srv_callback(std_srvs.srv.SetBoolRequest(data=False))
        self.stop_imu()
        return std_srvs.srv.TriggerResponse(success=True)

    def set_running_srv_callback(self, req: std_srvs.srv.SetBoolRequest):
        rospy.loginfo("set_running as " + str(req.data))
        rsp = std_srvs.srv.SetBoolResponse()
        try:
            try:
                self.imu_sub.unregister()
            except Exception as e:
                rospy.logerr(str(e))

            if req.data:
                    if self.robot is None:
                        self.robot = rospider.rospider(self, pwm=False)
                    try:
                        self.oled_on_off(data=False)
                    except Exception as e:
                        pass
                    try:
                        self.start_imu()
                    except Exception as e:
                        pass
                    rospy.sleep(0.2)
                    self.robot.set_build_in_pose('DEFAULT_POSE_M', duration=0.5)
                    self.timestamp = time.time()
                    self.pid_pitch.clear()
                    self.pid_roll.clear()
                    self.duration = 0.5
                    self.pitch = 0
                    self.roll = 0
                    rsp.success = True
                    self.imu_sub = rospy.Subscriber(self.imu_topic, sensor_msgs.msg.Imu, self.imu_callback, queue_size=1)
                    self.start_imu()
            else:
                if self.robot is not None:
                    self.robot.set_build_in_pose('DEFAULT_POSE', duration=1)
                    rospy.sleep(0.1)
                    self.robot.loop_enable = False
                    self.robot = None
                try:
                    self.stop_imu()
                except Exception as e:
                    pass
                try:
                    self.oled_on_off(data=True)
                except Exception as e:
                    pass
                rospy.sleep(1)

        except Exception as e:
            rospy.logerr(str(e))
            rsp.message = str(e)
        return rsp

    def imu_callback(self, imu_msg):
        if time.time() - self.timestamp < 2.5:
            return
        try:
            q = imu_msg.orientation
            r = R.from_quat((q.x, q.y, q.z, q.w))
            x, y, z = r.as_euler('xyz')

            self.pid_pitch.update(y)
            self.pid_roll.update(x)
            try:
                pitch = self.pitch - self.pid_pitch.output
                roll = self.roll - self.pid_roll.output
                new_pose = kinematics_api.transform_euler(build_in_pose.DEFAULT_POSE_M, (0, 0, 0), 'xyz', (roll, pitch, 0), degrees=False)
                self.robot.set_pose_base(new_pose, self.duration)
                self.duration = max(self.duration - 0.005, 0.02)
                self.pitch, self.roll = pitch, roll
            except Exception as e:
                self.pitch = 0
                self.roll = 0
                self.pid_pitch.clear()
                self.pid_roll.clear()

            '''
            x_out = min(0.02, x_out)
            x_out = max(-0.02, x_out)
            y_out = min(0.02, y_out)
            y_out = max(-0.02, y_out)
            '''
            #self.robot.transform_pose_2((0, 0, 0), 'xy', (x_out, -y_out), 0.02, degrees=False, interrupt=True)
            #self.robot.set_pose_base(self, new_pose, duration, pseudo=False, update_pose=True):

        except Exception as e:
            rospy.logerr(str(e))


def main():
    try:
        self_balancing_node = SelfBalancingNode('self_balancing')
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))


if __name__ == "__main__":
    main()
