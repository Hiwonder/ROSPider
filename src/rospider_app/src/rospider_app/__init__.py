import time
import rospy
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse


class Heart:
    def __init__(self, srv_name, timeout, callback):
        self.heartbeat_stamp = 0
        self.callback = callback
        self.timeout = timeout
        self.heartbeat_timer = rospy.Timer(rospy.Duration(1), self.heartbeat_timeout_check)
        self.heartbeat_srv = rospy.Service(srv_name, SetBool, self.heartbeat_srv_callback)

    def heartbeat_srv_callback(self, msg: SetBoolRequest):
        if msg.data:
            self.heartbeat_stamp = time.time() + self.timeout
        else:
            self.heartbeat_stamp = 0
        return SetBoolResponse(success=True)

    def heartbeat_timeout_check(self, timer_event):
        if self.heartbeat_stamp != 0 and self.heartbeat_stamp < time.time():
            rospy.loginfo("heartbeat timeout")
            self.heartbeat_stamp = 0
            self.callback(timer_event)



