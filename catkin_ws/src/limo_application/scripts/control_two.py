#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gap(Int16) + rotate(Bool)  →  cmd_vel(Twist)  (PID)
"""
import rospy, numpy as np
from std_msgs.msg import Int16, Bool
from geometry_msgs.msg import Twist

class LaneControllerPID:
    def __init__(self):
        # ─ 파라미터 ─
        self.kp = rospy.get_param("~kp", 0.004)
        self.ki = rospy.get_param("~ki", 0.00005)
        self.kd = rospy.get_param("~kd", 0.0006)
        self.i_lim = rospy.get_param("~i_lim", 8000.0)

        self.base_speed   = rospy.get_param("~base_speed",   0.25)
        self.max_ang      = rospy.get_param("~max_ang",      2.0)
        self.rotate_speed = rospy.get_param("~rotate_speed", 1.2)
        self.timeout      = rospy.get_param("~timeout",      0.4)

        # ─ 상태 ─
        self.err_px = 0.0
        self.integral = 0.0
        self.prev_err = 0.0
        self.rot_flag = True
        self.last_upd = rospy.Time.now()

        rospy.Subscriber("gap",    Int16, self.cb_gap,    queue_size=1)
        rospy.Subscriber("rotate", Bool,   self.cb_rot,   queue_size=1)
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)

        self.timer = rospy.Timer(rospy.Duration(0.05), self.loop)  # 20 Hz

    def cb_gap(self, msg):
        self.err_px  = float(msg.data)
        self.last_upd = rospy.Time.now()

    def cb_rot(self, msg):
        self.rot_flag = msg.data
        self.last_upd = rospy.Time.now()

    def loop(self, _):
        dt = (rospy.Time.now() - self.last_upd).to_sec()
        searching = self.rot_flag or (dt > self.timeout)

        cmd = Twist()

        if searching:
            self.integral = self.prev_err = 0.0
            cmd.angular.z =  self.rotate_speed
        else:
            # PID
            self.integral += self.err_px * dt
            self.integral = np.clip(self.integral, -self.i_lim, self.i_lim)
            diff = (self.err_px - self.prev_err) / dt if dt > 1e-4 else 0.0
            self.prev_err = self.err_px

            ang = -(self.kp*self.err_px + self.ki*self.integral + self.kd*diff)
            ang = np.clip(ang, -self.max_ang, self.max_ang)

            cmd.angular.z = ang
            cmd.linear.x  = self.base_speed

        self.cmd_pub.publish(cmd)

if __name__ == "__main__":
    rospy.init_node("lane_controller_pid")
    LaneControllerPID()
    rospy.loginfo("lane_controller_pid node started")
    rospy.spin()
