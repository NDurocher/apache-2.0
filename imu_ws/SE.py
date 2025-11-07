#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
ROS1 Extended Kalman Filter (EKF) state estimator for a differential-drive robot.

Python 2.7 compatible.
"""

from __future__ import print_function
import math
import numpy as np
import rospy
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler
import tf


def normalize_angle(a):
    """Normalize angle to [-pi, pi)."""
    return (a + math.pi) % (2 * math.pi) - math.pi


class DiffDriveEKF(object):
    def __init__(self):
        # Parameters
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.child_frame_id = rospy.get_param("~child_frame_id", "base_link")
        self.publish_tf = rospy.get_param("~publish_tf", False)

        self.topic_odom = rospy.get_param("~wheel_odom_topic", "/wheel_odom")
        self.topic_imu = rospy.get_param("~imu_topic", "/sensor/imu")
        self.topic_cmd = rospy.get_param("~cmd_vel_topic", "/cmd_vel")

        Q_diag = rospy.get_param("~Q_diag", [1e-3, 1e-3, 1e-4])
        R_odom_diag = rospy.get_param("~R_odom_diag", [5e-3, 5e-3, 1e-3])
        R_imu_yaw = rospy.get_param("~R_imu_yaw", 2e-3)

        self.Q = np.diag(Q_diag).astype(float)
        self.R_odom = np.diag(R_odom_diag).astype(float)
        self.R_imu = np.array([[float(R_imu_yaw)]])

        # State [x, y, theta]
        self.x = np.zeros((3, 1))
        self.P = np.eye(3) * 1e-2

        # Controls
        self.v = 0.0
        self.omega = 0.0

        # Time
        self.last_time = None

        # ROS pubs/subs
        self.pub = rospy.Publisher("~odom_filtered", Odometry, queue_size=10)
        rospy.Subscriber(self.topic_cmd, Twist, self.cb_cmd_vel, queue_size=50)
        rospy.Subscriber(self.topic_odom, Odometry, self.cb_wheel_odom, queue_size=50)
        rospy.Subscriber(self.topic_imu, Imu, self.cb_imu, queue_size=50)

        # TF broadcaster
        self.br = tf.TransformBroadcaster() if self.publish_tf else None

        rospy.loginfo("DiffDriveEKF initialized: wheel_odom=%s imu=%s cmd_vel=%s",
                      self.topic_odom, self.topic_imu, self.topic_cmd)

    # --------- Callbacks ----------
    def cb_cmd_vel(self, msg):
        self.v = msg.linear.x
        self.omega = msg.angular.z

    def cb_wheel_odom(self, msg):
        try:
            self.v = msg.twist.twist.linear.x
            self.omega = msg.twist.twist.angular.z
        except Exception:
            pass

        self._predict_to_time(msg.header.stamp)

        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        z = np.array([[px], [py], [yaw]])
        self.update_odom(z)

    def cb_imu(self, msg):
        self._predict_to_time(msg.header.stamp)
        q = msg.orientation
        yaw = self._yaw_from_quat(q)
        if yaw is None:
            return
        z = np.array([[yaw]])
        self.update_imu_yaw(z)

    # --------- EKF Core ----------
    def _predict_to_time(self, stamp):
        t = stamp.to_sec() if stamp and stamp.to_sec() > 0 else rospy.Time.now().to_sec()
        if self.last_time is None:
            self.last_time = t
            return
        dt = t - self.last_time
        if dt <= 0:
            return

        th = float(self.x[2, 0])
        v = self.v
        w = self.omega

        # State prediction
        self.x[0, 0] += v * dt * math.cos(th)
        self.x[1, 0] += v * dt * math.sin(th)
        self.x[2, 0] = normalize_angle(self.x[2, 0] + w * dt)

        # Jacobian
        F = np.array([
            [1.0, 0.0, -v*dt*math.sin(th)],
            [0.0, 1.0,  v*dt*math.cos(th)],
            [0.0, 0.0,  1.0]
        ])

        Qd = self.Q.copy()
        Qd[0, 0] += abs(v) * dt + 1e-9
        Qd[1, 1] += abs(v) * dt + 1e-9
        Qd[2, 2] += abs(w) * dt + 1e-9

        self.P = F.dot(self.P).dot(F.T) + Qd
        self.last_time = t

        self.publish_odom(stamp=rospy.Time.from_sec(t))

    def update_odom(self, z):
        H = np.eye(3)
        y = z - self.x
        y[2, 0] = normalize_angle(float(y[2, 0]))
        S = H.dot(self.P).dot(H.T) + self.R_odom
        K = self.P.dot(H.T).dot(np.linalg.inv(S))
        self.x = self.x + K.dot(y)
        self.x[2, 0] = normalize_angle(self.x[2, 0])
        I = np.eye(3)
        self.P = (I - K.dot(H)).dot(self.P)
        self.publish_odom()

    def update_imu_yaw(self, z):
        H = np.array([[0.0, 0.0, 1.0]])
        y = z - np.array([[self.x[2, 0]]])
        y[0, 0] = normalize_angle(float(y[0, 0]))
        S = H.dot(self.P).dot(H.T) + self.R_imu
        K = self.P.dot(H.T).dot(np.linalg.inv(S))
        self.x = self.x + K.dot(y)
        self.x[2, 0] = normalize_angle(self.x[2, 0])
        I = np.eye(3)
        self.P = (I - K.dot(H)).dot(self.P)
        self.publish_odom()

    # --------- Helpers ----------
    def _yaw_from_quat(self, q):
        if (q.w, q.x, q.y, q.z) == (0.0, 0.0, 0.0, 0.0):
            return None
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def publish_odom(self, stamp=None):
        msg = Odometry()
        msg.header.stamp = stamp if stamp is not None else rospy.Time.now()
        msg.header.frame_id = self.frame_id
        msg.child_frame_id = self.child_frame_id
        msg.pose.pose.position.x = float(self.x[0, 0])
        msg.pose.pose.position.y = float(self.x[1, 0])
        q = quaternion_from_euler(0.0, 0.0, float(self.x[2, 0]))
        msg.pose.pose.orientation = Quaternion(*q)

        P = self.P
        cov = [0.0] * 36
        cov[0] = float(P[0, 0])
        cov[1] = float(P[0, 1])
        cov[5] = float(P[0, 2])
        cov[6] = float(P[1, 0])
        cov[7] = float(P[1, 1])
        cov[11] = float(P[1, 2])
        cov[30] = float(P[2, 0])
        cov[31] = float(P[2, 1])
        cov[35] = float(P[2, 2])
        msg.pose.covariance = cov

        self.pub.publish(msg)

        if self.publish_tf and self.br is not None:
            self.br.sendTransform(
                (msg.pose.pose.position.x, msg.pose.pose.position.y, 0.0),
                (q[0], q[1], q[2], q[3]),
                msg.header.stamp,
                self.child_frame_id,
                self.frame_id
            )


def main():
    rospy.init_node("state_estimator")
    ekf = DiffDriveEKF()
    rospy.loginfo("state_estimator (EKF) running.")
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
