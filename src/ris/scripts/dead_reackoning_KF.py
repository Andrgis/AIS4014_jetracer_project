#!/usr/bin/env python
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion

class KalmanFilterNode:
    def __init__(self):
        rospy.init_node('kalman_filter_node')
        self.pub = rospy.Publisher('/odom_kalman', Odometry, queue_size=10)

        # State: [x, y, theta]
        self.x = np.zeros((3,1))
        self.P = np.eye(3) * 0.1

        # Process noise (tunable)
        self.Q = np.diag([0.01, 0.01, 0.005])

        # Last timestamps and measurements
        self.last_time = None
        self.latest_imu_omega = 0.0

        # Subscribers
        rospy.Subscriber('/imu', Imu, self.imu_cb)
        rospy.Subscriber('/odom_raw', Odometry, self.odom_raw_cb)

        rospy.loginfo("Kalman Filter Node started")
        rospy.spin()

    def imu_cb(self, msg):
        # use IMU yaw rate as our best omega
        self.latest_imu_omega = msg.angular_velocity.z

    def odom_raw_cb(self, msg):
        t = msg.header.stamp.to_sec()
        if self.last_time is None:
            self.last_time = t
            return

        dt = t - self.last_time
        self.last_time = t

        # 1) Prediction using velocity from odometry and omega from IMU
        v = msg.twist.twist.linear.x
        omega = self.latest_imu_omega
        self.predict(v, omega, dt)

        # 2) Measurement update with raw pose (x, y, yaw)
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        quat = msg.pose.pose.orientation
        # convert quaternion to yaw
        _, _, yaw =  euler_from_quaternion(
            [quat.x, quat.y, quat.z, quat.w]
        )
        z = np.array([[px], [py], [yaw]])

        # build measurement covariance R from the incoming message
        cov = msg.pose.covariance  # list of 36
        R = np.diag([
            cov[0],    # x variance
            cov[7],    # y variance
            cov[35]    # yaw variance
        ])

        self.update(z, R)

        # 3) Publish filtered pose
        self.publish(msg.header.stamp, msg.header.frame_id)

    def predict(self, v, omega, dt):
        theta = self.x[2,0]
        # State transition
        # x' = x + v*cos(theta)*dt
        # y' = y + v*sin(theta)*dt
        # theta' = theta + omega*dt
        F = np.array([
            [1, 0, -v * np.sin(theta) * dt],
            [0, 1,  v * np.cos(theta) * dt],
            [0, 0,  1]
        ])
        self.x += np.array([
            [v * np.cos(theta) * dt],
            [v * np.sin(theta) * dt],
            [omega * dt]
        ])
        self.P = F.dot(self.P).dot(F.T) + self.Q

    def update(self, z, R):
        # H: we measure x, y, theta directly
        H = np.eye(3)
        y = z - H.dot(self.x)
        S = H.dot(self.P).dot(H.T) + R
        K = self.P.dot(H.T).dot(np.linalg.inv(S))
        self.x += K.dot(y)
        self.P = (np.eye(3) - K.dot(H)).dot(self.P)

    def publish(self, stamp, frame_id):
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = frame_id
        # pose
        odom.pose = PoseWithCovariance()
        odom.pose.pose.position.x = self.x[0,0]
        odom.pose.pose.position.y = self.x[1,0]
        q = quaternion_from_euler(0, 0, self.x[2,0])
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        # TODO: optionally fill pose.covariance from self.P
        # twist (we publish nothing here)
        odom.twist = TwistWithCovariance()
        self.pub.publish(odom)

if __name__ == '__main__':
    try:
        KalmanFilterNode()
    except rospy.ROSInterruptException:
        pass
