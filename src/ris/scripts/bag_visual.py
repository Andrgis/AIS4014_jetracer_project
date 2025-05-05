#!/usr/bin/env python

import rosbag
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import numpy as np
import tf.transformations

bag = rosbag.Bag('/home/jetson/workspace/data/circle.bag')

odom_combined_x, odom_combined_y = [], []
odom_raw_x, odom_raw_y = [], []
lid_r, lid_a = [], []

# Read bag
for topic, msg, t in bag.read_messages():
    if topic == "/odom_combined":
        pos = msg.pose.pose.position
        odom_combined_x.append(pos.x)
        odom_combined_y.append(pos.y)

    elif topic == "/odom_raw":
        pos = msg.pose.pose.position
        odom_raw_x.append(pos.x)
        odom_raw_y.append(pos.y)

    elif topic == "/scan":
        lid_r.append(msg.ranges[0])
        lid_a.append(msg.angle_min + msg.angle_increment * msg.ranges[0])


bag.close()

# Plotting
plt.figure(figsize=(10, 8))
plt.plot(odom_combined_x, odom_combined_y, label="odom_combined", linestyle='--')
plt.plot(odom_raw_x, odom_raw_y, label="odom_raw", linestyle=':')


plt.xlabel("x position (m)")
plt.ylabel("y position (m)")
plt.title("Position Trajectories")
plt.legend()
plt.grid()
plt.axis('equal')
plt.tight_layout()
plt.show()
