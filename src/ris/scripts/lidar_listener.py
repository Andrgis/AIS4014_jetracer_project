#!/usr/bin/env

import rospy
import pandas as pd
from sensor_msgs.msg import LaserScan

# Initialize an empty DataFrame
columns = ['timestamp', 'angle_min', 'angle_max', 'angle_increment', 'range_min', 'range_max', 'ranges']
lidar_df = pd.DataFrame(columns=columns)

# Callback function to process incoming LiDAR data
def lidar_callback(msg):
    global lidar_df

    # Extract relevant data
    data = {
        'timestamp': rospy.Time.now().to_sec(),
        'angle_min': msg.angle_min,
        'angle_max': msg.angle_max,
        'angle_increment': msg.angle_increment,
        'range_min': msg.range_min,
        'range_max': msg.range_max,
        'ranges': list(msg.ranges)  # Store full range data as a list
    }

    # Append data to DataFrame
    lidar_df = pd.concat([lidar_df, pd.DataFrame([data])], ignore_index=True)

    # Save to CSV
    lidar_df.to_csv('data/lidar_data.csv', index=False)
    rospy.loginfo("LiDAR data saved.")

# ROS Node Initialization
def lidar_listener():
    rospy.init_node('lidar_data_logger', anonymous=True)
    rospy.loginfo("LiDAR data logger node initialized.")
    rospy.Subscriber('/scan', LaserScan, lidar_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        lidar_listener()
    except rospy.ROSInterruptException:
        pass
