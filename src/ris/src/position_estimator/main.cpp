#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <fstream>

// Open a CSV file for writing
std::ofstream csv_file("odom_data.csv");

// Callback function to process odometry data
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    if (csv_file.is_open()) {
        csv_file << msg->header.seq << ","  // Sequence number
                 << msg->header.stamp.sec << "." << msg->header.stamp.nsec << ","  // Timestamp
                 << msg->pose.pose.position.x << "," << msg->pose.pose.position.y << "," << msg->pose.pose.position.z << ","
                 << msg->pose.pose.orientation.x << "," << msg->pose.pose.orientation.y << ","
                 << msg->pose.pose.orientation.z << "," << msg->pose.pose.orientation.w << ","
                 << msg->twist.twist.linear.x << "," << msg->twist.twist.linear.y << "," << msg->twist.twist.linear.z << ","
                 << msg->twist.twist.angular.x << "," << msg->twist.twist.angular.y << "," << msg->twist.twist.angular.z << "\n";
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "odom_logger");
    ros::NodeHandle nh;
    ROS_INFO("Odometry logger node started");

    // Write CSV header
    if (csv_file.is_open()) {
        csv_file << "Seq,Timestamp,Position_x,Position_y,Position_z,"
                    "Orientation_x,Orientation_y,Orientation_z,Orientation_w,"
                    "LinearVelocity_x,LinearVelocity_y,LinearVelocity_z,"
                    "AngularVelocity_x,AngularVelocity_y,AngularVelocity_z\n";
    } else {
        ROS_ERROR("Failed to open CSV file for writing!");
        return 1;
    }

    // Subscribe to the odometry topic (change if needed)
    ros::Subscriber odom_sub = nh.subscribe("/odom", 1000, odomCallback);

    ros::spin();  // Keep the node running

    csv_file.close();  // Close the file when the node shuts down
    return 0;
}
