#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <fstream>


// Open a CSV file for writing
std::ofstream csv_file("imu_data.csv");

// Callback function to process IMU data
void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    if (csv_file.is_open()) {
        csv_file << msg->header.seq << ","  // Sequence number
                 << msg->header.stamp.sec << "." << msg->header.stamp.nsec << ","  // Timestamp
                 << msg->orientation.x << "," << msg->orientation.y << ","
                 << msg->orientation.z << "," << msg->orientation.w << ","
                 << msg->angular_velocity.x << "," << msg->angular_velocity.y << ","
                 << msg->angular_velocity.z << ","
                 << msg->linear_acceleration.x << "," << msg->linear_acceleration.y << ","
                 << msg->linear_acceleration.z << "\n";
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "imu_logger");
    ros::NodeHandle nh;
    ROS_INFO("IMU logger node started");
    // Write CSV header
    if (csv_file.is_open()) {
        csv_file << "Seq,Timestamp,Orientation_x,Orientation_y,Orientation_z,Orientation_w,"
                    "AngularVelocity_x,AngularVelocity_y,AngularVelocity_z,"
                    "LinearAcceleration_x,LinearAcceleration_y,LinearAcceleration_z\n";
    } else {
        ROS_ERROR("Failed to open CSV file for writing!");
        return 1;
    }

    // Subscribe to the IMU topic
    ros::Subscriber imu_sub = nh.subscribe("/imu", 1000, imuCallback);

    ros::spin();  // Keep the node running

    csv_file.close();  // Close the file when the node shuts down
    return 0;
}
