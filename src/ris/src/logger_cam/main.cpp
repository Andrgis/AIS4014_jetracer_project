#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <sys/stat.h>

// Directory for saving images
const std::string log_dir = "/workspace/ros_logs/camera/";
std::ofstream csv_file;

// Function to create a directory if it doesn’t exist
void createDirectory(const std::string& path) {
    struct stat info;
    if (stat(path.c_str(), &info) != 0) {
        ROS_INFO("Creating directory: %s", path.c_str());
        mkdir(path.c_str(), 0777);
    }
}

// Get timestamp for filenames
std::string getTimestamp(const ros::Time& stamp) {
    std::ostringstream oss;
    oss << stamp.sec << "_" << std::setw(9) << std::setfill('0') << stamp.nsec;
    return oss.str();
}

// Callback function for image processing
void imageCallback(const sensor_msgs::Image::ConstPtr& msg) {
    // Convert ROS Image message to OpenCV format
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Generate filename
    std::string timestamp = getTimestamp(msg->header.stamp);
    std::string image_filename = log_dir + "image_" + timestamp + ".jpg";

    // Save image
    cv::imwrite(image_filename, cv_ptr->image);

    // Log metadata in CSV
    if (csv_file.is_open()) {
        csv_file << msg->header.seq << "," << msg->header.stamp.sec << "." << msg->header.stamp.nsec << ","
                 << image_filename << "\n";
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "camera_logger");
    ros::NodeHandle nh;
    ROS_INFO("Camera logger node started");

    // Create the directory for storing images
    createDirectory(log_dir);

    // Open CSV file for logging metadata
    csv_file.open(log_dir + "camera_data.csv");
    if (!csv_file.is_open()) {
        ROS_ERROR("Failed to open CSV file for writing!");
        return 1;
    }

    // Write CSV header
    csv_file << "Seq,Timestamp,ImageFilename\n";

    // Subscribe to the camera topic (change if needed)
    ros::Subscriber image_sub = nh.subscribe("/camera/image_raw", 10, imageCallback);

    ros::spin();

    csv_file.close();  // Close file when node shuts down
    return 0;
}
