#include <ros/ros.h> // Ros library
#include <std_msgs/String.h> // For writing a standard string msg

void writeMsgToLog(const std_msgs::String::ConstPtr& msg){
  ROS_INFO("We have recieved and confirmed that: '%s'", msg->data.c_str());
}

int main(int argc, char **argv){
  ros::init(argc, argv, "Subscriber"); // Initiating ros node
  ros::NodeHandle node; // Declaring the node handle

  ros::Subscriber topic_subscriber = node.subscribe("learning_publisher", 100, writeMsgToLog);

  ros::spin(); // Checks for update?
}