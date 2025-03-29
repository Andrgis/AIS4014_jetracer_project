#include <ros/ros.h> // Ros library
#include <ris_msgs/Addition_service_node.h> // For writing a standard string msg

bool addition(ris_msgs::Addition_service_node::Request &req, ris_msgs::Addition_service_node::Response &res){
  res.sum = req.a + req.b;
  ROS_INFO("Addition sum is %li", res.sum);
  return true;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "Service_node"); // Initiating ros node
  ros::NodeHandle node; // Declaring the node handle

  ros::ServiceServer service_server = node.advertiseService("addition_service", addition);
  ROS_INFO("Service node started.");

  ros::spin(); // Checks for update?
}