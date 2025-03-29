#include <ros/ros.h> // Ros library
#include <ris_msgs/Dog_data.h> // For writing a standard string msg

void writeMsgToLog(const ris_msgs::Dog_data &dog_data){
  ROS_INFO("The ros dog's name is: %s", dog_data.name.c_str());
  ROS_INFO("The ros dog's breed is: '%s'", dog_data.breed.c_str());
  ROS_INFO("The ros dog's age is: %i years", dog_data.age);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "Subscriber"); // Initiating ros node
  ros::NodeHandle node; // Declaring the node handle

  ros::Subscriber topic_subscriber = node.subscribe("learning_topic", 100, writeMsgToLog);

  ros::spin(); // Checks for update?
}