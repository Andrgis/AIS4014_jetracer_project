#include <ros/ros.h> // Ros library
#include <std_msgs/String.h> // For writing a standard string msg


int main(int argc, char **argv){
  ros::init(argc, argv, "Publisher"); // Initiating ros node
  ros::NodeHandle node; // Declaring the node handle

  // Declaring the publisher node using the node handle. Topic name: "learning_publiser". Max number of msg: 100.
  ros::Publisher topic_publisher = node.advertise<std_msgs::String>("learning_publisher", 100);
  ros::Rate loop_rate(1); // setting how often the publisher should publish

  while(ros::ok()){
    std_msgs::String msg; // Declaring message
    msg.data = "Andreas is the ROS champ!"; // Setting value for the message
    topic_publisher.publish(msg); // Publishing message to topic
    ros::spinOnce(); // Ros spins ???
    loop_rate.sleep(); // Sleeps for 1 second before looping again
  }
}