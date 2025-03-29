#include <ros/ros.h> // Ros library
#include <ris_msgs/Dog_data.h> // For writing a standard string msg

int main(int argc, char **argv){
  ros::init(argc, argv, "Publisher"); // Initiating ros node
  ros::NodeHandle node; // Declaring the node handle

  // Declaring the publisher node using the node handle. Topic name: "learning_publiser". Max number of msg: 100.
  ros::Publisher topic_publisher = node.advertise<ris_msgs::Dog_data>("learning_topic", 100);
  ros::Rate loop_rate(2); // setting how often the publisher should publish

  while(ros::ok()){
    ris_msgs::Dog_data dog; // Declaring message
    dog.name = "Rocky"; // Setting value for the message
    dog.breed = "Bishon Havanais";
    dog.age = 12;
    topic_publisher.publish(dog); // Publishing message to topic
    ros::spinOnce(); // Ros spins ???
    loop_rate.sleep(); // Sleeps for 1 second before looping again
  }
}
