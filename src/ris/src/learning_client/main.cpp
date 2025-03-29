#include <ros/ros.h> // Ros library
#include <ris_msgs/Addition_service_node.h> // For writing a standard string msg

int main(int argc, char **argv){
  ros::init(argc, argv, "Client_node"); // Initiating ros node
  ros::NodeHandle node; // Declaring the node handle

  ros::ServiceClient client = node.serviceClient<ris_msgs::Addition_service_node>("addition_service");

  ris_msgs::Addition_service_node srv;
  srv.request.a = 18;
  srv.request.b = 333;

  if (client.call(srv)){
    ROS_INFO("Addition service call succeeded. Result is %li", srv.response.sum);
  }
  else {
    ROS_INFO("Addition service call failed");
    return 1;
  }

  return 0;
}