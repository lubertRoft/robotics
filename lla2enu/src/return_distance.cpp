#include "ros/ros.h"
//#include "ros/console.h"
#include "lla2enu/ReturnDistance.h"

bool distance(//lla2enu::ReturnDistance::Request &req,
              lla2enu::ReturnDistance::Response &res)
{
  //ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  ROS_INFO("sending back response: [%ld]", (long int)res.distance);
  return true;  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "return_distance");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("return_distance", distance); //name which will be displayed in the terminal
  ROS_INFO("Ready to add two ints.");
  ros::spin();

  return 0;
}