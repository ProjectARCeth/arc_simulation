#include "ros/ros.h"
#include "eRodGZInterface.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "GZInterface");
  ros::NodeHandle node;
  // Create a object of the GZInterface class with the name interface_object.
  GZInterface interface_object(node);
  ros::spin();

  return 0;
}
