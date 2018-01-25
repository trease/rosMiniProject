#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <iostream>
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  //ROS_INFO("I heard: [%s]", msg->range_min.c_str());
  float test = msg->range_min;
  //ROS_INFO(test);
  std::cout << test << std::endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "scanSub");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/scan", 1000, scanCallback);
  ros::spin();

  return 0;
}
