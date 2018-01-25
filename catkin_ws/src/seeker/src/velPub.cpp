#include "ros/ros.h"
#include "geometry_msgs/Twist.h"



int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 1000);

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
    {

      //ROS_INFO("%s", msg.data.c_str());

      geometry_msgs::Twist msg;
      msg.linear.x = 0.0;
      msg.linear.y = 0.0;
      msg.linear.z = 0.0;
      msg.angular.x = 0.0;
      msg.angular.y = 0.0;
      msg.angular.z = 5.0;
      
      vel_pub.publish(msg);

      ros::spinOnce();

      loop_rate.sleep();
      ++count;
    }


  return 0;
}
