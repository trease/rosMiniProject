#include <iostream>
#include <math.h>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/LaserScan.h"
#include "kobuki_msgs/BumperEvent.h"
#include "std_srvs/SetBool.h"

class Seeker {
 public:

  Seeker();
  ~Seeker();

  //function to spin the turtle in place
  //speed changes the rate of the turn
  void turtleSpin(float speed);

  //writes the provided x and y values in the form of a vector3 message to the displaement topic
  void writeDisplacement(float x, float y);
  
  // function to move the turtle forward and make slight course adjustments
  // speed is the forward speed only, has no affect on turning speed
  // dir is the direction of the turn, 0=forward, 1=right, 2=left
  void turtleForward(float speed, int dir);

  // function to stop the turtle
  void stopTurtle();

  // message and service callbacks
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
  void bumpCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg);
  bool srvCallback(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response);

  //basic getters
  bool getBumpFlag();
  bool getFoundFlag();
  int getDirFlag();
  bool getEnableFlag();
  
 private:

  bool bumpFlag;
  bool foundFlag;
  int dirFlag;  // 0 = straigt, 1 = right, 2 = left
  bool enableFlag;

};
