#include "interview.h"



Seeker::Seeker(){
  bumpFlag = false;
  foundFlag = false;
  dirFlag = 0;
  enableFlag = false;
}
Seeker::~Seeker(){
}


//function to spin the turtle in place
//speed changes the rate of the turn

void Seeker::turtleSpin(float speed){

  ros::NodeHandle n;
  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 1000);
  geometry_msgs::Twist msg;
  
    msg.linear.x = 0.0;
    msg.linear.y = 0.0;
    msg.linear.z = 0.0;
    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = speed;
    
    vel_pub.publish(msg);

    ros::spinOnce();
}

void Seeker::writeDisplacement(float x, float y){
  ros::NodeHandle n;
  ros::Publisher displacement = n.advertise<geometry_msgs::Vector3>("displacement", 1000);
  geometry_msgs::Vector3 msg;
  
  msg.x = x;
  msg.y = y;
  msg.z = 0.0;
    
    displacement.publish(msg);

    ros::spinOnce();
}


// function to move the turtle forward and make slight course adjustments
// speed is the forward speed only, has no affect on turning speed
// dir is the direction of the turn, 0=forward, 1=right, 2=left

void Seeker::turtleForward(float speed, int dir){

  ros::NodeHandle n;
  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 1000);
  geometry_msgs::Twist msg;
  
    msg.linear.x = speed;
    msg.linear.y = 0.0;
    msg.linear.z = 0.0;
    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = 0.0;
    if (dir == 0 ){
      msg.angular.z = 0.0;
      msg.linear.y = 0.0;
    }
    else if (dir == 1){
      msg.angular.z = 2.0;
      msg.linear.y = 10.0;
    }
    else if (dir == 2){
      msg.angular.z = -2.0;
      msg.linear.y = -10.0;
    }
    
    vel_pub.publish(msg);

    ros::spinOnce();
}


void Seeker::stopTurtle(){ // function to stop the turtle

  ros::NodeHandle n;
  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 1000);
  geometry_msgs::Twist msg;
  
  msg.linear.x = 0.0;
  msg.linear.y = 0.0;
  msg.linear.z = 0.0;
  msg.angular.x = 0.0;
  msg.angular.y = 0.0;
  msg.angular.z = 0.0;
  
  vel_pub.publish(msg);
  ros::spinOnce();
}

bool Seeker::getBumpFlag(){
  return bumpFlag;
}

bool Seeker::getFoundFlag(){
  return foundFlag;
}

int Seeker::getDirFlag(){
  return dirFlag;
}

bool Seeker::getEnableFlag(){
  return enableFlag;
}

void Seeker::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
  if(enableFlag){
    //std::cout << "----------------- scan called --------------"  << std::endl;
    float centerRange = 0;
    int cCount = 0;
    float maxRange = 0;
    int maxPos = 0;
    
    for ( int i=200; i < 400; i++){ // pulls a specific part (range of values) from the scan data and averages all valid data points, right,  left, and center data to enable couse correction when moving
      if ( msg->ranges[i] < msg->range_max && msg->ranges[i] > msg->range_min) {
	centerRange = centerRange + msg->ranges[i];
	if (msg->ranges[i] > maxRange){
	  maxRange = msg->ranges[i];
	  maxPos = i;
	}
	cCount++;
      }
    }
    centerRange = centerRange/cCount;

    float leftRange = 0;
    int lCount = 0;
    for ( int i=90; i < 150; i++){
      if ( msg->ranges[i] < msg->range_max && msg->ranges[i] > msg->range_min) {
	leftRange = leftRange + msg->ranges[i];
	lCount++;
      }
    }
    leftRange = leftRange/lCount;

    float rightRange = 0;
    int rCount = 0;
    for ( int i=490; i < 550; i++){
      if ( msg->ranges[i] < msg->range_max && msg->ranges[i] > msg->range_min) {
	rightRange = rightRange + msg->ranges[i];
	rCount++;
      }
    }
    rightRange = rightRange/rCount;

    //std::cout << rightRange << "  " << centerRange << " " << leftRange << " " << msg->ranges[300] << std::endl;
    if ( centerRange > msg->range_min || foundFlag) {
      //std::cout << "found object" << std::endl;

      if(isnan(rightRange)) // corrects bad data values
	rightRange = -10;
      if(isnan(leftRange))
	leftRange = -10;

      if(centerRange < 0.2 || bumpFlag){ // case for once ball is found to stop turtlebot
	stopTurtle();
      }
      else if(rightRange > 0.1){ // if the ball is in the right side of the scan then turn left
	turtleForward(1.0, 1);
	//std::cout << "right" << std::endl;
	dirFlag = 2;
      }
      else if(leftRange > 0.1){ // if the ball is in the left side of the scan then turn right
	turtleForward(1.0, 2);
	//std::cout << "left" << std::endl;
	dirFlag = 1;
      }
      else if (centerRange > 0.1){ // if the ball is only in the center of the scan then move rapidly towards the ball
	turtleForward(1.0, 0);
	dirFlag = 0;
      }
      else if (dirFlag == 1){ // additional case if turtle overcorrects so the ball is out of the the sensors view, turtle rememebers the last turn and adjusts
	turtleForward(0.5, 2);
	//std::cout << "mem right" << std::endl;
      }
      else if (dirFlag == 2){
	turtleForward(0.5, 1);
	//std::cout << "mem left" << std::endl;
      }
      else
	stopTurtle(); // defualt case
      
      foundFlag = true;
      
    }
    else{ // if turtlebot can't see the ball keep spinning in a circle 
      //std::cout << "empty space" << std::endl;
      turtleSpin(0.5);
    }

    if(bumpFlag)
      maxRange = 0;

    // displacement finds and publishes the distance from turtlebot to the ball
    // displacement is based off turtlebot with the x-axis being the front of the robot

    float xValue = 0.0;
    float yValue = 0.0;
    if(maxPos <= 300){
      xValue = maxRange * cos((300-maxPos)*msg->angle_increment);
      yValue = maxRange * sin((300-maxPos)*msg->angle_increment);// finds the distance off the center the ball is, multiplies that by the angle increment to find the angle
      // off center the ball is to find the y component of the vector.
    }
    else{
      xValue = maxRange * cos((maxPos-300)*msg->angle_increment);
      yValue = maxRange * sin((maxPos-300)*msg->angle_increment);
    }
    writeDisplacement(maxRange, yValue);
  }
  
}

void Seeker::bumpCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg){
  bool pressed = msg->PRESSED;
  foundFlag = true;
  if( pressed == true){
    bumpFlag = true;
  }
}

bool Seeker::srvCallback(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response){

  if ((bool)request.data == true)
    enableFlag = true;

  response.success = true;
  response.message = "Turtlebot should be searching for the ball now";
  
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");

  Seeker seeker;
  ros::NodeHandle n;
  
  ros::Publisher displacement = n.advertise<geometry_msgs::Vector3>("displacement", 1000);
  ros::Publisher velPub = n.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 1000);
  ros::Subscriber scanSub = n.subscribe("/scan", 1000, &Seeker::scanCallback, &seeker);
  ros::Subscriber bumpSub = n.subscribe("/mobile_base/events/bumper", 1000, &Seeker::bumpCallback, &seeker);
  ros::Rate loop_rate(10);

  ros::ServiceServer service = n.advertiseService("/enable", &Seeker::srvCallback, &seeker);


  while (ros::ok()){
    if(seeker.getEnableFlag()){
      if (seeker.getBumpFlag() == false && seeker.getFoundFlag() == false)
	seeker.turtleSpin(0.25);
    }
    ros::spin();
  }


  return 0;
}
