
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <cmath>
#include <algorithm>
#include <sstream>
#include <geometry_msgs/Vector3Stamped.h>
double qe1 = 0;
 double qe2 = 0;
 double dt = 0;
 double dx = 0;
 double dy = 0;
 double dth = 0;
 double dist = 0;
 double x = 0.0;
 double y = 0.0;
 double th = 0.0;
 double vx = 0.0;
 double vy = 0.0;
 double vth = 0.0;
 double dxp, dyp, vartheta, rcurv;
 double rate=20.0;
 ros::Time current_time, last_time;
 ros::Publisher odom_pub;

void speedCallback(const geometry_msgs::Vector3Stamped& speed_msg){
//  tf::TransformBroadcaster odom_broadcaster;

vx=speed_msg.vector.x;
vy=speed_msg.vector.y;
vth=speed_msg.vector.z;
}
void positionCallback(const geometry_msgs::Vector3Stamped& position_msg){
tf::TransformBroadcaster odom_broadcaster;
current_time = ros::Time::now();
last_time= ros::Time::now();
x=position_msg.vector.x;
y=position_msg.vector.y;
th=position_msg.vector.z;
   geometry_msgs::Quaternion odom_quat =tf::createQuaternionMsgFromYaw(th);
   dt =(current_time-last_time).toSec(); //calc velocities

   nav_msgs::Odometry odom; //create nav_msgs::odometry 
   odom.header.stamp = current_time;
   odom.header.frame_id = "odom";
 
   odom.pose.pose.position.x = x; //set positions 
   odom.pose.pose.position.y = y;
   odom.pose.pose.position.z = 0.0;
   odom.pose.pose.orientation = odom_quat;
 
   odom.child_frame_id = "base_link"; // set child frame and set velocity in twist message
   odom.twist.twist.linear.x =vx;
   odom.twist.twist.linear.y =vy;
   odom.twist.twist.angular.z =vth;
 //ROS_INFO("%d", msg.data.c_str());

   odom_pub.publish(odom);  //publish odom message
   last_time = current_time;
 //ROS_INFO("Received [%d]",msg->data)
}
 //we need a quaternion to describe rotation in 3d
 
int main(int argc, char **argv)
{

  ros::init(argc, argv, "testing");
  tf::TransformBroadcaster odom_broadcaster;
  
  ros::NodeHandle nh;
  
  odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
  ros::Subscriber speed = nh.subscribe("Speed", 50, speedCallback);
  ros::Subscriber position = nh.subscribe("Position", 50, positionCallback);
  current_time = ros::Time::now();
  last_time= ros::Time::now();

    


 
 ros::spin();

 


  return 0;
}
// %EndTag(FULLTEXT)%

