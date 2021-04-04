#include "ros/ros.h"
#include "geometry_msgs/Vector3Stamped.h"
#include <geometry_msgs/Vector3.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <testing_cpp/Speed.h>
//void MotorBackCallback(const std_msgs::Int8::ConstPtr& msg){
   
// 	ROS_INFO("The value is [%d]", msg->data);

//}
	double dt=0.0;
	float V1=0.0;
	float V2=0.0;
	float V3=0.0;
	float V4=0.0;

	float Vx=0.0;
	float Vy=0.0;
	float Omega=0.0;

	float x=0.0;
	float y=0.0;
	float th=0.0;
	float roll=0.0;
	float pitch=0.0;
	float yaw=0.0;
	const float a=0.4;
	const float b=0.4;

void SpeedCallback(const testing_cpp::Speed& speed_msg)
{
       V1=speed_msg.v1;
       V2=speed_msg.v2;
       ROS_INFO("Speed Motor 1= %f", V1);
       ROS_INFO("Speed Motor 2= %f", V2);
       
}
/*void angleCallback(const geometry_msgs::Vector3& orient)
{
	roll=orient.x;
	pitch=orient.y;
	yaw=orient.z;
}*/
float calculateVx( float v1, float v2, float v3, float v4, float a, float b) {
  return (1.0/4.0)*(v1+v2+v3+v4);
}


float calculateVy( float v1, float v2, float v3, float v4, float a, float b) {
  return (1.0/4.0)*(-v1+v2+v3-v4);
}


float calculateOmega( float v1, float v2, float v3, float v4, float a,  float b) {
  return  (1.0/(4.0))*(-v1+v2-v3+v4);
}




int main(int argc, char **argv){

ros::init(argc,argv,"encoder_subscriber");
tf::TransformBroadcaster odom_broadcaster;
ros::NodeHandle nh;
ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
//ros::Subscriber Speed_motor_back=nh.subscribe("speed_motor_back",50,SpeedMotorBackCallback);
//ros::Subscriber Position_motor_back=nh.subscribe("position_motor_back",1000,PoseMotorBackCallback);
ros::Subscriber Speed=nh.subscribe("speed",50,SpeedCallback);
//ros::Subscriber theta_robot=nh.subscribe("imu_data",50, angleCallback);
//ros::Subscriber Position_motor_front=nh.subscribe("position_motor_front",1000,PoseMotorFrontCallback);
 
	ros::Time current_time;
	ros::Time last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();


	tf::TransformBroadcaster broadcaster;
	ros::Rate loop_rate(20);

	const double degree = M_PI/180;

	// message declarations
	geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link"; 

  while(nh.ok()){

       ros::spinOnce();
       current_time = ros::Time::now();
       dt=(current_time-last_time).toSec();
  
       Vx=(1.0/2.0)*(V1+V2)*cos(th);
       Vy=(1.0/2.0)*(V1+V2)*sin(th);
       Omega=(1.0/((a+b)*(2.0)))*(-V1+V2);
       x+=Vx*dt;
       y+=Vy*dt;
       th+=Omega*dt;
       
       geometry_msgs::Quaternion odom_quat;
       odom_quat=tf::createQuaternionMsgFromYaw(th);
       
       //update transform
       odom_trans.header.stamp = current_time; 
       odom_trans.transform.translation.x = x; 
       odom_trans.transform.translation.y = y; 
       odom_trans.transform.translation.z = 0.0;
       odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(th);
       
       
       nav_msgs::Odometry odom; //create nav_msgs::odometry 
       odom.header.stamp = current_time;
       odom.header.frame_id = "odom";
       odom.child_frame_id = "base_link"; // set child frame and set velocity in twist message
       
       odom.pose.pose.position.x = x; //set positions 
       odom.pose.pose.position.y = y;
       odom.pose.pose.position.z = 0.0;
       odom.pose.pose.orientation = odom_quat;
       

       odom.twist.twist.linear.x =Vx;
       odom.twist.twist.linear.y =0.0;
       odom.twist.twist.angular.z =Omega; 
       last_time = current_time;
       
      // broadcaster.sendTransform(odom_trans);
       odom_pub.publish(odom);  //publish odom message
       loop_rate.sleep();
  }

return 0;

}
