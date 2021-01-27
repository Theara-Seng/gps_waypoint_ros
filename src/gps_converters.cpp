#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Vector3.h>
ros::Publisher gps_pub;
float z=0.0;
float y=0.0;
float x=0.0;


void gps_datacallback2(const sensor_msgs::NavSatFix::ConstPtr& gps_msgs){
 ros::Time current_time = ros::Time::now();

 sensor_msgs::NavSatFix gps;
 gps.header.stamp = current_time;
 gps.header.frame_id = "gps_data";
 //gps.child_frame_id="gps_data";
 gps.latitude = gps_msgs->latitude;
 gps.longitude = gps_msgs->longitude;
 gps.altitude = gps_msgs->altitude;
 gps.position_covariance_type=gps_msgs->position_covariance_type;

 gps_pub.publish(gps);
}

int main(int argc, char** argv){
 ros::init(argc, argv, "gps_publishers");
 ros::NodeHandle nh; 
 gps_pub = nh.advertise<sensor_msgs::NavSatFix>("/gps_datas", 50);
  ros::Subscriber imu_sub1 = nh.subscribe("android/fix",10,gps_datacallback2);
 ros::spin();
 return 0;
}
