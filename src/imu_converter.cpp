#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Vector3.h>
//#include <sensor_msgs/NavSatFix.h>
ros::Publisher imu_pub;
//ros::Publisher gps_pub;
float z=0.0;
float y=0.0;
float x=0.0;

/*void imu_datacallback(const geometry_msgs::Vector3& orient){
   
   z=orient.z;
   y=orient.y;
   x=orient.x;

}*/
void imu_datacallback(const sensor_msgs::Imu::ConstPtr& imu_msgs){
 ros::Time current_time = ros::Time::now();

 sensor_msgs::Imu imu;
 imu.header.stamp = current_time;
 imu.header.frame_id = "base_link";
// imu.child_frame_id="imu_data";
 imu.orientation.x = imu_msgs->orientation.x;
 imu.orientation.y = imu_msgs->orientation.y;
 imu.orientation.z = imu_msgs->orientation.z;
 imu.orientation.w = imu_msgs->orientation.w;
 imu.angular_velocity.x = imu_msgs->angular_velocity.x;
 imu.angular_velocity.y = imu_msgs->angular_velocity.y;
 imu.angular_velocity.z = imu_msgs->angular_velocity.z;
 imu.linear_acceleration.x = imu_msgs->linear_acceleration.x;
 imu.linear_acceleration.y = imu_msgs->linear_acceleration.y;
 imu.linear_acceleration.z = imu_msgs->linear_acceleration.z;
 ROS_INFO("quaternion z= %f",imu.orientation.z);
 ROS_INFO("quaternion w= %f",imu.orientation.w);
 imu_pub.publish(imu);
}
/*void gps_datacallback2(const sensor_msgs::NavSatFix::ConstPtr& gps_msgs){
 ros::Time current_time = ros::Time::now();

 sensor_msgs::NavSatFix gps;
 gps.header.stamp = current_time;
 gps.header.frame_id = "base_link";
 gps.latitude = gps_msgs->latitude;
 gps.longitude = gps_msgs->longitude;
 gps.altitude = gps_msgs->altitude;
 gps.position_covariance_type= gps_msgs->position_covariance_type;

 gps_pub.publish(gps);

}*/
int main(int argc, char** argv){
 ros::init(argc, argv, "IMU_publishers");
 ros::NodeHandle nh; 
//  gps_pub = nh.advertise<sensor_msgs::NavSatFix>("gps_datas", 50);
   imu_pub = nh.advertise<sensor_msgs::Imu>("imu_data", 50);
 // ros::Subscriber gps_sub1 = nh.subscribe("android/fix",10,gps_datacallback2);

//  ros::Subscriber imu_sub = nh.subscribe("imu_data",10,imu_datacallback);
  ros::Subscriber imu_sub = nh.subscribe("imu_msgs",10,imu_datacallback);
 ros::spin();
 return 0;
}
