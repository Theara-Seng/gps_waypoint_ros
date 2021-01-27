
#define USE_USBCON
#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif
#include <math.h>
#include <ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>

#include <Encoder.h>

#include "kinematic.h"
#include "robot_specs.h"
#include "pid.h"
#include "motor.h"

#define a 0.2
#define b 0.2

//
//motor and encoder

Motor Motor3(5, 6, 28, 26);
Motor Motor4(4, 3, 22, 24);

long enc_value1 = Motor3.enc.read();
long enc_value2 = Motor4.enc.read();


long enc_last_value1 = enc_value1;
long enc_last_value2 = enc_value2;

float l1 = 0.0;
float l2 = 0.0;

float dl1 = 0.0;
float dl2 = 0.0;

float v1 = 0.0;
float v2 = 0.0;

float v_target3 = 0.0;
float v_target4 = 0.0;

float v_motor3 = 0.0;
float v_motor4 = 0.0;

float cmd1 = 0.0;
float cmd2 = 0.0;





double Vx = 0.0;
double Vy = 0.0;
double Omega = 0.0;
double gain = 2.2;
double gain1 = 5.0;
unsigned long last_time = millis();
unsigned long dt = 0.0;
pid pid1(100.0, 2.0, 0.0);
pid pid2(100.0, 2.0, 0.0);


void handle_cmd( const geometry_msgs::Twist& msg) {
  Vx = msg.linear.x;
  Vy = msg.linear.y;
   Omega = msg.angular.z;
       v_target3 =gain*calculateV3(Vx,Vy,Omega,a,b) ;
       v_target4  = gain*calculateV4(Vx,Vy,Omega,a,b) ;

}

ros::NodeHandle nh;

ros::Subscriber<geometry_msgs::Twist> speeds("cmd_vel", handle_cmd);
geometry_msgs::Vector3Stamped pose_msg;
geometry_msgs::Vector3Stamped speed_msg;
ros::Publisher pose_pub("position", &pose_msg);
ros::Publisher speed_pub("speed_motor_back", &speed_msg);


void setup() {

   Serial.begin(57600);
  for (int i = 3; i <= 10; i++) {
    pinMode(i, OUTPUT);
  }

  Motor3.initPins();
  Motor4.initPins();
 

  nh.initNode();
  nh.subscribe(speeds);
  nh.advertise(pose_pub);
  nh.advertise(speed_pub);
}

void loop() {

  dt = millis() - last_time;
  if (dt >= 50) {
    last_time = millis();
    enc_value1 = Motor3.enc.read();
    enc_value2 = Motor4.enc.read();

    dl1 = 3.14 * 0.117 * (enc_value1 - enc_last_value1) / 19.2 / 14.0 / 2.0;
    v1 =  1000 * dl1 / dt;
    enc_last_value1 = enc_value1;
    dl2 =  3.14 * 0.117 * (enc_value2 - enc_last_value2) / 19.2 / 14.0 / 2.0;
    v2 = 1000 * dl2 / dt;
    enc_last_value2 = enc_value2;

    l1 += v1 * dt / 1000.0;
    l2 += v2 * dt / 1000.0;
    v_motor3 = pid1.calculate(v_target3, v1, dt);
    v_motor4 = pid2.calculate(v_target4, v2, dt);
    Motor3.commandMotor(v_motor3);
    Motor4.commandMotor(v_motor4);
    Serial.print("v_target1=");
    Serial.print(v_target3);
    Serial.print(";");
    Serial.print("v_target2=");
    Serial.print(v_target4);
    Serial.print(";");

    Serial.print("vmotor1=");
    Serial.print(v1);
    Serial.print(";");
    Serial.print("vmotor2=");
    Serial.print(v2);
    Serial.print(";");
    Serial.print("l1=");
    Serial.print(l1);
    Serial.print(";");
    Serial.print("l2=");
    Serial.println(l2);
    speed_msg.vector.x=v1;
    speed_msg.vector.y=v2;
    pose_msg.vector.x=l1;
    pose_msg.vector.y=l2;
    pose_pub.publish(&pose_msg);
    speed_pub.publish(&speed_msg);
    
  }
   



  nh.spinOnce();
  delay(10);



}
