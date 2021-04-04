
#define USE_USBCON
#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif
#include <math.h>
#include <ros.h>
#include <testing_cpp/Speed.h>
#include <testing_cpp/Position.h>
#include <geometry_msgs/Twist.h>

#include <Encoder.h>

#include "kinematic.h"
#include "robot_specs.h"
#include "pid.h"
#include "motor.h"

#define a 0.4
#define b 0.4


//motor and encoder
Motor Motor1(4, 3, 28, 26);
Motor Motor2(5, 6, 24, 22);


long enc_value1 = Motor1.enc.read();
long enc_value2 = Motor2.enc.read();


long enc_last_value1 = enc_value1;
long enc_last_value2 = enc_value2;

float l1 = 0.0;
float l2 = 0.0;

float dl1 = 0.0;
float dl2 = 0.0;

float v1 = 0.0;
float v2 = 0.0;

float v_target1 = 0.0;
float v_target2 = 0.0;

float v_motor1 = 0.0;
float v_motor2 = 0.0;

float cmd1 = 0.0;
float cmd2 = 0.0;





double Vx = 0.0;
double Vy = 0.0;
double Omega = 0.0;
double gain = 12;
double gain1 = 11.5;
double gain2 =10.6;
unsigned long last_time = millis();
unsigned long dt = 0.0;
pid pid1(30.0, 2.0, 0.0);
pid pid2(30.0, 2.0, 0.0);


void handle_cmd( const geometry_msgs::Twist& msg) {
  Vx = msg.linear.x;
  Omega = msg.angular.z;
  if (Vx >= 0 ) {
    v_target1 = gain * (Vx + gain1 * (a + b) * Omega);
    v_target2  = gain1 * (Vx - gain1 * (a + b) * Omega);
  }
  else if (Vx < 0 ) {
    v_target1 = gain2 * (Vx + gain1 * (a + b) * Omega);
    v_target2  = gain * (Vx - gain1 * (a + b) * Omega);
  }
  else{
     v_target1 = gain * (Vx + gain1 * (a + b) * Omega);
    v_target2  = gain * (Vx - gain1 * (a + b) * Omega);
  }
}

ros::NodeHandle nh;

ros::Subscriber<geometry_msgs::Twist> speeds("cmd_vel", handle_cmd);
testing_cpp::Position pose_msg;
testing_cpp::Speed speed_msg;
ros::Publisher pose_pub("position", &pose_msg);
ros::Publisher speed_pub("speed", &speed_msg);


void setup() {

  Serial.begin(57600);
  for (int i = 3; i <= 10; i++) {
    pinMode(i, OUTPUT);
  }

  Motor1.initPins();
  Motor2.initPins();


  nh.initNode();
  nh.subscribe(speeds);
  nh.advertise(pose_pub);
  nh.advertise(speed_pub);
}

void loop() {

  dt = millis() - last_time;
  if (dt >= 50) {
    last_time = millis();
    enc_value1 = Motor1.enc.read();
    enc_value2 = Motor2.enc.read();

    dl1 = PI * 0.27 * (enc_value1 - enc_last_value1) / 1440.0 ;
    v1 =  1000 * dl1 / dt;
    enc_last_value1 = enc_value1;


    dl2 = PI * 0.27 * (enc_value2 - enc_last_value2) / 1440.0;
    v2 = 1000 * dl2 / dt;
    enc_last_value2 = enc_value2;

    l1 += v1 * dt / 1000.0;
    l2 += v2 * dt / 1000.0;
    v_motor1 = pid1.calculate(v_target1, v1, dt);
    v_motor2 = pid2.calculate(v_target2, v2, dt);
    Motor1.commandMotor(v_motor1);
    Motor2.commandMotor(v_motor2);
    Serial.print("v_target1=");
    Serial.print(v_target1);
    Serial.print(";");
    Serial.print("v_target2=");
    Serial.print(v_target2);
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
    speed_msg.v1 = v1;
    speed_msg.v1 = v2;
    pose_msg.l1 = l1;
    pose_msg.l2 = l2;
    pose_pub.publish(&pose_msg);
    speed_pub.publish(&speed_msg);

  }




  nh.spinOnce();
  delay(10);



}
