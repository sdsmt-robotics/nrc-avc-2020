/*
 * rosserial Servo Control Example
 *
 * This sketch demonstrates the control of hobby R/C servos
 * using ROS and the arduiono
 * 
 * For the full tutorial write up, visit
 * www.ros.org/wiki/rosserial_arduino_demos
 *
 * For more information on the Arduino Servo Library
 * Checkout :
 * http://www.arduino.cc/en/Reference/Servo
 */

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

// Define which ROS messages will be used
#include <Servo.h> 
#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

// Define global variables
ros::NodeHandle  nh;
std_msgs::Int16 servo_return;
std_msgs::Int16 rpm_return;
ros::Publisher servo_pub("servo_return", &servo_return);
ros::Publisher rpm_pub("rpm", &rpm_return);
float target_vel = 0;

// Define the servo callback function
Servo servo;

void servo_cb( const std_msgs::Int16& cmd_msg){
  servo.write(cmd_msg.data); //set servo angle, should be from 0-180
  digitalWrite(13, HIGH-digitalRead(13));  //toggle led
  servo_return.data = cmd_msg.data + 10;
  servo_pub.publish(&servo_return);
}

ros::Subscriber<std_msgs::Int16> servo_sub("avc_angle", servo_cb);

// Define the servo callback function
Servo motor_controller;

void motor_controller_cb( const std_msgs::Int16& cmd_msg){
  // Map input to vehicle speed
  // 90 is 0
  // 110 is reasonably fast
  // 
  target_vel = cmd_msg.data
}

ros::Subscriber<std_msgs::Int16> motor_sub("avc_angle", servo_cb);

void speed_control()
{
  // Map input to vehicle speed
  // 90 is 0
  // 110 is reasonably fast
  // 
  motor_controller.write(cmd_msg.data); //set servo angle, should be from 0-180
}

void setup(){
  pinMode(13, OUTPUT);

  // Initiate all publishers and subscribers
  nh.initNode();
  nh.subscribe(servo_sub);
  nh.subscribe(motor_sub);
  nh.advertise(servo_pub);
  nh.advertise(rpm_pub);
  
  servo.attach(9); //attach it to pin 9
  motor_controller.attach(10); //attach it to pin 9
}

void loop(){

  // Check for new information every 1 ms
  nh.spinOnce();
  speed_control();
  delay(1);
}
