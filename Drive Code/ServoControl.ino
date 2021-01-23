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
#include <std_msgs/UInt16.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

// Define global variables
ros::NodeHandle  nh;
std_msgs::String str_msg;
std_msgs::UInt16 servo_return;
ros::Publisher returner("servo_return", &servo_return);

// Define the servo callback function
Servo servo;

void servo_cb( const std_msgs::UInt16& cmd_msg){
  servo.write(cmd_msg.data); //set servo angle, should be from 0-180  
  digitalWrite(13, HIGH-digitalRead(13));  //toggle led  
  servo_return.data = cmd_msg.data + 10;
  returner.publish(&servo_return);
}

ros::Subscriber<std_msgs::UInt16> sub("servo", servo_cb);

void setup(){
  pinMode(13, OUTPUT);

  // Initiate all publishers and subscribers
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(returner);
  
  servo.attach(9); //attach it to pin 9
}

void loop(){

  // Check for new information every 1 ms
  nh.spinOnce();
  delay(1);
}
