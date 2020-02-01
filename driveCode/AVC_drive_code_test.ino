
#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

/*
 *  Subscribes to "speed_set" a Float32 that sets target speed 0 to 20 m/s
 *  Subscribes to "turn_angle" a Int16 that sets target dive angle -20 to 20
 *  Publishes speed new to "speed_current" as a Float32
 *
 */
#include <Servo.h>

#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int16.h>

//set up servo objects and default values found through testing
Servo turnR;
const int default_angle_R = 87;
const int default_lower_angle_R = 0;
const int default_upper_angle_R = 170;
Servo turnL;
const int default_angle_L = 87;
const int default_lower_angle_L = 0;
const int default_upper_angle_L = 170;
Servo motor;
const int default_angle_M = 0;
const int default_lower_angle_M = 0;
const int default_upper_angle_M = 170;

//pin to be used to increment the rev count
const int interrupt_pin = 2;

//Maximum velocity allowed
const int maxV = 20;

//ratio of rotational distance of wheel to one revolution from measured gear
const double rev_to_speed = 1;

//number of pulses from the revolution sensor triggered by interrupts
volatile int revolution_count = 0;

// Define global variables for ross
ros::NodeHandle  node_handle;
std_msgs::Float32 speed_current;
ros::Publisher returner("speed_current", &speed_current);

//Attempts to set the motor to a reasonable speed based on input.
//Returns false on invalid input although velocity will be set to max or min depending.
void set_speed_AVC(const std_msgs::Float32& velocity) //m/s
{
  bool  valid = true;
  int   angM = 0;

  //check input value for validity and constrain it if it is not valid
  if ((velocity.data > maxV) || (velocity.data < 0))
  {
    valid = false;
    constrain(velocity.data, 0, maxV);
  }  

  //map speed to servo angle (probably need to fit this to a function instead of just liner mapping it)
  //also could do feedback on the arduino side. Speed is set externally and then the car just
  //tries to reach said speed on the arduino side
  angM = int(map(velocity.data,0,maxV,default_lower_angle_M,default_upper_angle_M));
  motor.write(angM);

  return;
}

//Called when the rpm sensor pulses
void rev_interupt()
{
  ++revolution_count;
  return;
}

double get_speed()
{
  static double last_time; //in milliseconds
  double speed_new = rev_to_speed * revolution_count / (1000*(millis()- last_time));//m/s

  //reset for the next call
  revolution_count = 0;
  last_time = millis();
 
  return speed_new;
}

//Attempts to set the steering to the imputed angle (approximate). Zero is measured from forward on vehicle.
//Returns false under invalid input although angle will still be set to max or min depending.
void set_turn_AVC(const std_msgs::Int16& dir) // max -25(left) to 25(right)
{
  bool  valid = true;
  int   angR = 0;
  int   angL = 0;

  //check input value for validity and constrain it if it is not valid
  if ((dir.data > 25) || (dir.data < -25))
  {
    valid = false;
    constrain(dir.data, -25, 25);
  }

  //map input angle to corresponding servo angle
   angR = int(map(dir.data,-25,25,default_lower_angle_R,default_upper_angle_R));
   angL = int(map(dir.data,-25,25,default_lower_angle_L,default_upper_angle_L));

   //turn servos
   turnR.write(angR);
   turnL.write(angL);

   return;
}

ros::Subscriber<std_msgs::Int16> sub1("turn_angle", set_turn_AVC);
ros::Subscriber<std_msgs::Float32> sub2("speed_set", set_speed_AVC);


void setup() {

  //Attach each servo and set to reasonable initial values
  turnR.attach(9);
  turnR.write(default_angle_R);
  turnL.attach(10);
  turnL.write(default_angle_L);
  motor.attach(11);
  motor.write(default_angle_M);

  //set up for interrupts from revolution sensor
  attachInterrupt(digitalPinToInterrupt(interrupt_pin), rev_interupt, FALLING);

  //set up ros node
  node_handle.initNode();
  node_handle.subscribe(sub1);
  node_handle.subscribe(sub2);
  node_handle.advertise(returner);
}

void loop() {
    int i;
    if ((i > 50) || (i < 0))
    {
      i = 0;
      speed_current.data = get_speed();
      returner.publish(&speed_current);
    }
    delay(1);
    node_handle.spinOnce();
}
