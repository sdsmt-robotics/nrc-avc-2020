#include <Arduino_LSM9DS1.h>
#include <Servo.h>
#include "LedStrip.h"
#include "PID.h"
#include "Encoder.h"
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

#define DEBUG true

//set up servo objects and default values found through testing
Servo turnR;
const int default_angle_R = 91;
const int default_lower_angle_R = 0;
const int default_upper_angle_R = 170;
Servo turnL;
const int default_angle_L = 91;
const int default_lower_angle_L = 0;
const int default_upper_angle_L = 170;
Servo motor;
const int default_angle_M = 90;
const int default_lower_angle_M = 0;
const int default_upper_angle_M = 170;


//Maximum power allowed
const float maxPower = 0.2;

// Conversion from RPM to M/s
const float RPM_TO_M_S = (140 * PI) / 1000 / 60;

// Controller update interval in milliseconds
const unsigned long UPDATE_INTERVAL = 10;

LedStrip ledStrip(8, 7, 6);

float kp = 0.06, ki = 0.02, kd = 0.0, N = 10;
PID speedController(kp, ki, kd, N, UPDATE_INTERVAL);

Encoder encoder(5, 4.5, 100);

// ROS variables
// Set up master node handler
ros::NodeHandle  nh;

// Create variables for publishers
std_msgs::Float32 servo_return;
std_msgs::Float32 velocity_return;
sensor_msgs::Imu imu_return;

// Create publishers
ros::Publisher servo_pub("wheel_angle_current", &servo_return);
ros::Publisher velocity_pub("speed_current", &velocity_return);
ros::Publisher imu_pub("imu", &imu_return);

// Create subscribers
void send_imu_data();
void motor_cb( const std_msgs::Float32& msg);
void servo_cb( const std_msgs::Float32& msg);
ros::Subscriber<std_msgs::Float32> motor_sub("target_velocity", motor_cb);
ros::Subscriber<std_msgs::Float32> servo_sub("target_wheel_angle", servo_cb);

//================SETUP==============================
void setup() {
  // Initialize the LED Strip
  ledStrip.init();
  ledStrip.setBrightness(255);
  ledStrip.setColorRGB(255, 0, 0);
  
  Serial.begin(115200);
  if (DEBUG) Serial.println("Start.");
  
  //Initialize servos and main motor
  if (DEBUG) Serial.println("Initializing motors...");
  speedController.setLimits(0, 0.25);
  initMotors();
  
  if (DEBUG) Serial.println("Initializing encoder...");
  encoder.init();

  ledStrip.setColorRGB(0, 255, 0);
  if (DEBUG) Serial.println("Ready!");

  IMU.begin();

  // Initialize all publishers and subscribers
  nh.initNode();
  nh.subscribe(servo_sub);
  nh.subscribe(motor_sub);
  nh.advertise(servo_pub);
  nh.advertise(velocity_pub);
  nh.advertise(imu_pub);
}


//================LOOP==============================
void loop() {
  nh.spinOnce();
  send_imu_data();
  delay(1);
}


//================CAR Functions======================
//Attempts to set the motor to a reasonable speed based on input.
//Returns false on invalid input although velocity will be set to max or min depending.
float power;
void setDriveSpeed(float speed) //m/s
{
  static unsigned long lastUpdate = millis();
  speedController.setTarget(speed);

  // Do a reset if it's been a while
  if (millis() - lastUpdate > UPDATE_INTERVAL * 5) {
    speedController.reset();
  }

  if (millis() - lastUpdate > UPDATE_INTERVAL) {
    encoder.estimateSpeed();
    
    //Get the power level from the speed controller
    power = speedController.calculateOutput(getDriveSpeed());

    //set the motor power
    setDrivePower(power);
    
    lastUpdate = millis();
  }
}

/**
* Get the current speed of the car in m/s.
*/
float getDriveSpeed() {
  return encoder.getFilteredSpeed() * RPM_TO_M_S;
}

/**
* Set the power level for the main motor.
* 
* @param power - power level for the motor [0 to 1.0] 
*/
void setDrivePower(float power)
{
  int angM = 0;

  // Convert to a motor power level (0 to 180)
  angM = default_angle_M + power * (default_upper_angle_M - default_angle_M);
  
  // Constrain to valid
  angM = constrain(angM, default_angle_M, default_upper_angle_M);

  motor.write(angM);
}


/**
* Set the steering angle for the servos. Zero is measured from forward on vehicle.
* 
* @param dir - Angle to set steering [-25 to 25] 
*/
int setDriveAngle(int dir) // max -25(left) to 25(right)
{
  bool  valid = true;
  int   angR = 0;
  int   angL = 0;

  //check input value for validity and constrain it if it is not valid
  if ((dir > 25) || (dir < -25))
  {
    dir = constrain(dir, -25, 25);
  }

  //map input angle to corresponding servo angle
   angR = int(map(dir,-25,25,default_lower_angle_R,default_upper_angle_R));
   angL = int(map(dir,-25,25,default_lower_angle_L,default_upper_angle_L));

   //turn servos
   turnR.write(angR);
   turnL.write(angL);
   return dir;
}

/**
 * Initialize AVC servos and main motor.
 */
void initMotors() {
  // Steering servos
  turnR.attach(9);
  turnR.write(default_angle_R);
  turnL.attach(10);
  turnL.write(default_angle_L);

  // Main motor
  motor.attach(11);
  motor.write(90);

  // Wait for ESC to be OK with life
  delay(2500);
}

////================TESTING FUNCTIONS===================
///**
// * Test the various car functions.
// */
//void testCarFunctions() {
//  Serial.println("Running Servos...");
//  delay(2000);
//  setDriveAngle(-25);
//  delay(800);
//  setDriveAngle(25);
//  delay(800);
//  setDriveAngle(0);
//  
//  Serial.println("Running LEDs...");
//  delay(1000);
//  ledStrip.setBrightness(255);
//  ledStrip.setColorRGB(255, 0, 0);
//  
//  Serial.println("Driving...");
//  delay(500);
//  setDrivePower(0.12);
//
//  delay(3000);
//  setDrivePower(0);
//  while (true) {
//    Serial.println("Done.");
//    delay(1000);
//  }
//}
//
///**
// * Test outputing encoder speeds.
// */
//void testEncoder() {
//  static unsigned long lastUpdate = millis();
//
//
//  if (millis() - lastUpdate > UPDATE_INTERVAL) {
//    Serial.print("0, ");
//    Serial.print(encoder.estimateSpeed());
//    Serial.print(", ");
//    Serial.println(encoder.getSpeed());
//    //Serial.print(", ");
//
//    //set the motor power
//    setDrivePower(0.1);
//    
//    lastUpdate = millis();
//  }
//}
//
///**
// * Test changing the speed to a new value every three seconds.
// */
//void testChangingSpeed() {
//  const float speeds[] = {2, 3, 0.8};
//  const int numSpeeds = 3;
//  static int speedIndex = 0;
//  static unsigned long lastSpeedChange = millis();
//  static unsigned long lastPrint = millis();
//  static unsigned long lastUpdate = millis();
//
//  // Adjust PID vals
//  // Format: Xd.dddd where X is the thing to change and d.dddd is the new value.
//  if (Serial.available() > 0) {
//    char change = Serial.read();
//    Serial.setTimeout(10);
//    float val = Serial.parseFloat();
//
//    switch (change) {
//      case 'p':
//        kp = val;
//      break;
//      case 'i':
//        ki = val;
//      break;
//      case 'd':
//        kd = val;
//      break;
//      case 'n':
//        N = val;
//      break;
//      
//    }
//    speedController.setConstants(kp, ki, kd, N, UPDATE_INTERVAL);
//    delay(500);
//  }
//
//  //go to the next speed if past time
//  if (millis() - lastSpeedChange > 2000) {
//    speedIndex++;
//    lastSpeedChange = millis();
//    if (speedIndex >= numSpeeds) {
//      speedIndex = 0;
//    }
//  }
//
//
//  //Get the power level from the speed controller
//  if (millis() - lastUpdate > UPDATE_INTERVAL) {
//    //set the target speed
//    setDriveSpeed(speeds[speedIndex]);
//    lastUpdate = millis();
//  }
//  
//  //Output current speed
//  if (millis() - lastPrint > 20) {
//    Serial.print("0,\t");
//    Serial.print(speedController.getTarget()*100);
//    Serial.print("0,\t");
//    Serial.print(power*100);
//    Serial.print(",\t");
//    Serial.println(getDriveSpeed()*100);
//    
//    lastPrint = millis();
//  }
//}


//================ROS Functions======================
/**
 * Test changing the speed to a new value every three seconds.
 */
void servo_cb( const std_msgs::Float32& msg){
  int deg = int(msg.data) * 180.0 / Pi;
  deg = setDriveAngle(deg);
  servo_return.data = deg * Pi / 180.0;
  servo_pub.publish(&servo_return);
}

/**
 * Test changing the speed to a new value every three seconds.
 */
void motor_cb( const std_msgs::Float32& msg){
  setDriveSpeed(msg.data);
  velocity_return.data = msg.data;
  velocity_pub.publish(&velocity_return);
}

/**
 * Test changing the speed to a new value every three seconds.
 */
void send_imu_data()
{
    float mag_offset_x = 0;
    float mag_offset_y = 0;
    float mag_offset_z = 0;
    float gyro_x, gyro_y, gyro_z;
    float acc_x, acc_y, acc_z;
    float mag_x, mag_y, mag_z;
    if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable() && IMU.magneticFieldAvailable())
    {
      IMU.readAcceleration(acc_x, acc_y, acc_z);
      IMU.readGyroscope(gyro_x, gyro_y, gyro_z);
      IMU.readMagneticField(mag_x, mag_y, mag_z);
    }
    mag_x += mag_offset_x;
    mag_y += mag_offset_y;
    mag_z += mag_offset_z;
    float mag_yaw = atan2(mag_y, mag_x);
    // Assume variance of 0.01 (std = 0.1 radians or ~10 degrees)
    // For unknown values assume 1 (std = 1 degree)
    float orientation_covariance [9] = {1, 0, 0, 0, 1, 0, 0, 0, 0.01};
    // We don't have values, so the first element should be -1
    float angular_velocity_covariance [9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    // Assume variance of 0.01 (std = 0.1 m/s)
    // For unknown values assume 0.000001 (std = 1 mm/s)
    float linear_acceleration_covariance [9] = {0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01};
    //imu_return.header = std_msgs::Header;

    // Equations to convert just a yaw angle to a quaternion (assumes others are zero)
    imu_return.orientation.x = 0;
    imu_return.orientation.y = 0;
    imu_return.orientation.z = sin(mag_yaw / 2);
    imu_return.orientation.w = cos(mag_yaw / 2);
    *imu_return.orientation_covariance = *orientation_covariance;

    // Gyroscope angular velocities
    imu_return.angular_velocity.x = gyro_x;
    imu_return.linear_acceleration.y = gyro_y;
    imu_return.linear_acceleration.z = gyro_z;
    *imu_return.angular_velocity_covariance = *angular_velocity_covariance;

    // X is forward, Y is Left, Z is up
    // The accelerometer does weird things (gravity is 8-9 Gs instead of 1)
    // For now we assume that we can scale that and all other measurements back to 1 and get reasonable values. 
    imu_return.linear_acceleration.x = acc_x;
    imu_return.linear_acceleration.y = acc_y;
    imu_return.linear_acceleration.z = acc_z;
    *imu_return.linear_acceleration_covariance = *linear_acceleration_covariance;

    // Publish the IMU data
    imu_pub.publish(&imu_return);
}
