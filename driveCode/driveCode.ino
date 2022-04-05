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
const int default_lower_period_motor = 1000;
const int default_upper_period_motor = 2000;

// Magnetometer calibration variables
int calibration_duration = 30;
float mag_offset_x = 0;
float mag_offset_y = 0;

//Maximum power allowed
const float maxPower = 0.25;

// Conversion from RPM to M/s
const float RPM_TO_M_S = (140 * PI) / 1000 / 60;

// Controller update interval in milliseconds
const unsigned long UPDATE_INTERVAL = 10;

LedStrip ledStrip(8, 7, 6);

//float kp = 0.06, ki = 0.02, kd = 0.0, N = 10;
float kp = 0.08, ki = 0.1, kd = 0.0, N = 10;
PID speedController(kp, ki, kd, N, UPDATE_INTERVAL);

Encoder encoder(4, 4.5, 100);

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

float timeFloat = 0; // s
float increment = 100; // ms
float powerMultiplier = 0.1;
uint32_t timeStart = 0; // ms
uint32_t timeToRun = 10000; // ms

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
  speedController.setLimits(0, 0.3);
  initMotors();

  if (DEBUG) Serial.println("Calibrating IMU, Please spin me around!");
  ledStrip.setColorRGB(0, 0, 255);
  IMU.begin();
  delay(1000);
  //imu_calibrate_hard_iron(calibration_duration);

  if (DEBUG) Serial.println("Calibrated, initializing ROS...");


  // Initialize all publishers and subscribers
  nh.initNode();
  nh.subscribe(servo_sub);
  nh.subscribe(motor_sub);
  nh.advertise(servo_pub);
  nh.advertise(velocity_pub);
  nh.advertise(imu_pub);

  if (DEBUG) Serial.println("ROS Initialized!");

  if (DEBUG) Serial.println("Initializing encoder...");
  encoder.init();
  pinMode(4, INPUT);
  if (DEBUG) Serial.println("Ready!");

//  // hold the throttle at its max for ESC startup, this will "train" a new max throttle value
//  motor.writeMicroseconds(2000);
//  delay(3000);
//  Serial.println("Calibrating high throttle value, waiting for input...");
//  while (Serial.read() == -1);
//
//  // take the throttle low when a key is pressed
//  motor.writeMicroseconds(1000);
//  Serial.println("Calibrating low throttle value...");
//  delay(5000);
//  Serial.println("Begin!");
//  ledStrip.setColorRGB(0, 255, 0);
//  timeStart = millis();
  
  // wait for something to come in on the serial terminal to start
  while (Serial.read() != -1); // clear the buffer
  Serial.println("Waiting for input to start...");
  while (Serial.read() == -1);
  Serial.println("Begin!");
  ledStrip.setColorRGB(0, 255, 0);
  timeStart = millis();
}

//uint32_t prevTime = 0;
//uint32_t period = 4000;
//bool state = 1;
//float vel = 0;

//================LOOP==============================
void loop() {
  //  nh.spinOnce();
  //  send_imu_data();
  //  delay(1);

  // output a happy little sine wave to the ESC
//  timeFloat = (float)(millis()) / 1000.0;
//  float drivePower = abs(sin(timeFloat / 3.0) * powerMultiplier);
//  setDrivePower(drivePower);
//  Serial.print("Power: ");
//  Serial.println(drivePower);
//  delay(100);
//
//  // stop the car after a while
//  if (millis() - timeStart > timeToRun)
//  {
//    setDrivePower(0);
//    while (Serial.read() != -1); // clear the serial buffer
//    
//    Serial.println("Stopped!");
//    Serial.println("Waiting for input to start...");
//    ledStrip.setColorRGB(0, 0, 255);
//
//    // wait for something to come in on the serial terminal
//    while (Serial.read() == -1);
//    
//    ledStrip.setColorRGB(0, 255, 0);
//    timeStart = millis();
//  }

  setDriveAngle(25);
  delay(2000);
  setDriveAngle(-25);
  delay(2000);
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
    float currentSpeed = getDriveSpeed();
    power = speedController.calculateOutput(currentSpeed);

    //set the motor power
    setDrivePower(power);

    lastUpdate = millis();
  }
}

/**
  Get the current speed of the car in m/s.
*/
float getDriveSpeed() {
  return encoder.getFilteredSpeed() * RPM_TO_M_S;
}

/**
  Set the power level for the main motor.

  @param power - power level for the motor [0 to 1.0]
*/
void setDrivePower(float power)
{
  // Convert to a motor PWM period
  int period = map(int(power * 1000), 0, 1000, 1000, 2000);

  motor.writeMicroseconds(period);
}


/**
  Set the steering angle for the servos. Zero is measured from forward on vehicle.

  @param dir - Angle to set steering [-25 to 25]
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
  angR = int(map(dir, -25, 25, default_lower_angle_R, default_upper_angle_R));
  angL = int(map(dir, -25, 25, default_lower_angle_L, default_upper_angle_L));

  //turn servos
  turnR.write(angR);
  turnL.write(angL);
  return dir;
}

/**
   Initialize AVC servos and main motor.
*/
void initMotors() {
  // Steering servos
  turnR.attach(9);
  turnR.write(default_angle_R);
  turnL.attach(10);
  turnL.write(default_angle_L);

  // Main motor
  motor.attach(11, 1000, 2000);
  motor.setDrivePower(0);  

  //SET UP THE STARTUP PROCEDURE TO HOLD THROTTLE HIGH WHEN ESC IS POWERED ON, HOLD THERE FOR AT LEAST TWO SECONDS, THEN GO LOW
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
   Get the target wheel angle from ROS, update the local wheel angle and return the actual wheel angle to ROS
*/
void servo_cb( const std_msgs::Float32& msg) {
  int deg = int(msg.data) * 180.0 / 3.1415926535898;
  deg = setDriveAngle(deg);
  servo_return.data = deg * 3.1415926535898 / 180.0;
  servo_pub.publish(&servo_return);
}

/**
   Get the target speed from ROS, update the local speed and return the actual speed to ROS
*/
void motor_cb( const std_msgs::Float32& msg) {
  setDriveSpeed(msg.data);
  velocity_return.data = getDriveSpeed();
  velocity_pub.publish(&velocity_return);
}

/**
   Send IMU data over the imu_pub ROS topic
*/
void send_imu_data()
{
  float gyro_x, gyro_y, gyro_z;
  float acc_x, acc_y, acc_z;
  float mag_x, mag_y, mag_z;
  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable() && IMU.magneticFieldAvailable())
  {
    IMU.readAcceleration(acc_x, acc_y, acc_z);
    IMU.readGyroscope(gyro_x, gyro_y, gyro_z);
    IMU.readMagneticField(mag_x, mag_y, mag_z);
    mag_x -= mag_offset_x;
    mag_y -= mag_offset_y;
    float mag_yaw = atan2(mag_y, mag_x);
    //      Serial.print("mag yaw:");
    //      Serial.println(mag_yaw);
    // Assume variance of 0.0025 (std = 0.05 radians or ~3 degrees)
    // For unknown values assume 1 (std = 1 degree)
    float orientation_covariance [9] = {0.0025, 0, 0, 0, 0.0025, 0, 0, 0, 0.0025};
    // Assume variance of 0.0025 (std = 0.05 rad/s or ~3 deg/s)
    float angular_velocity_covariance [9] = {0.0025, 0, 0, 0, 0.0025, 0, 0, 0, 0.0025};
    // Assume variance of 0.0025 (std = 0.05 m/s^2)
    float linear_acceleration_covariance [9] = {0.0025, 0, 0, 0, 0.0025, 0, 0, 0, 0.0025};
    //imu_return.header = std_msgs::Header;

    // Equations to convert just a yaw angle to a quaternion (assumes others are zero)
    imu_return.orientation.x = 0;
    imu_return.orientation.y = 0;
    imu_return.orientation.z = sin(mag_yaw / 2);
    imu_return.orientation.w = cos(mag_yaw / 2);
    *imu_return.orientation_covariance = *orientation_covariance;

    // Gyroscope angular velocities
    imu_return.angular_velocity.x = gyro_x;
    imu_return.angular_velocity.y = gyro_y;
    imu_return.angular_velocity.z = gyro_z;
    *imu_return.angular_velocity_covariance = *angular_velocity_covariance;

    // Accelerometer linear accelerations
    imu_return.linear_acceleration.x = acc_x;
    imu_return.linear_acceleration.y = acc_y;
    imu_return.linear_acceleration.z = acc_z;
    *imu_return.linear_acceleration_covariance = *linear_acceleration_covariance;

    // Publish the IMU data
    imu_pub.publish(&imu_return);
  }
}

/**
   Hard iron IMU calibration (centering the circle, not scaling it)
*/
void imu_calibrate_hard_iron(int duration)
{
  float mx_min = 0;
  float my_min = 0;
  float mx_max = 0;
  float my_max = 0;
  float mag_x, mag_y, mag_z;
  float end_time = duration * 1000;
  float current_time = 0;
  while (current_time < end_time)
  {
    if (IMU.magneticFieldAvailable())
    {
      IMU.readMagneticField(mag_x, mag_y, mag_z);
      if (mag_x > mx_max) mx_max = mag_x;
      if (mag_y > my_max) my_max = mag_y;
      if (mag_x < mx_min) mx_min = mag_x;
      if (mag_y < my_min) my_min = mag_y;
      mag_offset_x = (mx_min + mx_max) / 2;
      mag_offset_y = (my_min + my_max) / 2;
    }
    delay(10);
    current_time += 10;
  }
  return;
}
